import time
import struct
import numpy as np
import casadi as ca
from acados_template import AcadosOcp, AcadosModel, AcadosOcpSolver
from scipy.linalg import solve_continuous_are
from multiprocessing import shared_memory
import signal

from .config import CORE_CONTROLLER, SHM_NAME, NUM_SENSORS, ANGLE2DIS
from .utils import set_realtime, graceful_shutdown
from .ekf import EKF

# Controller logic process
def controller_logic():
    time.sleep(0.5)
    signal.signal(signal.SIGINT,  _graceful_shutdown)
    signal.signal(signal.SIGTERM, _graceful_shutdown)
    set_realtime(CORE_CONTROLLER, 80)
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    buf = shm.buf
    # print(f"[Controller] Running on core {CORE_CONTROLLER}")
    duty = 0
    reached_end = False
    start_control = False
    offset_duty = NUM_SENSORS * 8
    control_freq = 50 #Hz
    lim = 100 #%
    prev_est = [0,0,0,0]
    Ts = 1/control_freq

    # System parameters
    mc_mp               = 3.3039   
    mp_l                = 0.0163
    mp_l2_plus_Icom     = 0.0056
    cc                  = 20.4037
    cp                  = 3.6e-4
    k                   = 0.3809
    g                   = 9.81

    # Linearized model matrices 
    # Common denominator
    Delta = mc_mp*mp_l2_plus_Icom - mp_l**2 
    # State-space A matrix
    A = np.zeros([4, 4])
    A[0,1] = 1
    A[1,0] = mp_l * g * (mc_mp) / Delta
    A[1,1] = -cp * (mc_mp) / Delta
    A[1,3] = -cc * mp_l / Delta
    A[2,3] = 1
    A[3,0] = (mp_l)**2 * g / Delta
    A[3,1] = -cp * mp_l / Delta
    A[3,3] = -cc * (mp_l2_plus_Icom) / Delta

    # Input B matrix
    B = np.zeros([4,1])
    B[1] = k * mp_l / Delta
    B[3] = k*(mp_l2_plus_Icom) / Delta
    A_c = A
    B_c = B 

    A = ca.DM(A_c)
    B = ca.DM(B_c)

    # Build Acados model 
    model = AcadosModel()
    model.name = "pendulum_cart_mpc"

    # States & controls
    x      = ca.MX.sym("x", 4)
    u      = ca.MX.sym("u", 1)
    xdot   = A @ x + B @ u
    xdot_v = ca.MX.sym("xdot", 4)

    model.x           = x
    model.u           = u
    model.xdot        = xdot_v
    model.f_impl_expr = xdot_v - xdot
    model.f_expl_expr = xdot

    # Set up OCP
    ocp = AcadosOcp()
    ocp.model = model

    # dimensions
    ocp.dims.nx = model.x.size1()
    ocp.dims.nu = model.u.size1()
    ocp.solver_options.N_horizon = 40       # horizon length

    ocp.dims.lbx_0 = ocp.dims.nx   # = 4
    ocp.dims.ubx_0 = ocp.dims.nx   # = 4

    # 1) control bounds
    ocp.constraints.lbu     = np.array([-100.0])
    ocp.constraints.ubu     = np.array([100.0])
    ocp.constraints.idxbu   = np.array([0])

    # 2) state bounds (only on x[2])
    ocp.constraints.idxbx   = np.array([2])
    ocp.constraints.lbx     = np.array([-0.4])
    ocp.constraints.ubx     = np.array([ 0.4])

    # stage cost weights
    Q = np.diag([1, 0.001, 100, 0.001])
    R = np.array([[0.01]])
    W = np.block([[Q,               np.zeros((4,1))],
                [np.zeros((1,4)),            R      ]])

    P = solve_continuous_are(A_c, B_c, Q, R) # terminal cost weight

    # 4.1: Stage cost
    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    ocp.cost.W           = W
    ocp.cost.W_e         = P      # terminal weight on x

    ocp.cost.yref   = np.zeros(5)   # [x; u]
    ocp.cost.yref_e = np.zeros(4)   # terminal only on x

    ocp.cost.Vx = np.vstack([np.eye(4), np.zeros((1,4))])  # shape (5,4)
    ocp.cost.Vu = np.vstack([np.zeros((4,1)), np.eye(1)])  # shape (5,1)

    ocp.cost.Vx_e = np.eye(4)        # maps the 4×1 state into the 4×1 y_e
    ocp.cost.Vu_e = np.zeros((4,1))  # no control at the terminal cost

    # 4.2: **Initial‐stage cost** (to fix W_0 / Vx_0 mismatch)
    ocp.cost.cost_type_0 = "LINEAR_LS"
    ocp.cost.W_0         = W
    ocp.cost.yref_0      = np.zeros(5)
    # map x→y and u→y for initial stage
    ocp.cost.Vx_0 = np.vstack([np.eye(4), np.zeros((1,4))])  # 5×4
    ocp.cost.Vu_0 = np.vstack([np.zeros((4,1)), np.eye(1)])  # 5×1

    # solver options
    ocp.solver_options.tf                  = 0.02     # sampling time
    ocp.solver_options.integrator_type     = "IRK"
    ocp.solver_options.nlp_solver_type     = "SQP_RTI" # RTI since it is linear and more iterations not needed
    ocp.solver_options.hessian_approx      = "GAUSS_NEWTON"
    ocp.solver_options.qp_solver           = "FULL_CONDENSING_QPOASES"

    x0        = np.array([0.0, 0.0, np.deg2rad(5.0), 0.0])
    ocp.constraints.x0 = x0  # initial condition

    # export & create solver (will generate & compile)
    ocp.code_export_directory = "acados_ocp_pendulum_real"
    ocp.model_export_format   = "json"
    solver = AcadosOcpSolver(ocp, json_file="pendulum_cart_mpc.json")

    EKF_ = EKF()
    prev_PA, prev_DA, prev_PAvel = 0, 0, 0
    alpha_PA, alpha_DA, alpha_PAvel = 0.15, 0.87, 0.87
    K_lqr = np.array([1.5*471, 115, -3, -100])
    I = 0
    i=0
    ref_offs = 0
    MPC_active = False
    try:
        while True:
            start_time  = time.perf_counter()

            # get sensor values and previous control input for use in Extended Kalman Filter
            sensors     = [struct.unpack_from('d', buf, i*8)[0] for i in range(NUM_SENSORS)]
            PA_vel      = struct.unpack_from('d', buf, 7*8)[0]
            DA_vel      = struct.unpack_from('d', buf, 12*8)[0]*ANGLE2DIS
            u           = struct.unpack_from('d', buf, 4*8)[0]

            # Filter sensors with Extended Kalman Filter
            EKF_vals = EKF_.filter(np.array([sensors[0], PA_vel, sensors[1]*ANGLE2DIS, DA_vel]), u, Ts)
            PA = EKF_vals[0]
            DA = EKF_vals[2]
            PA_vel = EKF_vals[1]
            DA_vel = EKF_vals[3]

            # get reference value depending on which direction swing up is
            ref = 180 if abs(PA - 180) < abs(PA + 180) else -180
            if MPC_active:
                    # slow-varying adaptive set-point to overcome calibration error
                    ref_offs += (2/(3*control_freq)*DA) #2 deg per 3s per 1 meter
                    ref += ref_offs    
                    # print(ref)
            x = np.array([[(PA-ref)*np.pi/180],[PA_vel*np.pi/180],[DA],[DA_vel]])

            # # LPF for angle sensors
            # PA = sensors[0]*(1-alpha_PA) + prev_PA*alpha_PA
            # DA = sensors[1]*(1-alpha_DA) + prev_DA*alpha_DA
            # PA_vel = PA_vel*(1-alpha_PAvel) + prev_PAvel*alpha_PAvel

            # write filtered states into shared memory for plotting
            struct.pack_into("d", buf, 8*8, PA)
            struct.pack_into("d", buf, 9*8, DA)
            struct.pack_into("d", buf, 10*8, PA_vel)
            struct.pack_into("d", buf, 11*8, DA_vel)
            prev_PA = PA
            prev_DA = DA
            prev_PAvel = PA_vel

            if sensors[2] < 1 or sensors[3] < 1:
                reached_end = True

            if reached_end:         
                duty = 0 #%
            else:
                if abs(DA) > 0.4:
                    duty = 0 #%
                else:
                    if True:
                        # I += (PA-180)*1/control_freq
                        # duty = 0.02*DA +35.5*(PA-180) + 30*I
                        # duty = 10*np.sin(start_time*0.5

                        # Energy based swing-up controller:
                        mc_mp               = 3.3039   
                        mp_l                = 0.0163
                        mp_l2_plus_Icom     = 0.0056
                        cc                  = 20.4037
                        cp                  = 3.6e-4
                        k                   = 0.3809
                        g                   = 9.81
                        Ed = 2*mp_l*9.81
                        E = 0.42*mp_l2_plus_Icom*(PA_vel*np.pi/180)**2 + mp_l*9.81*(1-np.cos(PA*np.pi/180))
                        if abs(PA) < 165:
                            duty = min(55,max(-55,-9*(Ed - E)*PA_vel*np.cos(PA*np.pi/180)))
                            MPC_active = False 
                        else:
                            MPC_active = True
                            # -K_lqr @ x
                            # fix initial state in the first shooting interval
                            solver.set(0, "lbx", x)
                            solver.set(0, "ubx", x)

                            # t_start = time.time()
                            solver.solve()
                            # comp_time[k] = time.time() - t_start

                            uopt = solver.get(0, "u").item()
                            # add a small input smoothing to combat noise propagation
                            lpf_u = 0.3
                            duty = uopt*(1-lpf_u) + lpf_u*duty

                        
                        # duty = 40*np.sin(5*time.time())
                        # duty = 0
                        # if i<50:
                        #     duty = 20
                        # elif i>=50 and i < 70:
                        #     duty = 0
                        # elif i>=70 and i<110:
                        #     duty = -40
                        # elif i>=110 and i<160:
                        #     duty = 0
                        # elif i>=160 and i<175:
                        #     duty = 70
                        # else:
                        #     duty = 0 
                    else:
                        duty = 0
        
            # write duty to shared memory for PWM generator
            struct.pack_into("d", buf, offset_duty, max(-lim, min(duty,lim)))
            # i +=1
            
            # SLeep for 70% of  remaining sampling time followed by 30% busy wait for accurate timing
            sleep_time = 1/control_freq - (time.perf_counter()-start_time)
            if sleep_time < 0:
                print("\n\n Controller + filter took longer than sampling time!!! \n\n")
            elif sleep_time < 0.75/control_freq:
                print("\n\n Controller + filter took longer than 25% sampling time!!! \n\n")
            time.sleep(max(0,sleep_time*0.7))
            while time.perf_counter() < start_time + 1/control_freq:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        print(f"reference is now: {ref}")
        shm.close()
