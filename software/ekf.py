# software/ekf.py
import numpy as np
import time

class EKF:
    def __init__(self):
        # identified parameters
        mc_mp               = 3.3039   
        mp_l                = 0.0163
        mp_l2_plus_Icom     = 0.0056
        cc                  = 20.4037
        cp                  = 3.6e-4
        k                   = 0.3809
        g                   = 9.81
        def f(x, u):
            F = k*u
            return np.array([x[1], 
                    ( - (mp_l*np.cos(x[0])) * ( F + mp_l*np.sin(x[0])*x[1]**2 - cc*x[3] ) - (mc_mp) * ( mp_l*g*np.sin(x[0]) + cp*x[1] ) )  / ( (mp_l2_plus_Icom)*(mc_mp) - (mp_l*np.cos(x[0]))**2 ), 
                    x[3], 
                    (  (mp_l*np.cos(x[0])) * ( mp_l*g*np.sin(x[0]) + cp*x[1] ) + (mp_l2_plus_Icom) * ( F + mp_l*np.sin(x[0])*x[1]**2 - cc*x[3] ) )   / ( (mp_l2_plus_Icom)*(mc_mp) - (mp_l*np.cos(x[0]))**2 )])
        self.f = f

        def F_lin(x, u):
            theta       = x[0]
            thetadot    = x[1]
            ydot        = x[3]
            col1 = [0,
                    (2*mp_l**2*np.cos(theta)*np.sin(theta)*(mc_mp*(cp*thetadot + g*mp_l*np.sin(theta)) + mp_l*np.cos(theta)*(mp_l*np.sin(theta)*thetadot**2 - cc*ydot + k*u)))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2)**2 - (mp_l*(mp_l*(2*np.cos(theta)**2 - 1)*thetadot**2 + g*mc_mp*np.cos(theta) + cc*ydot*np.sin(theta) - k*u*np.sin(theta)))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2),
                    0,
                    (g*mp_l**2*np.cos(theta)**2 - mp_l*np.sin(theta)*(cp*thetadot + g*mp_l*np.sin(theta)) + mp_l*mp_l2_plus_Icom*thetadot**2*np.cos(theta))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2) - (2*mp_l**2*np.cos(theta)*np.sin(theta)*(mp_l2_plus_Icom*(mp_l*np.sin(theta)*thetadot**2 - cc*ydot + k*u) + mp_l*np.cos(theta)*(cp*thetadot + g*mp_l*np.sin(theta))))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2)**2]
            col2 = [1,
                    -(thetadot*np.sin(2*theta)*mp_l**2 + cp*mc_mp)/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2),
                    0,
                    (mp_l*(cp*np.cos(theta) + 2*mp_l2_plus_Icom*thetadot*np.sin(theta)))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2)]
            col3 = [0,0,0,0]
            col4 = [0,
                    (cc*mp_l*np.cos(theta))/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2),
                    1,
                    -(cc*mp_l2_plus_Icom)/(mc_mp*mp_l2_plus_Icom - mp_l**2*np.cos(theta)**2)]
            F_l = np.array([col1, col2, col3, col4]).T
            return F_l*1/50 + np.eye(4)
        self.F_lin = F_lin
        self.prev_est = np.array([0,0,0,0])

        self.Q = np.diag([1e-1, 1e-1, 1e-1, 1e-1])
        self.R = np.diag([0.5e-1, 2e0, 2e-2, 6e-1])
        self.P = np.diag([1e3, 1e3, 1e3, 1e3])
        self.C = np.diag([1,1,1,1])


    def filter(self, meas, u, Ts):
        # ---------------Time update--------------------
        meas[0] = meas[0]*np.pi/180     # convert from deg to radians for model
        meas[1] = meas[1]*np.pi/180     # convert from deg to radians for model
        # forward evaluation
        k1 = self.f( self.prev_est,               u )
        k2 = self.f( self.prev_est + 0.5*Ts*k1,   u )
        k3 = self.f( self.prev_est + 0.5*Ts*k2,   u )
        k4 = self.f( self.prev_est + Ts*k3,       u )
        x_est_model = self.prev_est + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4)
        # linearized matrix F around self.prev_est (x_k-1)
        F_l = self.F_lin(self.prev_est, u)
        # update covariance
        self.P = F_l @ self.P @ F_l.T + self.Q

        # compute inovation for outlier rejection
        v = meas - x_est_model
        S = self.P + self.R
        d2 = float(v.T @ np.linalg.inv(S) @ v)

        # test for outliers (lower value for threshold is more rejection)
        if d2 > 3.5:
            # skip update step because measurement is an outlier
            x_est = x_est_model
            print(f"outlier detected at time: {round(time.perf_counter(),1)}")
        else:
            # ---------- Measurement update ----------- (C is not added since it is identity)
            # make kalman gain (C matrix is excluded due to identity)
            Kgain = self.P @ np.linalg.inv(self.P + self.R)
            # combine model estimate and measurement
            x_est = x_est_model + Kgain @ (v)
            # update estimate covariance for next iteration
            self.P = self.P - Kgain @ self.P

        # save estimate for next iteration
        self.prev_est = x_est
        return [x_est[0]*180/np.pi, x_est[1]*180/np.pi, x_est[2], x_est[3]]
