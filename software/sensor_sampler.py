import time
import struct
from multiprocessing import shared_memory
import signal

import RPi.GPIO as GPIO

from .config import CORE_SENSOR, SHM_NAME, bus1, bus15
from .utils import set_realtime, graceful_shutdown, read_angle

# Sensor sampling process
def sensor_sampling():
    signal.signal(signal.SIGINT,  _graceful_shutdown)
    signal.signal(signal.SIGTERM, _graceful_shutdown)
    set_realtime(CORE_SENSOR, 90)
    # print(f"[Sensor] Running on core {CORE_SENSOR}")
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    buf = shm.buf
    step = 8
    sampling_freq = 100 #hz
    DIS_vel_samp_prev1 = DIS_vel_samp_prev2 = DIS_vel_samp_prev3 = DIS_vel_samp_prev4 \
        = DIS_vel_samp_prev5 = DIS_vel_samp_prev6 = 0
    PA_vel_samp_prev1 = PA_vel_samp_prev2 = 0
    alpha1, alpha2 = 0.4, 0.4

    # log_filename = f"pendulum_log.csv"                  #LOGGING IS EXPENSIVE!!!
    # log_path = os.path.join("./plots/", log_filename)
    # log_file = open(log_path, mode='w', newline='')
    # csv_writer = csv.writer(log_file)
    # csv_writer.writerow(["time_s", "PA_deg", "DA_deg", "PWM_duty"])
    # log_start_time = time.time()

    # Limit switch pins
    LIMIT_PINS = [17, 27]       

    PA_rotations    = 0
    DIS_rotations   = 0
    prev_PA_angle   = prev_PA_angle_full    = read_angle(bus15)         #EXPENSIVE!!! TRY TO MINIM(SMBus.read_i2c_block_data FOR BOTH IN ONE)
    prev_DIS_angle  = prev_DIS_angle_full   =  read_angle(bus1)         #EXPENSIVE!!! TRY TO MINIMIZE
    GPIO.setmode(GPIO.BCM)

    PA_vel, DIS_vel = 0, 0
    # set each up as input with pull-down
    for pin in LIMIT_PINS:
        GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    try:
        while True:
            start_time = time.perf_counter()
            PA_ANGLE_OFFSET = struct.unpack_from('d', buf, 5*8)[0]
            DIS_ANGLE_OFFSET = struct.unpack_from('d', buf, 6*8)[0]

            # ------------limit switches-------------------------------
            lval_left = GPIO.input(17)
            lval_right = GPIO.input(27)

            # -------------- pendulum angle & vel ---------------------
            # read raw cumulative pendulum angle
            PA_raw_angle = read_angle(bus15)
            if PA_raw_angle is None:            # some error handling
                PA_raw_angle = prev_PA_angle

            # check if a full rotation crossing happened
            if PA_raw_angle - prev_PA_angle < -180:
                PA_rotations +=1
            elif PA_raw_angle - prev_PA_angle > 180:
                PA_rotations -= 1
            
            # obtaining cumulative angle and velocity
            prev_PA_angle = PA_raw_angle
            PA_angle = PA_raw_angle + PA_rotations * 360
            PA_vel_samp  = (PA_angle - prev_PA_angle_full)*sampling_freq
            PA_vel = (PA_vel_samp)*(1-alpha1) + alpha1 * PA_vel
            prev_PA_angle_full = PA_angle


            # -------------------- Distance & velocity -------------------
            # read raw cumulative distance wheel angle
            DIS_raw_angle = read_angle(bus1)
            if DIS_raw_angle - prev_DIS_angle < -180:
                DIS_rotations +=1
            elif DIS_raw_angle - prev_DIS_angle > 180:
                DIS_rotations -= 1
            prev_DIS_angle = DIS_raw_angle

            # obtaining cumulative angle and velocity
            DIS_angle = DIS_raw_angle + DIS_rotations * 360
            DIS_vel_samp = (DIS_angle - prev_DIS_angle_full)*sampling_freq
            # moving avg filter
            DIS_vel = (DIS_vel_samp)*(1-alpha2) + alpha2 * DIS_vel
            # DIS_vel = (DIS_vel_samp + DIS_vel_samp_prev1 + DIS_vel_samp_prev2 + DIS_vel_samp_prev3 \
            #            + DIS_vel_samp_prev4 + DIS_vel_samp_prev5 + DIS_vel_samp_prev6)/7
            # DIS_vel_samp_prev6 = DIS_vel_samp_prev5
            # DIS_vel_samp_prev5 = DIS_vel_samp_prev4
            # DIS_vel_samp_prev4 = DIS_vel_samp_prev3
            # DIS_vel_samp_prev3 = DIS_vel_samp_prev2
            # DIS_vel_samp_prev2 = DIS_vel_samp_prev1
            # DIS_vel_samp_prev1 = DIS_vel_samp
            prev_DIS_angle_full = DIS_angle
            # print(f"DA: {DIS_angle-DIS_ANGLE_OFFSET}")

            # pack into shared memory for reading by controller
            struct.pack_into('d', buf, 0*step, PA_angle-PA_ANGLE_OFFSET)
            struct.pack_into('d', buf, 1*step, (DIS_angle-DIS_ANGLE_OFFSET))
            struct.pack_into('d', buf, 7*step, PA_vel)
            struct.pack_into('d', buf, 12*step, DIS_vel)
            struct.pack_into('d', buf, 2*step, lval_left)
            struct.pack_into('d', buf, 3*step, lval_right)
            # Get current time
            # t_log = time.time() - log_start_time

            # # Read current PWM duty from shared memory for logging
            # pwm_duty = struct.unpack_from("d", buf, 4*step)[0]

            # # Write to CSV: time, PA angle, DA angle, PWM duty
            # csv_writer.writerow([round(t_log, 4),
            #                     round(PA_angle - PA_ANGLE_OFFSET, 3),
            #                     round((DIS_angle - DIS_ANGLE_OFFSET)*ANGLE2DIS, 3),
            #                     round(pwm_duty, 2)])

            sleep_time = 1/sampling_freq - (time.perf_counter()-start_time)
            if sleep_time < 0:
                print("\n\n Sensor sampling took longer than sampling time!!! \n\n")
            elif sleep_time < 0.3/sampling_freq:
                print("\n\n Sensor sampling took longer than 70% sampling time!!! \n\n")
            time.sleep(max(0,sleep_time*0.7))
            while time.perf_counter() < start_time + 1/sampling_freq:
                pass
    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
        # log_file.close()
    # no special cleanup
