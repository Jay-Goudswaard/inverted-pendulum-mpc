import os
import time
import struct
import shutil
from multiprocessing import Process, shared_memory

from .config import SHM_NAME, NUM_SENSORS, PLOTS_DIR
from .plotter import plotter_process
from .pwm_driver import pwm_driver
from .sensor_sampler import sensor_sampling
from .mpc_controller import controller_logic
from .system_init import system_init

# Main spawns and manages processes
if __name__ == "__main__":
    if os.path.isdir("./plots"):
        shutil.rmtree("./plots")
    os.makedirs("./plots", exist_ok=True)
    # Setup shared memory
    size = (NUM_SENSORS+2+1+3+2)*8 + 8
    try:
        shm = shared_memory.SharedMemory(name=SHM_NAME, create=True, size=size)
        struct.pack_into('d', shm.buf, 0*8, 0.0) #encoder 1 pendulum
        struct.pack_into('d', shm.buf, 1*8, 0.0) #encoder 2 distance
        struct.pack_into('d', shm.buf, 2*8, 1.0) #ls 1
        struct.pack_into('d', shm.buf, 3*8, 1.0) #ls 2
        struct.pack_into('d', shm.buf, 4*8, 0.0) # pwm
        struct.pack_into('d', shm.buf, 5*8, 0.0) # PA_offset
        struct.pack_into('d', shm.buf, 6*8, 0.0) # DA_offset
        struct.pack_into('d', shm.buf, 7*8, 0.0) # pendulum_vel
        struct.pack_into('d', shm.buf, 8*8, 0.0) # PA filtered
        struct.pack_into('d', shm.buf, 9*8, 0.0) # DA fitlered
        struct.pack_into('d', shm.buf, 10*8, 0.0) # pendulum_vel filtered
        struct.pack_into('d', shm.buf, 11*8, 0.0) # dis_vel filtered
        struct.pack_into('d', shm.buf, 12*8, 0.0) # dis_vel
        os.chmod(f"/dev/shm/{SHM_NAME}", 0o666)
        shm.close()
    except FileExistsError:
        pass

    # Spawn pwm and sensor process
    p_pwm = Process(target=pwm_driver,     name="PWMDriver")
    p_pwm.start()
    p_sen = Process(target=sensor_sampling,name="SensorSampler")
    p_sen.start()
    p_plot = Process(target=plotter_process, name="Plotter")
    p_plot.start()
    # initialize the system
    system_init()

    # start controller processes
    p_ctl = Process(target=controller_logic,name="Controller")
    p_ctl.start()

    # Wait and handle shutdown
    procs = [p_pwm, p_sen, p_ctl, p_plot]
    try:
        for p in procs:
            p.join()
    except KeyboardInterrupt:
        print("Main: shutting down children...")
        # ask nicely
        for p in procs:
            p.terminate()
        time.sleep(0.1)
        # force kill any still alive
        for p in procs:
            if p.is_alive():
                p.kill()
        for p in procs:
            p.join()
        # cleanup shared memory
        try:
            shared_memory.SharedMemory(name=SHM_NAME).unlink()
        except FileNotFoundError:
            pass
