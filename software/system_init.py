import time
import struct
from multiprocessing import shared_memory

from .config import SHM_NAME

def system_init():
    # Move the cart till left LS clicks
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    buf = shm.buf
    duty = -10
    ls_val = 1
    while not ls_val == 0:
        struct.pack_into("d", buf, 4*8, max(-100, min(duty,100)))
        # print(duty)
        ls_val = struct.unpack_from('d', buf, 3*8)[0]
        time.sleep(0.01)
    struct.pack_into("d", buf, 4*8, max(-100, min(0,100)))

    # Set that angle to -1226
    DIS_OFFSET = struct.unpack_from('d', buf, 1*8)[0] + 1226
    # print(DIS_OFFSET)
    struct.pack_into('d', buf, 6*8, DIS_OFFSET) # DA_offset
    time.sleep(0.5)

    # Move cart to new 0 point
    da_val = -1000
    while da_val < 0:
        struct.pack_into("d", buf, 4*8, max(-100, min(-duty,100)))
        # print(-duty)
        da_val = struct.unpack_from('d', buf, 1*8)[0]
        # print(f"da_val = {da_val}")
        time.sleep(0.01)
    struct.pack_into("d", buf, 4*8, max(-100, min(0,100)))


    # wait 15 sec for oscillations to stop
    time.sleep(15)

    # set zero point for pendulum angle by taking average of 20 samples
    PA_OFFSET = 0
    for _ in range(20):
        PA_OFFSET += struct.unpack_from('d', buf, 0*8)[0]
        time.sleep(0.05)
    struct.pack_into('d', buf, 5*8, PA_OFFSET/20) # PA_offset
    
    shm.close()
