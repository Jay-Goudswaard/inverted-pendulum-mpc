import os
import numpy as np

from .config import AS5600_ADDRESS, ANGLE_REG_HIGH, ANGLE_REG_LOW

def set_realtime(core, priority=None):
    os.sched_setaffinity(0, {core})
    if priority is not None:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))

def read_angle(bus):
    retries = 2
    for attempt in range(retries):
        try:
            high = bus.read_byte_data(AS5600_ADDRESS, ANGLE_REG_HIGH)
            low  = bus.read_byte_data(AS5600_ADDRESS, ANGLE_REG_LOW)
            angle_raw = ((high << 8) | low) & 0x0FFF  # 12-bit resolution
            return angle_raw * 360.0 / 4096
        except OSError as e:
            if e.errno == 5 and attempt < retries-1:
                continue
            print(f"Read error: {e}")
            return None

def graceful_shutdown(signum, frame):
    raise KeyboardInterrupt()

def pwm_deadbandcomp(pwm):
    comp_val = 30
    pwm_comp = np.tanh(0.75*pwm)*comp_val + pwm*(100-comp_val)/100
    return min(100, max(-100, pwm_comp))
