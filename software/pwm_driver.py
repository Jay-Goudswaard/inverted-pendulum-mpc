import time
import struct
import signal
from multiprocessing import shared_memory

import lgpio

from .config import (CORE_DRIVER, PRIO_DRIVER, SHM_NAME, NUM_SENSORS,
                     PERIOD_NS, PWM_PIN_FORWARD, PWM_PIN_REVERSE)
from .utils import set_realtime, graceful_shutdown, pwm_deadbandcomp

# software timed PWM driver process (hardware PWM not yet readily available on pi 5)
def pwm_driver():
    # catch termination signals
    signal.signal(signal.SIGINT,  _graceful_shutdown)
    signal.signal(signal.SIGTERM, _graceful_shutdown)

    set_realtime(CORE_DRIVER, PRIO_DRIVER)
    h = lgpio.gpiochip_open(0)

    shm = shared_memory.SharedMemory(name=SHM_NAME)
    buf = shm.buf
    offset_duty = NUM_SENSORS * 8

    t_next = time.monotonic_ns()
    try:
        while True:
            duty = struct.unpack_from("d", buf, offset_duty)[0]
            duty = pwm_deadbandcomp(-duty)
            duty = duty * PERIOD_NS / 100
            abs_duty = abs(duty)
            # print(f"{duty}, {abs_duty}")

            if abs_duty > 0:
                if duty < 0:
                    #set pin forward to 0
                    lgpio.gpio_write(h, PWM_PIN_FORWARD, 0)

                    # ------- PWM on reverse -------------------
                    # rising edge
                    lgpio.gpio_write(h, PWM_PIN_REVERSE, 1)
                    # busy-wait high
                    deadline = t_next + abs_duty
                    while time.monotonic_ns() < deadline:
                        pass
                    # falling edge
                    lgpio.gpio_write(h, PWM_PIN_REVERSE, 0)
                    # schedule next
                    t_next += PERIOD_NS
                    while time.monotonic_ns() < t_next:
                        pass
                    # ----- END PWM on forward ----------------
                elif duty > 0:
                    # set pin reverse to 0
                    lgpio.gpio_write(h, PWM_PIN_REVERSE, 0)

                    # ------- PWM on reverse -------------------
                    # rising edge
                    lgpio.gpio_write(h, PWM_PIN_FORWARD, 1)
                    # busy-wait high
                    deadline = t_next + abs_duty
                    while time.monotonic_ns() < deadline:
                        pass
                    # falling edge
                    lgpio.gpio_write(h, PWM_PIN_FORWARD, 0)
                    # schedule next
                    t_next += PERIOD_NS
                    while time.monotonic_ns() < t_next:
                        pass
                    # ----- END PWM on forward ----------------

                else:
                    # set both pins to 0 (for safety)
                    lgpio.gpio_write(h, PWM_PIN_REVERSE, 0)
                    lgpio.gpio_write(h, PWM_PIN_FORWARD, 0)

                    # ----------- WAIT --------
                    # busy-wait
                    deadline = t_next + abs_duty
                    while time.monotonic_ns() < deadline:
                        pass
                    # schedule next
                    t_next += PERIOD_NS
                    while time.monotonic_ns() < t_next:
                        pass
                    # --------- END WAIT -----------
            else:
                # set both pins to 0 (for safety)
                    lgpio.gpio_write(h, PWM_PIN_REVERSE, 0)
                    lgpio.gpio_write(h, PWM_PIN_FORWARD, 0)

                    # ----------- WAIT --------
                    # busy-wait
                    deadline = t_next + abs_duty
                    while time.monotonic_ns() < deadline:
                        pass
                    # schedule next
                    t_next += PERIOD_NS
                    while time.monotonic_ns() < t_next:
                        pass
                    # --------- END WAIT -----------

    except KeyboardInterrupt:
        pass
    finally:
        # always set PWM to 0
        lgpio.gpio_write(h, PWM_PIN_REVERSE, 0)
        lgpio.gpio_write(h, PWM_PIN_FORWARD, 0)
        lgpio.gpiochip_close(h)
        shm.close()
