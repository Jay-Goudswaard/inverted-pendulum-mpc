# Global configuration and shared constants

import os
from smbus2 import SMBus

# ── CORE / PRIORITY / SHM ─────────────────────────────────────
CORE_DRIVER     = 1              # CPU core for PWM driver
CORE_SENSOR     = 2              # CPU core for sensor sampling
CORE_CONTROLLER = 3              # CPU core for controller logic
PRIO_DRIVER     = 95             # SCHED_FIFO priority for PWM
SHM_NAME        = "multi_shm"    # shared-memory name
NUM_SENSORS     = 4

# ── TIMING / IO ───────────────────────────────────────────────
PERIOD_NS       = 1_000_000      # PWM period (1 kHz)
PWM_PIN_FORWARD = 13
PWM_PIN_REVERSE = 12

SAVE_INTERVAL   = 0.25           # CSV save interval [s]
VIEWBACK        = 7.5            # visible window in plots [s]
SAMPLE_RATE     = 150            # plotting loop sample rate [Hz]
ANGLE2DIS       = 0.000343       # angle→distance conversion

# ── PATHS ─────────────────────────────────────────────────────
PLOTS_DIR = "./plots"
CSV_PATH  = os.path.join(PLOTS_DIR, "log.csv")

# ── I2C / SENSORS ─────────────────────────────────────────────
AS5600_ADDRESS = 0x36
ANGLE_REG_HIGH = 0x0E
ANGLE_REG_LOW  = 0x0F

# I2C buses (same names as before)
bus1  = SMBus(1)    # hardware I²C (GPIO 2/3)
bus15 = SMBus(15)   # software I²C (GPIO 10/11)
