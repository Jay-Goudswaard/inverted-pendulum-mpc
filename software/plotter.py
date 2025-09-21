import os
import time
import csv
import struct
from collections import deque
from multiprocessing import shared_memory

import matplotlib
matplotlib.use('Agg')  # disables GUI backends
import matplotlib.pyplot as plt  # (kept even if currently unused)

from .config import SHM_NAME, CSV_PATH, PLOTS_DIR, SAVE_INTERVAL, VIEWBACK, SAMPLE_RATE, ANGLE2DIS

# ensure plots directory exists
os.makedirs(PLOTS_DIR, exist_ok=True)


def plotter_process():                 
    shm = shared_memory.SharedMemory(name=SHM_NAME)
    buf = shm.buf
    step = 8

    # slots to plot and their labels
    var_indices = [0, 1, 2, 4, 7, 12, 8, 9, 10, 3, 11]
    var_names   = [
        "PA angle (°)",
        "Distance (m)",
        "LS right",
        "PWM duty (%)",
        "PA ang. vel (°/s)",
        "Distance vel.",
        "PA filtered (°)",
        "Distance filtered (m)",
        "PA ang. vel filtered (°/s)",
        "LS left",
        "Distance vel. filtered"
    ]
    if not os.path.exists(CSV_PATH):
        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_s"] + var_names)

    # rolling buffers for times and data
    times = deque()
    data  = [deque() for _ in var_indices]

    script_start   = time.time()
    next_save_rel  = SAVE_INTERVAL
    plot_count     = 0

        # which subplot (0–5) gets an overlaid filtered trace, and at which data-index
    overlay_map = { 
        0: 6,  # PA angle: overlay data[6]
        1: 7,  # DA angle: overlay data[7]
        2: 9,  # encoders
        4: 8,   # PA vel:   overlay data[8]
        5: 10,   # DA vel:   overlay data[12]
    }

    # fixed y-limits per subplot
    ylims = {
        # 0: (-270,  270),
        # 1: (-0.45, 0.45),
        2: (-0.1,   1.1),
        #3: (-0.1,   1.1),
        3: (-110,  110),
        # 5 (vel) left auto
    }

    try:
        while True:
            t_now_rel = time.time() - script_start
            t_start_plot = time.time()
            # sample
            times.append(t_now_rel)
            for i, shm_idx in enumerate(var_indices):
                if i == 1 or i == 5:
                    data[i].append(struct.unpack_from('d', buf, shm_idx*step)[0]*ANGLE2DIS)
                else:
                    data[i].append(struct.unpack_from('d', buf, shm_idx*step)[0])

            # drop old
            cutoff = t_now_rel - VIEWBACK
            while times and times[0] < cutoff:
                times.popleft()
                for dq in data:
                    dq.popleft()

            
            # time to save?
            if t_now_rel >= next_save_rel:
                with open(CSV_PATH, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["timestamp_s"] + var_names)
                    # write each timestep row
                    for idx, t in enumerate(times):
                        row = [t] + [data[j][idx] for j in range(len(var_indices))]
                        writer.writerow(row)
                next_save_rel += SAVE_INTERVAL
            time_elapsed = time.time() - t_start_plot
            time.sleep(max(0,1.0 / SAMPLE_RATE - time_elapsed))

    except KeyboardInterrupt:
        pass
    finally:
        shm.close()
