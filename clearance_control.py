#region SAVIOR 
    # printe("SAVIOR INIT")

    # tof_ok = False
    # tof_offset = 78
    # min_tof_clearance = 20
    # sink_tof_clearance = 16
    # watch_tof_clearance = 30
    # max_tof_clearance = 25
    # clearance_array = []
    # time_clearance_array = []
    # clearance_window = 2
    # clearance_timer = time.perf_counter()
    # time_to_sink = 3
    # sink_slope = (sink_tof_clearance-max_tof_clearance)/time_to_sink
    # try:
    #     tof = VL53L0X.VL53L0X(i2c_bus=4,i2c_address=0x29)
    #     tof.open()
    #     tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
    #     tof_ok = True
    #     printe("Tof inicio bien")
    # except Exception as ex:
    #     errorwriter(ex, "Error al iniciar medidor tof")
    #     printe("Error al inciar el medidor tof")
    #     tof_ok = False

    # if tof_ok:
    #     while True:
    #         clearance.value = tof.get_distance() - tof_offset
    #         if not clearance_stuck_flag.value:
    #             if clearance.value < min_tof_clearance:
    #                 clearance_stuck_flag.value = True
    #                 time_clearance_array = []
    #                 clearance_array = []
    #             elif clearance.value < watch_tof_clearance:
    #                 time_clearance_array.append(time.perf_counter())
    #                 clearance_array.append(clearance.value)
    #                 if (time_clearance_array[-1] - time_clearance_array[0]) > clearance_window:
    #                     time_clearance_array.pop(0)
    #                     clearance_array.pop(0)
    #                 if len(clearance_array) > 5:
    #                     y = np.array(clearance_array)
    #                     x = np.array([count for count, i in enumerate(clearance_array)])
    #                     A = np.stack([x, np.ones(len(x))]).T
    #                     slope, point_zero = np.linalg.lstsq(A, y, rcond=None)[0]
    #                     if slope < sink_slope:
    #                         clearance_stuck_flag.value = True
    #                         time_clearance_array = []
    #                         clearance_array = []
    #         else:
    #             if clearance.value > max_tof_clearance:
    #                 clearance_stuck_flag.value = False
            
    #             # t = [count for count, i in enumerate(clearance_array)]
    #             # ty = [count * i for count, i in enumerate(clearance_array)]
    #             # t_sqrt = [i**2 in t]
    #             # n = len(clearance_array)
    #             # slope = (n*sum(ty)-sum(t)*sum(clearance_array)/(n*sum(t_sqrt)-sum(t)**2))

    #             y = np.array(clearance_array)
    #             x = np.array([count for count, i in enumerate(clearance_array)])
    #             A = np.stack([x, np.ones(len(x))]).T
    #             slope, point_zero = np.linalg.lstsq(A, y, rcond=None)[0]
                
    #             pass
    #         if clearance.value < min_tof_clearance:
    #             clearance_stuck_flag.value = True
    #         elif clearance.value > max_tof_clearance:
    #             clearance_stuck_flag.value = False
    #endregion 