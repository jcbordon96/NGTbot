import time
import SDL_DS3231

ds3231 = SDL_DS3231.SDL_DS3231(1, 0x68)
# ds3231.write_now()

while True:

    print(ds3231.read_datetime())
    print(ds3231.getTemp())
    time.sleep(1.0)
# from datetime import datetime

# today = datetime.now()

# # dd/mm/YY
# d1 = today.strftime("%d%m%Y_%H%M%S")
# print("d1 =", d1)

# # Textual month, day and year
# d2 = today.strftime("%H%M%S")
# print("d2 =", d2)

# print(d1+"_"+d2)

# # Month abbreviation, day and year
# d4 = today.strftime("%b-%d-%Y")
# print("d4 =", d4)
# import sensors

# sensors.init()
# try:
#     for chip in sensors.iter_detected_chips():

#         for feature in chip:
#             if feature.label == "temp1":
#                 print('%s at %s' % (chip, chip.adapter_name))
#                 print('  %s: %.2f' % (feature.label, feature.get_value()))
# finally:
#     sensors.cleanup()
