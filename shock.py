#!/usr/bin/env python

from picamera.array import PiRGBArray
from picamera import PiCamera
import asyncio
import json
import logging
import websockets
import math
import multiprocessing
import cv2
import sys
from datetime import date, datetime as dt
import time
import random
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Button
import smbus
import math
from datetime import datetime
import sensors
import os
import csv
from scipy.linalg import norm
from numpy import sum, average
import Adafruit_ADS1x15
import board
import adafruit_dht


PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
INT_ENABLE = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H = 0x43
GYRO_YOUT_H = 0x45
GYRO_ZOUT_H = 0x47
motors_enable = DigitalOutputDevice("BOARD8", active_high=False)
flash_enable = DigitalOutputDevice("BOARD32", active_high=True)


def last_on():
    now_logdate = datetime.now()
    log_date = now_logdate.strftime("%Y-%m-%d")
    log_hour = now_logdate.strftime("%H:%M:%S")
    date_name = now_logdate.strftime("%Y%m%d")
    json_last = {"Fecha": log_date, "Hora": log_hour, "Name": date_name}
    with open('last_on.json', 'w') as outfile:
        json.dump(json_last, outfile)


def logwriter(event, id, t_cpu=0, t_clock=0, t_ambiente=0, humedad=0, amoniaco=0, watch_dog=False, last_date=-1, last_hour=-1, last_name=-1):
    nowlogdate = datetime.now()
    if watch_dog:
        logdate = last_date
        loghour = last_hour
        datename = last_name
    else:
        logdate = nowlogdate.strftime("%Y-%m-%d")
        loghour = nowlogdate.strftime("%H:%M:%S")
        datename = nowlogdate.strftime("%Y%m%d")
    stringdatelog = "log/diario/"+datename+".csv"
    stringdatelogbackup = "log/backup/"+datename+".csv"

    if not os.path.exists(stringdatelog):
        print("No existe el logfile diario")
        header = ["#", "Fecha", "Hora", "Evento", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open(stringdatelog, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelog, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, t_cpu,
                    t_clock, t_ambiente, humedad, amoniaco, id])

    if not os.path.exists(stringdatelogbackup):
        print("No existe el logfile diario de backup")
        header = ["#", "Fecha", "Hora", "Evento", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open(stringdatelogbackup, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelogbackup, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, t_cpu,
                    t_clock, t_ambiente, humedad, amoniaco, id])

    with open('log/log.csv', 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, t_cpu,
                    t_clock, t_ambiente, humedad, amoniaco, id])


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class cmd_vel:
    x = 0
    y = 0


PROCESSES = []
STATE = {"value": 0}

USERS = set()


def command(cam_req, cam_rate, auto_req, auto_rate, imu_req, imu_flag, stuck_flag, flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco):

    motor_1_dir = DigitalOutputDevice("BOARD40")
    motor_1_pwm = DigitalOutputDevice("BOARD38")
    motor_2_dir = DigitalOutputDevice("BOARD37")
    motor_2_pwm = DigitalOutputDevice("BOARD35")
    print("COMMAND INIT")

    def state_event():
        return json.dumps({"type": "state", **STATE})

    def users_event():
        return json.dumps({"type": "users", "count": len(USERS)})

    async def notify_state():
        if USERS:  # asyncio.wait doesn't accept an empty list
            message = state_event()
            await asyncio.wait([user.send(message) for user in USERS])

    async def notify_users():
        if USERS:  # asyncio.wait doesn't accept an empty list
            message = users_event()
            await asyncio.wait([user.send(message) for user in USERS])

    async def register(websocket):
        USERS.add(websocket)
        await notify_users()

    async def unregister(websocket):
        USERS.remove(websocket)
        await notify_users()

    async def counter(websocket, path):
        print("COMMAND SOCKET INIT")

        await register(websocket)
        try:
            await websocket.send(state_event())

            async for message in websocket:
                data = json.loads(message)
                print(data)
                if data["action"] == "move":
                    x = data["x"]
                    z = data["z"]
                    STATE["value"] = "x:{}/z:{}".format(
                        data["x"], data["z"])
                    move(x, z)
                    await notify_state()
                elif data["action"] == "stop":
                    STATE["value"] = "STOP"
                    x = 0
                    z = 0
                    move(x, z)
                    await notify_state()
                elif data["action"] == "config":
                    config = data["req"]
                    if config == True:
                        with open('/var/www/html/state.json') as json_file:
                            state = json.load(json_file)
                        with open('/var/www/html/config.json', 'w') as outfile:
                            json.dump(state, outfile)
                    # auto(auto_req)
                    pass
                elif data["action"] == "auto":
                    auto_req.value = data["req"]
                    print(auto_req.value)
                    # auto(auto_req)
                    pass
                elif data["action"] == "camera":
                    cam_req.value = data["req"]
                    # print(cam_req.value)
                    pass
                elif data["action"] == "auto_rate":
                    auto_rate.value = int(data["req"])
                    print(auto_rate.value)
                    # auto(auto_req)
                    pass
                elif data["action"] == "camera_rate":
                    cam_rate.value = int(data["req"])
                    # print(cam_req.value)
                    pass
                elif data["action"] == "imu_req":
                    imu_req.value = data["req"]
                    # print(cam_req.value)
                    pass
                elif data["action"] == "imu_flag":
                    imu_flag.value = int(data["req"])
                    # print(cam_req.value)
                    pass
                elif data["action"] == "flash":
                    flash_req.value = data["req"]
                    if flash_req.value == True:
                        flash_enable.on()
                        logwriter("Prendi luces", 12)
                    else:
                        flash_enable.off()
                        logwriter("Apague luces", 13)
                    # print(cam_req.value)
                    pass
                elif data["action"] == "temp_req":
                    state_string = ""
                    state_string = str(temp_cpu.value) + "°C/" + str(temp_clock.value) + "°C/" + str(temp_out.value) + "°C/" + str(humedad.value) + "%HR/" + str(amoniaco.value) + "ppm"
                    print(state_string)
 
                    await websocket.send(json.dumps({"type": "temp", "data": state_string}))

                    pass
                elif data["action"] == "save_req":
                    try:
                        if os.path.exists("/dev/sda"):
                            os.system("sudo mount /dev/sda /media/usb")
                            print("lo monte")
                            os.system("mkdir /media/usb/backup ")
                            os.system("cp -r log /media/usb/backup")
                            if not os.path.exists("/media/usb/backup/resources"):
                                os.system("mkdir /media/usb/backup/resources ")
                            os.system(
                                "rsync -aP --ignore-existing resources/ /media/usb/backup/resources")
                            os.system("sudo umount /media/usb")
                            print("Termine backup")
                            await websocket.send(json.dumps({"type": "save_state", "data": True}))
                        else:
                            print("No esta conectado el usb")
                            await websocket.send(json.dumps({"type": "save_state", "data": False}))
                    except:
                        print("No pude montar el usb")
                elif data["action"] == "reboot":
                    logwriter("Recibi pedido de reiniciar", 7)
                    time.sleep(1)
                    os.system("reboot now")
                    # print(cam_req.value)
                    pass
                elif data["action"] == "shutdown":
                    logwriter("Recibi pedido de apagado", 8)
                    time.sleep(1)
                    os.system("shutdown now")

                    # print(cam_req.value)
                    pass

                else:
                    logging.error("unsupported event: %s", data)
                json_state = {"flash": flash_req.value, "auto": auto_req.value, "auto_rate": auto_rate.value,
                              "camera": cam_req.value, "camera_rate": cam_rate.value, "imu_req": imu_req.value, "imu_flag": imu_flag.value}
                with open('/var/www/html/state.json', 'w') as outfile:
                    json.dump(json_state, outfile)
        finally:
            await unregister(websocket)

    def move(x, z):
        if(z == 0):
            pwm1 = x
            pwm2 = pwm1
        elif (x == 0):
            pwm1 = z
            pwm2 = -pwm1
        else:
            if (x > 0):
                if (z > 0):
                    pwm1 = x
                    pwm2 = x - abs(0.5*z)
                else:
                    pwm2 = x
                    pwm1 = x - abs(0.5*z)
            else:
                if (z > 0):
                    pwm1 = x
                    pwm2 = x + abs(0.5*z)
                else:
                    pwm2 = x
                    pwm1 = x + abs(0.5*z)
        # print("PWM1 {}".format(pwm1))
        # print("PWM2 {}".format(pwm2))
        if (pwm1 > 0):
            motor_1_pwm.on()
            motor_1_dir.off()
        elif(pwm1 < 0):
            motor_1_pwm.on()
            motor_1_dir.on()
        else:
            motor_1_pwm.off()
        if (pwm2 < 0):
            motor_2_pwm.on()
            motor_2_dir.off()
        elif(pwm2 > 0):
            motor_2_pwm.on()
            motor_2_dir.on()
        else:
            motor_1_pwm.off()
        motor_1_pwm.value = abs(pwm1)
        motor_2_pwm.value = abs(pwm2)

    start_server2 = websockets.serve(counter, "192.168.4.1", 9001)
    asyncio.get_event_loop().run_until_complete(start_server2)
    asyncio.get_event_loop().run_forever()


def pitch(man, imu_req, imu_flag, stuck_flag, cam_req, cam_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco):
    counter = 0

    GAIN = 1
    adc_ok = False
    dht_ok = False
    imu_ok = False
    dht_init = False
    dht_fail_counter = 0
    try:
        adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=4)
        print("El ADC inicio")
        adc_ok = True
    except: 
        print("Error al inicilizar el ADC")
        pass
    try:
        dhtDevice = adafruit_dht.DHT11(board.D14, use_pulseio=False)
        dht_init = True
    except:
        print("Error al iniciar el DHT")
        pass

    def compare_images(img1, img2):
        # normalize to compensate for exposure difference
        img1 = normalize(img1)
        img2 = normalize(img2)
        # print(img1)
        # calculate the difference and its norms
        diff = img1 - img2  # elementwise for scipy arrays
        m_norm = sum(abs(diff))  # Manhattan norm
        return (m_norm)

    def to_grayscale(arr):
        "If arr is a color image (3D array), convert it to grayscale (2D array)."
        if len(arr.shape) == 3:
            # average over the last axis (color channels)
            return average(arr, -1)
        else:
            return arr

    def normalize(arr):
        rng = arr.max()-arr.min()
        amin = arr.min()
        return (arr-amin)*255/rng

    def MPU_Init():
        # write to sample rate register
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)

        # Write to power management register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)

        # Write to Configuration register
        bus.write_byte_data(Device_Address, CONFIG, 0)

        # Write to Gyro configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)

        # Write to interrupt enable register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)

    def read_raw_data(addr):
        # Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)

        # concatenate higher and lower value
        value = ((high << 8) | low)

        # to get signed value from mpu6050
        if(value > 32768):
            value = value - 65536
        return value
    try:
        bus = smbus.SMBus(5) 	# or bus = smbus.SMBus(0) for older version boards
        Device_Address = 0x68   # MPU6050 device address
        MPU_Init()
        print("IMU INIT")
        imu_ok = True
    except:
        print("El IMU no pudo inicializarse")


    with open('counter.json') as json_file:
        img_index = json.load(json_file)
    img_index_num.value = img_index["num"]
    print("CAMERA INIT")
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    raw_capture = PiRGBArray(camera, size=(640, 480))

    last_pic = time.perf_counter()
    last_imu = time.perf_counter()
    was_taking = False
    first = True
    log_imu_stuck = True
    log_cam_stuck = True
    log_cam = False
    temp_timer = time.perf_counter()
    state_timer = time.perf_counter()
    is_hot.value = False
    cam_count = 0
    imu_count = 0
    while True:
        for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
            f = frame.array

            cv2.waitKey(1)
            # cv2.imshow('frame', f)
            f = cv2.resize(f, (640, 480))
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
            # The function imencode compresses the image and stores it in the memory buffer that is resized to fit

            man[0] = cv2.imencode('.jpg', f, encode_param)[1]
            if first:
                image_to_compare0 = f
                first = False
                compare_timer = time.perf_counter()
            if time.perf_counter() - compare_timer > 2.5:
                img0 = to_grayscale(image_to_compare0.astype(float))
                img1 = to_grayscale(f.astype(float))
                n_m = compare_images(img0, img1)
                # print("Manhattan norm per pixel:", n_m/img0.size)
                if not is_stopped.value:
                    if ((n_m/img0.size) < 10) or math.isnan(n_m):
                        print("Estoy trabado!!! Dos fotos iguales")
                        if log_cam_stuck:
                            # logwriter("Cam Stuck", 14)
                            log_cam_stuck = False
                            cam_count += 1
                            json_stuck_line = {
                                "IMU": imu_count, "Cam": cam_count}
                            with open('stuck_count.json', 'w') as outfile:
                                json.dump(json_stuck_line, outfile)
                        stuck_flag.value = True
                    else:
                        if not log_cam_stuck:
                            print("destuck")
                            # logwriter("Cam Destuck", 15)
                            log_cam_stuck = True
                        stuck_flag.value = False

                image_to_compare0 = f
                compare_timer = time.perf_counter()
            if (cam_req.value == True and was_taking == False):
                img_index_num.value = img_index_num.value + 1
                json_string = {"num": int(img_index_num.value)}
                with open('counter.json', 'w') as outfile:
                    json.dump(json_string, outfile)
                img_counter = 0
                logwriter("Empece a sacar fotos", 3)
                log_cam = True
            if cam_req.value == True:
                was_taking = True
                if time.perf_counter() - last_pic > cam_rate.value:
                    taking_pics.value = True
                    if is_stopped.value == True:
                        now = datetime.now()
                        d1 = now.strftime("%Y%m%d_%H%M%S")
                        img_name = "resources/{}_{}_{}.png".format(d1,
                                                                   img_index_num.value, img_counter)
                        camera.capture(img_name)
                        last_pic = time.perf_counter()
                        img_counter += 1
                        taking_pics.value = False
                        print("Just take a pic")
            else:
                was_taking = False
                if log_cam:
                    logwriter("Termine de sacar fotos", 4)
                    log_cam = False
            raw_capture.truncate(0)
            if imu_req.value == True and imu_ok == True:
                if time.perf_counter()-last_imu > 0.5:
                    try:
                        last_imu = time.perf_counter()
                        acc_x = read_raw_data(ACCEL_XOUT_H)
                        acc_y = read_raw_data(ACCEL_YOUT_H)
                        acc_z = read_raw_data(ACCEL_ZOUT_H)

                        # Full scale range +/- 250 degree/C as per sensitivity scale factor
                        Ax = acc_x/16384.0
                        Ay = acc_y/16384.0
                        Az = acc_z/16384.0
                        

                        pitch = math.atan2(Ay,  Az) * 57.3
                        # print(
                        #     "AX: {0:2.2f} / AY: {1:2.2f} / AZ: {2:2.2f}".format(Ax, Ay, Az))
                        if pitch > imu_flag.value:
                            counter += 1
                        else:
                            counter = 0
                            stuck_flag.value = False
                            if log_imu_stuck == False:
                                # logwriter("IMU Destuck", 17)
                                pass
                            log_imu_stuck = True
                        if counter > 3:
                            print(
                                "Estoy trabado!!! Detecte inclinacion mayor a la safe")
                            if log_imu_stuck:
                                # logwriter("IMU Stuck", 16)
                                log_imu_stuck = False
                                imu_count += 1
                                json_stuck_line = {
                                    "IMU": imu_count, "Cam": cam_count}
                                with open('stuck_count.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)

                            stuck_flag.value = True
                    except:
                        print("Ups! El IMU no pudo tomar lectura")
                        
            if time.perf_counter() - temp_timer > 60:
                temp_timer = time.perf_counter()
                last_on()
                sensors.init()
                try:
                    for chip in sensors.iter_detected_chips():
                        for feature in chip:
                            if feature.label == "temp1":
                                print("el nombre del chip es:")
                                print(chip.adapter_name)
                                if chip.adapter_name == "bcm2835 (i2c@7e804000)":
                                    temp_clock.value = round(feature.get_value(), 1)
                                if chip.adapter_name == "Virtual device":
                                    temp_cpu.value = round(feature.get_value(), 1)
                finally:
                    if (temp_cpu.value > 80 or temp_clock.value > 80) and not is_hot.value:
                        is_hot.value = True
                        logwriter("Alta temperatura", 9, temp_cpu.value, temp_clock.value)
                    sensors.cleanup()
            if time.perf_counter() - state_timer > 60:

                state_timer = time.perf_counter()
                if adc_ok:
                    try:
                        value_adc = adc.read_adc(0, gain=GAIN)
                        volt = (value_adc/32768)*4.096
                        RS = ((3.3/volt)-1)*47
                        ro = 196.086
                        ratio = RS/ro
                        amoniaco.value = pow((math.log(ratio, 10)-0.323)/(-0.243), 10)
                    except:
                        print("Algo salio mal con el sensor de amoniaco")
                        pass
                if dht_init:
                    dht_fail_counter = 0
                    while not dht_ok:
                        dht_ok = True
                        try:
                            temp_out.value = dhtDevice.temperature
                            humedad.value = dhtDevice.humidity
                        except:
                            dht_ok = False
                            dht_fail_counter =+ 1
                        if dht_fail_counter > 20:
                            print("No pude sacar medicion del DHT")
                            break
                logwriter("Estado", 14, temp_cpu.value, temp_clock.value,
                          temp_out.value, humedad.value, amoniaco.value)


def auto(auto_req, auto_rate, taking_pics, is_stopped, stuck_flag, is_hot):
    was_auto = False
    motor_1_dir = DigitalOutputDevice("BOARD40")
    motor_1_pwm = DigitalOutputDevice("BOARD38")
    motor_2_dir = DigitalOutputDevice("BOARD37")
    motor_2_pwm = DigitalOutputDevice("BOARD35")
    button_left = Button("BOARD15")
    button_middle = Button("BOARD13")
    button_right = Button("BOARD11")
    motor_1_pwm.off()
    motor_2_pwm.off()
    print("AUTO INIT")
    first_auto = True
    is_rest = False

    def move(x, z):
        if(z == 0):
            pwm1 = x
            pwm2 = pwm1
        elif (x == 0):
            pwm1 = z
            pwm2 = -pwm1
        else:
            if (x > 0):
                if (z > 0):
                    pwm1 = x
                    pwm2 = x - abs(0.5*z)
                else:
                    pwm2 = x
                    pwm1 = x - abs(0.5*z)
            else:
                if (z > 0):
                    pwm1 = x
                    pwm2 = x + abs(0.5*z)
                else:
                    pwm2 = x
                    pwm1 = x + abs(0.5*z)
        # print("PWM1 {}".format(pwm1))
        # print("PWM2 {}".format(pwm2))
        if (pwm1 > 0):
            motor_1_pwm.on()
            motor_1_dir.off()
        elif(pwm1 < 0):
            motor_1_pwm.on()
            motor_1_dir.on()
        else:
            motor_1_pwm.off()
        if (pwm2 < 0):
            motor_2_pwm.on()
            motor_2_dir.off()
        elif(pwm2 > 0):
            motor_2_pwm.on()
            motor_2_dir.on()
        else:
            motor_1_pwm.off()
        motor_1_pwm.value = abs(pwm1)
        motor_2_pwm.value = abs(pwm2)
        # print("Is active pwm1: {}".format(motor_1_pwm.value))
        # print("Is active pwm2: {}".format(motor_2_pwm.value))
        # print("Is active dir1: {}".format(motor_1_dir.value))
        # print("Is active dir2: {}".format(motor_2_dir.value))

    while True:

        if (auto_req.value == False and was_auto == True):
            move(0, 0)
            was_auto = False
            print("Stop auto")
            is_stopped.value = True
            first_auto = True
            logwriter("Termine de andar autonomo", 4)
        elif(auto_req.value == True):
            if first_auto:
                last_time_rest = time.perf_counter()
                logwriter("Empece a andar autonomo", 2)
                print("first auto")
                first_auto = False
            if is_rest and (time.perf_counter()-last_time_on > 600):
                is_rest = False
                last_time_rest = time.perf_counter()
                is_stopped.value = False
                is_hot.value = False
                print("Vuelvo a andar")
                logwriter("Termine descanso", 11)
            was_auto = True
            timer = time.perf_counter()
            while auto_req.value == True and not is_rest:

                if ((time.perf_counter() - last_time_rest > 1200) or is_hot.value) and not is_rest:
                    is_rest = True
                    move(0, 0)
                    time.sleep(1)
                    is_stopped.value = True
                    last_time_on = time.perf_counter()
                    print("Voy a descansar")
                    logwriter("Empece descanso", 10)
                    break

                if time.perf_counter() - timer < auto_rate.value:
                    backward_counter = 0
                    steer_counter = 0

                    if stuck_flag.value == True:
                        move(-1.0, 0)
                        print("Retrocediendo para desencajarme")
                        while stuck_flag.value == True:
                            time.sleep(1)
                        time.sleep(1)
                        go_right = random.choice([True, False])
                        if go_right == True:
                            move(0, -1.0)
                            print("Going right")
                        else:
                            move(0, 1.0)
                            print("Going left")
                        while (steer_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            steer_counter += 1
                        timer = time.perf_counter()
                    move(1.0, 0)
                    # print("Going forward")
                    while taking_pics.value == True:
                        move(0, 0)
                        time.sleep(1)
                        is_stopped.value = True
                    is_stopped.value = False
                    if button_left.is_pressed and not (button_middle.is_pressed or button_right.is_pressed):
                        print("Me apretaron de izquierda")
                        timer = time.perf_counter()
                        move(-1.0, 0)
                        print("Going backwards")
                        while (backward_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            backward_counter += 1
                        move(0, 1.0)
                        print("Going right")
                        while (steer_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            steer_counter += 1
                    elif button_right.is_pressed and not (button_middle.is_pressed or button_left.is_pressed):
                        timer = time.perf_counter()
                        print("Me apretaron de derecha")
                        move(-1.0, 0)
                        print("Going backwards")
                        while (backward_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            backward_counter += 1
                        move(0, -1.0)
                        print("Going left")
                        while (steer_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            steer_counter += 1
                    elif not (button_middle.is_pressed or button_left.is_pressed or button_right.is_pressed):
                        pass
                    else:
                        print("Choque randomn")
                        timer = time.perf_counter()
                        move(-1.0, 0)
                        print("Going backwards")
                        while (backward_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            backward_counter += 1
                        go_right = random.choice([True, False])
                        if go_right == True:
                            move(0, -1.0)
                            print("Going right")
                        else:
                            move(0, 1.0)
                            print("Going left")
                        while (steer_counter < 1 and auto_req.value == True):
                            time.sleep(1)
                            steer_counter += 1
                else:
                    print('No paso nada')
                    timer = time.perf_counter()
                    move(-1.0, 0)
                    print("Going backwards")
                    while (backward_counter < 1 and auto_req.value == True):
                        time.sleep(1)
                        backward_counter += 1
                    go_right = random.choice([True, False])
                    if go_right == True:
                        move(0, -1.0)
                        print("Going right")
                    else:
                        move(0, 1.0)
                        print("Going left")
                    while (steer_counter < 1 and auto_req.value == True):
                        time.sleep(1)
                        steer_counter += 1
        time.sleep(1)


def camera(man, cam_req, cam_rate, img_index_num, taking_pics, is_stopped, stuck_flag):

    print("CAMERA SENDER INIT")

    async def handler(websocket, path):
        print("CAMERA SOCKET INIT")
        while True:
            try:
                # try:
                await websocket.send(man[0].tobytes())
                await asyncio.sleep(0.1)  # 30 fps
                # except:
                #     print("Ups!")
                #     time.sleep(1)
                #     # start_server = websockets.serve(
                #     #     ws_handler=handler, host='192.168.4.1', port=9000)
                #     # # Start the server, add it to the event loop
                #     # asyncio.get_event_loop().run_until_complete(start_server)
                #     # # Registered our websocket connection handler, thus run event loop forever
                #     # asyncio.get_event_loop().run_forever()
                #         print(sys.exc_info()[0])

            except websockets.exceptions.ConnectionClosed:
                print("Socket closed")
                break
        # vc.release()
        # cv2.destroyAllWindows()
    start_server = websockets.serve(
        ws_handler=handler, host='192.168.4.1', port=9000)
    # Start the server, add it to the event loop
    asyncio.get_event_loop().run_until_complete(start_server)
    # Registered our websocket connection handler, thus run event loop forever
    asyncio.get_event_loop().run_forever()


def main():
    # queue = multiprocessing.Queue()
    img_index_num = multiprocessing.Value('i', 0)
    cam_req = multiprocessing.Value('b', False)
    cam_rate = multiprocessing.Value('i', 0)
    auto_req = multiprocessing.Value('b', False)
    flash_req = multiprocessing.Value('b', False)
    auto_rate = multiprocessing.Value('i', 0)
    imu_req = multiprocessing.Value('b', False)
    imu_flag = multiprocessing.Value('i', 0)
    stuck_flag = multiprocessing.Value('b', False)
    taking_pics = multiprocessing.Value('b', False)
    is_stopped = multiprocessing.Value('b', True)
    is_hot = multiprocessing.Value('b', False)
    temp_cpu = multiprocessing.Value('d', 0)
    temp_clock = multiprocessing.Value('d', 0)
    temp_out = multiprocessing.Value('d', 0)
    humedad = multiprocessing.Value('d', 0)
    amoniaco = multiprocessing.Value('d', 0)
    manager = multiprocessing.Manager()
    lst = manager.list()
    lst.append(None)
    flash_req.value = config["flash"]
    cam_req.value = config["camera"]
    cam_rate.value = config["camera_rate"]
    auto_req.value = config["auto"]
    auto_rate.value = config["auto_rate"]
    imu_req.value = config["imu_req"]
    imu_flag.value = config["imu_flag"]
    json_state = {"flash": flash_req.value, "auto": auto_req.value, "auto_rate": auto_rate.value,
                  "camera": cam_req.value, "camera_rate": cam_rate.value, "imu_req": imu_req.value, "imu_flag": imu_flag.value}
    with open('/var/www/html/state.json', 'w') as outfile:
        json.dump(json_state, outfile)
    # Set up our websocket handler
    command_handler = multiprocessing.Process(
        target=command, args=(cam_req, cam_rate, auto_req, auto_rate, imu_req, imu_flag, stuck_flag, flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco,))
    # Set up our camera
    camera_handler = multiprocessing.Process(
        target=camera, args=(lst, cam_req, cam_rate, img_index_num, taking_pics, is_stopped, stuck_flag,))
    auto_handler = multiprocessing.Process(
        target=auto, args=(auto_req, auto_rate, taking_pics, is_stopped, stuck_flag, is_hot,))
    pitch_handler = multiprocessing.Process(
        target=pitch, args=(lst, imu_req, imu_flag, stuck_flag, cam_req, cam_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco,))
    # Add 'em to our list
    PROCESSES.append(camera_handler)
    PROCESSES.append(command_handler)
    PROCESSES.append(auto_handler)
    PROCESSES.append(pitch_handler)
    for p in PROCESSES:
        p.start()
    while True:
        time.sleep(0.1)


if __name__ == '__main__':
    # time.sleep(20)
    print(bcolors.OKGREEN + "CHICKENBOT 1.0 APPELIE ROBOTICS - 2021" + bcolors.ENDC)
    flash_enable.off()
    if not os.path.exists("log/log.csv"):
        print("No existe el logfile")
        header = ["#", "Fecha", "Hora", "Evento", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open('log/log.csv', 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open('/var/www/html/config.json') as json_file:
        config = json.load(json_file)
    print(config)
    flash_enable.on()
    time.sleep(0.5)
    flash_enable.off()
    time.sleep(0.5)
    flash_enable.on()
    time.sleep(0.5)
    flash_enable.off()
    time.sleep(0.5)
    flash_enable.on()
    time.sleep(0.5)
    flash_enable.off()
    time.sleep(0.5)
    if not os.path.exists("last_on.json"):
        last_on()
    else:
        with open('last_on.json') as json_on:
            last_watch = json.load(json_on)
        with open('stuck_count.json') as json_stuck:
            last_stuck = json.load(json_stuck)
        start_log = "Me apague, atascos detectados por imu: " + \
            str(last_stuck["IMU"]) + \
            ", detectados por camara: " + str(last_stuck["Cam"])
        logwriter(start_log, 15, watch_dog=True,
                  last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    json_stuck_line = {"IMU": 0, "Cam": 0}
    with open('stuck_count.json', 'w') as outfile:
        json.dump(json_stuck_line, outfile)
    logwriter("Me prendi con esta configuracion: " + str(config), 1)
    last_on()
    if config["flash"] == True:
        flash_enable.on()
        logwriter("Prendi luces", 12)
    motors_enable.on()
    try:
        main()
    except KeyboardInterrupt:
        motor_1_pwm = DigitalOutputDevice("BOARD38")
        motor_2_pwm = DigitalOutputDevice("BOARD35")
        motor_1_pwm.off()
        motor_2_pwm.off()
        for p in PROCESSES:
            p.terminate()
