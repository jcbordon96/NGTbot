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
# motors_enable = DigitalOutputDevice("BOARD8", active_high=False)
flash_enable = DigitalOutputDevice("BOARD32", active_high=True)
led_ground = DigitalOutputDevice("BOARD12", active_high=True)
led_enable = DigitalOutputDevice("BOARD10", active_high=True)


def last_on():
    now_logdate = datetime.now()
    log_date = now_logdate.strftime("%Y-%m-%d")
    log_hour = now_logdate.strftime("%H:%M:%S")
    date_name = now_logdate.strftime("%Y%m%d")
    json_last = {"Fecha": log_date, "Hora": log_hour, "Name": date_name}
    with open('last_on.json', 'w') as outfile:
        json.dump(json_last, outfile)
    with open('last_on_backup.json', 'w') as outfile:
        json.dump(json_last, outfile)

def errorwriter(error, comentario = ""):
    error_date = str(datetime.now())
    err = str(error)
    errlog = error_date + " Error: "+ err + " Comentario: "+ comentario
    with open("log/error.log",'a', newline='') as logerror:
        logerror.write(errlog)

def logwriter(event, id, t_cpu=0, minutos =0, t_clock=0, t_ambiente=0, humedad=0, amoniaco=0, watch_dog=False, last_date=-1, last_hour=-1, last_name=-1):
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
        header = ["#", "Fecha", "Hora", "Evento", "Minutos", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open(stringdatelog, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelog, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos, t_cpu,
                    t_clock, t_ambiente, humedad, amoniaco, id])

    if not os.path.exists(stringdatelogbackup):
        print("No existe el logfile diario de backup")
        header = ["#", "Fecha", "Hora", "Evento","Minutos", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open(stringdatelogbackup, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelogbackup, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos, t_cpu,
                    t_clock, t_ambiente, humedad, amoniaco, id])

    with open('log/log.csv', 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos, t_cpu,
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


def command(cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag,  flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, timer_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, timer_rest, timer_wake, steer_counter, backwards_counter, timer_boring, crash_timeout):

    # motor_1_dir = DigitalOutputDevice("BOARD31")
    # motor_1_pwm = DigitalOutputDevice("BOARD38")
    # motor_2_dir = DigitalOutputDevice("BOARD35")
    # motor_2_pwm = DigitalOutputDevice("BOARD35")
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
                # print(data)
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
                        with open('/var/www/html/backup_config.json', 'w') as outfile:
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
                    timer_boring.value = int(data["req"])
                    print(timer_boring.value)
                    # auto(auto_req)
                    pass
                elif data["action"] == "camera_rate":
                    camera_rate.value = int(data["req"])
                    # print(cam_req.value)
                    pass
                elif data["action"] == "imu_req":
                    imu_req.value = data["req"]
                    # print(cam_req.value)
                    pass
                elif data["action"] == "imu_flag":
                    pitch_flag.value = int(data["req"])
                    # print(cam_req.value)
                    pass
                elif data["action"] == "flash":
                    flash_req.value = data["req"]
                    if flash_req.value == True:
                        flash_enable.on()
                        logwriter("Prendi luces", id=12)
                    else:
                        flash_enable.off()
                        logwriter("Apague luces", id=13)
                    # print(cam_req.value)
                    pass
                elif data["action"] == "temp_req":
                    state_string = ""
                    state_string = str(temp_cpu.value) + "°C/" + str(temp_clock.value) + "°C/" + str(temp_out.value) + "°C/" + str(humedad.value) + "%HR/" + str(amoniaco.value) + "ppm"
                    # print(state_string)
 
                    await websocket.send(json.dumps({"type": "temp", "data": state_string}))

                    pass
                elif data["action"] == "save_req":
                    try:
                        if os.path.exists("/dev/sda"):
                            os.system("sudo mount /dev/sda1 /media/usb")
                            print("lo monte")
                            if not os.path.exists("/media/usb/backup"):
                                os.system("mkdir /media/usb/backup ")
                            os.system("sudo cp -r log /media/usb/backup")
                            if not os.path.exists("/media/usb/backup/resources"):
                                os.system("mkdir /media/usb/backup/resources")
                            os.system(
                                "sudo rsync -aP --ignore-existing resources/ /media/usb/backup/resources")
                            os.system("sudo umount /media/usb")
                            print("Termine backup")
                            await websocket.send(json.dumps({"type": "save_state", "data": True}))
                        else:
                            print("No esta conectado el usb")
                            await websocket.send(json.dumps({"type": "save_state", "data": False}))
                    except Exception as ex:
                        errorwriter(ex, "No se monto el USB")
                        print("No pude montar el usb")
                elif data["action"] == "reboot":
                    logwriter("Recibi pedido de reiniciar", id=7)
                    time.sleep(1)
                    os.system("sudo reboot now")
                    # print(cam_req.value)
                    pass
                elif data["action"] == "shutdown":
                    logwriter("Recibi pedido de apagado", id=8)
                    time.sleep(1)
                    os.system("sudo shutdown now")

                    # print(cam_req.value)
                    pass

                else:
                    logging.error("unsupported event: %s", data)
                json_state = {"flash": flash_req.value, "auto": auto_req.value, "auto_rate": timer_boring.value,
                              "camera": cam_req.value, "camera_rate": camera_rate.value, "imu_req": imu_req.value, "imu_flag": pitch_flag.value}
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


def pitch(man, imu_req, pitch_flag, cam_stuck_flag, imu_stuck_flag, cam_req, camera_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, timer_stuck_pic, pitch_counter, timer_temp, timer_log, pic_sensibility):
    
    counter = 0
    GAIN = 1
    adc_ok = False
    dht_ok = False
    imu_ok = False
    dht_init = False
    dht_fail_counter = 0
    try:
        adc = Adafruit_ADS1x15.ADS1015(address=0x48, busnum=4)
        print("El ADC inicio")
        adc_ok = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el ADC") 
        print("Error al iniciar el ADC")
        pass
    try:
        dhtDevice = adafruit_dht.DHT11(board.D14, use_pulseio=False)
        dht_init = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el DHT")
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
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el IMU")
        print("El IMU no pudo iniciar")

    try:
        with open('counter.json') as json_file:
            img_index = json.load(json_file)
    except:
        json_string = {"num": 0}
        with open('counter.json', 'w') as outfile:
            json.dump(json_string, outfile)
        with open('counter.json') as json_file:
            img_index = json.load(json_file)

    img_index_num.value = img_index["num"]
    print("CAMERA INIT")
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    raw_capture = PiRGBArray(camera, size=(640, 480))

    last_pic = 0
    last_imu = 0
    was_taking = False
    first = True
    log_imu_stuck = True
    log_cam_stuck = True
    log_cam = False
    temp_timer = 0
    state_timer = 0
    is_hot.value = False
    start_cam_stuck = 0
    start_imu_stuck = 0
    elapsed_cam_stuck = 0
    elapsed_imu_stuck = 0
    total_elapsed_cam_stuck = 0
    total_elapsed_imu_stuck = 0
    last_total_elapsed_cam_stuck = 0
    last_total_elapsed_imu_stuck = 0
    is_tails = False
    while True:
        try:
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
                if time.perf_counter() - compare_timer > timer_stuck_pic.value:
                    img0 = to_grayscale(image_to_compare0.astype(float))
                    img1 = to_grayscale(f.astype(float))
                    n_m = compare_images(img0, img1)
                    # print("Manhattan norm per pixel:", n_m/img0.size)
                    if not is_stopped.value:
                        if not math.isnan(n_m):
                            # print((n_m/img0.size))
                            if ((n_m/img0.size) < pic_sensibility.value):
                                if log_cam_stuck:
                                    print("Estoy trabado!!! Dos fotos iguales")
                                    logwriter("Me trabe, camara", id=16)
                                    log_cam_stuck = False
                                    start_cam_stuck = time.perf_counter()
                                    last_total_elapsed_cam_stuck = total_elapsed_cam_stuck
                                cam_stuck_flag.value = True
                                elapsed_cam_stuck = round((time.perf_counter() - start_cam_stuck)/60.0, 2)
                                total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck}
                                with open('stuck_count.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                with open('stuck_count_backup.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                
                            else:
                                if not log_cam_stuck:
                                    elapsed_cam_stuck = round((time.perf_counter() - start_cam_stuck)/60.0, 2)
                                    total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                    json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck}
                                    with open('stuck_count.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    with open('stuck_count_backup.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    print(" Cam destuck")
                                    logwriter("Me destrabe, camara, minutos:", minutos=elapsed_cam_stuck, id=17)
                                    log_cam_stuck = True
                                cam_stuck_flag.value = False
                    else:
                        cam_stuck_flag.value = False

                    image_to_compare0 = f
                    compare_timer = time.perf_counter()
                if (cam_req.value == True and was_taking == False):
                    img_index_num.value = img_index_num.value + 1
                    json_string = {"num": int(img_index_num.value)}
                    with open('counter.json', 'w') as outfile:
                        json.dump(json_string, outfile)
                    img_counter = 0
                    logwriter("Empece a sacar fotos", id=3)
                    log_cam = True
                if cam_req.value == True:
                    was_taking = True
                    if time.perf_counter() - last_pic > camera_rate.value:
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
                        logwriter("Termine de sacar fotos", id=4)
                        log_cam = False
                raw_capture.truncate(0)
                if imu_req.value == True and imu_ok == True:
                    if time.perf_counter()-last_imu > 0.5:
                        try:
                            last_imu = time.perf_counter()
                            # acc_x = read_raw_data(ACCEL_XOUT_H)
                            acc_y = read_raw_data(ACCEL_YOUT_H)
                            acc_z = read_raw_data(ACCEL_ZOUT_H)

                            # Full scale range +/- 250 degree/C as per sensitivity scale factor
                            # Ax = acc_x/16384.0
                            Ay = acc_y/16384.0
                            Az = acc_z/16384.0
                            

                            pitch = math.atan2(Ay,  Az) * 57.3
                            # print(
                            #     "AX: {0:2.2f} / AY: {1:2.2f} / AZ: {2:2.2f}".format(Ax, Ay, Az))
                            if pitch > pitch_flag.value:
                                counter += 1
                            else:
                                counter = 0
                                imu_stuck_flag.value = False
                                if log_imu_stuck == False:
                                    elapsed_imu_stuck = round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                                    total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                                    json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck}
                                    with open('stuck_count.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    with open('stuck_count_backup.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    print("IMU destuck")
                                    logwriter("Me destrabe, IMU, minutos", minutos=elapsed_imu_stuck, id=19)
                                    log_imu_stuck = True
                            if counter > pitch_counter.value:
                                if log_imu_stuck:
                                    print("Estoy trabado!!! Detecte inclinacion mayor a la safe")
                                    logwriter("Me trabe, IMU", id=18)
                                    log_imu_stuck = False
                                    start_imu_stuck = time.perf_counter()
                                    last_total_elapsed_imu_stuck = total_elapsed_imu_stuck
                                elapsed_imu_stuck =  round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                                total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                                json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck}
                                with open('stuck_count.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                with open('stuck_count_backup.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                imu_stuck_flag.value = True
                        except Exception as ex:
                            errorwriter(ex, "El IMU no pudo tomar lectura")
                            print(ex, "Ups! El IMU no pudo tomar lectura")
                            
                if time.perf_counter() - temp_timer > timer_temp.value:
                    temp_timer = time.perf_counter()
                    last_on()
                    sensors.init()
                    try:
                        for chip in sensors.iter_detected_chips():
                            for feature in chip:
                                if feature.label == "temp1":
                                    # print("el nombre del chip es:")
                                    # print(chip.adapter_name)
                                    if chip.adapter_name == "bcm2835 (i2c@7e804000)" or chip.adapter_name == "i2c-gpio-rtc@0":
                                        temp_clock.value = round(feature.get_value(), 1)
                                        if chip.adapter_name == "i2c-gpio-rtc@0":
                                            is_tails = True
                                    if chip.adapter_name == "Virtual device":
                                        temp_cpu.value = round(feature.get_value(), 1)
                    finally:
                        if (temp_cpu.value > 80 or temp_clock.value > 80) and not is_hot.value:
                            is_hot.value = True
                            logwriter("Alta temperatura", id=9, t_cpu=temp_cpu.value, t_clock=temp_clock.value)
                        sensors.cleanup()
                if time.perf_counter() - state_timer > timer_log.value:

                    state_timer = time.perf_counter()
                    if adc_ok:
                        try:
                            value_adc = adc.read_adc(0, gain=GAIN)
                            volt = (value_adc/32768)*4.096
                            RS = ((3.3/volt)-1)*47
                            if is_tails:
                                ro = 326.52
                            else:
                                ro = 260.5
                            ratio = RS/ro
                            amoniaco.value = round(pow((math.log(ratio, 10)-0.323)/(-0.243), 10),2)
                        except Exception as ex:
                            print(ex)
                            errorwriter(ex, "No se pudo tomar medicion de amoniaco")
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
                                dht_fail_counter += 1
                            if dht_fail_counter > 50:
                                print("No pude sacar medicion del DHT")
                                errorwriter("DHT", "No se pudo tomar medicion de Humedad y Temperatura")
                                break
                    logwriter("Estado", id=14, t_cpu=temp_cpu.value, t_clock=temp_clock.value,
                            t_ambiente=temp_out.value, humedad=humedad.value, amoniaco=amoniaco.value)
        except Exception as ex:
            # errorwriter("Camara", "Fallo timeout")
            # print(ex)
            pass



def auto(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, is_hot, timer_rest, timer_wake, steer_counter, backwards_counter, crash_timeout, last_touch_timeout, last_touch_counter, last_touch_osc_counter, flash_req ):
    was_auto = False
    # motor_1_dir = DigitalOutputDevice("BOARD40")
    # motor_1_pwm = DigitalOutputDevice("BOARD38")
    # motor_2_dir = DigitalOutputDevice("BOARD37")
    # motor_2_pwm = DigitalOutputDevice("BOARD35")
    button_left = Button("BOARD15")
    button_middle = Button("BOARD13")
    button_right = Button("BOARD11")
    motor_1_pwm.off()
    motor_2_pwm.off()
    print("AUTO INIT")
    first_auto = True
    is_rest = False
    crash_confirmed = False
    last_touch = ""
    last_touch_timer = time.perf_counter()
    last_touch_count = 0
    last_touch_osc_count = 0
    last_touch_osc_timer = time.perf_counter()
    led_on = True
    second_back = False
    back_change = 0
    

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
    def antiloop(mode):
        backward_count = 0
        steer_count = 0
        print("Antiloop")
        print(mode)
        if mode == "IZQ":
            move(-1.0, 0)
            print("Going backwards")
            while (backward_count < backwards_counter.value and auto_req.value == True):
                time.sleep(1)
                backward_count += 1
            move(0, -1.0)
            print("Going left")
            while (steer_count < steer_counter.value and auto_req.value == True):
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                move(0, -1.0)
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                move(0, -1.0)
                steer_count += 1

        elif mode == "OSC":
            move(-1.0, 0)
            print("Going backwards")
            while (backward_count < backwards_counter.value and auto_req.value == True):
                time.sleep(1)
                backward_count += 1
            if last_touch == "IZQ":
                go_right = False
            elif last_touch == "DER":
                go_right = True
            else:
                go_right = random.choice([True, False])
            if go_right == True:
                move(0, 1.0)
                print("Going right")
            else:
                move(0, -1.0)
                print("Going left")
            while (steer_count < steer_counter.value and auto_req.value == True):
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                if go_right == True:
                    move(0, 1.0)
                    print("Going right")
                else:
                    move(0, -1.0)
                    print("Going left")
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                if go_right == True:
                    move(0, 1.0)
                    print("Going right")
                else:
                    move(0, -1.0)
                    print("Going left")
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                if go_right == True:
                    move(0, 1.0)
                    print("Going right")
                else:
                    move(0, -1.0)
                    print("Going left")
                steer_count += 1
        elif mode == "DER":
            move(-1.0, 0)
            print("Going backwards")
            while (backward_count < backwards_counter.value and auto_req.value == True):
                time.sleep(1)
                backward_count += 1
            move(0, 1.0)
            print("Going right")
            while (steer_count < steer_counter.value and auto_req.value == True):
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                move(0, 1.0)
                time.sleep(0.25)
                move(1.0, 0)
                time.sleep(0.45)
                move(0, 1.0)
                steer_count += 1 
        last_touch_count = 0
        last_touch_osc_count = 0   
    while True:
        if is_rest:
            if led_on:
                led_enable.off()
                led_on = False
            else:
                led_enable.on()
                led_on = True

        if (auto_req.value == False and was_auto == True):
            move(0, 0)
            was_auto = False
            print("Stop auto")
            is_stopped.value = True
            first_auto = True
            logwriter("Termine de andar autonomo", id=4)
        elif(auto_req.value == True):
            if first_auto:
                last_time_rest = time.perf_counter()
                logwriter("Empece a andar autonomo", id=2)
                print("first auto")
                first_auto = False
                is_stopped.value = False
            if is_rest and (time.perf_counter()-last_time_on > timer_rest.value):
                is_rest = False
                last_time_rest = time.perf_counter()
                is_stopped.value = False
                is_hot.value = False
                print("Vuelvo a andar")
                logwriter("Termine descanso", id=11)
                led_enable.on()
                if flash_req.value == True:
                    flash_enable.on()
            was_auto = True
            timer = time.perf_counter()
            while auto_req.value == True and not is_rest:

                if ((time.perf_counter() - last_time_rest > timer_wake.value) or is_hot.value) and not is_rest:
                    is_rest = True
                    move(0, 0)
                    time.sleep(1)
                    is_stopped.value = True
                    last_time_on = time.perf_counter()
                    print("Voy a descansar")
                    flash_enable.off()
                    logwriter("Empece descanso", id=10)
                    break
                if time.perf_counter() - last_touch_timer > last_touch_timeout.value:
                    last_touch_count = 0
                if time.perf_counter() - last_touch_osc_timer > last_touch_timeout.value:
                    last_touch_osc_count = 0
                if time.perf_counter() - timer < timer_boring.value:
                    backward_count = 0
                    steer_count = 0
                    stuck_seq = 0

                    if cam_stuck_flag.value == True or imu_stuck_flag.value == True:
                        move(-1.0, 0)
                        print("Retrocediendo para desencajarme")
                        while ((cam_stuck_flag.value == True or imu_stuck_flag == True) and auto_req.value == True):
                            if stuck_seq == 0:
                                move(-1.0, 0)
                                # print("Going backwards")
                                while (backward_count < backwards_counter.value and auto_req.value == True):
                                    time.sleep(1)
                                    backward_count += 1
                                stuck_seq = 1
                                backward_count = 0
                                
                            elif stuck_seq == 1:
                                move(1.0, 0)
                                # print("Going forward")
                                while (backward_count < backwards_counter.value and auto_req.value == True):
                                    time.sleep(1)
                                    backward_count += 1
                                stuck_seq = 2
                                backward_count = 0
                            elif stuck_seq == 2:
                                move(0, 1.0)
                                # print("Going left idk")
                                while (backward_count < backwards_counter.value and auto_req.value == True):
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, 1.0)
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, 1.0)
                                    backward_count += 1
                                backward_count = 0
                                break
                        if stuck_seq != 1:
                            move(-1.0, 0)
                            # print("Going backwards")
                            while (backward_count < backwards_counter.value and auto_req.value == True):
                                time.sleep(1)
                                backward_count += 1
                        stuck_seq = 0
                        
                        #while (backward_count < 2 and auto_req.value == True):
                            #   time.sleep(3)
                            #  backward_count += 1
                            # move(0,1)
                            # time.sleep(0.5)
                            #move(1,0)
                            #time.sleep(1)
                            #move(0,-1.0)
                            #time.sleep(1)
                            #move(-1.0, 0)
                            #time.sleep(1)
                            
                        backward_count = 0
                            

                        go_right =  True #random.choice([True, False])
                        if go_right == True:
                            move(0, 1.0)
                            # print("Going right")
                        else:
                            move(0, -1.0)
                            # print("Going left")
                        while (steer_count < steer_counter.value and auto_req.value == True):
                            time.sleep(0.25)
                            move(1.0, 0)
                            time.sleep(0.45)
                            if go_right == True:
                                move(0, 1.0)
                                # print("Going right")
                            else:
                                move(0, -1.0)
                                # print("Going left")
                            time.sleep(0.25)
                            move(1.0, 0)
                            time.sleep(0.45)
                            if go_right == True:
                                move(0, 1.0)
                                # print("Going right")
                            else:
                                move(0, -1.0)
                                # print("Going left")
                            time.sleep(0.25)
                            move(1.0, 0)
                            time.sleep(0.45)
                            if go_right == True:
                                move(0, 1.0)
                                # print("Going right")
                            else:
                                move(0, -1.0)
                                # print("Going left")
                            steer_count += 1
                        timer = time.perf_counter()
                    move(1.0, 0)
                    # print("Going forward")
                    while taking_pics.value == True:
                        move(0, 0)
                        time.sleep(1)
                        is_stopped.value = True
                    is_stopped.value = False
                    if button_left.is_pressed and not button_right.is_pressed:
                        crash_confirmed = False
                        crash_timer = time.perf_counter()
                        # print("Me apretaron de izquierda")
                        if crash_timeout.value > 0:
                            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                time.sleep(0.25)
                                if button_left.is_pressed and not button_right.is_pressed:
                                    crash_confirmed = True
                                else:
                                    crash_confirmed = False
                                    break
                        else:
                            crash_confirmed = True
                        if crash_confirmed:
                            # print("Choque confirmado de izquierda")
                            timer = time.perf_counter()
                            if last_touch == "IZQ":
                                last_touch_count +=1
                                last_touch_osc_count = 0
                                last_touch_timer = time.perf_counter()
                                # print("Izquierda count: {}".format(last_touch_count))
                            elif last_touch == "DER":
                                last_touch_osc_count += 1
                                last_touch_count = 0
                                last_touch_osc_timer = time.perf_counter()
                                # print("Osci count: {}".format(last_touch_osc_count))
                            last_touch = "IZQ"
                            if last_touch_count >= last_touch_counter.value:
                                antiloop("IZQ")
                                last_touch = "DER"
                            elif last_touch_osc_count >= last_touch_osc_counter.value:
                                antiloop("OSC")
                                last_touch_osc_count = 0
                            else:
                                move(-1.0, 0)
                                # print("Going backwards")
                                if second_back == False:
                                    back_change = backwards_counter.value
                                    second_back = True
                                else:
                                    back_change = int(backwards_counter.value * 1.5)
                                    second_back = False
                                while (backward_count < back_change and auto_req.value == True):
                                    time.sleep(1)
                                    backward_count += 1
                                move(0, 1.0)
                                # print("Going right")
                                while (steer_count < steer_counter.value and auto_req.value == True):
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, 1.0)
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, 1.0)
                                    steer_count += 1

                    elif button_right.is_pressed and not button_left.is_pressed:
                        crash_confirmed = False
                        crash_timer = time.perf_counter()
                        # print("Me apretaron de derecha")
                        if crash_timeout.value > 0:
                            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                time.sleep(0.25)
                                if button_right.is_pressed and not button_left.is_pressed:
                                    crash_confirmed = True
                                else:
                                    crash_confirmed = False
                                    break
                        else:
                            crash_confirmed = True
                        if crash_confirmed:
                            timer = time.perf_counter()
                            # print("Choque confirmado de derecha")
                            if last_touch == "DER":
                                last_touch_count +=1
                                last_touch_osc_count = 0
                                last_touch_timer = time.perf_counter()
                                # print("Derecha count: {}".format(last_touch_count))
                            elif last_touch == "IZQ":
                                last_touch_osc_count += 1
                                last_touch_count = 0
                                last_touch_osc_timer = time.perf_counter()
                                # print("Oscilation count: {}".format(last_touch_osc_count))
                            last_touch = "DER"
                            if last_touch_count >= last_touch_counter.value:
                                antiloop("DER")
                                last_touch = "IZQ"
                            elif last_touch_osc_count >= last_touch_osc_counter.value:
                                antiloop("OSC")
                                last_touch_osc_count = 0
                            else:
                                if second_back == False:
                                    back_change = backwards_counter.value
                                    second_back = True
                                else:
                                    back_change = int(backwards_counter.value * 1.5)
                                    second_back = False
                                move(-1.0, 0)
                                # print("Going backwards")
                                while (backward_count < back_change and auto_req.value == True):
                                    time.sleep(1)
                                    backward_count += 1
                                move(0, -1.0)
                                # print("Going left")
                                while (steer_count < steer_counter.value and auto_req.value == True):
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, -1.0)
                                    time.sleep(0.25)
                                    move(1.0, 0)
                                    time.sleep(0.45)
                                    move(0, -1.0)
                                    steer_count += 1
                    elif (button_middle.is_pressed and not (button_left.is_pressed or button_right.is_pressed)):
                        crash_confirmed = False
                        crash_timer = time.perf_counter()

                        
                        # print("Me apretaron de frente")
                        if crash_timeout.value > 0:
                            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                time.sleep(0.25)
                                if (button_middle.is_pressed and not (button_left.is_pressed or button_right.is_pressed)):
                                    crash_confirmed = True
                                else:
                                    crash_confirmed = False
                                    break
                        else:
                            crash_confirmed = True
                            time.sleep(0.2)
                            if button_left.is_pressed and not button_right.is_pressed:
                                crash_confirmed = False
                                crash_timer = time.perf_counter()
                                # print("Me apretaron de izquierda")
                                if crash_timeout.value > 0:
                                    while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                        time.sleep(0.25)
                                        if button_left.is_pressed and not button_right.is_pressed:
                                            crash_confirmed = True
                                        else:
                                            crash_confirmed = False
                                            break
                                else:
                                    crash_confirmed = True
                                if crash_confirmed:
                                    # print("Choque confirmado de izquierda")
                                    timer = time.perf_counter()
                                    if last_touch == "IZQ":
                                        last_touch_count +=1
                                        last_touch_osc_count = 0
                                        last_touch_timer = time.perf_counter()
                                        # print("Izquierda count: {}".format(last_touch_count))
                                    elif last_touch == "DER":
                                        last_touch_osc_count += 1
                                        last_touch_count = 0
                                        last_touch_osc_timer = time.perf_counter()
                                        # print("Osci count: {}".format(last_touch_osc_count))
                                    last_touch = "IZQ"
                                    if last_touch_count >= last_touch_counter.value:
                                        antiloop("IZQ")
                                        last_touch = "DER"
                                    elif last_touch_osc_count >= last_touch_osc_counter.value:
                                        antiloop("OSC")
                                        last_touch_osc_count = 0
                                    else:
                                        move(-1.0, 0)
                                        # print("Going backwards")
                                        if second_back == False:
                                            back_change = backwards_counter.value
                                            second_back = True
                                        else:
                                            back_change = int(backwards_counter.value * 1.5)
                                            second_back = False
                                        while (backward_count < back_change and auto_req.value == True):
                                            time.sleep(1)
                                            backward_count += 1
                                        move(0, 1.0)
                                        # print("Going right")
                                        while (steer_count < steer_counter.value and auto_req.value == True):
                                            time.sleep(0.25)
                                            move(1.0, 0)
                                            time.sleep(0.45)
                                            move(0, 1.0)
                                            time.sleep(0.25)
                                            move(1.0, 0)
                                            time.sleep(0.45)
                                            move(0, 1.0)
                                            steer_count += 1
                                crash_confirmed = False

                            elif button_right.is_pressed and not button_left.is_pressed:
                                crash_confirmed = False
                                crash_timer = time.perf_counter()
                                # print("Me apretaron de derecha")
                                if crash_timeout.value > 0:
                                    while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                        time.sleep(0.25)
                                        if button_right.is_pressed and not button_left.is_pressed:
                                            crash_confirmed = True
                                        else:
                                            crash_confirmed = False
                                            break
                                else:
                                    crash_confirmed = True
                                if crash_confirmed:
                                    timer = time.perf_counter()
                                    # print("Choque confirmado de derecha")
                                    if last_touch == "DER":
                                        last_touch_count +=1
                                        last_touch_osc_count = 0
                                        last_touch_timer = time.perf_counter()
                                        # print("Derecha count: {}".format(last_touch_count))
                                    elif last_touch == "IZQ":
                                        last_touch_osc_count += 1
                                        last_touch_count = 0
                                        last_touch_osc_timer = time.perf_counter()
                                        # print("Oscilation count: {}".format(last_touch_osc_count))
                                    last_touch = "DER"
                                    if last_touch_count >= last_touch_counter.value:
                                        antiloop("DER")
                                        last_touch = "IZQ"
                                    elif last_touch_osc_count >= last_touch_osc_counter.value:
                                        antiloop("OSC")
                                        last_touch_osc_count = 0
                                    else:
                                        if second_back == False:
                                            back_change = backwards_counter.value
                                            second_back = True
                                        else:
                                            back_change = int(backwards_counter.value * 1.5)
                                            second_back = False
                                        move(-1.0, 0)
                                        # print("Going backwards")
                                        while (backward_count < back_change and auto_req.value == True):
                                            time.sleep(1)
                                            backward_count += 1
                                        move(0, -1.0)
                                        # print("Going left")
                                        while (steer_count < steer_counter.value and auto_req.value == True):
                                            time.sleep(0.25)
                                            move(1.0, 0)
                                            time.sleep(0.45)
                                            move(0, -1.0)
                                            time.sleep(0.25)
                                            move(1.0, 0)
                                            time.sleep(0.45)
                                            move(0, -1.0)
                                            steer_count += 1
                                crash_confirmed = False
                        if crash_confirmed:
                            # print("Choque frontal")
                            timer = time.perf_counter()
                            move(-1.0, 0)
                            # print("Going backwards")
                            if second_back == False:
                                back_change = backwards_counter.value
                                second_back = True
                            else:
                                back_change = int(backwards_counter.value * 1.5)
                                second_back = False
                            while (backward_count < back_change and auto_req.value == True):
                                time.sleep(1)
                                backward_count += 1
                            if last_touch == "IZQ":
                                go_right = True
                            elif last_touch == "DER":
                                go_right = False
                            else:
                                go_right = random.choice([True, False])
                            last_touch = "FRO"
                            if go_right == True:
                                move(0, 1.0)
                                # print("Going right")
                            else:
                                move(0, -1.0)
                                # print("Going left")
                            while (steer_count < steer_counter.value and auto_req.value == True):
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                steer_count += 1

                    elif not (button_middle.is_pressed or button_left.is_pressed or button_right.is_pressed):
                        pass
                    elif (button_left.is_pressed and button_right.is_pressed):
                        # print("Toque randomn")
                        crash_confirmed = False
                        crash_timer = time.perf_counter()
                        if crash_timeout.value > 0:
                            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                                time.sleep(0.25)
                                if (button_left.is_pressed and button_right.is_pressed):
                                    crash_confirmed = True
                                else:
                                    crash_confirmed = False
                                    break
                        else:
                            crash_confirmed = True
                        if crash_confirmed:
                            # print("Choque confirmado randomn")
                            timer = time.perf_counter()
                            move(-1.0, 0)
                            # print("Going backwards")
                            if second_back == False:
                                back_change = backwards_counter.value
                                second_back = True
                            else:
                                back_change = int(backwards_counter.value * 1.5)
                                second_back = False
                            while (backward_count < back_change and auto_req.value == True):
                                time.sleep(1)
                                backward_count += 1
                            if last_touch == "IZQ":
                                go_right = True
                            elif last_touch == "DER":
                                go_right = False
                            else:
                                go_right = random.choice([True, False])
                            last_touch = "FRO"
                            if go_right == True:
                                move(0, 1.0)
                                # print("Going right")
                            else:
                                move(0, -1.0)
                                # print("Going left")
                            while (steer_count < steer_counter.value and auto_req.value == True):
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                time.sleep(0.25)
                                move(1.0, 0)
                                time.sleep(0.45)
                                if go_right == True:
                                    move(0, 1.0)
                                    # print("Going right")
                                else:
                                    move(0, -1.0)
                                    # print("Going left")
                                steer_count += 1
                else:
                    # print('No paso nada')
                    timer = time.perf_counter()
                    move(-1.0, 0)
                    # print("Going backwards")
                    if second_back == False:
                        back_change = backwards_counter.value
                        second_back = True
                    else:
                        back_change = int(backwards_counter.value * 1.5)
                        second_back = False
                    while (backward_count < back_change and auto_req.value == True):
                        time.sleep(1)
                        backward_count += 1
                    go_right = random.choice([True, False])
                    if go_right == True:
                        move(0, -1.0)
                        # print("Going right")
                    else:
                        move(0, 1.0)
                        # print("Going left")
                    while (steer_count < steer_counter.value and auto_req.value == True):
                        time.sleep(0.25)
                        move(1.0, 0)
                        time.sleep(0.45)
                        if go_right == True:
                            move(0, 1.0)
                            # print("Going right")
                        else:
                            move(0, -1.0)
                            # print("Going left")
                        time.sleep(0.25)
                        move(1.0, 0)
                        time.sleep(0.45)
                        if go_right == True:
                            move(0, 1.0)
                            # print("Going right")
                        else:
                            move(0, -1.0)
                            # print("Going left")
                        time.sleep(0.25)
                        move(1.0, 0)
                        time.sleep(0.45)
                        if go_right == True:
                            move(0, 1.0)
                            # print("Going right")
                        else:
                            move(0, -1.0)
                            # print("Going left")
                        steer_count += 1
        time.sleep(1)


def camera(man):

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
    camera_rate = multiprocessing.Value('i', 0)
    auto_req = multiprocessing.Value('b', False)
    flash_req = multiprocessing.Value('b', False)
    timer_boring = multiprocessing.Value('i', 0)
    imu_req = multiprocessing.Value('b', False)
    pitch_flag = multiprocessing.Value('i', 0)
    pitch_counter = multiprocessing.Value('i', 0)
    cam_stuck_flag = multiprocessing.Value('b', False)
    imu_stuck_flag = multiprocessing.Value('b', False)
    taking_pics = multiprocessing.Value('b', False)
    is_stopped = multiprocessing.Value('b', True)
    is_hot = multiprocessing.Value('b', False)
    temp_cpu = multiprocessing.Value('d', 0)
    temp_clock = multiprocessing.Value('d', 0)
    temp_out = multiprocessing.Value('d', 0)
    humedad = multiprocessing.Value('d', 0)
    amoniaco = multiprocessing.Value('d', 0)
    timer_stuck_pic = multiprocessing.Value('d', 0)
    timer_temp = multiprocessing.Value('i', 0)
    timer_log = multiprocessing.Value('i', 0)
    timer_rest = multiprocessing.Value('i', 0)
    timer_wake = multiprocessing.Value('i', 0)
    steer_counter = multiprocessing.Value('i', 0)
    backwards_counter = multiprocessing.Value('i', 0)
    crash_timeout = multiprocessing.Value('d', 0)
    last_touch_timeout = multiprocessing.Value('i', 0)
    last_touch_counter = multiprocessing.Value('i', 0)
    last_touch_osc_counter = multiprocessing.Value('i', 0)
    pic_sensibility = multiprocessing.Value('i', 0)

    manager = multiprocessing.Manager()
    lst = manager.list()
    lst.append(None)
    flash_req.value = config["flash"]
    cam_req.value = config["camera"]
    camera_rate.value = admin["camera_rate"]
    auto_req.value = config["auto"]
    timer_boring.value = admin["timer_boring"]
    imu_req.value = config["imu_req"]
    pitch_flag.value = admin["pitch_flag"]
    pitch_counter.value = admin["pitch_counter"]
    timer_stuck_pic.value = admin["timer_stuck_pic"]
    timer_temp.value = admin["timer_temp"]
    timer_log.value = admin["timer_log"]
    timer_rest.value = admin["timer_rest"]
    timer_wake.value = admin["timer_wake"]
    steer_counter.value = admin["steer_counter"]
    backwards_counter.value = admin["backwards_counter"]
    date_crash_timeout = admin["date_crash_timeout"]
    crash_timeout_before = admin["crash_timeout_before"]
    crash_timeout_after = admin["crash_timeout_after"]
    last_touch_timeout.value = admin["last_touch_timeout"]
    last_touch_counter.value = admin["last_touch_counter"]
    last_touch_osc_counter.value = admin["last_touch_osc_counter"]
    pic_sensibility.value = admin["pic_sensibility"]
    today = datetime.now()
    date_int = today.year*10000+today.month*100+today.day
    if date_int >= date_crash_timeout:
        crash_timeout.value = crash_timeout_after
        print("SENSIBILIDAD BAJA")
        logwriter("Sensibilidad baja", id=15)
    else:
        print("SENSIBILIDAD ALTA")
        crash_timeout.value = crash_timeout_before
        logwriter("Sensibilidad alta", id=15)


    json_state = {"flash": flash_req.value, "auto": auto_req.value,
                  "camera": cam_req.value, "imu_req": imu_req.value}
    with open('/var/www/html/state.json', 'w') as outfile:
        json.dump(json_state, outfile)
    # Set up our websocket handler
    command_handler = multiprocessing.Process(
        target=command, args=(cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag, flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, timer_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, timer_rest, timer_wake, steer_counter, backwards_counter, timer_boring, crash_timeout,))
    # Set up our camera
    camera_handler = multiprocessing.Process(
        target=camera, args=(lst,))
    auto_handler = multiprocessing.Process(
        target=auto, args=(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, is_hot, timer_rest, timer_wake, steer_counter, backwards_counter, crash_timeout, last_touch_timeout,last_touch_counter, last_touch_osc_counter, flash_req ))
    pitch_handler = multiprocessing.Process(
        target=pitch, args=(lst, imu_req, pitch_flag, cam_stuck_flag, imu_stuck_flag, cam_req, camera_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, timer_stuck_pic, pitch_counter, timer_temp, timer_log, pic_sensibility,))
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
    motor_1_pwm = DigitalOutputDevice("BOARD38")
    motor_2_pwm = DigitalOutputDevice("BOARD35")
    motor_1_dir = DigitalOutputDevice("BOARD40")
    motor_2_dir = DigitalOutputDevice("BOARD37")
    start_time = time.perf_counter()
    led_ground.off()
    motor_1_pwm.off()
    motor_2_pwm.off()
    try:
        if os.path.exists("/dev/sda"):
            led_enable.on()
            os.system("sudo mount /dev/sda1 /media/usb")
            print("lo monte")
            if not os.path.exists("/media/usb/backup"):
                os.system("mkdir /media/usb/backup ")
            os.system("sudo cp -r log /media/usb/backup")
            if not os.path.exists("/media/usb/backup/resources"):
                os.system("mkdir /media/usb/backup/resources")
            os.system(
                "sudo rsync -aP --ignore-existing resources/ /media/usb/backup/resources")
            os.system("sudo umount /media/usb")
            print("Termine backup")
            while True:
                led_enable.on()
                time.sleep(0.2)
                led_enable.off()
                time.sleep(0.2)
        else:
            pass
    except:
        pass
    while time.perf_counter()-start_time < 20:
        led_enable.on()
        time.sleep(0.5)
        led_enable.off()
        time.sleep(0.5)
    print(bcolors.OKGREEN + "CHICKENBOT 2.0 APPELIE ROBOTICS - 2022" + bcolors.ENDC)
    flash_enable.off()
    if not os.path.exists("log/log.csv"):
        print("No existe el logfile")
        header = ["#", "Fecha", "Hora", "Evento","Minutos", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open('log/log.csv', 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    try:
        with open('/var/www/html/config.json') as json_file:
            config = json.load(json_file)
        with open('/var/www/html/backup_config.json', 'w') as outfile:
            json.dump(config, outfile)
    except:
        with open('/var/www/html/backup_config.json') as json_file:
            config = json.load(json_file)
        with open('/var/www/html/config.json', 'w') as outfile:
            json.dump(config, outfile)
    try:
        with open('/var/www/html/admin.json') as admin_file:
            admin = json.load(admin_file)
        with open('/var/www/html/backup_admin.json', 'w') as outfile:
            json.dump(admin, outfile)
    except:
        with open('/var/www/html/backup_admin.json') as admin_file:
            admin = json.load(admin_file)
        with open('/var/www/html/admin.json', 'w') as outfile:
            json.dump(admin, outfile)
    print(config)
    print(admin)
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
    led_enable.on()
    #  with open('/var/www/html/config.json') as json_file:
    #         config = json.load(json_file)
    #     with open('/var/www/html/backup_config.json', 'w') as outfile:
    #         json.dump(config, outfile)
    # except:
    #     with open('/var/www/html/backup_config.json') as json_file:
    #         config = json.load(json_file)
    #     with open('/var/www/html/config.json', 'w') as outfile:
    #         json.dump(config, outfile)
    if not os.path.exists("log/error.log"):
        with open('log/error.log', 'w') as errlog:
            errlog.write("START ERROR LOG")
    if not os.path.exists("stuck_count.json"):
        json_stuck_line = {"IMU": 0, "Cam": 0}
        with open('stuck_count.json', 'w') as outfile:
            json.dump(json_stuck_line, outfile)
    try:
        with open('stuck_count.json') as json_stuck:
            last_stuck = json.load(json_stuck)
        with open('stuck_count_backup.json', 'w') as outfile:
            json.dump(last_stuck, outfile)
    except:
        try:
            with open('stuck_count_backup.json') as json_stuck:
                last_stuck = json.load(json_stuck)
            with open('stuck_count.json', 'w') as outfile:
                json.dump(last_stuck, outfile)
        except:
            json_stuck_line = {"IMU": -1, "Cam": -1}
            with open('stuck_count.json', 'w') as outfile:
                json.dump(json_stuck_line, outfile)
            with open('stuck_count.json') as json_stuck:
                last_stuck = json.load(json_stuck)
    try:
        with open('last_on.json') as json_on:
            last_watch = json.load(json_on)
    except:
        try:
            with open('last_on_backup.json') as json_on:
                last_watch = json.load(json_on)
            with open('last_on.json', 'w') as outfile:
                json.dump(last_watch, outfile)
        except:
            last_on()
            with open('last_on.json') as json_on:
                last_watch = json.load(json_on)
    

    start_log = "Me apague, minutos trabado camara / IMU " 
    minutos_start = str(last_stuck["Cam"]) + "/" + str(last_stuck["IMU"])
    logwriter(start_log, id=6, watch_dog=True, minutos= minutos_start, 
                last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    json_stuck_line = {"IMU": 0, "Cam": 0}
    with open('stuck_count.json', 'w') as outfile:
        json.dump(json_stuck_line, outfile)
    logwriter("Me prendi con esta configuracion: " + str(config)+'/'+str(admin), id=1)
    last_on()
    if config["flash"] == True:
        flash_enable.on()
        logwriter("Prendi luces", id=12)
    # motors_enable.on()
    try:
        main()
    except KeyboardInterrupt:
        motor_1_pwm = DigitalOutputDevice("BOARD38")
        motor_2_pwm = DigitalOutputDevice("BOARD35")
        motor_1_pwm.off()
        motor_2_pwm.off()
        led_enable.off()
        for p in PROCESSES:
            p.terminate()

