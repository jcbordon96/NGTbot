#!/usr/bin/env python3

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
import time
import random
from gpiozero import DigitalOutputDevice, PWMOutputDevice, Button
import smbus
import smbus2
import math
from datetime import date, datetime
import sensors
import os
import csv
from scipy.linalg import norm
from scipy import optimize
import numpy as np
import Adafruit_ADS1x15
import board
import adafruit_dht
from mlx90614 import MLX90614
import psutil
from bmp280 import BMP280
import bme280
import requests
import subprocess
from zipfile import ZipFile
import VL53L0X
import sys
from bluetooth import *


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

flash_enable = DigitalOutputDevice("BOARD12", active_high=True)
led_enable = DigitalOutputDevice("BOARD16", active_high=True)
def printe(*what_to_print):
    if prints_enable:
        string = ""
        for items in what_to_print:
            string += str(items) + " "
        print(string)
def open_json(filename):
    place = filename.strip(filename.split('/')[-1])
    stripped_name = filename.split('/')[-1].split('.')[0]
    stripped_extension = '.' + filename.split('/')[-1].split('.')[-1]
    stripped_name_backup = stripped_name +'_backup'
    try:
        with open(filename) as json_file:
            config = json.load(json_file)
        with open(place + stripped_name_backup + stripped_extension , 'w') as outfile:
            json.dump(config, outfile)
    except:
        with open(place + stripped_name_backup + stripped_extension) as json_file:
            config = json.load(json_file)
        with open(filename, 'w') as outfile:
            json.dump(config, outfile)
    return config

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
    errlog = error_date + " Error: "+ err + " Comentario: "+ comentario + '\n'
    with open("log/error.log",'a', newline='') as logerror:
        logerror.write(errlog)

def logwriter(event, id,  minutos =0, t_cpu=0, t_clock=0, t_dht=0, t_bme=0, t_bmp=0, t_laser_surf = 0, t_laser_amb = 0, h_dht=0, h_bme=0, p_bme=0, p_bmp=0, thi = 0, score_temp_amb_rt = 0, score_temp_bed_rt = 0, score_hum_rt = 0, score_thi_rt = 0, score_general_rt = 0, score_temp_amb_prom = 0, score_temp_bed_prom = 0, score_hum_prom = 0, score_thi_prom = 0, score_general_prom = 0, t_total= 0, t_active = 0, t_rest = 0, t_stuck = 0, watch_dog=False, last_date=-1, last_hour=-1, last_name=-1):
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
        printe("No existe el logfile diario")
        header = ["#", "Fecha", "Hora", "Evento", "Minutos", "CPU", "RAM" ,"T. CPU",
                  "T. Clock", "T. DHT", "T. BME", "T. BMP", 'T. Ambiente Laser', "T. Laser", "H. DHT", "H. BME", "P. BME", "P. BMP", "THI", "Score Temp. Amb. RT", "Score Temp. Cama RT", "Score Hum. RT", "Score THI RT", "Score General RT", "Score Temp. Amb. PROM", "Score Temp. Cama PROM", "Score Hum. PROM", "Score THI PROM", "Score General PROM", "Tiempo Total", "Tiempo Activo", "Tiempo Descansando", "Tiempo Trabado", "ID"]
        with open(stringdatelog, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelog, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        psutil.cpu_percent(percpu = True)
        wr.writerow(["", logdate, loghour, event, minutos,str(os.getloadavg()[0]), str(psutil.virtual_memory().percent), t_cpu,
                    t_clock, t_dht, t_bme, t_bmp, t_laser_amb, t_laser_surf, h_dht, h_bme, p_bme, p_bmp, thi, score_temp_amb_rt, score_temp_bed_rt, score_hum_rt, score_thi_rt, score_general_rt, score_temp_amb_prom, score_temp_bed_prom, score_hum_prom, score_thi_prom, score_general_prom, t_total, t_active, t_rest, t_stuck, id])

    if not os.path.exists(stringdatelogbackup):
        printe("No existe el logfile diario de backup")
        header = ["#", "Fecha", "Hora", "Evento", "Minutos", "CPU", "RAM" ,"T. CPU",
                  "T. Clock", "T. DHT", "T. BME", "T. BMP", 'T. Ambiente Laser', "T. Laser", "H. DHT", "H. BME", "P. BME", "P. BMP", "THI", "Score Temp. Amb. RT", "Score Temp. Cama RT", "Score Hum. RT", "Score THI RT", "Score General RT", "Score Temp. Amb. PROM", "Score Temp. Cama PROM", "Score Hum. PROM", "Score THI PROM", "Score General PROM", "Tiempo Total", "Tiempo Activo", "Tiempo Descansando", "Tiempo Trabado", "ID"]
        with open(stringdatelogbackup, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelogbackup, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos,str(os.getloadavg()[0]), str(psutil.virtual_memory().percent), t_cpu,
                    t_clock, t_dht, t_bme, t_bmp, t_laser_amb, t_laser_surf, h_dht, h_bme, p_bme, p_bmp, thi, score_temp_amb_rt, score_temp_bed_rt, score_hum_rt, score_thi_rt, score_general_rt, score_temp_amb_prom, score_temp_bed_prom, score_hum_prom, score_thi_prom, score_general_prom, t_total, t_active, t_rest, t_stuck, id])

    with open('log/log.csv', 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos,str(os.getloadavg()[0]), str(psutil.virtual_memory().percent), t_cpu,
                    t_clock, t_dht, t_bme, t_bmp, t_laser_amb, t_laser_surf, h_dht, h_bme, p_bme, p_bmp, thi, score_temp_amb_rt, score_temp_bed_rt, score_hum_rt, score_thi_rt, score_general_rt, score_temp_amb_prom, score_temp_bed_prom, score_hum_prom, score_thi_prom, score_general_prom, t_total, t_active, t_rest, t_stuck, id])

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


PROCESSES = []
STATE = {"value": 0}
USERS = set()


def command(cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag,  flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, timer_rest, timer_wake, backwards_counter, timer_boring, crash_timeout, x_com, z_com):


    printe("COMMAND INIT")

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
        printe("COMMAND SOCKET INIT")

        await register(websocket)
        # await websocket.send(man[0].tobytes())
        try:
            await websocket.send(state_event())

            async for message in websocket:
                data = json.loads(message)
                # printe(data)
                if data["action"] == "move":
                    x_com.value = data["x"]
                    z_com.value = data["z"]
                    STATE["value"] = "x:{}/z:{}".format(
                        data["x"], data["z"])
                    # move(x, z)
                    await notify_state()
                elif data["action"] == "stop":
                    STATE["value"] = "STOP"
                    x_com.value = 0
                    z_com.value = 0
                    # move(x, z)
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
                    printe(auto_req.value)
                    # auto(auto_req)
                    pass
                elif data["action"] == "camera":
                    cam_req.value = data["req"]
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "auto_rate":
                    timer_boring.value = int(data["req"])
                    printe(timer_boring.value)
                    # auto(auto_req)
                    pass
                elif data["action"] == "camera_rate":
                    camera_rate.value = int(data["req"])
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "imu_req":
                    imu_req.value = data["req"]
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "imu_flag":
                    pitch_flag.value = int(data["req"])
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "flash":
                    flash_req.value = data["req"]
                    if flash_req.value == True:
                        flash_enable.on()
                        logwriter("Prendi luces", id=12)
                    else:
                        flash_enable.off()
                        logwriter("Apague luces", id=13)
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "temp_req":
                    state_string = ""
                    state_string = str(temp_cpu.value) + "°C/" + str(temp_clock.value) + "°C/" + str(temp_out.value) + "°C/" + str(humedad.value) + "%HR/" + str(amoniaco.value) + "ppm"
                    # printe(state_string)

                    await websocket.send(json.dumps({"type": "temp", "data": state_string}))

                    pass
                elif data["action"] == "save_req":
                    try:
                        if os.path.exists("/dev/sda"):
                            os.system("sudo mount /dev/sda1 /media/usb")
                            printe("lo monte")
                            if not os.path.exists("/media/usb/backup"):
                                os.system("mkdir /media/usb/backup ")
                            os.system("sudo cp -r log /media/usb/backup")
                            if not os.path.exists("/media/usb/backup/resources"):
                                os.system("mkdir /media/usb/backup/resources")
                            os.system(
                                "sudo rsync -aP --ignore-existing resources/ /media/usb/backup/resources")
                            os.system("sudo umount /media/usb")
                            printe("Termine backup")
                            await websocket.send(json.dumps({"type": "save_state", "data": True}))
                        else:
                            printe("No esta conectado el usb")
                            await websocket.send(json.dumps({"type": "save_state", "data": False}))
                    except Exception as ex:
                        errorwriter(ex, "No se monto el USB")
                        printe("No pude montar el usb")
                elif data["action"] == "reboot":
                    logwriter("Recibi pedido de reiniciar", id=7)
                    time.sleep(1)
                    os.system("sudo reboot now")
                    # printe(cam_req.value)
                    pass
                elif data["action"] == "shutdown":
                    logwriter("Recibi pedido de apagado", id=8)
                    time.sleep(1)
                    os.system("sudo shutdown now")

                    # printe(cam_req.value)
                    pass

                else:
                    logging.error("unsupported event: %s", data)
                json_state = {"flash": flash_req.value, "auto": auto_req.value, "auto_rate": timer_boring.value,
                              "camera": cam_req.value, "camera_rate": camera_rate.value, "imu_req": imu_req.value, "imu_flag": pitch_flag.value}
                with open('/var/www/html/state.json', 'w') as outfile:
                    json.dump(json_state, outfile)
        finally:
            await unregister(websocket)

    start_server2 = websockets.serve(counter, "192.168.4.1", 9001)
    asyncio.get_event_loop().run_until_complete(start_server2)
    asyncio.get_event_loop().run_forever()

def pitch(man, imu_req, pitch_flag, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, clearance, cam_req, camera_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_counter, timer_temp, timer_log, pic_sensibility, stucks_to_confirm, stuck_window, is_rest, flash_req, current_date, score_config, zero_date, day_score_config, breeding_day, campaign_id):
    #region Iniciar Variables
    printe("CAMERA INIT")
    shutdown_button = Button('BOARD5')
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    raw_capture = PiRGBArray(camera, size=(640, 480))
    is_stuck_confirm = False
    last_pic = 0
    last_imu = 0
    was_taking = False
    first_img = True
    log_imu_stuck = True
    log_cam_stuck = True
    log_cam = False
    temp_timer = 0
    state_timer = time.perf_counter()
    is_hot.value = False
    start_cam_stuck = 0
    start_imu_stuck = 0
    elapsed_cam_stuck = 0
    elapsed_imu_stuck = 0
    total_elapsed_cam_stuck = 0
    total_elapsed_imu_stuck = 0
    last_total_elapsed_cam_stuck = 0
    last_total_elapsed_imu_stuck = 0
    confirm_elapsed_cam_stuck = 0
    confirm_total_elapsed_cam_stuck = 0
    confirm_last_total_elapsed_cam_stuck = 0
    confirm_log_cam_stuck = True
    confirm_start_cam_stuck = 0
    is_tails = False
    reference_stuck = 0
    stuck_count = 0
    img_to_filter = []
    # Cargo configuracion de scoring
    t_amb_max = float(day_score_config['T. Ambiente Max'])
    t_amb_min = float(day_score_config['T. Ambiente Min'])
    t_amb_delta= float(day_score_config['Amplitud Termica Amb.'])
    t_amb_optimum = float(day_score_config['Promedio Optimo Amb.'])
    h_max = float(day_score_config['Humedad Max'])
    h_min = float(day_score_config['Humedad Min'])
    h_delta= float(day_score_config['Amplitud Humedad'])
    h_optimum = float(day_score_config['Promedio Optimo Humedad'])
    thi_optimum = float(day_score_config['THI Optimo'])
    if day_score_config['T. Cama Max'] != '':
        t_bed_max = float(day_score_config['T. Cama Max'])
        t_bed_min = float(day_score_config['T. Cama Min'])
        t_bed_delta = float(day_score_config['Amplitud Termica Cama'])
        t_bed_optimum = float(day_score_config['Promedio Optimo Cama'])
        bed_check = True
    else:
        bed_check = False
    score_temp_amb_rt = 0
    score_temp_bed_rt = 0
    score_hum_rt = 0
    score_thi_rt = 0
    score_general_rt = 0
    score_temp_amb_prom = 0
    score_temp_bed_prom = 0
    score_hum_prom = 0
    score_thi_prom = 0
    score_general_prom = 0
    t_amb_list = []
    t_bed_list = []
    h_list = []
    thi_list = []
    t_init = time.perf_counter()
    t_total = 0
    t_active = 0
    t_rest = 0
    t_stuck = 0
    last_t_stuck = 0
    last_t_active = 0
    last_t_rest = 0
    t_stuck_sec = 0
    t_active_sec = 0
    t_rest_sec = 0
    counter = 0
    dht_ok = False
    imu_ok = False
    dht_init = False
    dht_fail_counter = 0
    moving_img = False
    t_mlx_amb = 0 
    t_mlx_surface = 0
    last_measure = 0
    measure_rate = 1 #Hz
    t_bme_list = []
    h_bme_list = []
    p_bme_list = []
    t_bmp_list = []
    p_bmp_list = []
    t_dht_list = []
    h_dht_list = []
    t_mlx_amb_list = []
    t_mlx_surface_list = []
    t_bme_mean = 0
    h_bme_mean = 0
    p_bme_mean = 0
    t_bmp_mean = 0
    p_bmp_mean = 0
    t_dht_mean = 0
    h_dht_mean = 0
    t_mlx_amb_mean = 0
    t_mlx_surface_mean = 0
    measurements_list = []
    robot_id = 2
    day_info = {"day": {"breeding_day": breeding_day, "config": day_score_config, "total_time": 0, "active_time": 0, "rest_time": 0, "stuck_time": 0, "date": current_date, "campaign_id": campaign_id}}
    day_info_list = []
    url = "http://192.168.0.191:4000/loadMeasurements"
    url_img = "http://192.168.0.191:4000/loadImages"
    data_was_sended = False
    img_to_compress = []
    imu_debug = False
    
    #endregion
    #region Levanto mediciones que han quedado sin enviar antes de apagarse
    if os.path.exists("send_queue/logs"):
        for file in os.listdir("send_queue/logs"):
            if file.find("_backup") == -1:
                try:
                    with open('send_queue/logs/{}'.format(file)) as send_file:
                        day_info_list.append(json.load(send_file))
                except:
                    try:
                        backup_string = 'send_queue/logs/'+file.split(".")[0]+"_backup.json"
                        with open(backup_string) as send_file:
                            day_info_list.append(json.load(send_file))
                    except:
                        pass
    day_info_list.append("")
    #endregion
    #region Levanto lista de fotos que todavia no han sido enviadas
    try:
        with open('send_queue/imgs/list/img_to_compress.json') as send_file:
            img_to_compress += json.load(send_file)
    except:
        try:

            with open('send_queue/imgs/list/img_to_compress_backup.json') as send_file:
                img_to_compress += json.load(send_file)
        except:
            pass

    #endregion
    #region Funciones
    def mean_check(value_list):
        try:
            if len(value_list) > 0:
                return np.mean(value_list)
            else:
                return 0
        except:
            printe("Error mean check")
            return 0

    def thi_calc(temperatura, humedad):
        thi = temperatura+0.348*((humedad/100)*6.105*math.exp((17.27*temperatura)/(237.7+temperatura)))
        return thi

    def compare_images(img1, img2):
        # normalize to compensate for exposure difference
        try:
            img1 = normalize(img1)
            img2 = normalize(img2)
            diff = img1 - img2  # elementwise for scipy arrays
            m_norm = np.sum(abs(diff))  # Manhattan norm
            return (m_norm)
        except:
            return math.nan


    def to_grayscale(arr):
        if len(arr.shape) == 3:
            # average over the last axis (color channels)
            return np.average(arr, -1)
        else:
            return arr

    def normalize(arr):
        rng = arr.max()-arr.min()
        amin = arr.min()
        return (arr-amin)*255/rng

    def MPU_Init():
        bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)# write to sample rate register
        bus.write_byte_data(Device_Address, PWR_MGMT_1, 1) # Write to power management register
        bus.write_byte_data(Device_Address, CONFIG, 0)# Write to Configuration register
        bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)# Write to Gyro configuration register
        bus.write_byte_data(Device_Address, INT_ENABLE, 1)# Write to interrupt enable register

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
    #endregion
    #region Inicio de sensores
    
    try:
        mlxbus = smbus.SMBus(4)
        mlx = MLX90614(mlxbus, address=0x5A)
        printe("Laser inicio bien")
        mlx_ok = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar medidor laser")
        printe("Error al inciar el medidor laser")
        mlx_ok = False
    try:
        bmp280 = BMP280(i2c_dev=mlxbus, i2c_addr = 0x77)
        bmp280.setup(mode="forced")
        bmp_ok = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el BMP")
        printe(ex, "Error al iniciar el BMP")
        bmp_ok = False
    try:
        calibration_params = bme280.load_calibration_params(mlxbus,0x76)
        bme = bme280.sample(mlxbus, 0x76, calibration_params)
        bme_ok = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el BME")
        printe(ex, "Error al iniciar el BME")
        bme_ok = False
    try:
        dhtDevice = adafruit_dht.DHT22(board.D26, use_pulseio=False)
        dht_init = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el DHT")
        printe("Error al iniciar el DHT")
        pass
    try:
        bus = smbus.SMBus(5) 	# or bus = smbus.SMBus(0) for older version boards
        Device_Address = 0x68   # MPU6050 device address
        MPU_Init()
        printe("IMU INIT")
        imu_ok = True
    except Exception as ex:
        errorwriter(ex, "Error al iniciar el IMU")
        printe("El IMU no pudo iniciar")
    #endregion

    try:
        with open('counter.json') as json_file:
            img_index = json.load(json_file)
        img_index_num.value = img_index["num"]
    except:
        json_string = {"num": 0}
        with open('counter.json', 'w') as outfile:
            json.dump(json_string, outfile)
        with open('counter.json') as json_file:
            img_index = json.load(json_file)
        img_index_num.value = img_index["num"]


    while True:
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                f = frame.array
                cv2.waitKey(1)
                f = cv2.resize(f, (640, 480))
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                man[0] = cv2.imencode('.jpg', f, encode_param)[1]# The function imencode compresses the image and stores it in the memory buffer that is resized to fit
                if shutdown_button.is_pressed:
                    time.sleep(3)
                    if shutdown_button.is_pressed:
                        printe('Me voy a apagar')
                        logwriter("Recibi pedido de apagado", id=8)
                        os.system("sudo shutdown now")
                
                if first_img:
                    image_to_compare0 = f
                    first_img = False
                    compare_timer = time.perf_counter()
                # Comparo cuantas veces se detecto trabado en la ultima ventana
                if time.perf_counter() - reference_stuck > stuck_window.value:
                    if stuck_count >= stucks_to_confirm.value:
                        if confirm_log_cam_stuck:
                            logwriter("Me trabe, camara CONFIRMADO", id=20)
                            printe("Me trabe confirmado")
                            is_stuck_confirm = True
                            confirm_log_cam_stuck = False
                            confirm_start_cam_stuck = time.perf_counter() 
                            confirm_last_total_elapsed_cam_stuck = confirm_total_elapsed_cam_stuck
                        confirm_elapsed_cam_stuck = (time.perf_counter() - confirm_start_cam_stuck)/60.0
                        confirm_total_elapsed_cam_stuck = confirm_last_total_elapsed_cam_stuck + confirm_elapsed_cam_stuck
                        json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                        with open('stuck_count.json', 'w') as outfile:
                            json.dump(json_stuck_line, outfile)
                        with open('stuck_count_backup.json', 'w') as outfile:
                            json.dump(json_stuck_line, outfile)
                    elif not confirm_log_cam_stuck and stuck_count < stucks_to_confirm.value:
                        confirm_elapsed_cam_stuck = (time.perf_counter() - confirm_start_cam_stuck)/60.0
                        confirm_total_elapsed_cam_stuck = confirm_last_total_elapsed_cam_stuck + confirm_elapsed_cam_stuck
                        json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                        with open('stuck_count.json', 'w') as outfile:
                            json.dump(json_stuck_line, outfile)
                        with open('stuck_count_backup.json', 'w') as outfile:
                            json.dump(json_stuck_line, outfile)
                        confirm_log_cam_stuck = True
                        is_stuck_confirm = False
                        logwriter("Me destrabe, camara CONFIRMADO, minutos:", minutos=round(confirm_elapsed_cam_stuck,2), id=21)
                    reference_stuck = time.perf_counter()
                    stuck_count = 0
                # Comparo imagenes
                if time.perf_counter() - compare_timer > window_stuck_pic.value:
                    # Si no esta frenado vamos a comparar las imagenes para ver si esta trabado
                    #region Comparacion de imagenes
                    if not is_stopped.value:
                        img0 = to_grayscale(image_to_compare0.astype(float))
                        img1 = to_grayscale(f.astype(float))
                        n_m = compare_images(img0, img1)
                        if not math.isnan(n_m):
                            if ((n_m/img0.size) < pic_sensibility.value):
                            # if False:
                                stuck_count += 1
                                if log_cam_stuck:
                                    logwriter("Me trabe, camara", id=16)
                                    log_cam_stuck = False
                                    start_cam_stuck = time.perf_counter()
                                    last_total_elapsed_cam_stuck = total_elapsed_cam_stuck
                                cam_stuck_flag.value = True
                                elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                                total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                                with open('stuck_count.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                with open('stuck_count_backup.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                
                            else:
                                if not log_cam_stuck:
                                    elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                                    total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                    json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                                    with open('stuck_count.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    with open('stuck_count_backup.json', 'w') as outfile:
                                        json.dump(json_stuck_line, outfile)
                                    # printe(" Cam destuck")
                                    logwriter("Me destrabe, camara, minutos:", minutos=round(elapsed_cam_stuck,2), id=17)
                                    log_cam_stuck = True
                                cam_stuck_flag.value = False
                        image_to_compare0 = f
                        compare_timer = time.perf_counter()
                    else:
                        if not log_cam_stuck:
                            elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                            total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                            json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                            with open('stuck_count.json', 'w') as outfile:
                                json.dump(json_stuck_line, outfile)
                            with open('stuck_count_backup.json', 'w') as outfile:
                                json.dump(json_stuck_line, outfile)
                            printe(" Cam destuck")
                            logwriter("Me destrabe, camara, minutos:", minutos=round(elapsed_cam_stuck,2), id=17)
                            log_cam_stuck = True
                        cam_stuck_flag.value = False
                    #endregion
                #region Guardar fotos
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
                    if time.perf_counter() - last_pic > (camera_rate.value - 1):
                        flash_enable.on()
                    if time.perf_counter() - last_pic > camera_rate.value:
                        now = datetime.now()
                        img_folder = now.strftime("%Y%m%d")
                        # printe(img_folder)
                        if not os.path.exists("resources/{}".format(img_folder)):
                            os.mkdir("resources/{}".format(img_folder))
                        if not is_stopped.value and not is_rest.value and not moving_img:
                            now = datetime.now()
                            d1 = now.strftime("%Y%m%d_%H%M%S")
                            img_type = "M"
                            img_name = "resources/{}/{}_{}_{}_{}.png".format(img_folder,d1,img_type,
                                                                        img_index_num.value, img_counter)
                            # printe(img_name)
                            img_to_filter.append(img_name)
                            moving_img = True
                            camera.capture(img_name)
                            img_to_compress.append(img_name)
                            with open('send_queue/imgs/list/img_to_compress.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                            with open('send_queue/imgs/list/img_to_compress_backup.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                        taking_pics.value = True      
                        if is_stopped.value == True:
                            last_pic = time.perf_counter()
                            printe("Voy a sacar foto detenida")
                            moving_img = False
                            now = datetime.now()
                            d1 = now.strftime("%Y%m%d_%H%M%S")
                            if is_rest.value:
                                img_type = "R"
                            elif is_stuck_confirm:
                                img_type = "S"
                            else:
                                img_type = "P"
                            img_name = "resources/{}/{}_{}_{}_{}.png".format(img_folder,d1,img_type,
                                                                    img_index_num.value, img_counter)
                            img_to_filter.append(img_name)
                            img_to_compress.append(img_name)
                            camera.capture(img_name)
                            with open('send_queue/imgs/list/img_to_compress.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                            with open('send_queue/imgs/list/img_to_compress_backup.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                            if is_rest.value or not flash_req.value:
                                flash_enable.off()
                            img_counter += 1
                            taking_pics.value = False
                            printe("Just take a pic")
                else:
                    was_taking = False
                    if log_cam:
                        logwriter("Termine de sacar fotos", id=4)
                        log_cam = False
                raw_capture.truncate(0)
                #endregion 
                
                #region Esta va a ser la rutina de envio de data cuando esta descansado
                if is_rest.value and not data_was_sended:
                    
                    try:
                        printe("Enabling wifi")
                        subprocess.call("./enablewifi_silent", timeout=40)
                    except Exception as e:
                        printe(e, "No me pude conectar para mandar data")
                        errorwriter(e, "No me pude conectar para mandar data")
                    
                    if len(day_info_list)>0:
                        day_info_to_send = day_info_list[0]
                        head = {u'content-type': u'application/json'}
                        printe("Envio esta data", day_info_to_send)
                        r = requests.post(url=url, data=json.dumps(day_info_to_send), headers=head)
                        printe(r)
                        if r.status_code == 200:
                            day_info_list.pop(0)
                    else:
                        # Borrar todo el contenido de la carpeta send queue
                        printe("No hay mas logs que enviar")
                        for file in os.listdir("send_queue/logs"):
                            os.remove(os.path.join("send_queue/logs",file))
                        # Ya no hay mas elementos para enviar, voy a esperar a que vuelva a entrar a descanso para entrar
                    if len(img_to_compress) > 0:
                        start_new_compress = True
                        printe("Hay imagenes para comprimir")
                        while start_new_compress:
                            start_new_compress = False
                            zip_name = datetime.now().strftime("%Y%m%d_%H%M%S")
                            printe(img_to_compress)
                            last_zip_name = img_to_compress[0].split('/')[1]
                            with ZipFile("send_queue/imgs/zipfiles/{}_{}.zip".format(last_zip_name,zip_name),'w') as zip:
                                while img_to_compress:
                                    if img_to_compress[0].split('/')[1] != last_zip_name:
                                        printe("Cambio de nombre")
                                        start_new_compress = True
                                        break
                                    printe("Agregando {} al zip".format(img_to_compress[0]))
                                    zip.write(img_to_compress[0])
                                    img_to_compress.pop(0)
                                    with open('send_queue/imgs/list/img_to_compress.json', 'w') as outfile:
                                        json.dump( img_to_compress, outfile)
                                    with open('send_queue/imgs/list/img_to_compress_backup.json', 'w') as outfile:
                                        json.dump( img_to_compress, outfile)
                            printe("Acabo de crear zip:", "{}_{}.zip".format(last_zip_name,zip_name))
                    else:
                        printe("No hay mas imagenes para comprimir")
                    if os.path.exists("send_queue/imgs/zipfiles"):
                        if len(os.listdir("send_queue/imgs/zipfiles")) > 0:
                            for zipfile in os.listdir("send_queue/imgs/zipfiles"):
                                delete_zip = False
                                date_zip_file = datetime.strptime(zipfile.split('_')[0], "%Y%m%d").strftime("%Y-%m-%d")
                                with open("send_queue/imgs/zipfiles/"+ zipfile, 'rb') as file:
                                    fileobj = [('zip', (zipfile, file, 'zip'))]
                                    head = {u'content-type': u'multipart/form-data'}
                                    printe("Voy a enviar imagenes, zipfile:", zipfile)
                                    r = requests.post(url=url_img, data={"robot_identifier": robot_id, "campaign_id": campaign_id, "date":date_zip_file}, files = fileobj)
                                    printe(r)
                                    if r.status_code == 200:
                                        delete_zip = True
                                if delete_zip:
                                    printe("Borre zip: ", zipfile)
                                    os.remove("send_queue/imgs/zipfiles/"+ zipfile)
                        else:
                            printe('No hay mas imagenes que mandar')
                    if len(os.listdir("send_queue/imgs/zipfiles")) == 0 and len(img_to_compress) == 0 and len(day_info_list) == 0:
                        data_was_sended = True
                        printe("No queda mas para hacer en reposo")

                elif not is_rest.value and data_was_sended:
                    data_was_sended = False
                    printe("Enabling hotspot")
                    subprocess.call("./enablehotspot_silent", timeout=40)
                #endregion
                #region Rutina de filtrado de imagenes
                if is_rest.value and len(img_to_filter) > 0:
                    try:
                        string_array = img_to_filter[0].split("/")
                        img = cv2.imread(img_to_filter[0])
                        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                        reduccion = cv2.resize(gris, (300, 300), interpolation = cv2.INTER_CUBIC)
                        filtrado = cv2.medianBlur(reduccion, 3)
                        umbral = cv2.Laplacian(filtrado, ddepth = cv2.CV_16S).var()
                        if not os.path.exists("resources/{}/ok".format(string_array[1])):
                            os.mkdir("resources/{}/ok".format(string_array[1]))
                        if not os.path.exists("resources/{}/not_ok".format(string_array[1])):
                            os.mkdir("resources/{}/not_ok".format(string_array[1]))
                        if umbral >= 90:
                            save_dir = string_array[0]+ "/" + string_array[1] + "/ok/" + string_array[2] 
                        else:
                            save_dir = string_array[0] + "/" + string_array[1] + "/not_ok/" + string_array[2] 
                        
                        cv2.imwrite(save_dir, img)
                        img_to_filter.pop(0)

                    except:
                        img_to_filter.pop(0)
                #endregion
                if imu_req.value == True and imu_ok == True and time.perf_counter()-last_imu > 0.5:
                    try:
                        last_imu = time.perf_counter()
                        acc_y = read_raw_data(ACCEL_YOUT_H)
                        acc_z = read_raw_data(ACCEL_ZOUT_H)
                        
                        
                        Ay = acc_y/16384.0
                        Az = acc_z/16384.0
                        
                        if imu_debug:
                            
                            gyro_x = read_raw_data(GYRO_XOUT_H)
                            gyro_y = read_raw_data(GYRO_YOUT_H)
                            gyro_z = read_raw_data(GYRO_ZOUT_H)
                            acc_x = read_raw_data(ACCEL_XOUT_H)
                            gyro_x = read_raw_data(GYRO_XOUT_H)
                            gyro_y = read_raw_data(GYRO_YOUT_H)
                            gyro_z = read_raw_data(GYRO_ZOUT_H)
                            Ax = acc_x/16384.0
                            Gx = gyro_x/131.0
                            Gy = gyro_y/131.0
                            Gz = gyro_z/131.0
                            imu_array = [Ax, Ay, Az, Gx, Gy, Gz]
                            imu_array_list = []
                        pitch = math.atan2(Ay,  Az) * 57.3

                        if pitch > pitch_flag.value:
                            counter += 1
                        else:
                            counter = 0
                            imu_stuck_flag.value = False
                            if log_imu_stuck == False:
                                elapsed_imu_stuck = round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                                total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                                json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                                with open('stuck_count.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                with open('stuck_count_backup.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                                printe("IMU destuck")
                                logwriter("Me destrabe, IMU, minutos", minutos=elapsed_imu_stuck, id=19)
                                log_imu_stuck = True
                        if counter > pitch_counter.value:
                            if log_imu_stuck:
                                printe("Estoy trabado!!! Detecte inclinacion mayor a la safe")
                                logwriter("Me trabe, IMU", id=18)
                                log_imu_stuck = False
                                start_imu_stuck = time.perf_counter()
                                last_total_elapsed_imu_stuck = total_elapsed_imu_stuck
                            elapsed_imu_stuck =  round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                            total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                            json_stuck_line = {"IMU": total_elapsed_imu_stuck, "Cam": total_elapsed_cam_stuck, "CamConf": confirm_total_elapsed_cam_stuck}
                            with open('stuck_count.json', 'w') as outfile:
                                json.dump(json_stuck_line, outfile)
                            with open('stuck_count_backup.json', 'w') as outfile:
                                    json.dump(json_stuck_line, outfile)
                            imu_stuck_flag.value = True
                    except Exception as ex:
                        errorwriter(ex, "El IMU no pudo tomar lectura")
                        printe(ex, "Ups! El IMU no pudo tomar lectura")
                if time.perf_counter() - temp_timer > timer_temp.value:
                    temp_timer = time.perf_counter()
                    last_on()
                    sensors.init()
                    try:
                        for chip in sensors.iter_detected_chips():
                            for feature in chip:
                                if feature.label == "temp1":
                                    if chip.adapter_name == "bcm2835 (i2c@7e804000)" or chip.adapter_name == "i2c-gpio-rtc@0":
                                        temp_clock.value = round(feature.get_value(), 1)
                                        if chip.adapter_name == "i2c-gpio-rtc@0":
                                            is_tails = True
                                    if chip.adapter_name == "Virtual device":
                                        temp_cpu.value = round(feature.get_value(), 1)
                    except:
                        pass
                    finally:
                        if (temp_cpu.value > 80 or temp_clock.value > 80) and not is_hot.value:
                            is_hot.value = True
                            logwriter("Alta temperatura", id=9, t_cpu=temp_cpu.value, t_clock=temp_clock.value)
                        sensors.cleanup()
                if time.perf_counter()-last_measure > measure_rate:
                    last_measure = time.perf_counter()
                    
                    if mlx_ok:
                        try:
                            t_mlx_surface = round(mlx.get_obj_temp(),2)
                            t_mlx_amb = round(mlx.get_amb_temp(),2)
                            if t_mlx_surface < 80:
                                t_mlx_surface_list.append(t_mlx_surface)
                            if t_mlx_amb < 80:
                                t_mlx_amb_list.append(t_mlx_amb)
                        except Exception as ex:
                            errorwriter(ex, "No se pudo tomar mediciones laser")
                            printe("Algo salio mal con el medidor laser")
                            printe(ex)
                    if bmp_ok:
                        t_bmp = round(bmp280.get_temperature(),2)
                        p_bmp = round(bmp280.get_pressure(),2)
                        t_bmp_list.append(t_bmp)
                        p_bmp_list.append(p_bmp)
                    if bme_ok:
                        bme = bme280.sample(mlxbus, 0x76, calibration_params)
                        t_bme = round(bme.temperature,2)
                        p_bme = round(bme.pressure,2)
                        h_bme = round(bme.humidity,2)
                        t_bme_list.append(t_bme)
                        p_bme_list.append(p_bme)
                        h_bme_list.append(h_bme)
                    if dht_init:
                        dht_fail_counter = 0
                        dht_ok = False
                        while not dht_ok:
                            dht_ok = True
                            try:
                                t_dht = dhtDevice.temperature
                                h_dht = dhtDevice.humidity
                            except:
                                dht_ok = False
                                dht_fail_counter += 1
                            if dht_fail_counter > 10:
                                printe("No pude sacar medicion del DHT")
                                errorwriter("DHT", "No se pudo tomar medicion de Humedad y Temperatura")
                                break
                        if dht_ok:
                            t_dht_list.append(t_dht)
                            h_dht_list.append(h_dht)
                if time.perf_counter() - state_timer > timer_log.value:
                    state_timer = time.perf_counter()

                    if current_date != datetime.now().strftime("%Y%m%d"):
                        day_info_list.append("")
                        try:
                            measurements_list = []
                            current_date = datetime.now().strftime("%Y%m%d")
                            breeding_day = (datetime.now() - datetime.strptime(zero_date, "%Y%m%d")).days
                            if breeding_day > 60 or breeding_day < 0:
                                printe("Fecha invalida")
                                raise Exception
                            day_score_config = score_config[breeding_day]
                            t_amb_max = float(day_score_config['T. Ambiente Max'])
                            t_amb_min = float(day_score_config['T. Ambiente Min'])
                            t_amb_delta= float(day_score_config['Amplitud Termica Amb.'])
                            t_amb_optimum = float(day_score_config['Promedio Optimo Amb.'])
                            h_max = float(day_score_config['Humedad Max'])
                            h_min = float(day_score_config['Humedad Min'])
                            h_delta= float(day_score_config['Amplitud Humedad'])
                            h_optimum = float(day_score_config['Promedio Optimo Humedad'])
                            thi_optimum = float(day_score_config['THI Optimo'])
                            if day_score_config['T. Cama Max'] != '':
                                t_bed_max = float(day_score_config['T. Cama Max'])
                                t_bed_min = float(day_score_config['T. Cama Min'])
                                t_bed_delta = float(day_score_config['Amplitud Termica Cama'])
                                t_bed_optimum = float(day_score_config['Promedio Optimo Cama'])
                                bed_check = True
                            else:
                                bed_check = False
                            logwriter(id=0, event=str(day_score_config))
                        except:
                            printe("Fallo la carga de configuracion scoring")

                    t_bme_mean = mean_check(t_bme_list)
                    h_bme_mean = mean_check(h_bme_list)
                    p_bme_mean = mean_check(p_bme_list)
                    t_bmp_mean = mean_check(t_bmp_list)
                    p_bmp_mean = mean_check(p_bmp_list)
                    t_dht_mean = mean_check(t_dht_list)
                    h_dht_mean = mean_check(h_dht_list)
                    t_mlx_amb_mean = mean_check(t_mlx_amb_list)
                    t_mlx_surface_mean = mean_check(t_mlx_surface_list)
                    t_bme_list = []
                    h_bme_list = []
                    p_bme_list = []
                    t_bmp_list = []
                    p_bmp_list = []
                    t_dht_list = []
                    h_dht_list = []
                    t_mlx_amb_list = []
                    t_mlx_surface_list = []

                    temp_out.value = t_bme_mean
                    humedad.value = h_bme_mean
                    amoniaco.value = t_mlx_surface_mean
                    

                    if t_bme_mean != 0 and h_bme_mean != 0:
                        thi = thi_calc(temperatura=t_bme_mean, humedad=h_bme_mean)
                        t_amb_list.append(t_bme_mean)
                        h_list.append(h_bme_mean)
                        thi_list.append(thi)
                        if abs(t_bme_mean-t_amb_optimum) < t_amb_delta/2:
                            score_temp_amb_rt = 10
                        else:
                            score_temp_amb_rt = max(10+t_amb_delta/2 - abs(t_bme_mean - t_amb_optimum), 0)
                        if abs(h_bme_mean-h_optimum) < h_delta/2:
                            score_hum_rt = 10
                        else:
                            score_hum_rt = max(10+h_delta/2 - abs(h_bme_mean - h_optimum), 0)
                        score_thi_rt = max(10 - abs(thi-thi_optimum),0)
                        t_amb_prom = mean_check(t_amb_list)
                        h_prom = mean_check(h_list)
                        thi_prom = mean_check(thi_list)
                        if abs(t_amb_prom-t_amb_optimum) < t_amb_delta/2:
                            score_temp_amb_prom = 10
                        else:
                            score_temp_amb_prom = max(10+t_amb_delta/2 - abs(t_amb_prom - t_amb_optimum), 0)
                        if abs(h_prom-h_optimum) < h_delta/2:
                            score_hum_prom = 10
                        else:
                            score_hum_prom = max(10+h_delta/2 - abs(h_prom - h_optimum), 0)
                        score_thi_prom = max(10 - abs(thi_prom-thi_optimum),0)
                    if bed_check:
                        if t_mlx_surface_mean != 0:
                            t_bed_list.append(t_mlx_surface_mean)
                            if abs(t_mlx_surface_mean-t_bed_optimum) < t_bed_delta/2:
                                score_temp_bed_rt = 10
                            else:
                                score_temp_bed_rt = max(10+t_bed_delta/2 - abs(t_mlx_surface_mean - t_bed_optimum), 0)
                            temp_bed_prom = mean_check(t_bed_list)
                            if abs(temp_bed_prom-t_bed_optimum) < t_bed_delta/2:
                                score_temp_bed_prom = 10
                            else:
                                score_temp_bed_prom = max(10+t_bed_delta/2 - abs(temp_bed_prom - t_bed_optimum), 0)
                        score_general_rt = score_temp_bed_rt * 0.5 + score_temp_amb_rt * 0.2 + score_hum_rt * 0.2 + score_thi_rt * 0.1
                        score_general_prom = score_temp_bed_prom * 0.5 + score_temp_amb_prom * 0.2 + score_hum_prom * 0.2 + score_thi_prom * 0.1
                    else:
                        score_general_rt = score_temp_amb_rt * 0.4 + score_hum_rt * 0.4 + score_thi_rt * 0.2
                        score_general_prom = score_temp_amb_prom * 0.4 + score_hum_prom * 0.4 + score_thi_prom * 0.2
                    if is_stuck_confirm:
                        if last_t_stuck != 0:
                            t_stuck_sec += time.perf_counter()-last_t_stuck
                            t_stuck = t_stuck_sec/3600
                        last_t_stuck = time.perf_counter()
                    else:
                        if last_t_stuck != 0:
                            t_stuck_sec += time.perf_counter()-last_t_stuck
                            t_stuck = t_stuck_sec/3600
                        last_t_stuck = 0
                    if is_rest.value:
                        if last_t_rest != 0:
                            t_rest_sec += time.perf_counter()-last_t_rest
                            t_rest = t_rest_sec/3600
                        last_t_rest = time.perf_counter()
                        if last_t_active != 0:
                            t_active_sec += time.perf_counter()-last_t_active
                            t_active = t_active_sec/3600
                        last_t_active = 0
                    else:
                        if last_t_rest != 0:
                            t_rest_sec += time.perf_counter()-last_t_rest
                            t_rest = t_rest_sec/3600
                        last_t_rest = 0
                        if last_t_active != 0:
                            t_active_sec += time.perf_counter()-last_t_active
                            t_active = t_active_sec/3600
                        last_t_active = time.perf_counter()
                    t_total = (time.perf_counter() - t_init)/3600
                    if len(day_info_list) == 0:
                        printe("Se envio todo lo actual, reseteo las variables")
                        day_info_list.append("")
                        measurements_list = []
                    current_date_server = datetime.strptime(current_date, "%Y%m%d").strftime("%Y-%m-%d")
                    measurement = {"temp_environment": t_bme_mean, "temp_surface": t_mlx_surface_mean, "humidity": h_bme_mean, "comfort": thi, "score_temp_environment": round(score_temp_amb_rt,2), "score_temp_surface": round(score_temp_bed_rt,2), "score_humidity": round(score_hum_rt,2), "score_comfort": round(score_thi_rt,2), "time":datetime.now().strftime("%H:%M:%S"), "robot_identifier": robot_id, "score_overall": round(score_general_rt,2)}
                    measurements_list.append(measurement)
                    day_info ={"day": {"breeding_day": breeding_day, "config": day_score_config, "total_time": t_total, "active_time": t_active, "rest_time": t_rest, "stuck_time": t_stuck, "date":current_date_server, "campaign_id": campaign_id, "measurements": measurements_list}}
                    
                    day_info_list[-1] = day_info
                    with open('send_queue/logs/{}.json'.format(current_date), 'w') as outfile:
                            json.dump(day_info, outfile)
                    with open('send_queue/logs/{}_backup.json'.format(current_date), 'w') as outfile:
                            json.dump(day_info, outfile)
                    if not is_rest.value:
                        printe("Estado")
                        logwriter("Estado", id=14, t_cpu=temp_cpu.value, t_clock=temp_clock.value, t_bme=t_bme_mean, t_bmp=t_bmp_mean,
                            t_dht=t_dht_mean, t_laser_surf= t_mlx_surface_mean, t_laser_amb=t_mlx_amb_mean, h_dht=h_dht_mean, h_bme=h_bme_mean, p_bme=p_bme_mean, p_bmp=p_bmp_mean, thi=thi, score_temp_amb_rt=round(score_temp_amb_rt,2), score_temp_bed_rt = round(score_temp_bed_rt,2), score_hum_rt = round(score_hum_rt,2), score_thi_rt = round(score_thi_rt,2), score_general_rt = round(score_general_rt,2), score_temp_amb_prom=round(score_temp_amb_prom,2), score_temp_bed_prom = round(score_temp_bed_prom,2), score_hum_prom = round(score_hum_prom,2), score_thi_prom = round(score_thi_prom,2), score_general_prom = round(score_general_prom,2), t_total = t_total, t_active = t_active, t_rest = t_rest, t_stuck = t_stuck)
                    else:
                        printe("Estado descansando")
                        logwriter("Estado, descansando", id=15, t_cpu=temp_cpu.value, t_clock=temp_clock.value, t_bme=t_bme_mean, t_bmp=t_bmp_mean,
                            t_dht=t_dht_mean, t_laser_surf= t_mlx_surface_mean, t_laser_amb=t_mlx_amb_mean, h_dht=h_dht_mean, h_bme=h_bme_mean, p_bme=p_bme_mean, p_bmp=p_bmp_mean, thi=thi, score_temp_amb_rt=round(score_temp_amb_rt,2), score_temp_bed_rt = round(score_temp_bed_rt,2), score_hum_rt = round(score_hum_rt,2), score_thi_rt = round(score_thi_rt,2), score_general_rt = round(score_general_rt,2), score_temp_amb_prom=round(score_temp_amb_prom,2), score_temp_bed_prom = round(score_temp_bed_prom,2), score_hum_prom = round(score_hum_prom,2), score_thi_prom = round(score_thi_prom,2), score_general_prom = round(score_general_prom,2), t_total = t_total, t_active = t_active, t_rest = t_rest, t_stuck = t_stuck)
        except Exception as ex:
            printe("Error in line:", sys.exc_info()[-1].tb_lineno)
            printe(ex)
            # errorwriter("Camara", "Fallo timeout")
            # printe(ex)
            pass

def auto(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, clearance, is_hot, timer_rest, timer_wake, backwards_counter, crash_timeout, last_touch_window_timeout, last_touch_counter, last_touch_osc_counter, flash_req, vel_array, time_turn, x_com, z_com, is_rest):
    motor_1_pwm = PWMOutputDevice("BOARD35")
    motor_2_pwm = PWMOutputDevice("BOARD33")
    motor_1_pwm.frequency = 20000
    motor_2_pwm.frequency = 20000
    
    was_auto = False
    motor_1_dir = DigitalOutputDevice("BOARD31")
    motor_2_dir = DigitalOutputDevice("BOARD29")
    button_left = Button("BOARD40")
    button_middle = Button("BOARD38")
    button_right = Button("BOARD36")
    motor_1_pwm.off()
    motor_2_pwm.off()
    printe("AUTO INIT")
    first_auto = True
    crash_confirmed = False
    last_touch = ""
    last_touch_timer = time.perf_counter()
    last_touch_count = 0
    last_touch_osc_count = 0
    last_touch_osc_timer = time.perf_counter()
    led_on = True
    second_back = False
    back_change = 0
    move_status = ''
    sinking = False
    second_back = False
    class Velocity:
        def __init__(self, forward, backward, left, right):
            self.forward = self.VelocityData(forward)
            self.left = -1
            self.right = 1
            self.backward = self.VelocityData(backward)
        class VelocityData:
            def __init__(self, velArray):
                self.stuck = velArray[0]
                self.normal = velArray[1]
    vel = Velocity(vel_array[0], vel_array[1], vel_array[2], vel_array[3])
    time_turn = time_turn.value

    

    def move(x = 0, z = 0, t = 0):
        global move_status
        if x < 0 and z == 0:
            move_status = 'B'
        elif x > 0 and z == 0:
            move_status = 'F'
        elif x > 0 and z > 0:
            move_status = 'R'
        elif x > 0 and z < 0:
            move_status = 'L'
        check_rate = 0.5
        if abs(x) > 1:
            x = x/abs(x)
        if abs(z) > 1:
            z = z/abs(z)
        if(z == 0):
            pwm1 = x 
            pwm2 = pwm1
        elif (x == 0):
            pwm1 = z 
            pwm2 = -pwm1
        else:
            if (x > 0):
                if (z > 0):
                    pwm1 = vel_array[3] 
                    pwm2 = vel_array[2]
                else:
                    pwm2 = vel_array[3]
                    pwm1 = vel_array[2]
            else:
                if (z > 0):
                    pwm1 = vel_array[2]
                    pwm2 = vel_array[3]
                else:
                    pwm2 = vel_array[3]
                    pwm1 = vel_array[2]

        if (pwm1 > 0):
            motor_1_dir.off()
        elif(pwm1 < 0):
            motor_1_dir.on()
        else:
            motor_1_pwm.off()
        if (pwm2 < 0):
            motor_2_dir.off()
        elif(pwm2 > 0):
            motor_2_dir.on()
        else:
            motor_2_pwm.off()
        if motor_1_pwm.value != abs(pwm1):
            # printe(pwm1,pwm2)
            motor_1_pwm.value = abs(pwm1)
        if motor_2_pwm.value != abs(pwm2):
            motor_2_pwm.value = abs(pwm2)
        if t > 0:
            number_check_rate = int(t / check_rate)
            rest = t - number_check_rate * check_rate
            counter_check_rate = 0
            if x > 0:
                while (counter_check_rate <= number_check_rate and auto_req.value == True and not taking_pics.value and not (button_left.is_pressed or button_right.is_pressed or button_middle.is_pressed)):
                    time.sleep(check_rate)
                    counter_check_rate += 1
            else:
                while (counter_check_rate <= number_check_rate and auto_req.value == True and not taking_pics.value):
                    time.sleep(check_rate)
                    counter_check_rate += 1
            if counter_check_rate == number_check_rate and auto_req.value == True:
                    time.sleep(rest)
            motor_1_pwm.off()
            motor_2_pwm.off()
    def crash(crash_side):
        if crash_side == "DER":
            pass
        elif crash_side == "IZQ":
            pass
        crash_confirmed = False
        crash_timer = time.perf_counter()
        # printe("Me apretaron de izquierda")
        if crash_timeout.value > 0:
            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                time.sleep(0.25)
                if crash_side == "IZQ" and button_left.is_pressed and not button_right.is_pressed:
                    crash_confirmed = True
                elif crash_side == "DER" and not button_left.is_pressed and button_right.is_pressed:
                    crash_confirmed = True
                else:
                    crash_confirmed = False
                    break
        else:
            crash_confirmed = True
        if crash_confirmed:
            timer = time.perf_counter()
            time_turn_crash = time_turn.value
            if crash_side != last_crash and crash_side != "":
                time_turn_crash = 0.5 * time_turn.value
                move_sequence(crash_side)
            last_crash = crash_side
        
        
    def move_sequence(type):
        second_back = False
        if type == "TURN_STUCK":

            move(vel.forward.stuck, vel.left, time_turn)

        if type == "IZQ":
            if second_back == False:
                back_change = backwards_counter.value
                second_back = True
            else:
                back_change = backwards_counter.value * 1.5
                second_back = False
            move(vel.backward.normal, 0, back_change)
            move(vel.forward.normal, vel.right, time_turn_crash)
        if type == "DER":
            if second_back == False:
                back_change = backwards_counter.value
                second_back = True
            else:
                back_change = backwards_counter.value * 1.5
                second_back = False
            move(vel.backward.normal, 0, back_change)
            move(vel.forward.normal, vel.left, time_turn_crash)

    def antiloop(mode):
        printe("Antiloop")
        printe(mode)
        if mode == "IZQ":
            move(vel.backward.normal, 0, backwards_counter.value)
            move_sequence('TOUCH_DER')
        elif mode == "OSC":
            move(vel.backward.normal, 0, backwards_counter.value)
            if last_touch == "IZQ":
                go_right = False
            elif last_touch == "DER":
                go_right = True
            else:
                go_right = random.choice([True, False])
            if go_right == True:
                move_sequence('TOUCH_IZQ')
            else:
                move_sequence('TOUCH_DER')
        elif mode == "DER":
            move(vel.backward.normal, 0, backwards_counter.value)
            move_sequence('TOUCH_IZQ')
        last_touch_count = 0
        last_touch_osc_count = 0   
    while True:
        if is_rest.value:
            if led_on:
                led_enable.off()
                led_on = False
            else:
                led_enable.on()
                led_on = True
        if auto_req.value == False:
            move(x_com.value, z_com.value)
        if (auto_req.value == False and was_auto == True):
            move(0, 0)
            was_auto = False
            printe("Stop auto")
            is_stopped.value = True
            first_auto = True
            logwriter("Termine de andar autonomo", id=4)
        elif(auto_req.value == True):
            if first_auto:
                last_time_rest = time.perf_counter()
                logwriter("Empece a andar autonomo", id=2)
                printe("first auto")
                first_auto = False
                is_stopped.value = False
            if is_rest.value and (time.perf_counter()-last_time_on > timer_rest.value):
                is_rest.value = False
                last_time_rest = time.perf_counter()
                is_stopped.value = False
                is_hot.value = False
                printe("Vuelvo a andar")
                logwriter("Termine descanso", id=11)
                led_enable.on()
                if flash_req.value == True:
                    flash_enable.on()
            was_auto = True
            timer = time.perf_counter()
            while auto_req.value == True and not is_rest.value:

                if ((time.perf_counter() - last_time_rest > timer_wake.value) or is_hot.value) and not is_rest.value:
                    is_rest.value = True
                    move(0, 0)
                    time.sleep(1)
                    is_stopped.value = True
                    last_time_on = time.perf_counter()
                    printe("Voy a descansar")
                    flash_enable.off()
                    logwriter("Empece descanso", id=10)
                    break
                if time.perf_counter() - last_touch_timer > last_touch_window_timeout.value:
                    last_touch_count = 0
                    last_touch = ''
                if time.perf_counter() - last_touch_osc_timer > last_touch_window_timeout.value:
                    last_touch_osc_count = 0
                if time.perf_counter() - timer < timer_boring.value:
                    backward_count = 0
                    stuck_seq = 0
                    
                    if cam_stuck_flag.value == True or imu_stuck_flag.value == True or clearance_stuck_flag.value == True:
                        move(0, 0)
                        start_clearance = clearance.value
                        if move_status in ['R','L','F']:
                            move(vel.backward.stuck, 0, 0)
                        else:
                            move(vel.forward.stuck, 0, 0)
                        if clearance_stuck_flag.value:
                            while clearance_stuck_flag.value and not sinking:
                                pass
                            pass
                        elif imu_stuck_flag.value:
                            pass
                        else:
                            pass
                        
                        
                            

                    while taking_pics.value == True:
                        move(0, 0)
                        time.sleep(1)
                        is_stopped.value = True
                        printe("Stop esperando foto")
                    is_stopped.value = False
                    if button_left.is_pressed and not button_right.is_pressed: # Choque izquierdo
}                       crash("IZQ")
                        if crash_confirmed:
                            # printe("Choque confirmado de izquierda")
                            crash("IZQ")
                            timer = time.perf_counter()
                            if last_touch == "IZQ":
                                last_touch_count +=1
                                last_touch_osc_count = 0
                                last_touch_timer = time.perf_counter()
                            elif last_touch == "DER":
                                last_touch_osc_count += 1
                                last_touch_count = 0
                                last_touch_osc_timer = time.perf_counter()
                            last_touch = "IZQ"
                            if last_touch_count >= last_touch_counter.value:
                                antiloop("IZQ")
                                last_touch = "DER"
                            elif last_touch_osc_count >= last_touch_osc_counter.value:
                                antiloop("OSC")
                                last_touch_osc_count = 0
                            else:
                            
                                move_sequence('TOUCH_IZQ')
                    elif button_right.is_pressed and not button_left.is_pressed: #Choque derecho
                        crash_confirmed = False
                        crash_timer = time.perf_counter()
                        # printe("Me apretaron de derecha")
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
                            if last_touch == "DER":
                                last_touch_count +=1
                                last_touch_osc_count = 0
                                last_touch_timer = time.perf_counter()
                                # printe("Derecha count: {}".format(last_touch_count))
                                last_touch_count = 0
                                last_touch_osc_timer = time.perf_counter()
                                # printe("Oscilation count: {}".format(last_touch_osc_count))
                            last_touch = "DER"
                            if last_touch_count >= last_touch_counter.value:
                                antiloop("DER")
                                last_touch = "IZQ"
                            elif last_touch_osc_count >= last_touch_osc_counter.value:
                                antiloop("OSC")
                                last_touch_osc_count = 0
                            else:
                                move_sequence('TOUCH_DER')
                    elif (button_left.is_pressed and button_right.is_pressed): #Choque frontal
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
                            # printe("Choque confirmado randomn")
                            timer = time.perf_counter()
                            # printe("Going backwards")
                            if last_touch == "IZQ":
                                go_right = True
                            elif last_touch == "DER":
                                go_right = False
                            else:
                                go_right = random.choice([True, False])
                            last_touch = "FRO"
                            if go_right == True:
                                move_sequence('TOUCH_IZQ')
                                # printe("Going right")
                            else:
                                move_sequence('TOUCH_DER')
                else:
                    timer = time.perf_counter()
                    go_right = random.choice([True, False])
                    if go_right == True:
                        move_sequence('TOUCH_IZQ')
                    else:
                        move_sequence('TOUCH_DER')
                    
        time.sleep(1)


def savior(flash_req, cam_req, imu_req, auto_req, camera_rate, timer_boring, pitch_flag, pitch_counter, window_stuck_pic, timer_temp, timer_log, timer_rest, timer_wake, backwards_counter, day_crash_timeout , crash_timeout_before , crash_timeout_after, last_touch_window_timeout, last_touch_counter, last_touch_osc_counter, pic_sensibility, stucks_to_confirm, stuck_window, vel_array , time_turn):

    printe("Inicio BT Server")
    printe("savior",sys.argv)
    subprocess.Popen(['hciconfig', 'hci0', 'piscan'], stdout=subprocess.PIPE)


    # returns JSON object as 
    # a dictionary


    server_sock=BluetoothSocket( RFCOMM )
    server_sock.bind(("",1))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]

    uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

    advertise_service( server_sock, "SampleServer",
                       service_id = uuid,
                       service_classes = [ uuid, SERIAL_PORT_CLASS ],
                       profiles = [ SERIAL_PORT_PROFILE ], 
    #                   protocols = [ OBEX_UUID ] 
                        )
                       
    printe("Waiting for connection on RFCOMM channel %d" % port)

    client_sock, client_info = server_sock.accept()
    printe("Accepted connection from ", client_info)
    first = True
    build_string = ''
    new_string_timer = time.perf_counter()
    while True:
        try:
            if not first:
                printe("Waiting for connection")
                client_sock, client_info = server_sock.accept()
                printe("Accepted connection from ", client_info)
            first = False    
            while True:
                data_input = client_sock.recv(1024)
                if len(data_input) == 0: break
                # printe(data_input)
                try:
                    if len(data_input) == 990:
                        data_input = data_input.decode('utf8').replace("'",'"')
                        build_string += data_input
                        new_string_timer = time.perf_counter()
                        printe('Esperando el resto del mensaje')
                    elif len(data_input) != 990 or time.perf_counter() - new_string_timer > 4:
                        data_input = data_input.decode('utf8').replace("'",'"')
                        build_string += data_input
                        data_input = json.loads(build_string)
                        build_string = ""
                        printe(data_input["request"])
                        if (data_input["request"] == "GET_ROBOT_CONFIG"):
                            try:
                                
                                with open('/var/www/html/default_behavior.json') as default_behavior_file:
                                    default_behavior = json.load(default_behavior_file)
                                if os.path.exists('/var/www/html/actual_behavior.json'):
                                    with open('/var/www/html/actual_behavior.json') as actual_behavior_file:
                                        actual_behavior = json.load(actual_behavior_file)
                                else:
                                    actual_behavior = None
                                with open ('/var/www/html/default_config_scoring.csv') as default_breeding_config_file:
                                    default_breeding_config = default_breeding_config_file.readlines()
                                if os.path.exists('/var/www/html/actual_config_scoring.csv'):
                                    with open ('/var/www/html/actual_config_scoring.csv') as actual_breeding_config_file:
                                        actual_breeding_config = actual_breeding_config_file.readlines()
                                else:
                                    actual_breeding_config = None
                                with open ('/var/www/html/campaign_status.json') as campaign_status_file:
                                    campaign_status = json.load(campaign_status_file)
                                campaign_status['zero_date'] = datetime.strptime(campaign_status['zero_date'], "%Y%m%d").strftime("%Y-%m-%d")
                            
                                send_info = client_sock.send(str({"request": "GET_ROBOT_CONFIG", "data": {"behavior_config":{"default": default_behavior, "actual":actual_behavior}, "breeding_config": {"default": default_breeding_config, "actual": actual_breeding_config}, "campaign_config": campaign_status}}))
                                printe("send",send_info)
                            except Exception as ex:
                                printe(ex)
                                pass
                        elif (data_input["request"] == "SET_BEHAVIOR_CONFIG"):
                            try:
                                printe(data_input["data"])
                                if data_input["data"]["default"] == True:
                                    printe("Seteando behavior default")
                                    if os.path.exists("/var/www/html/actual_behavior.json"):
                                        os.remove("/var/www/html/actual_behavior.json")
                                    behavior = open_json('/var/www/html/default_behavior.json')
                                else:
                                    printe("Seteando behavior personalizada")
                                    behavior = data_input["data"]['behavior_config']
                                    with open('/var/www/html/actual_behavior.json',  'w') as json_file:
                                        json.dump(behavior,json_file)
                                    with open('/var/www/html/actual_behavior_backup.json',  'w') as json_file:
                                        json.dump(behavior,json_file)
                                    
                                flash_req.value = behavior["flash_enable"]
                                cam_req.value = behavior["camera_enable"]
                                imu_req.value = behavior["imu_enable"]
                                auto_req.value = behavior["auto_enable"]
                                camera_rate.value = behavior["camera_rate"]
                                timer_boring.value = behavior["timer_boring"]
                                pitch_flag.value = behavior["pitch_flag"]
                                pitch_counter.value = behavior["pitch_counter"]
                                window_stuck_pic.value = behavior["window_stuck_pic"]
                                timer_temp.value = behavior["timer_temp"]
                                timer_log.value = behavior["timer_log"]
                                timer_rest.value = behavior["timer_rest"]
                                timer_wake.value = behavior["timer_wake"]
                                backwards_counter.value = behavior["backwards_counter"]
                                day_crash_timeout = behavior["day_crash_timeout"]
                                crash_timeout_before = behavior["crash_timeout_before"]
                                crash_timeout_after= behavior["crash_timeout_after"]
                                last_touch_window_timeout.value = behavior["last_touch_window_timeout"]
                                last_touch_counter.value = behavior["last_touch_counter"]
                                last_touch_osc_counter.value = behavior["last_touch_osc_counter"]
                                pic_sensibility.value = behavior["pic_sensibility"]
                                stucks_to_confirm.value = behavior['stucks_to_confirm']
                                stuck_window.value = behavior['stuck_window']
                                vel_array = [[behavior["vel_forward_stuck"], behavior["vel_forward_normal"]], [-behavior["vel_backward_stuck"], -behavior["vel_backward_normal"]], [behavior["vel_turn_inner"], behavior["vel_turn_outter"]], ]
                                time_turn.value = behavior["time_turn"]


                                printe("Termino seteo de behavior")
                                client_sock.send(str({"request": "SET_BEHAVIOR_CONFIG_STATUS", "data": 1}))
                            except Exception as ex:
                                printe(ex)
                                client_sock.send(str({"request": "SET_BEHAVIOR_CONFIG_STATUS", "data": 0}))
                        elif (data_input["request"] == "SET_NEW_CAMPAIGN"):
                            try:
                                printe(data_input["data"])
                                campaign= {"is_active": data_input["data"]["is_active"], "zero_date": data_input["data"]["zero_date"], "end_date": data_input["data"]["end_date"], "campaign_id": data_input["data"]["campaign_id"], "baby_origin": data_input["data"]["baby_origin"], "batch": data_input["data"]["batch"], "shed_number": data_input["data"]["shed_number"]}
                                with open('/var/www/html/campaign_status.json', 'w') as outfile:
                                    json.dump(campaign, outfile)
                                with open('/var/www/html/campaign_status_backup.json', 'w') as outfile:
                                    json.dump(campaign, outfile)
                                if data_input["data"]["breeding_config"] == "default":
                                    printe("Seteando breeding default")
                                    if os.path.exists("/var/www/html/actual_config_scoring.csv"):
                                        os.remove("/var/www/html/actual_config_scoring.csv")
                                elif data_input["data"]["breeding_config"] == "actual":
                                    pass
                                else:
                                    lines = data_input["data"]["breeding_config"].splitlines()
                                    rows = csv.reader(lines)
                                    printe(rows)
                                    headers = next(rows)
                                    printe(headers)
                                    record = dict(zip())
                                    score_config = []
                                    day_list=[]
                                    with open("/var/www/html/actual_config_scoring.csv", 'w') as scoringfile:
                                        wr = csv.writer(scoringfile)
                                        wr.writerow(headers)
            
                                    for i,row in enumerate(rows):
                                        printe(i,row)
                                        with open("/var/www/html/actual_config_scoring.csv", 'a', newline='') as scoringfile:
                                            wr = csv.writer(scoringfile)
                                            wr.writerow(row)
                                    with open ('/var/www/html/actual_config_scoring.csv') as f:
                                        copy = f.read()
                                        with open ('/var/www/html/actual_config_scoring_backup.csv', 'w') as file:
                                            file.write(copy)
                                client_sock.send(str({"request": "SET_NEW_CAMPAIGN_STATUS", "data": 1}))
                            except Exception as ex:
                                printe(ex)
                                client_sock.send(str({"request": "SET_NEW_CAMPAIGN_STATUS", "data": 0}))
                        elif (data_input["request"] == "SET_END_CAMPAIGN"):
                            try:
                                printe(data_input["data"])
                                campaign= {"is_active": data_input["data"]["is_active"], "zero_date": data_input["data"]["zero_date"], "end_date": data_input["data"]["end_date"], "campaign_id": data_input["data"]["campaign_id"], "baby_origin": data_input["data"]["baby_origin"], "batch": data_input["data"]["batch"], "shed_number": data_input["data"]["shed_number"]}
                                with open('/var/www/html/campaign_status.json', 'w') as outfile:
                                    json.dump(campaign, outfile)
                                with open('/var/www/html/campaign_status_backup.json', 'w') as outfile:
                                    json.dump(campaign, outfile)
                                client_sock.send(str({"request": "SET_END_CAMPAIGN_STATUS", "data": 1}))
                            except Exception as ex:
                                printe(ex)
                                client_sock.send(str({"request": "SET_END_CAMPAIGN_STATUS", "data": 0}))
                        elif (data_input["request"] == "SET_BREEDING_CONFIG"):
                            try:
                                printe(data_input)
                                if data_input["data"]["default"] == True:
                                    printe("Seteando breeding default")
                                    if os.path.exists("/var/www/html/actual_config_scoring.csv"):
                                        os.remove("/var/www/html/actual_config_scoring.csv")
                                else:

                                    lines = data_input["data"]["breeding_config"].splitlines()
                                    rows = csv.reader(lines)
                                    printe(rows)
                                    headers = next(rows)
                                    printe(headers)
                                    record = dict(zip())
                                    score_config = []
                                    day_list=[]
                                    with open("/var/www/html/actual_config_scoring.csv", 'w') as scoringfile:
                                        wr = csv.writer(scoringfile)
                                        wr.writerow(headers)
            
                                    for i,row in enumerate(rows):
                                        printe(i,row)
                                        with open("/var/www/html/actual_config_scoring.csv", 'a', newline='') as scoringfile:
                                            wr = csv.writer(scoringfile)
                                            wr.writerow(row)
                                    with open ('/var/www/html/actual_config_scoring.csv') as f:
                                        copy = f.read()
                                        with open ('/var/www/html/actual_config_scoring_backup.csv', 'w') as file:
                                            file.write(copy)

                                    # if int(record['Dia']) != i:
                                    #     printe("Archivo corrupto")
                                    #     raise Exception

                                client_sock.send(str({"request": "SET_BREEDING_CONFIG_STATUS", "data": 1}))
                            except Exception as ex:
                                printe(ex)
                                client_sock.send(str({"request": "SET_BREEDING_CONFIG_STATUS", "data": 0}))
                        
                            
                except Exception as ex:
                    printe("REQUEST_PARSING_ERROR")
                    printe("len:",len(data_input), "data_input:", data_input)
                    client_sock.send(str({"request": "REQUEST_PARSING_ERROR", "data": 1}))
                # printe(data_input)
               
                
        except Exception as ex:
            printe(ex)
            printe("disconnected")
            pass
        

    printe("disconnected")

    client_sock.close()
    server_sock.close()
    printe("all done")
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

def main():
    # queue = multiprocessing.Queue()
    vel_array = multiprocessing.Array('d', [])
    time_turn = multiprocessing.Value('d', 0)
    img_index_num = multiprocessing.Value('i', 0)
    cam_req = multiprocessing.Value('b', False)
    camera_rate = multiprocessing.Value('i', 0)
    auto_req = multiprocessing.Value('b', False)
    flash_req = multiprocessing.Value('b', False)
    timer_boring = multiprocessing.Value('i', 0)
    imu_req = multiprocessing.Value('b', False)
    pitch_flag = multiprocessing.Value('d', 0.1)
    pitch_counter = multiprocessing.Value('i', 0)
    cam_stuck_flag = multiprocessing.Value('b', False)
    imu_stuck_flag = multiprocessing.Value('b', False)
    clearance_stuck_flag = multiprocessing.Value('b', False)
    clearance = multiprocessing.Value('i', 0)
    taking_pics = multiprocessing.Value('b', False)
    is_stopped = multiprocessing.Value('b', True)
    is_hot = multiprocessing.Value('b', False)
    temp_cpu = multiprocessing.Value('d', 0)
    temp_clock = multiprocessing.Value('d', 0)
    temp_out = multiprocessing.Value('d', 0)
    humedad = multiprocessing.Value('d', 0)
    amoniaco = multiprocessing.Value('d', 0)
    window_stuck_pic = multiprocessing.Value('d', 0)
    timer_temp = multiprocessing.Value('i', 0)
    timer_log = multiprocessing.Value('i', 0)
    timer_rest = multiprocessing.Value('i', 0)
    timer_wake = multiprocessing.Value('i', 0)
    backwards_counter = multiprocessing.Value('d', 0)
    crash_timeout = multiprocessing.Value('d', 0)
    last_touch_window_timeout = multiprocessing.Value('i', 0)
    last_touch_counter = multiprocessing.Value('i', 0)
    last_touch_osc_counter = multiprocessing.Value('i', 0)
    pic_sensibility = multiprocessing.Value('d', 0)
    stucks_to_confirm = multiprocessing.Value('i', 0)
    stuck_window = multiprocessing.Value('d', 0)
    x_com = multiprocessing.Value('d', 0)
    z_com = multiprocessing.Value('d', 0)
    is_rest = multiprocessing.Value('b', False)
    manager = multiprocessing.Manager()
    lst = manager.list()
    lst.append(None)
    flash_req.value = behavior["flash_enable"]
    cam_req.value = behavior["camera_enable"]
    imu_req.value = behavior["imu_enable"]
    auto_req.value = behavior["auto_enable"]
    camera_rate.value = behavior["camera_rate"]
    timer_boring.value = behavior["timer_boring"]
    pitch_flag.value = behavior["pitch_flag"]
    pitch_counter.value = behavior["pitch_counter"]
    window_stuck_pic.value = behavior["window_stuck_pic"]
    timer_temp.value = behavior["timer_temp"]
    timer_log.value = behavior["timer_log"]
    timer_rest.value = behavior["timer_rest"]
    timer_wake.value = behavior["timer_wake"]
    backwards_counter.value = behavior["backwards_counter"]
    day_crash_timeout = behavior["day_crash_timeout"]
    crash_timeout_before = behavior["crash_timeout_before"]
    crash_timeout_after = behavior["crash_timeout_after"]
    last_touch_window_timeout.value = behavior["last_touch_window_timeout"]
    last_touch_counter.value = behavior["last_touch_counter"]
    last_touch_osc_counter.value = behavior["last_touch_osc_counter"]
    pic_sensibility.value = behavior["pic_sensibility"]
    stucks_to_confirm.value = behavior['stucks_to_confirm']
    stuck_window.value = behavior['stuck_window']
    vel_array = [[behavior["vel_forward_stuck"], behavior["vel_forward_normal"]], [-behavior["vel_backward_stuck"], -behavior["vel_backward_normal"]], behavior["vel_turn_inner"], behavior["vel_turn_outter"], ]
    time_turn.value = behavior["time_turn"]
    if breeding_day >= day_crash_timeout:
        crash_timeout.value = crash_timeout_after
        printe("SENSIBILIDAD BAJA")
        logwriter("Sensibilidad baja", id=23)
    else:
        printe("SENSIBILIDAD ALTA")
        crash_timeout.value = crash_timeout_before
        logwriter("Sensibilidad alta", id=23)


    json_state = {"flash": flash_req.value, "auto": auto_req.value,
                  "camera": cam_req.value, "imu_req": imu_req.value}
    with open('/var/www/html/state.json', 'w') as outfile:
        json.dump(json_state, outfile)
    # Set up our websocket handler
    command_handler = multiprocessing.Process(
        target=command, args=(cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag, flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, timer_rest, timer_wake, backwards_counter, timer_boring, crash_timeout, x_com, z_com,))
    # Set up our camera
    savior_handler = multiprocessing.Process(target=savior, args=(flash_req, cam_req, imu_req, auto_req, camera_rate, timer_boring, pitch_flag, pitch_counter, window_stuck_pic, timer_temp, timer_log, timer_rest, timer_wake, backwards_counter, day_crash_timeout , crash_timeout_before , crash_timeout_after, last_touch_window_timeout, last_touch_counter, last_touch_osc_counter, pic_sensibility, stucks_to_confirm, stuck_window, vel_array , time_turn,))
    auto_handler = multiprocessing.Process(target=auto, args=(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, clearance, is_hot, timer_rest, timer_wake, backwards_counter, crash_timeout, last_touch_window_timeout,last_touch_counter, last_touch_osc_counter, flash_req, vel_array, time_turn, x_com, z_com, is_rest,))
    pitch_handler = multiprocessing.Process(target=pitch, args=(lst, imu_req, pitch_flag, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, clearance, cam_req, camera_rate, img_index_num, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_counter, timer_temp, timer_log, pic_sensibility, stucks_to_confirm, stuck_window, is_rest, flash_req, current_date, score_config, zero_date, day_score_config, breeding_day, campaign_id,))
    # Add 'em to our list
    PROCESSES.append(savior_handler)
    if campaign_is_active:
        PROCESSES.append(command_handler)
        PROCESSES.append(auto_handler)
        PROCESSES.append(pitch_handler)
    for p in PROCESSES:
        p.start() 
    while True:
        time.sleep(0.1)
        # printe("che")


if __name__ == '__main__':
    prints_enable = True
    wait_to_run = False  
    if len(sys.argv) > 1:
        if sys.argv[1] == "-s":
            # silent mode, no hay prints, es el modo que se pone en produccion
            prints_enable = False
        elif sys.argv[1] == "-d":
            # debug mode, hay prints
            prints_enable = True
        elif sys.argv[1] == "-nw":
            # no wait mode, no espera 20 segundos para empezar
            wait_to_run = False
        elif sys.argv[1] == "-w":
            # no wait mode, no espera 20 segundos para empezar
            wait_to_run = True
        elif sys.argv[1] == "-snw":
            prints_enable = False
            wait_to_run = False
        elif sys.argv[1] == "-dnw":
            prints_enable = True
            wait_to_run = False        
        elif sys.argv[1] == "-sw":
            prints_enable = False
            wait_to_run = True
        elif sys.argv[1] == "-dw":
            prints_enable = True
            wait_to_run = True        
    printe("main",len(sys.argv))
    start_time = time.perf_counter()
    #region Detecto si hay pendrive conectado y si es asi le copio la data
    try:
        if os.path.exists("/dev/sda"):
            led_enable.on()
            os.system("sudo mount /dev/sda1 /media/usb")
            printe("lo monte")
            if not os.path.exists("/media/usb/backup"):
                os.system("mkdir /media/usb/backup ")
            os.system("sudo cp -r log /media/usb/backup")
            if not os.path.exists("/media/usb/backup/resources"):
                os.system("mkdir /media/usb/backup/resources")
            os.system(
                "sudo rsync -aP --ignore-existing resources/ /media/usb/backup/resources")
            os.system("sudo umount /media/usb")
            printe("Termine backup")
            while True:
                led_enable.on()
                time.sleep(0.2)
                led_enable.off()
                time.sleep(0.2)
        else:
            pass
    except:
        pass
    #endregion
    while wait_to_run and time.perf_counter()-start_time < 20:
        led_enable.on()
        time.sleep(0.5)
        led_enable.off()
        time.sleep(0.5)
    printe(bcolors.OKGREEN + "AVI-Sense 1.0 APELIE ROBOTICS - 2022" + bcolors.ENDC)
    flash_enable.off()
    #region Si no existe el log master lo creo
    if not os.path.exists("log/log.csv"):
        printe("No existe el logfile")
        header = ["#", "Fecha", "Hora", "Evento","Minutos", "T. CPU",
                  "T. Clock", "T. Ambiente", "Humedad", "NH3", "ID"]
        with open('log/log.csv', 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    if not os.path.exists("log/error.log"):
        with open('log/error.log', 'w') as errlog:
            errlog.write("START ERROR LOG")
    #endregion
    # Hacer funcion para poder levantar archivos de configuracion y backupearlos, si esta mal el archivo levantar el backup y sobreescribir el corrupto

    if os.path.exists("/var/www/html/actual_behavior.json"):
        behavior = open_json('/var/www/html/actual_behavior.json')
    else:
        behavior = open_json('/var/www/html/default_behavior.json')

    printe(behavior)
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
    try:
        printe("Enabling hotspot")
        subprocess.call("./enablehotspot_silent", timeout=40)
    except Exception as e:
        printe(e, "No se pudo iniciar hotspot")
        errorwriter(e,"No inicio el hotspot")

    
    if not os.path.exists("stuck_count.json"):
        json_stuck_line = {"IMU": 0, "Cam": 0, "CamConf": 0}
        with open('stuck_count.json', 'w') as outfile:
            json.dump(json_stuck_line, outfile)
    last_stuck = open_json('stuck_count.json')
    last_watch = open_json('last_on.json')

    start_log = "Me apague, minutos trabado camara / IMU " 
    minutos_start = str(last_stuck["Cam"]) + "/" + str(last_stuck["IMU"])
    logwriter(start_log, id=6, watch_dog=True, minutos= minutos_start, 
                last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    logwriter("Me apague, minutos trabado camara confirmados", id=22, watch_dog=True, minutos= str(last_stuck["CamConf"]), 
                last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    # Vamos a tomar la configuracion de camapaña
    campaign_status = open_json('/var/www/html/campaign_status.json')

    campaign_is_active = campaign_status["is_active"]
    zero_date = campaign_status["zero_date"]
    end_date = campaign_status["end_date"]
    campaign_id = campaign_status["campaign_id"]
    baby_origin = campaign_status["baby_origin"]
    batch = campaign_status["batch"]
    shed_number = campaign_status["shed_number"]
    #region Cargo breeding config
    try:
        day_list=[]
        if os.path.exists("/var/www/html/actual_config_scoring.csv"):
            try:
                with open ('/var/www/html/actual_config_scoring.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('/var/www/html/actual_config_scoring.csv') as f:
                    copy = f.read()
                    with open ('/var/www/html/actual_config_scoring_backup.csv', 'w') as file:
                        file.write(copy)
                
            except:
                with open ('/var/www/html/actual_config_scoring_backup.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('/var/www/html/actual_config_scoring_backup.csv') as f:
                    copy = f.read()
                    with open ('/var/www/html/actual_config_scoring.csv', 'w') as file:
                        file.write(copy)
        else:  
            try:                         
                with open ('/var/www/html/default_config_scoring.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('/var/www/html/default_config_scoring.csv') as f:
                    copy = f.read()
                    with open ('/var/www/html/default_config_scoring_backup.csv', 'w') as file:
                        file.write(copy)
            except:
                with open ('/var/www/html/default_config_scoring_backup.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))  
                with open ('/var/www/html/default_config_scoring_backup.csv') as f:
                    copy = f.read()
                    with open ('/var/www/html/default_config_scoring.csv', 'w') as file:
                        file.write(copy)   
        breeding_day = (datetime.now() - datetime.strptime(zero_date, "%Y%m%d")).days
        printe("breeding_day:", breeding_day)
        if breeding_day > 60 or breeding_day < 0:
            printe("Fecha invalida")
            raise Exception
        if breeding_day in day_list:
            printe("si esta en la lista")
            breeding_day_index = day_list.index(breeding_day)
        else:
            printe("no esta en la lista")
            day_list.append(breeding_day)
            day_list.sort()
            breeding_day_index = day_list.index(breeding_day) - 1
        day_score_config = score_config[breeding_day_index]

        logwriter(id=0, event=str(day_score_config))
    except Exception as ex:
        printe(ex)
        printe("Error in line:", sys.exc_info()[-1].tb_lineno)
        printe("Fallo la carga de configuracion scoring")
    #endregion
    
    current_date = datetime.now().strftime("%Y%m%d")
    json_stuck_line = {"IMU": 0, "Cam": 0, "CamConf": 0}
    with open('stuck_count.json', 'w') as outfile:
        json.dump(json_stuck_line, outfile)
    # logwriter("Me prendi con esta configuracion: " + str(config)+'/'+str(behavior), id=1)
    last_on()
    if behavior["flash_enable"] == True:
        flash_enable.on()
        logwriter("Prendi luces", id=12)
    # motors_enable.on()
    try:
        main()
    except KeyboardInterrupt:
        # motor_1_pwm.off()
        # motor_2_pwm.off()
        led_enable.off()
        for p in PROCESSES:
            p.terminate()

