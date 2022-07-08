#!/usr/bin/env python3

#region Importacion de librerias
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
import board
from mlx90614 import MLX90614
import adafruit_mlx90614
import psutil
import bme280
import requests
import subprocess
from zipfile import ZipFile
import VL53L0X
import sys
from bluetooth import *
import adafruit_extended_bus
from socket import gethostname
import serial
#endregion

def buzzer(state = "off"):
    if state == "rand":
        state = random.choice(["on", "off"])
    if state == "off":
        buzz.off()
    elif state == "on":
        buzz.on()



def printe(*what_to_print, no_repeat = False): #Funcion auxiliar de printeo
    global last_string
    if prints_enable.value:
        string = ''
        for items in what_to_print:
            string += str(items) + " "
        if (no_repeat and last_string != string) or not no_repeat:
            last_string = string  
            print(datetime.now().strftime("%H:%M:%S "), string)
def open_json(filename): #Funcion para abrir jsons y backupearlos
    place = filename.replace(filename.split('/')[-1],"")
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

def last_on(): #Funcion que es como un watchdog, va escribiendo cual es el ultimo minuto activo
    now_logdate = datetime.now()
    log_date = now_logdate.strftime("%Y-%m-%d")
    log_hour = now_logdate.strftime("%H:%M:%S")
    date_name = now_logdate.strftime("%Y%m%d")
    json_last = {"Fecha": log_date, "Hora": log_hour, "Name": date_name}
    with open('config/actual/last_on.json', 'w') as outfile:
        json.dump(json_last, outfile)
    with open('config/actual/last_on_backup.json', 'w') as outfile:
        json.dump(json_last, outfile)

def errorwriter(error, comentario = "", no_repeat = False): #Funcion que escribe logs de errores
    global last_error
    if (no_repeat and last_error != error) or not no_repeat:
        error_date = datetime.now().strftime("%Y%m%d")
        error_hour = datetime.now().strftime("%H:%M:%S")
        err = str(error)
        errlog = error_hour + " Error: "+ err + " Comentario: "+ comentario + '\n'
        with open("log/error/{}.log".format(error_date),'a', newline='') as logerror:
            logerror.write(errlog)
# Funcion que escribe los logs
def logwriter(event, id,  minutos =0, t_cpu=0, t_clock=0, t_bme=0, t_laser_surf = 0, t_laser_amb = 0, h_bme=0, p_bme=0, thi = 0, clearance = 0, score_temp_amb_rt = 0, score_temp_bed_rt = 0, score_hum_rt = 0, score_thi_rt = 0, score_general_rt = 0, score_temp_amb_prom = 0, score_temp_bed_prom = 0, score_hum_prom = 0, score_thi_prom = 0, score_general_prom = 0, t_total= 0, t_active = 0, t_rest = 0, t_stuck = 0, watch_dog=False, last_date=-1, last_hour=-1, last_name=-1, battery_percent = 0, battery_voltage = 0):
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
                  "T. Clock", "T. BME", 'T. Ambiente Laser', "T. Laser", "H. BME", "P. BME", "THI", "Clearance", "Score Temp. Amb. RT", "Score Temp. Cama RT", "Score Hum. RT", "Score THI RT", "Score General RT", "Score Temp. Amb. PROM", "Score Temp. Cama PROM", "Score Hum. PROM", "Score THI PROM", "Score General PROM", "Tiempo Total", "Tiempo Activo", "Tiempo Descansando", "Tiempo Trabado", "Bateria", "Voltaje", "ID"]
        with open(stringdatelog, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelog, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        psutil.cpu_percent(percpu = True)
        wr.writerow(["", logdate, loghour, event, minutos,str(os.getloadavg()[0]), str(psutil.virtual_memory().percent), t_cpu,
                    t_clock, t_bme, t_laser_amb, t_laser_surf, h_bme, p_bme, thi, clearance, score_temp_amb_rt, score_temp_bed_rt, score_hum_rt, score_thi_rt, score_general_rt, score_temp_amb_prom, score_temp_bed_prom, score_hum_prom, score_thi_prom, score_general_prom, t_total, t_active, t_rest, t_stuck,battery_percent, battery_voltage, id])

    if not os.path.exists(stringdatelogbackup):
        printe("No existe el logfile diario de backup")
        header = ["#", "Fecha", "Hora", "Evento", "Minutos", "CPU", "RAM" ,"T. CPU",
                  "T. Clock", "T. BME", 'T. Ambiente Laser', "T. Laser", "H. BME", "P. BME", "THI", "Clearance", "Score Temp. Amb. RT", "Score Temp. Cama RT", "Score Hum. RT", "Score THI RT", "Score General RT", "Score Temp. Amb. PROM", "Score Temp. Cama PROM", "Score Hum. PROM", "Score THI PROM", "Score General PROM", "Tiempo Total", "Tiempo Activo", "Tiempo Descansando", "Tiempo Trabado", "Bateria", "Voltaje", "ID"]
        with open(stringdatelogbackup, 'w') as logfile:
            wr = csv.writer(logfile)
            wr.writerow(header)
    with open(stringdatelogbackup, 'a', newline='') as logfile:
        wr = csv.writer(logfile)
        wr.writerow(["", logdate, loghour, event, minutos,str(os.getloadavg()[0]), str(psutil.virtual_memory().percent), t_cpu,
                    t_clock, t_bme, t_laser_amb, t_laser_surf, h_bme, p_bme, thi, clearance, score_temp_amb_rt, score_temp_bed_rt, score_hum_rt, score_thi_rt, score_general_rt, score_temp_amb_prom, score_temp_bed_prom, score_hum_prom, score_thi_prom, score_general_prom, t_total, t_active, t_rest, t_stuck,battery_percent, battery_voltage, id])
def init_wifi():
    wifi_found = False
    try:
        printe("Iniciando wifi")
        try:
            subprocess.check_output('sudo iw dev "wlan0" scan | grep associated', shell=True)
            printe(bcolors.OKGREEN + "Ya conectado a la red {}".format(ssid) + bcolors.ENDC)
            errorwriter(error= "Ya estaba conectado a wifi")
            return True
        except subprocess.CalledProcessError:
            errorwriter(error="No estaba conectado a wifi")
            pass
        try:
            ssid_grep = "'SSID: " + ssid + "'"
            subprocess.check_output('sudo iw dev "wlan0" scan | grep {}'.format(ssid_grep), shell=True)
            wifi_found = True
            errorwriter(error="Se encontro la red wifi")
        except subprocess.CalledProcessError:
            printe(bcolors.FAIL + "No se encontro la red {}".format(ssid) + bcolors.ENDC)
            errorwriter(error="No se encontro la red wifi")
        if wifi_found:
            printe("Se encontro la red", ssid)  
        subprocess.call("./enablewifi_silent", timeout=40)
        try:
            subprocess.check_output('sudo iw dev "wlan0" scan | grep associated', shell=True)
            printe(bcolors.OKGREEN + "Conectado a la red {}".format(ssid) + bcolors.ENDC)
            errorwriter(error="Se conecto a la red")
            return True
        except subprocess.CalledProcessError:
            printe(bcolors.FAIL + "No se pudo conectar a la red {}".format(ssid) + bcolors.ENDC)
            errorwriter("No se pudo conectar a la red")
            return False

    except Exception as e:
        printe(e, "No se pudo iniciar wifi")
        errorwriter(e,"No inicio el wifi")

def bt_connection(prints_enable):
    printe("Inicio BT Server")
    subprocess.Popen(['hciconfig', 'hci0', 'piscan'], stdout=subprocess.PIPE)

    server_sock = BluetoothSocket( RFCOMM )
    server_sock.bind(("",1))
    server_sock.listen(1)

    port = server_sock.getsockname()[1]
    hotspot_status = 0

    uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

    advertise_service( server_sock, "SampleServer", service_id = uuid, service_classes = [ uuid, SERIAL_PORT_CLASS ], profiles = [ SERIAL_PORT_PROFILE ],) #protocols = [ OBEX_UUID ])
                    
    printe("Esperando conexion en el puerto %d" % port)
    server_sock.settimeout(60)
    try:
        client_sock, client_info = server_sock.accept()
    except:
        printe("Conexion Bluetooth timeout")
        return
    printe("Conexion aceptada desde: ", client_info)
    build_string = ''
    new_string_timer = time.perf_counter()
    while True:
        try:
            data_input = client_sock.recv(1024)
            if len(data_input) == 0: 
                break
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
                            
                            with open('config/default/default_behavior.json') as default_behavior_file:
                                default_behavior = json.load(default_behavior_file)
                            if os.path.exists('config/actual/actual_behavior.json'):
                                with open('config/actual/actual_behavior.json') as actual_behavior_file:
                                    actual_behavior = json.load(actual_behavior_file)
                            else:
                                actual_behavior = None
                            with open ('config/default/default_config_scoring.csv') as default_breeding_config_file:
                                default_breeding_config = default_breeding_config_file.readlines()
                            if os.path.exists('config/actual/actual_config_scoring.csv'):
                                with open ('config/actual/actual_config_scoring.csv') as actual_breeding_config_file:
                                    actual_breeding_config = actual_breeding_config_file.readlines()
                            else:
                                actual_breeding_config = None
                            with open ('config/actual/campaign_status.json') as campaign_status_file:
                                campaign_status = json.load(campaign_status_file)
                            campaign_status['zero_date'] = datetime.strptime(campaign_status['zero_date'], "%Y%m%d").strftime("%Y-%m-%d")
                        
                            client_sock.send(str({"request": "GET_ROBOT_CONFIG", "data": {"behavior_config":{"default": default_behavior, "actual":actual_behavior}, "breeding_config": {"default": default_breeding_config, "actual": actual_breeding_config}, "campaign_config": campaign_status, "hotspot": hotspot_status, "debug": prints_enable.value}}))
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            
                    elif (data_input["request"] == "SET_BEHAVIOR_CONFIG"):
                        try:
                            printe(data_input["data"])
                            if data_input["data"]["default"] == True:
                                printe("Seteando behavior default")
                                if os.path.exists("config/actual/actual_behavior.json"):
                                    os.remove("config/actual/actual_behavior.json")
                                behavior = open_json('config/default/default_behavior.json')
                            else:
                                printe("Seteando behavior personalizada")
                                behavior = data_input["data"]['behavior_config']
                                with open('config/actual/actual_behavior.json',  'w') as json_file:
                                    json.dump(behavior,json_file)
                                with open('config/actual/actual_behavior_backup.json',  'w') as json_file:
                                    json.dump(behavior,json_file)
                                
                            printe("Termino seteo de behavior")
                            client_sock.send(str({"request": "SET_BEHAVIOR_CONFIG_STATUS", "data": 1}))
                            os.system("sudo pm2 restart shock -- -snw")
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            client_sock.send(str({"request": "SET_BEHAVIOR_CONFIG_STATUS", "data": 0}))
                    elif (data_input["request"] == "SET_NEW_CAMPAIGN"):
                        try:
                            printe(data_input["data"])
                            campaign= {"is_active": data_input["data"]["is_active"], "zero_date": data_input["data"]["zero_date"], "end_date": data_input["data"]["end_date"], "campaign_id": data_input["data"]["campaign_id"], "baby_origin": data_input["data"]["baby_origin"], "batch": data_input["data"]["batch"], "shed_number": data_input["data"]["shed_number"]}
                            with open('config/actual/campaign_status.json', 'w') as outfile:
                                json.dump(campaign, outfile)
                            with open('config/actual/campaign_status_backup.json', 'w') as outfile:
                                json.dump(campaign, outfile)
                            if data_input["data"]["breeding_config"] == "default":
                                printe("Seteando breeding default")
                                if os.path.exists("config/actual/actual_config_scoring.csv"):
                                    os.remove("config/actual/actual_config_scoring.csv")
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
                                with open("config/actual/actual_config_scoring.csv", 'w') as scoringfile:
                                    wr = csv.writer(scoringfile)
                                    wr.writerow(headers)
        
                                for i,row in enumerate(rows):
                                    printe(i,row)
                                    with open("config/actual/actual_config_scoring.csv", 'a', newline='') as scoringfile:
                                        wr = csv.writer(scoringfile)
                                        wr.writerow(row)
                                with open ('config/actual/actual_config_scoring.csv') as f:
                                    copy = f.read()
                                    with open ('config/actual/actual_config_scoring_backup.csv', 'w') as file:
                                        file.write(copy)
                            client_sock.send(str({"request": "SET_NEW_CAMPAIGN_STATUS", "data": 1}))
                            os.system("sudo pm2 restart shock -- -snw")
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            client_sock.send(str({"request": "SET_NEW_CAMPAIGN_STATUS", "data": 0}))
                    elif (data_input["request"] == "SET_END_CAMPAIGN"):
                        try:
                            printe(data_input["data"])
                            campaign= {"is_active": data_input["data"]["is_active"], "zero_date": data_input["data"]["zero_date"], "end_date": data_input["data"]["end_date"], "campaign_id": data_input["data"]["campaign_id"], "baby_origin": data_input["data"]["baby_origin"], "batch": data_input["data"]["batch"], "shed_number": data_input["data"]["shed_number"]}
                            with open('config/actual/campaign_status.json', 'w') as outfile:
                                json.dump(campaign, outfile)
                            with open('config/actual/campaign_status_backup.json', 'w') as outfile:
                                json.dump(campaign, outfile)
                            client_sock.send(str({"request": "SET_END_CAMPAIGN_STATUS", "data": 1}))
                            os.system("sudo pm2 restart shock -- -snw")
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            client_sock.send(str({"request": "SET_END_CAMPAIGN_STATUS", "data": 0}))
                    elif (data_input["request"] == "SET_BREEDING_CONFIG"):
                        try:
                            printe(data_input)
                            if data_input["data"]["default"] == True:
                                printe("Seteando breeding default")
                                if os.path.exists("config/actual/actual_config_scoring.csv"):
                                    os.remove("config/actual/actual_config_scoring.csv")
                            else:

                                lines = data_input["data"]["breeding_config"].splitlines()
                                rows = csv.reader(lines)
                                printe(rows)
                                headers = next(rows)
                                printe(headers)
                                record = dict(zip())
                                score_config = []
                                day_list=[]
                                with open("config/actual/actual_config_scoring.csv", 'w') as scoringfile:
                                    wr = csv.writer(scoringfile)
                                    wr.writerow(headers)
        
                                for i,row in enumerate(rows):
                                    printe(i,row)
                                    with open("config/actual/actual_config_scoring.csv", 'a', newline='') as scoringfile:
                                        wr = csv.writer(scoringfile)
                                        wr.writerow(row)
                                with open ('config/actual/actual_config_scoring.csv') as f:
                                    copy = f.read()
                                    with open ('config/actual/actual_config_scoring_backup.csv', 'w') as file:
                                        file.write(copy)
                            client_sock.send(str({"request": "SET_BREEDING_CONFIG_STATUS", "data": 1}))
                            os.system("sudo pm2 restart shock -- -snw")
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            client_sock.send(str({"request": "SET_BREEDING_CONFIG_STATUS", "data": 0}))
                    elif (data_input["request"] == "SET_ENABLED_HOTSPOT"):
                        set_enabled_hotspot_status = 1
                        printe(data_input["data"])
                        if data_input["data"] == 1 and not hotspot_status:
                            try:
                                printe("Enabling hotspot")
                                subprocess.call("./enablehotspot_silent", timeout=40)
                                hotspot_status = 1
                            except Exception as e:
                                set_enabled_hotspot_status = 0
                                printe(e, "No se pudo iniciar hotspot")
                                errorwriter(e,"No inicio el hotspot")
                                hotspot_status = 0
                        elif data_input["data"] == 0 and hotspot_status:
                            try:
                                printe("Enabling wifi")
                                subprocess.call("./enablewifi_silent", timeout=40)
                                hotspot_status = 0
                            except Exception as e:
                                set_enabled_hotspot_status = 0
                                hotspot_status = 1
                                printe(e, "No se pudo iniciar wifi")
                                errorwriter(e,"No inicio el wifi")
                        client_sock.send(str({"request": "SET_ENABLED_HOTSPOT_STATUS", "data": set_enabled_hotspot_status}))
                    elif (data_input["request"] == "SET_ENABLED_DEBUG"):
                        set_enabled_debug_status = 1
                        printe(data_input["data"])
                        if data_input["data"] == 1 and not prints_enable.value:
                            try:
                                prints_enable.value = bool(data_input["data"])
                                printe("Enable debug")
                                
                            except Exception as e:
                                set_enabled_hotspot_status = 0
                                printe(e, "No se pudo iniciar debug")
                                errorwriter(e,"No inicio el debug")
                        elif data_input["data"] == 0 and prints_enable.value:
                            try:
                                prints_enable.value = bool(data_input["data"])
                                printe("Enable silent")
                            except Exception as e:
                                set_enabled_debug_status = 0
                                printe(e, "No se pudo sacar debug")
                                errorwriter(e,"No se pudo sacar debug")
                        client_sock.send(str({"request": "SET_ENABLED_DEBUG_STATUS", "data": set_enabled_debug_status}))
            except Exception as ex:
                printe("REQUEST_PARSING_ERROR")
                printe("len:",len(data_input), "data_input:", data_input)
                client_sock.send(str({"request": "REQUEST_PARSING_ERROR", "data": 1}))
        except Exception as ex:
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
            printe("Desconexion")
            break
# Clase que permite logear con color
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
def command(man, cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag,  flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, rest_time, wake_time, time_backwards, timer_boring, crash_timeout, x_com, z_com):
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
        printe(bcolors.OKGREEN + "Conectado via WebSocket" + bcolors.ENDC)

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
                    STATE["value"] = "x:{}/z:{}".format(data["x"], data["z"])
                    await notify_state()
                elif data["action"] == "stop":
                    STATE["value"] = "STOP"
                    x_com.value = 0
                    z_com.value = 0
                    # move(x, z)
                    await notify_state()
                elif data["action"] == "img_req":
                    await websocket.send(man[0].tobytes())
                elif data["action"] == "config":
                    config = data["req"]
                    # if config == True:
                    #     with open('config/state.json') as json_file:
                    #         state = json.load(json_file)
                    #     with open('config/config.json', 'w') as outfile:
                    #         json.dump(state, outfile)
                    #     with open('config/backup_config.json', 'w') as outfile:
                    #         json.dump(state, outfile)
                    # auto(auto_req)
                    pass
                elif data["action"] == "auto":
                    auto_req.value = data["req"]
                    printe("Cambio en el estado de movimiento automatico:", auto_req.value)
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
                with open('config/actual/state.json', 'w') as outfile:
                    json.dump(json_state, outfile)
        finally:
            await unregister(websocket)

    start_server2 = websockets.serve(counter, "localhost", 9001)
    asyncio.get_event_loop().run_until_complete(start_server2)
    asyncio.get_event_loop().run_forever()
def pitch(man, cam_stuck_flag, clearance, cam_req, camera_rate, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, timer_temp, timer_log, pic_sensibility_in, pic_sensibility_out, stucks_to_confirm, stuck_window, stuck_watchdog_time, is_rest, flash_req, current_date, score_config, zero_date, day_score_config, breeding_day, campaign_id, is_stuck_confirm, camera_state, mlx_state, bme_state, vl53_state, imu_state):
    #region Iniciar Variables
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 4
    raw_capture = PiRGBArray(camera, size=(640, 480))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    pic_sensibility = pic_sensibility_in.value
    last_pic = 0
    was_taking = False
    first_img = True
    log_cam_stuck = True
    log_cam = False
    temp_timer = 0
    state_timer = time.perf_counter()
    is_hot.value = False
    start_cam_stuck = 0
    elapsed_cam_stuck = 0
    total_elapsed_cam_stuck = 0
    last_total_elapsed_cam_stuck = 0
    confirm_elapsed_cam_stuck = 0
    confirm_total_elapsed_cam_stuck = 0
    confirm_last_total_elapsed_cam_stuck = 0
    confirm_log_cam_stuck = True
    confirm_start_cam_stuck = 0
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
    t_mlx_amb = 0 
    t_mlx_surface = 0
    last_measure = 0
    measure_rate = 1 #Hz
    t_bme_list = []
    h_bme_list = []
    p_bme_list = []
    t_mlx_amb_list = []
    t_mlx_surface_list = []
    t_bme_mean = 0
    h_bme_mean = 0
    p_bme_mean = 0
    t_mlx_amb_mean = 0
    t_mlx_surface_mean = 0
    last_t_mlx_surface_mean = 0
    bad_t_mlx_surface_mean_counter = 0
    measurements_list = []
    robot_id = 0
    day_info = {"day": {"breeding_day": breeding_day, "config": day_score_config, "total_time": 0, "active_time": 0, "rest_time": 0, "stuck_time": 0, "date": current_date, "campaign_id": campaign_id}}
    day_info_list = []
    url = "http://192.168.150.102:4000/loadMeasurements"
    url_img = "http://192.168.150.102:4000/loadImages"
    url_states = "http://192.168.150.102:4000/loadStates"
    data_was_sended = False
    img_to_compress = []
    mlx90614_connected = False
    bme_connected = False
    wifi_ok = False
    last_wifi = time.perf_counter()
    send_data_was_called = False
    stuck_watchdog_cam = []
    cam_debug = True
    bme_state = {"status": False, "status_str": "No detectado"}
    camera_state = {"status": False, "status_str": "Iniciada, sin tomar imagenes"}
    mlx_state = {"status": False, "status_str": "No detectado"}
    stuck_state = False
    general_state = {"hour": datetime.now().strftime("%H:%M:%S"), "stuck":bool(stuck_state), "camera":camera_state, "bme":bme_state, "mlx":mlx_state, "imu":{"status":imu_state["status"], "status_str": imu_state["status_str"]}, "vl53":{"status":vl53_state["status"], "status_str": vl53_state["status_str"]} }
    state_developer = {"robot_id":robot_id,"campaign_id":campaign_id, "date": datetime.now().strftime("%Y-%m-%d"), "state": general_state}
    states_list = []
    state_setted = False
    first_measurements_counter = 0
    is_tempered = 0
    delta_tmlx_tbme_list = []
    battery_voltage = 0
    battery_percent = 0
    #endregion
    #region Levanto mediciones que han quedado sin enviar antes de apagarse
    try:
        with open('send_queue/logs/states_to_send.json') as send_file:
            states_list = json.load(send_file)
        printe("Quedo {} states para enviar".format(len(day_info_list)))
    except Exception as ex:
        printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
        try:
            with open('send_queue/logs/states_to_send_backup.json') as send_file:
                states_list = json.load(send_file)
        except Exception as ex:
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
    try:
        with open('send_queue/logs/logs_to_send.json') as send_file:
            day_info_list = json.load(send_file)
        printe("Quedo {} logs para enviar".format(len(day_info_list)))
    except Exception as ex:
        printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
        try:
            with open('send_queue/logs/logs_to_send_backup.json') as send_file:
                day_info_list = json.load(send_file)
        except Exception as ex:
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
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
    def mean_check(value_list = [], who_called =''): #Funcion que 
        try:
            if len(value_list) > 0:
                return np.mean(value_list)
            else:
                return 0
        except Exception as ex:
            printe(bcolors.FAIL + "Error mean check, quien llamo fue {}, la excepcion: {}".format(who_called, ex) + bcolors.ENDC)
            return 0

    def thi_calc(temperatura = 0, humedad = 0):
        thi = temperatura+0.348*((humedad/100)*6.105*math.exp((17.27*temperatura)/(237.7+temperatura)))
        return thi

    def compare_images(img1, img2):
        # normalize to compensate for exposure difference
        try:
            img1 = normalize(img1)
            img2 = normalize(img2)
            diff = img1 - img2  # elementwise for scipy arrays
            m_norm = np.sum(abs(diff))  # Manhattan norm
            return m_norm
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

    
    #endregion
    #region Inicio de sensores

    if bme_detected:
        bme_state["status"] = False
        bme_state["status_str"] = "Detectado pero no iniciado"
        try:
            bmebus = smbus.SMBus(4)
            calibration_params = bme280.load_calibration_params(bmebus,bme_address)
            bme = bme280.sample(bmebus, bme_address, calibration_params)
            printe(bcolors.OKGREEN + "BME iniciado" + bcolors.ENDC)
            bme_connected = True
            bme_state["status"] = False
            bme_state["status_str"] = "Iniciado, sin tomar mediciones"
        except Exception as ex:
            errorwriter(ex, "Error al iniciar BME")
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "Error al iniciar el BME" + bcolors.ENDC)
            bme_connected = False
            bme_state["status"] = False
            bme_state["status_str"] = "Detectado pero no se pudo iniciar"

    if mlx90614_detected:
        mlx_state["status"] = False
        mlx_state["status_str"] = "Detectado pero no iniciado"
        try:
            # mlxbus = smbus2.SMBus(4)
            # mlx = MLX90614(mlxbus, address=mlx90614_address)
            mlx_i2c = adafruit_extended_bus.ExtendedI2C(4)
            mlx = adafruit_mlx90614.MLX90614(mlx_i2c)
            printe(bcolors.OKGREEN + "MLX90614 iniciado" + bcolors.ENDC)
            mlx90614_connected = True
            mlx_state["status"] = False
            mlx_state["status_str"] = "Iniciado, sin tomar mediciones"
        except Exception as ex:
            errorwriter(ex, "Error al iniciar MLX90614")
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "Error al iniciar el MLX90614" + bcolors.ENDC)
            mlx90614_connected = False
            mlx_state["status"] = False
            mlx_state["status_str"] = "Detectado pero no se pudo iniciar"
    #endregion
    if not is_milka:
        battery_serial = serial.Serial('/dev/ttyS0', 9600, timeout= 5)
    while True:
        try:
            for frame in camera.capture_continuous(raw_capture, format="bgr", use_video_port=True):
                #region Tomo mediciones del ambiente
                camera_state["status"] = True
                camera_state["status_str"] = "OK"
                if time.perf_counter()-last_measure > measure_rate:
                    last_measure = time.perf_counter() 
                    if is_stopped.value:
                        buzzer("off")
                    else:
                        buzzer("rand")
                    if mlx90614_connected:
                        try:
                            # t_mlx_surface = round(mlx.get_obj_temp(),2)
                            # t_mlx_amb = round(mlx.get_amb_temp(),2)
                            t_mlx_surface = round(mlx.object_temperature,2)
                            t_mlx_amb = round(mlx.ambient_temperature,2)
                            if t_mlx_surface < 80:
                                t_mlx_surface_list.append(t_mlx_surface)
                            if t_mlx_amb < 80:
                                t_mlx_amb_list.append(t_mlx_amb)
                        except Exception as ex:
                            mlx_state["status"] = False
                            mlx_state["status_str"] = "Iniciado pero no puede tomar mediciones"
                            errorwriter(ex, "No se pudo tomar mediciones MLX90614")
                            printe("Algo salio mal con el medidor MLX90614")
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                    if bme_connected:
                        try:
                            bme = bme280.sample(bmebus, bme_address, calibration_params)
                            t_bme = round(bme.temperature,2)
                            p_bme = round(bme.pressure,2)
                            h_bme = round(bme.humidity,2)
                            t_bme_list.append(t_bme)
                            p_bme_list.append(p_bme)
                            h_bme_list.append(h_bme)
                        except Exception as ex:
                            bme_state["status"] = False
                            bme_state["status_str"] = "Iniciado pero no puede tomar mediciones"
                            errorwriter(ex, "No se pudo tomar mediciones bme")
                            printe("Algo salio mal con el medidor bme")
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                f = frame.array
                cv2.waitKey(1)
                f = cv2.resize(f, (640, 480))
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 65]
                man[0] = cv2.imencode('.jpg', f, encode_param)[1]
                if cam_debug:
                    stuck_watchdog_cam.append((time.perf_counter(),f))
                    if stuck_watchdog_cam[-1][0] - stuck_watchdog_cam[0][0] > stuck_watchdog_time:
                        stuck_watchdog_cam.pop(0)
                if first_img:
                    image_to_compare0 = f
                    first_img = False
                    compare_timer = time.perf_counter()
                # Comparo cuantas veces se detecto trabado en la ultima ventana
                if time.perf_counter() - reference_stuck > stuck_window.value:
                    if stuck_count >= stucks_to_confirm.value:
                        if confirm_log_cam_stuck:
                            logwriter("Me trabe, camara CONFIRMADO", id=20)
                            printe(bcolors.FAIL + "Me trabe confirmado" + bcolors.ENDC)
                            is_stuck_confirm.value = True
                            confirm_log_cam_stuck = False
                            confirm_start_cam_stuck = time.perf_counter() 
                            confirm_last_total_elapsed_cam_stuck = confirm_total_elapsed_cam_stuck
                            try:
                                if cam_debug:
                                    if not os.path.exists("cam_debug/{}".format(datetime.now().strftime("%Y%m%d"))):
                                        os.mkdir("cam_debug/{}".format(datetime.now().strftime("%Y%m%d")))
                                    out = cv2.VideoWriter('cam_debug/{}/{}.avi'.format(datetime.now().strftime("%Y%m%d"),datetime.now().strftime("%H:%M:%S")),fourcc,4.0, (640,480))
                                    for fr in stuck_watchdog_cam:
                                        out.write(fr[1])
                                    out.release()
                                    stuck_watchdog_cam = []
                                    printe("Guarde video de la traba")
                            except Exception as ex:
                                printe(bcolors.FAIL + "Falle al guardar el video, excepcion: {}, en linea: {}".format(ex, sys.exc_info()[-1].tb_lineno) + bcolors.ENDC)
                        confirm_elapsed_cam_stuck = (time.perf_counter() - confirm_start_cam_stuck)/60.0
                        confirm_total_elapsed_cam_stuck = confirm_last_total_elapsed_cam_stuck + confirm_elapsed_cam_stuck
                        json_stuck_line_camconf = {"CamConf": confirm_total_elapsed_cam_stuck}
                        with open('config/actual/stuck_count_camconf.json', 'w') as outfile:
                            json.dump(json_stuck_line_camconf, outfile)
                        with open('config/actual/stuck_count_camconf_backup.json', 'w') as outfile:
                            json.dump(json_stuck_line_camconf, outfile)
                    elif not confirm_log_cam_stuck and stuck_count < stucks_to_confirm.value:
                        confirm_elapsed_cam_stuck = (time.perf_counter() - confirm_start_cam_stuck)/60.0
                        confirm_total_elapsed_cam_stuck = confirm_last_total_elapsed_cam_stuck + confirm_elapsed_cam_stuck
                        json_stuck_line_camconf = {"CamConf": confirm_total_elapsed_cam_stuck}
                        with open('config/actual/stuck_count_camconf.json', 'w') as outfile:
                            json.dump(json_stuck_line_camconf, outfile)
                        with open('config/actual/stuck_count_backup_camconf.json', 'w') as outfile:
                            json.dump(json_stuck_line_camconf, outfile)
                        confirm_log_cam_stuck = True
                        is_stuck_confirm.value = False
                        logwriter("Me destrabe, camara CONFIRMADO, minutos:", minutos=round(confirm_elapsed_cam_stuck,2), id=21)
                        printe(bcolors.OKGREEN + "Me destrabe confirmado" + bcolors.ENDC)
                    reference_stuck = time.perf_counter()
                    stuck_count = 0
                # Comparo imagenes
                if time.perf_counter() - compare_timer > window_stuck_pic.value:
                    # Si no esta frenado vamos a comparar las imagenes para ver si esta trabado
                    #region Comparacion de imagenes
                    if not is_stopped.value:
                        stuck_state = is_stuck_confirm.value
                        img0 = to_grayscale(image_to_compare0.astype(float))
                        img1 = to_grayscale(f.astype(float))
                        n_m = compare_images(img0, img1)
                        if not math.isnan(n_m):
                            if ((n_m/img0.size) < pic_sensibility):
                            # if False:
                                stuck_count += 1
                                if log_cam_stuck:
                                    pic_sensibility = pic_sensibility_out.value
                                    printe("Me trabe, camara. Valor:", n_m/img0.size)
                                    logwriter("Me trabe, camara", id=16)
                                    log_cam_stuck = False
                                    start_cam_stuck = time.perf_counter()
                                    last_total_elapsed_cam_stuck = total_elapsed_cam_stuck
                                cam_stuck_flag.value = True
                                elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                                total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                json_stuck_line_cam = {"Cam": total_elapsed_cam_stuck,}
                                with open('config/actual/stuck_count_cam.json', 'w') as outfile:
                                    json.dump(json_stuck_line_cam, outfile)
                                with open('config/actual/stuck_count_cam_backup.json', 'w') as outfile:
                                    json.dump(json_stuck_line_cam, outfile)
                            else:
                                if not log_cam_stuck:
                                    pic_sensibility = pic_sensibility_in.value
                                    elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                                    total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                                    json_stuck_line_cam = {"Cam": total_elapsed_cam_stuck}
                                    with open('config/actual/stuck_count_cam.json', 'w') as outfile:
                                        json.dump(json_stuck_line_cam, outfile)
                                    with open('config/actual/stuck_count_backup_cam.json', 'w') as outfile:
                                        json.dump(json_stuck_line_cam, outfile)
                                    printe("Me destrabe, camara. Valor:", n_m/img0.size)
                                    logwriter("Me destrabe, camara, minutos:", minutos=round(elapsed_cam_stuck,2), id=17)
                                    log_cam_stuck = True
                                cam_stuck_flag.value = False
                        else:
                            camera_state["status"] = False
                            camera_state["status_str"] = "Imagen Negra"
                        image_to_compare0 = f
                        compare_timer = time.perf_counter()
                    else:
                        if not log_cam_stuck:
                            elapsed_cam_stuck = (time.perf_counter() - start_cam_stuck)/60.0
                            total_elapsed_cam_stuck = last_total_elapsed_cam_stuck + elapsed_cam_stuck
                            json_stuck_line_cam = {"Cam": total_elapsed_cam_stuck}
                            with open('config/actual/stuck_count_cam.json', 'w') as outfile:
                                json.dump(json_stuck_line_cam, outfile)
                            with open('config/actual/stuck_count_cam_backup.json', 'w') as outfile:
                                json.dump(json_stuck_line_cam, outfile)
                            printe("Me destrabe, camara. Valor:",n_m/img0.size)
                            logwriter("Me destrabe, camara, minutos:", minutos=round(elapsed_cam_stuck,2), id=17)
                            log_cam_stuck = True
                        cam_stuck_flag.value = False
                    #endregion
                #region Guardar fotos
                if (cam_req.value == True and was_taking == False):
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
                        taking_pics.value = True      
                        if is_stopped.value:
                            last_pic = time.perf_counter()
                            printe("Voy a sacar foto detenida")
                            now = datetime.now()
                            d1 = now.strftime("%Y%m%d_%H%M%S")
                            if is_rest.value:
                                img_type = "R"
                            elif is_stuck_confirm.value:
                                img_type = "S"
                            else:
                                img_type = "P"
                            img_name = "resources/{}/{}_{}.png".format(img_folder,d1,img_type)
                            img_to_filter.append(img_name)
                            img_to_compress.append(img_name)
                            camera.capture(img_name)
                            with open('send_queue/imgs/list/img_to_compress.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                            with open('send_queue/imgs/list/img_to_compress_backup.json', 'w') as outfile:
                                json.dump( img_to_compress, outfile)
                            if is_rest.value or not flash_req.value:
                                flash_enable.off()
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
                    if not state_setted:
                        state_setted = True
                        general_state = {"hour": datetime.now().strftime("%H:%M:%S"), "stuck":bool(stuck_state), "camera":camera_state, "bme":bme_state, "mlx":mlx_state, "imu":{"status":imu_state["status"], "status_str": imu_state["status_str"]}, "vl53":{"status":vl53_state["status"], "status_str": vl53_state["status_str"]} }
                        state_developer = {"robot_id":robot_id,"campaign_id":campaign_id, "date": datetime.now().strftime("%Y-%m-%d"), "state": general_state}
                        printe("Ultimo estado:", state_developer)
                        states_list.append(state_developer)
                        with open('send_queue/logs/states_to_send.json', 'w') as outfile:
                            json.dump(states_list, outfile)
                        with open('send_queue/logs/states_to_send_backup.json', 'w') as outfile:
                            json.dump(states_list, outfile)
                    
                    send_data_was_called = True
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
                        if len(img_to_compress) == 0:
                            printe("No hay mas imagenes para comprimir")
                    if wifi_ok or time.perf_counter()-last_wifi > 60:
                        wifi_ok = init_wifi()
                        last_wifi = time.perf_counter()
                        if wifi_ok:
                            if len(states_list) > 0:
                                state_to_send = states_list[0]
                                head = {u'content-type': u'application/json'}
                                printe("Envio estado", state_to_send)
                                try:
                                    r = requests.post(url=url_states, data=json.dumps(state_to_send), headers=head, timeout=30)
                                    if r.status_code == 200:
                                        states_list.pop(0)
                                        printe(bcolors.OKGREEN + "Se envio correctamente el estado" + bcolors.ENDC)
                                        printe("Quedan para enviar:",len(states_list), "estados")
                                        with open('send_queue/logs/states_to_send.json', 'w') as outfile:
                                                json.dump(states_list, outfile)
                                        with open('send_queue/logs/states_to_send_backup.json', 'w') as outfile:
                                                json.dump(states_list, outfile)
                                    else:
                                        printe(bcolors.FAIL + "Fallo envio de estado, la respuesta fue:{}".format(r)+bcolors.ENDC)
                                except Exception as ex:
                                    printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            else:
                                printe("No hay mas estados para enviar")
                            if len(day_info_list)>0:
                                day_info_to_send = day_info_list[0]
                                head = {u'content-type': u'application/json'}
                                printe("Envio log")
                                try:
                                    r = requests.post(url=url, data=json.dumps(day_info_to_send), headers=head, timeout=30)
                                    if r.status_code == 200:
                                        day_info_list.pop(0)
                                        printe(bcolors.OKGREEN + "Se envio correctamente el log" + bcolors.ENDC)
                                        printe("Quedan para enviar:",len(day_info_list), "logs")
                                        with open('send_queue/logs/logs_to_send.json', 'w') as outfile:
                                                json.dump(day_info_list, outfile)
                                        with open('send_queue/logs/logs_to_send_backup.json', 'w') as outfile:
                                                json.dump(day_info_list, outfile)
                                    else:
                                        printe(bcolors.FAIL + "Fallo envio de log, la respuesta fue:{}".format(r)+bcolors.ENDC)
                                except Exception as ex:
                                    printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            if len(day_info_list) == 0:
                                printe("No hay mas logs que enviar")
                            
                            if os.path.exists("send_queue/imgs/zipfiles"):
                                if len(os.listdir("send_queue/imgs/zipfiles")) > 0:
                                    for zipfile in os.listdir("send_queue/imgs/zipfiles"):
                                        delete_zip = False
                                        date_zip_file = datetime.strptime(zipfile.split('_')[0], "%Y%m%d").strftime("%Y-%m-%d")
                                        with open("send_queue/imgs/zipfiles/"+ zipfile, 'rb') as file:
                                            fileobj = [('zip', (zipfile, file, 'zip'))]
                                            head = {u'content-type': u'multipart/form-data'}
                                            printe("Voy a enviar imagenes, zipfile:", zipfile)
                                            try:
                                                r = requests.post(url=url_img, data={"robot_id": robot_id, "campaign_id": campaign_id, "date":date_zip_file}, files = fileobj, timeout=30)
                                                if r.status_code == 200:
                                                    delete_zip = True
                                                    printe(bcolors.OKGREEN + "Se envio correctamente el zip de imagenes" + bcolors.ENDC)
                                                else:
                                                    printe(bcolors.FAIL + "Fallo envio de zip de imagenes, la respuesta fue:{}".format(r)+bcolors.ENDC)
                                            except Exception as ex:
                                                printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)

                                        if delete_zip:
                                            printe("Borre zip: ", zipfile)
                                            os.remove("send_queue/imgs/zipfiles/"+ zipfile)
                                else:
                                    printe('No hay mas imagenes que mandar')
                            if len(os.listdir("send_queue/imgs/zipfiles")) == 0 and len(img_to_compress) == 0 and len(day_info_list) == 0 and len(states_list) == 0:
                                data_was_sended = True
                                printe("No queda mas para hacer en reposo")
                        else:
                            printe(bcolors.WARNING + "No fue posible conectarse a la red, se reintentara en 1 minuto" + bcolors.ENDC)
                elif not is_rest.value and send_data_was_called and not data_was_sended:
                    printe(bcolors.WARNING + "No se pudo enviar la data al servidor en este descanso" + bcolors.ENDC)
                    logwriter("No se pudo enviar la data al servidor en este descanso", id= 25)
                    send_data_was_called = False
                    state_setted = False
                elif not is_rest.value and data_was_sended:
                    printe(bcolors.OKGREEN + "La data fue enviada correctamente en este descanso" + bcolors.ENDC)
                    data_was_sended = False
                    send_data_was_called = False
                    state_setted = False
                
                #endregion
                
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
                                    if chip.adapter_name == "Virtual device":
                                        temp_cpu.value = round(feature.get_value(), 1)
                    except:
                        pass
                    finally:
                        if (temp_cpu.value > 80 or temp_clock.value > 80) and not is_hot.value:
                            is_hot.value = True
                            logwriter("Alta temperatura", id=9, t_cpu=temp_cpu.value, t_clock=temp_clock.value)
                        sensors.cleanup()
                if time.perf_counter() - state_timer > timer_log.value:
                    #region Bateria
                    if not is_milka:
                        try:
                            battery_serial.flushInput()
                            serial_string = battery_serial.readline().decode("utf-8")
                            if "ERROR" in serial_string:
                                printe(bcolors.FAIL + serial_string + bcolors.ENDC)
                                errorwriter(error="Error detectado en el ESP32", comentario=serial_string)
                            elif serial_string == '':
                                printe(bcolors.FAIL + "Timeout bateria" + bcolors.ENDC)
                                errorwriter(error="Timeout bateria")
                            else:
                                serial_string = serial_string.replace("'", '"')
                                serial_json = json.loads(serial_string)["STATE"]
                                battery_percent = round((float(serial_json["cur_cap"])/float(serial_json["max_cap"]))*100,2)
                                battery_voltage = serial_json['load_vol']
                                printe("Estado de bateria {}%, voltaje {} V".format(battery_percent, battery_voltage))
                        except Exception as ex:
                            printe(bcolors.FAIL + "Fallo la obtencion de estado de bateria" + bcolors.ENDC)
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
                            errorwriter(error= ex, comentario = "Fallo la obtencion de estado de bateria")
                    #endregion
                    state_timer = time.perf_counter()
                    if current_date != datetime.now().strftime("%Y%m%d"):
                        printe("Cambio de dia, voy a reiniciar")
                        os.system("sudo pm2 restart shock -- -snw")
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

                    t_bme_mean = mean_check(t_bme_list, who_called = "BME")
                    h_bme_mean = mean_check(h_bme_list, who_called = "BME")
                    p_bme_mean = mean_check(p_bme_list, who_called = "BME")
                    t_mlx_amb_mean = mean_check(t_mlx_amb_list, who_called = "MLX90614")
                    t_mlx_surface_mean = mean_check(t_mlx_surface_list, who_called = "MLX90614")
                    if t_bme_mean == 0 or h_bme_mean == 0 or p_bme_mean == 0:
                        bme_state["status"] = False
                        bme_state["status_str"] = "Medicion nula"
                    else:
                        bme_state["status"] = True
                        bme_state["status_str"] = "OK"
                    if t_mlx_amb_mean == 0 or t_mlx_surface_mean == 0:
                        mlx_state["status"] = False
                        mlx_state["status_str"] = "Medicion nula"
                    else:
                        mlx_state["status"] = True
                        mlx_state["status_str"] = "OK"
                    t_bme_list = []
                    h_bme_list = []
                    p_bme_list = []
                    t_mlx_amb_list = []
                    t_mlx_surface_list = []

                    temp_out.value = t_bme_mean
                    humedad.value = h_bme_mean
                    amoniaco.value = t_mlx_surface_mean
                    if not is_tempered:
                        first_measurements_counter += 1
                        delta_tmlx_tbme = t_mlx_surface - t_bme_mean
                        delta_tmlx_tbme_list.append(delta_tmlx_tbme)
                        if first_measurements_counter == 5:
                            first_meausure_prom = sum(delta_tmlx_tbme_list)/len(delta_tmlx_tbme_list)
                            if first_meausure_prom < 0:
                                is_tempered = True
                                printe(bcolors.OKGREEN + 'El robot ya esta templado en los primeros 5 minutos' + bcolors.ENDC)
                        elif first_measurements_counter > 5:
                            if abs(delta_tmlx_tbme) > 1:
                                is_tempered = True
                                printe(bcolors.OKGREEN + 'El robot ya esta templado, pasaron {} minutos'.format(first_measurements_counter) + bcolors.ENDC)
                            else:
                                printe(bcolors.WARNING + "EL robot no esta templado, pasaron {} minutos".format(first_measurements_counter)+ bcolors.ENDC)
                        if first_measurements_counter > 45:
                            is_tempered = True
                            printe(bcolors.OKGREEN + 'Se supone el robot templado luego de 45 minutos' + bcolors.ENDC)
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
                        t_amb_prom = mean_check(t_amb_list, who_called = "Temperatura promedio")
                        h_prom = mean_check(h_list, who_called = "Humedad promedio")
                        thi_prom = mean_check(thi_list, who_called = "THI promedio")
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
                            temp_bed_prom = mean_check(t_bed_list, who_called = "Temperatura de cama promedio")
                            if abs(temp_bed_prom-t_bed_optimum) < t_bed_delta/2:
                                score_temp_bed_prom = 10
                            else:
                                score_temp_bed_prom = max(10+t_bed_delta/2 - abs(temp_bed_prom - t_bed_optimum), 0)
                        score_general_rt = score_temp_bed_rt * 0.5 + score_temp_amb_rt * 0.2 + score_hum_rt * 0.2 + score_thi_rt * 0.1
                        score_general_prom = score_temp_bed_prom * 0.5 + score_temp_amb_prom * 0.2 + score_hum_prom * 0.2 + score_thi_prom * 0.1
                    else:
                        score_general_rt = score_temp_amb_rt * 0.4 + score_hum_rt * 0.4 + score_thi_rt * 0.2
                        score_general_prom = score_temp_amb_prom * 0.4 + score_hum_prom * 0.4 + score_thi_prom * 0.2
                    if is_stuck_confirm.value:
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
                    if abs(t_mlx_surface_mean - last_t_mlx_surface_mean) > 5 and bad_t_mlx_surface_mean_counter == 0:
                        t_mlx_surface_mean_measure = last_t_mlx_surface_mean
                        bad_t_mlx_surface_mean_counter += 1
                    else:
                        bad_t_mlx_surface_mean_counter = 0
                        t_mlx_surface_mean_measure = t_mlx_surface_mean
                        last_t_mlx_surface_mean = t_mlx_surface_mean
                    measurement = {"temp_environment": t_bme_mean, "temp_surface": t_mlx_surface_mean_measure, "humidity": h_bme_mean, "comfort": thi, "score_temp_environment": round(score_temp_amb_rt,2), "score_temp_surface": round(score_temp_bed_rt,2), "score_humidity": round(score_hum_rt,2), "score_comfort": round(score_thi_rt,2), "time":datetime.now().strftime("%H:%M:%S"), "robot_id": robot_id, "score_overall": round(score_general_rt,2)}
                    if is_tempered:
                        measurements_list.append(measurement)
                    day_info ={"day": {"breeding_day": breeding_day, "config": day_score_config, "total_time": t_total, "active_time": t_active, "rest_time": t_rest, "stuck_time": t_stuck, "date":current_date_server, "campaign_id": campaign_id, "measurements": measurements_list}}
                    
                    day_info_list[-1] = day_info
                    with open('send_queue/logs/logs_to_send.json', 'w') as outfile:
                            json.dump(day_info_list, outfile)
                    with open('send_queue/logs/logs_to_send_backup.json', 'w') as outfile:
                            json.dump(day_info_list, outfile)
                    if not is_rest.value:
                        printe("Estado: BME: T:{}/H:{}/P:{} MLX90614: T:{}".format(round(t_bme_mean,2), round(h_bme_mean,2), round(p_bme_mean,2), round(t_mlx_surface_mean,2)))
                        logwriter("Estado", id=14, t_cpu=temp_cpu.value, t_clock=temp_clock.value, t_bme=round(t_bme_mean,2),
                            t_laser_surf=round(t_mlx_surface_mean,2), t_laser_amb=round(t_mlx_amb_mean,2), h_bme=round(h_bme_mean,2), p_bme=round(p_bme_mean,2), thi=round(thi,2), clearance=clearance.value, score_temp_amb_rt=round(score_temp_amb_rt,2), score_temp_bed_rt = round(score_temp_bed_rt,2), score_hum_rt = round(score_hum_rt,2), score_thi_rt = round(score_thi_rt,2), score_general_rt = round(score_general_rt,2), score_temp_amb_prom=round(score_temp_amb_prom,2), score_temp_bed_prom = round(score_temp_bed_prom,2), score_hum_prom = round(score_hum_prom,2), score_thi_prom = round(score_thi_prom,2), score_general_prom = round(score_general_prom,2), t_total = round(t_total,2), t_active = round(t_active,2), t_rest = round(t_rest,2), t_stuck = round(t_stuck,2), battery_percent = battery_percent, battery_voltage = battery_voltage)
                    else:
                        printe("Estado descansando : BME: T:{}/H:{}/P:{} MLX90614: T:{}".format(round(t_bme_mean,2), round(h_bme_mean,2), round(p_bme_mean,2), round(t_mlx_surface_mean,2)))
                        logwriter("Estado, descansando", id=15, t_cpu=temp_cpu.value, t_clock=temp_clock.value, t_bme=round(t_bme_mean,2),
                            t_laser_surf=round(t_mlx_surface_mean,2), t_laser_amb=round(t_mlx_amb_mean,2), h_bme=round(h_bme_mean,2), p_bme=round(p_bme_mean,2), thi=round(thi,2), clearance=clearance.value, score_temp_amb_rt=round(score_temp_amb_rt,2), score_temp_bed_rt = round(score_temp_bed_rt,2), score_hum_rt = round(score_hum_rt,2), score_thi_rt = round(score_thi_rt,2), score_general_rt = round(score_general_rt,2), score_temp_amb_prom=round(score_temp_amb_prom,2), score_temp_bed_prom = round(score_temp_bed_prom,2), score_hum_prom = round(score_hum_prom,2), score_thi_prom = round(score_thi_prom,2), score_general_prom = round(score_general_prom,2), t_total = round(t_total,2), t_active = round(t_active,2), t_rest = round(t_rest,2), t_stuck = round(t_stuck,2), battery_percent = battery_percent, battery_voltage = battery_voltage)
        except Exception as ex:
            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
            camera_state["status"] = False
            camera_state["status_str"] = "Fallo la toma de imagenes, no deberia llegar al servidor"
            errorwriter(ex, "Fallo el try mayor de la camara en la linea {}".format(sys.exc_info()[-1].tb_lineno))
            
def savior(imu_req, is_rest, pitch_flag, pitch_counter, clearance, clearance_stuck_flag, imu_stuck_flag, is_stuck_confirm, stuck_watchdog_time, vl53_state, imu_state ):
    # Vamos a obtener las mediciones de clearance y del imu
    stuck_watchdog_clearance = []
    stuck_watchdog_clearance_saved = False
    imu_debug = True
    stuck_watchdog_imu = []
    stuck_watchdog_imu_saved = False
    clearance_control = False
    clearance_debug = True
    log_imu_stuck = True
    last_imu = 0
    start_imu_stuck = 0
    total_elapsed_imu_stuck = 0
    last_total_elapsed_imu_stuck = 0
    elapsed_imu_stuck = 0
    # imu_state = {"status": False, "status_str": "No detectado"}
    # vl53_state = {"status": False, "status_str": "No detectado"}
    #region Constantes de address del IMU
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
    #endregion
    #region Constantes relacionadas con el control de clearance
    vl53l0x_connected = False
    tof_offset = 78
    min_tof_clearance = 20
    sink_tof_clearance = 16
    watch_tof_clearance = 30
    max_tof_clearance = 25
    clearance_array = []
    time_clearance_array = []
    clearance_window = 2
    clearance_timer = time.perf_counter()
    time_to_sink = 3
    sink_slope = (sink_tof_clearance-max_tof_clearance)/time_to_sink
    #endregion
    #region Funciones relacionadas con el IMU
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
    #region Inicio IMU
    if imu_detected:
        imu_state["status"] = False
        imu_state["status_str"] = "Detecado pero no iniciado"
        try:
            bus = smbus.SMBus(4)	
            Device_Address = imu_address   
            MPU_Init()
            printe(bcolors.OKGREEN + "IMU iniciado" + bcolors.ENDC)
            imu_connected = True
            last_imu_measure = time.perf_counter()
            imu_state["status"] = False
            imu_state["status_str"] = "Iniciado, sin tomar mediciones"

        except Exception as ex:
            errorwriter(ex, "Error al iniciar el IMU")
            printe(bcolors.FAIL + "El IMU no pudo iniciar" + bcolors.ENDC)
            imu_connected = False
            imu_state["status"] = False
            imu_state["status_str"] = "Detectado, no se pudo iniciar"
    #endregion
    #region Inicio VL53
    if vl53l0x_detected:
        vl53_state["status"] = False
        vl53_state["status_str"] = "Detectado pero no iniciado"
        try:
            tof = VL53L0X.VL53L0X(i2c_bus=4,i2c_address=0x29)
            tof.open()
            tof.start_ranging(VL53L0X.Vl53l0xAccuracyMode.BETTER)
            vl53l0x_connected = True
            printe(bcolors.OKGREEN + "VL53L0X iniciado" + bcolors.ENDC)
            vl53_state["status"] = False
            vl53_state["status_str"] = "Iniciado, sin tomar mediciones"
        except Exception as ex:
            errorwriter(ex, "Error al iniciar el VL53L0X")
            printe(bcolors.FAIL + "Error al inciar el VL53L0X" + bcolors.ENDC)
            vl53l0x_connected = False
            vl53_state["status"] = False
            vl53_state["status_str"] = "Detectado, no se pudo iniciar"
    #endregion
    
    while True:
        time.sleep(0.05)
        if vl53l0x_connected:
            try:
                clearance_measure = tof.get_distance()
                if clearance_measure < 2 or clearance_measure > 200 :
                    vl53_state["status"] = False
                    vl53_state["status_str"] = "Medicion no valida"
                else:
                    vl53_state["status"] = True
                    vl53_state["status_str"] = "OK"
                    clearance.value = clearance_measure - tof_offset
            except Exception as ex:
                printe(bcolors.FAIL + "No se pudo tomar medicion clearance, excepcion {}".format(ex) + bcolors.ENDC)
                errorwriter(error=ex, comentario="No se pudo tomar medicion clearance")
                vl53_state["status"] = False
                vl53_state["status_str"] = "Iniciado pero no se pudo tomar medicion"
            if clearance_debug:
                stuck_watchdog_clearance.append((datetime.now().strftime("%H:%M:%S"),clearance.value))
                if (datetime.strptime(stuck_watchdog_clearance[-1][0], "%H:%M:%S") - datetime.strptime(stuck_watchdog_clearance[0][0], "%H:%M:%S")).seconds > stuck_watchdog_time:
                    stuck_watchdog_clearance.pop(0)
                if is_stuck_confirm.value and not stuck_watchdog_clearance_saved:
                    printe("Traba detectada, guardando la data de clearance")
                    stuck_watchdog_clearance_saved = True
                    if not os.path.exists("clearance_debug/{}".format(datetime.now().strftime("%Y%m%d"))):
                        os.mkdir("clearance_debug/{}".format(datetime.now().strftime("%Y%m%d")))
                    with open("clearance_debug/{}/{}.csv".format(datetime.now().strftime("%Y%m%d"),datetime.now().strftime("%H%M%S")), "w", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerows(stuck_watchdog_clearance)
                    stuck_watchdog_clearance = []
                if not is_stuck_confirm.value and stuck_watchdog_clearance_saved:
                    stuck_watchdog_clearance_saved = False
            if clearance_control:
                if not clearance_stuck_flag.value:
                    if clearance.value < min_tof_clearance:
                        clearance_stuck_flag.value = True
                        printe("Clearance menor a la minima, traba")
                        time_clearance_array = []
                        clearance_array = []
                    elif clearance.value < watch_tof_clearance:
                        time_clearance_array.append(time.perf_counter())
                        clearance_array.append(clearance.value)
                        if (time_clearance_array[-1] - time_clearance_array[0]) > clearance_window:
                            time_clearance_array.pop(0)
                            clearance_array.pop(0)
                        if len(clearance_array) > 5:
                            y = np.array(clearance_array)
                            x = np.array([count for count, i in enumerate(clearance_array)])
                            A = np.stack([x, np.ones(len(x))]).T
                            slope, point_zero = np.linalg.lstsq(A, y, rcond=None)[0]
                            if slope < sink_slope:
                                printe("La pendiente de clearance es mayor a la admisible, traba en accion")
                                clearance_stuck_flag.value = True
                                time_clearance_array = []
                                clearance_array = []
                else:
                    if clearance.value > max_tof_clearance:
                        printe("Aumente el clearance, me destrabe")
                        clearance_stuck_flag.value = False
        if imu_connected and imu_debug and not is_rest.value:
            try:
                acc_x = read_raw_data(ACCEL_XOUT_H)
                acc_y = read_raw_data(ACCEL_YOUT_H)
                acc_z = read_raw_data(ACCEL_ZOUT_H)
                gyro_x = read_raw_data(GYRO_XOUT_H)
                gyro_y = read_raw_data(GYRO_YOUT_H)
                gyro_z = read_raw_data(GYRO_ZOUT_H)
                stuck_watchdog_imu.append((datetime.now().strftime("%H:%M:%S"), acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z))
                if (datetime.strptime(stuck_watchdog_imu[-1][0], "%H:%M:%S") - datetime.strptime(stuck_watchdog_imu[0][0], "%H:%M:%S")).seconds > stuck_watchdog_time:
                    stuck_watchdog_imu.pop(0)
                if is_stuck_confirm.value and not stuck_watchdog_imu_saved:
                    printe("Traba detectada, guardando la data de IMU")
                    stuck_watchdog_imu_saved = True
                    if not os.path.exists("imu_debug/{}".format(datetime.now().strftime("%Y%m%d"))):
                        os.mkdir("imu_debug/{}".format(datetime.now().strftime("%Y%m%d")))
                    with open("imu_debug/{}/{}.csv".format(datetime.now().strftime("%Y%m%d"),datetime.now().strftime("%H%M%S")), "w", newline="") as f:
                        writer = csv.writer(f)
                        writer.writerows(stuck_watchdog_imu)
                    stuck_watchdog_imu = []
                if not is_stuck_confirm.value and stuck_watchdog_imu_saved:
                    stuck_watchdog_imu_saved = False
            except:
                pass
        if imu_connected and imu_req.value and time.perf_counter() - last_imu_measure > 1:
            last_imu_measure = time.perf_counter()
            try:
                acc_y = read_raw_data(ACCEL_YOUT_H)
                acc_z = read_raw_data(ACCEL_ZOUT_H)

                Ay = acc_y/16384.0
                Az = acc_z/16384.0

                    
                pitch = math.atan2(Ay,  Az) * 57.3
                imu_state["status"] = True
                imu_state["status_str"] = "OK"

                if pitch > pitch_flag.value:
                    counter += 1
                else:
                    counter = 0
                    imu_stuck_flag.value = False
                    if log_imu_stuck == False:
                        elapsed_imu_stuck = round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                        total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                        json_stuck_line_imu = {"IMU": total_elapsed_imu_stuck}
                        with open('config/actual/stuck_count_imu.json', 'w') as outfile:
                            json.dump(json_stuck_line_imu, outfile)
                        with open('config/actual/stuck_count_imu_backup.json', 'w') as outfile:
                            json.dump(json_stuck_line_imu, outfile)
                        printe("Me destrabe IMU, inclinacion {}°".format(pitch))
                        logwriter("Me destrabe, IMU, minutos", minutos=elapsed_imu_stuck, id=19)
                        log_imu_stuck = True
                if counter > pitch_counter.value:
                    if log_imu_stuck:
                        printe("Me trabe IMU, inclinacion {}°".format(pitch))
                        logwriter("Me trabe, IMU", id=18)
                        log_imu_stuck = False
                        start_imu_stuck = time.perf_counter()
                        last_total_elapsed_imu_stuck = total_elapsed_imu_stuck
                    elapsed_imu_stuck =  round((time.perf_counter() - start_imu_stuck)/60.0, 2)
                    total_elapsed_imu_stuck = last_total_elapsed_imu_stuck + elapsed_imu_stuck
                    json_stuck_line = {"IMU": total_elapsed_imu_stuck}
                    with open('config/actual/stuck_count_imu.json', 'w') as outfile:
                        json.dump(json_stuck_line, outfile)
                    with open('config/actual/stuck_count_imu_backup.json', 'w') as outfile:
                            json.dump(json_stuck_line, outfile)
                    imu_stuck_flag.value = True
            except Exception as ex:
                imu_state["status"] = False
                imu_state["status_str"] = "No se pudo tomar medicion"
                errorwriter(ex, "El IMU no pudo tomar lectura")
                printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "Ups! El IMU no pudo tomar lectura"+ bcolors.ENDC)
                if ex == "Error: [Errno 6] No such device or address":
                    printe("Vamos a tratar de iniciar de nuevo el IMU")
                    try:
                        MPU_Init()
                        printe(bcolors.OKGREEN + "IMU iniciado" + bcolors.ENDC)
                    except Exception as ex:
                        printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "El IMU no se pudo reiniciar"+ bcolors.ENDC)
                        errorwriter(ex, "El IMU no se pudo reiniciar")
def auto(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, is_hot, rest_time, wake_time, time_backwards, crash_timeout, last_touch_window_timeout, flash_req, vel_array, time_turn, x_com, z_com, is_rest, night_mode_enable, night_mode_start, night_mode_end, night_mode_rest_time, night_mode_wake_time, night_mode_vel_array, night_mode_reversed, prints_enable ):
    
    if is_milka:
        printe("Es MILKA")
        motor_stby = DigitalOutputDevice("BOARD40")
        motor_rf_pwm = PWMOutputDevice("BOARD35")  #1 ES EL MOTOR DERECHO
        motor_rb_pwm = PWMOutputDevice("BOARD33")  #1 ES EL MOTOR DERECHO
        motor_lf_pwm = PWMOutputDevice("BOARD32")  #1 ES EL MOTOR DERECHO
        motor_lb_pwm = PWMOutputDevice("BOARD12")
        motor_rf_cw_dir = DigitalOutputDevice("BOARD37")   #AIN1
        motor_rf_ccw_dir = DigitalOutputDevice("BOARD31")  #AIN2
        motor_rb_cw_dir = DigitalOutputDevice("BOARD38")   #BIN1
        motor_rb_ccw_dir = DigitalOutputDevice("BOARD36")  #BIN2
        motor_lf_cw_dir = DigitalOutputDevice("BOARD29")
        motor_lf_ccw_dir = DigitalOutputDevice("BOARD22")
        motor_lb_cw_dir = DigitalOutputDevice("BOARD18")
        motor_lb_ccw_dir = DigitalOutputDevice("BOARD16")
        bumper_l = Button("BOARD19", pull_up= False)
        bumper_r = Button("BOARD21", pull_up= False)
        shutdown_button = Button("BOARD15")
    else:
        printe("Es SIXPACK")
        motor_stby = DigitalOutputDevice("BOARD40")
        motor_lf_pwm = PWMOutputDevice("BOARD35")  #1 ES EL MOTOR DERECHO
        motor_lb_pwm = PWMOutputDevice("BOARD33")  #1 ES EL MOTOR DERECHO
        motor_rf_pwm = PWMOutputDevice("BOARD32")  #1 ES EL MOTOR DERECHO
        motor_rb_pwm = PWMOutputDevice("BOARD12")
        motor_lf_ccw_dir = DigitalOutputDevice("BOARD37")   #AIN1
        motor_lf_cw_dir = DigitalOutputDevice("BOARD31")  #AIN2
        motor_lb_ccw_dir = DigitalOutputDevice("BOARD38")   #BIN1
        motor_lb_cw_dir = DigitalOutputDevice("BOARD36")  #BIN2
        motor_rf_ccw_dir = DigitalOutputDevice("BOARD29")
        motor_rf_cw_dir = DigitalOutputDevice("BOARD22")
        motor_rb_ccw_dir = DigitalOutputDevice("BOARD18")
        motor_rb_cw_dir = DigitalOutputDevice("BOARD16")
        bumper_l = Button("BOARD27", pull_up= True)
        bumper_r = Button("BOARD28", pull_up= True)
        shutdown_button = Button("BOARD19")
    motor_rf_pwm.frequency = 1000
    motor_rb_pwm.frequency = 1000
    motor_lf_pwm.frequency = 1000
    motor_lb_pwm.frequency = 1000
    motor_rf_pwm.off()
    motor_rb_pwm.off()
    motor_lf_pwm.off()
    motor_lb_pwm.off()
    was_auto = False
    first_auto = True
    led_on = True
    global move_status
    move_status = ''
    global crash_counter
    crash_counter = 0
    global crash_timer
    crash_timer = 0
    global last_crash 
    last_crash = ''
    global timer
    timer = time.perf_counter()
    wake_time_watch = wake_time.value
    rest_time_watch = rest_time.value
    night_mode_setted = False
    no_crash = False

    class Velocity:
        def __init__(self, forward, backward):
            self.forward = self.VelocityData(forward)
            self.left = -1
            self.right = 1
            self.backward = self.VelocityData(backward)
        class VelocityData:
            def __init__(self, velArray):
                self.stuck = velArray[0]
                self.normal = velArray[1]
    vel = Velocity(vel_array[0], vel_array[1])
    


    def move(x = 0, z = 0, t = 0):
        global move_status
        motor_stby.off()
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
                    pwm1 = vel_array[2] 
                    pwm2 = vel_array[3]
                else:
                    pwm2 = vel_array[2]
                    pwm1 = vel_array[3]
            else:
                if (z > 0):
                    pwm1 = vel_array[3]
                    pwm2 = vel_array[2]
                else:
                    pwm2 = vel_array[3]
                    pwm1 = vel_array[2]
        if (pwm1 > 0):
            motor_rf_cw_dir.off()
            motor_rb_cw_dir.off()
            if not motor_rf_cw_dir.is_active:
                motor_rf_ccw_dir.on()
            if not motor_rb_cw_dir.is_active:
                motor_rb_ccw_dir.on()
        elif(pwm1 < 0):
            motor_rf_ccw_dir.off()
            motor_rb_ccw_dir.off()
            if not motor_rf_ccw_dir.is_active:
                motor_rf_cw_dir.on()
            if not motor_rb_ccw_dir.is_active:
                motor_rb_cw_dir.on()
        else:
            motor_rf_cw_dir.off()
            motor_rf_ccw_dir.off()
            motor_rf_pwm.off()
            motor_rb_cw_dir.off()
            motor_rb_ccw_dir.off()
            motor_rb_pwm.off()
        if (pwm2 > 0):
            motor_lf_ccw_dir.off()
            motor_lb_ccw_dir.off()
            if not motor_lf_ccw_dir.is_active:
                motor_lf_cw_dir.on()
            if not motor_lb_ccw_dir.is_active:
                motor_lb_cw_dir.on()
        elif(pwm2 < 0):
            motor_lf_cw_dir.off()
            motor_lb_cw_dir.off()
            if not motor_lf_cw_dir.is_active:
                motor_lf_ccw_dir.on()
            if not motor_lb_cw_dir.is_active:
                motor_lb_ccw_dir.on()
        else:
            motor_lf_cw_dir.off()
            motor_lf_ccw_dir.off()
            motor_lf_pwm.off()
            motor_lb_cw_dir.off()
            motor_lb_ccw_dir.off()
            motor_lb_pwm.off()
        if motor_rf_pwm.value != abs(pwm1):
            motor_rf_pwm.value = abs(pwm1)
        if motor_rb_pwm.value != abs(pwm1):
            motor_rb_pwm.value = abs(pwm1)
        if motor_lf_pwm.value != abs(pwm2):
            motor_lf_pwm.value = abs(pwm2)
        if motor_lb_pwm.value != abs(pwm2):
            motor_lb_pwm.value = abs(pwm2)
        if pwm1 != 0 and pwm2 != 0:
            if motor_rf_cw_dir.is_active != motor_rf_ccw_dir.is_active and motor_rb_cw_dir.is_active != motor_rb_ccw_dir.is_active and motor_lf_cw_dir.is_active != motor_lf_ccw_dir.is_active and motor_lb_cw_dir.is_active != motor_lb_ccw_dir.is_active:
                motor_stby.on()
                # pass
            else:
                printe(bcolors.FAIL + "ERROR, va a quemarse el driver porque hay dos pines de direccion HIGH, PWM1= {}, PWM2={}".format(pwm1,pwm2) + bcolors.ENDC)
                motor_rf_pwm.off()
                motor_rb_pwm.off()
                motor_lf_pwm.off()
                motor_lb_pwm.off()
                motor_rf_cw_dir.off()
                motor_rf_ccw_dir.off()
                motor_rb_cw_dir.off()
                motor_rb_ccw_dir.off()
                motor_lf_cw_dir.off()
                motor_lf_ccw_dir.off()
                motor_lb_cw_dir.off()
                motor_lb_ccw_dir.off()
        if t > 0:
            number_check_rate = int(t / check_rate)
            rest = t - number_check_rate * check_rate
            counter_check_rate = 0
            if x > 0:
                while (counter_check_rate <= number_check_rate and auto_req.value == True and not taking_pics.value and not (bumper_l.is_pressed or bumper_r.is_pressed) and not shutdown_button.is_pressed):
                    time.sleep(check_rate)
                    counter_check_rate += 1
            else:
                while (counter_check_rate <= number_check_rate and auto_req.value == True and not taking_pics.value and not shutdown_button.is_pressed):
                    time.sleep(check_rate)
                    counter_check_rate += 1
            if counter_check_rate == number_check_rate and auto_req.value == True:
                    time.sleep(rest)
            motor_rf_pwm.off()
            motor_rb_pwm.off()
            motor_lf_pwm.off()
            motor_lb_pwm.off()
            motor_rf_cw_dir.off()
            motor_rf_ccw_dir.off()
            motor_rb_cw_dir.off()
            motor_rb_ccw_dir.off()
            motor_lf_cw_dir.off()
            motor_lf_ccw_dir.off()
            motor_lb_cw_dir.off()
            motor_lb_ccw_dir.off()
    def crash(crash_side):
        global crash_counter
        global timer
        global crash_timer
        global last_crash
        printe('Toque de tipo :', crash_side)
        if time.perf_counter() - crash_timer >= last_touch_window_timeout.value:
            printe('Pasaron {} segundos desde el ultimo toque, el timeout esta en {} segundos, por lo que resetee variables'.format(int(time.perf_counter() - crash_timer), last_touch_window_timeout.value))
            last_crash = ''
            crash_counter = 0
        crash_confirmed = False
        crash_timer = time.perf_counter()
        if crash_timeout.value > 0:
            printe('Hay que esperar {} segundos para confirmar el choque'.format(crash_timeout.value))
            while (time.perf_counter() - crash_timer) < crash_timeout.value:
                time.sleep(0.25)
                if crash_side == "FRO" and bumper_l.is_pressed and bumper_r.is_pressed:
                    crash_confirmed = True
                    if last_crash != '' and last_crash != 'FRO':
                        crash_side = last_crash
                    else:
                        crash_side = random.choice(['IZQ', 'DER'])
                elif (crash_side == "IZQ" or crash_side == "FRO") and bumper_l.is_pressed:
                    crash_confirmed = True
                    crash_side = 'IZQ'
                elif (crash_side == "DER" or crash_side == "FRO") and bumper_r.is_pressed:
                    crash_confirmed = True
                    crash_side = 'DER'
                else:
                    printe('Se solto el toque antes, no hay choque confirmado')
                    crash_confirmed = False
                    break
        else:
            crash_confirmed = True
            if crash_side == 'FRO' and last_crash != '' and last_crash != 'FRO':
                crash_side = last_crash
            elif crash_side == 'FRO':
                crash_side = random.choice(['IZQ', 'DER'])
        if crash_confirmed:
            printe('Toque confirmado lado:', crash_side)
            timer = time.perf_counter()
            time_turn_crash = time_turn.value
            crash_counter += 1
            if crash_counter > 3:
                printe('{} toques seguidos, vamor a dar un giro mas pronunciado'.format(crash_counter))
                time_turn_crash = 2 * time_turn.value
            elif (crash_side != last_crash and last_crash != "") or crash_counter == 3:
                printe('Toques alternos seguidos, giro la mitad de tiempo')
                time_turn_crash = 0.5 * time_turn.value
            last_crash = crash_side
            move_sequence(crash_side, time_turn_crash)

        
    def move_sequence(type, time_turn_crash):
        if type == "IZQ_STUCK":
            move(vel.forward.normal, vel.left, time_turn.value)
        elif type == "DER_STUCK":
            move(vel.forward.normal, vel.right, time_turn.value)
        elif type == "IZQ":
            move(vel.backward.normal, 0, time_backwards.value)
            move(vel.forward.normal, vel.right, time_turn_crash)
        elif type == "DER":
            move(vel.backward.normal, 0, time_backwards.value)
            move(vel.forward.normal, vel.left, time_turn_crash)
    #region Cheque que los botones del bumper no esten apretados
    if bumper_l.is_pressed:
        printe(bcolors.FAIL + "Error, bumper izquierdo trabado al inciar" + bcolors.ENDC)
        errorwriter(error= "Error, bumper izquierdo trabado al inciar")
    if bumper_r.is_pressed:
        printe(bcolors.FAIL + "Error, bumper derecho trabado al inciar" + bcolors.ENDC)
        errorwriter(error= "Error, bumper derecho trabado al inciar")
    #endregion
    while True:
        buzzer("off")
        if shutdown_button.is_pressed:
            buzzer("off")
            time.sleep(3)
            if shutdown_button.is_pressed:
                printe('Me voy a apagar')
                logwriter("Recibi pedido de apagado", id=8)
                os.system("sudo shutdown now")
            else:
                try:
                    bt_connection(prints_enable)
                except Exception as ex:
                    printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "Problemas con conexion Bluetooth"+ bcolors.ENDC)
        if is_rest.value:
            if led_on:
                led_enable.off()
                led_on = False
            else:
                led_enable.on()
                led_on = True
        if auto_req.value == False:
            if z_com.value > 0:
                move(vel.forward.normal, vel.right, 0)
            elif z_com.value < 0:
                move(vel.forward.normal, vel.left, 0)
            else:
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
                first_auto = False
                is_stopped.value = False
            if is_rest.value and (time.perf_counter()-last_time_on > rest_time_watch):
                is_rest.value = False
                last_time_rest = time.perf_counter()
                is_stopped.value = False
                is_hot.value = False
                printe("Vuelvo a andar")
                logwriter("Termine descanso", id=11)
                led_enable.on()
                if flash_req.value == True:
                    flash_enable.on()
            if night_mode_enable:
                if not night_mode_setted:
                    if not night_mode_reversed and (datetime.time(datetime.now()) > night_mode_start and datetime.time(datetime.now()) < night_mode_end):
                        printe(bcolors.OKBLUE + "Habilitado Night Mode" + bcolors.ENDC)
                        night_mode_setted = True
                        wake_time_watch = night_mode_wake_time
                        rest_time_watch = night_mode_rest_time
                        vel = Velocity(night_mode_vel_array[0], night_mode_vel_array[1])
                    elif night_mode_reversed and (datetime.time(datetime.now()) > night_mode_start or datetime.time(datetime.now()) < night_mode_end):
                        printe(bcolors.OKBLUE + "Habilitado Night Mode" + bcolors.ENDC)
                        night_mode_setted = True
                        wake_time_watch = night_mode_wake_time
                        rest_time_watch = night_mode_rest_time
                        vel = Velocity(night_mode_vel_array[0], night_mode_vel_array[1])
                else:
                    if not night_mode_reversed and (datetime.time(datetime.now()) < night_mode_start or datetime.time(datetime.now()) > night_mode_end):
                        printe(bcolors.OKBLUE + "Deshabilitado Night Mode" + bcolors.ENDC)
                        night_mode_setted = False
                        wake_time_watch = wake_time.value
                        rest_time_watch = rest_time.value
                        vel = Velocity(vel_array[0], vel_array[1])
                    elif night_mode_reversed and (datetime.time(datetime.now()) < night_mode_start and datetime.time(datetime.now()) > night_mode_end):
                        printe(bcolors.OKBLUE + "Deshabilitado Night Mode" + bcolors.ENDC)
                        night_mode_setted = False
                        wake_time_watch = wake_time.value
                        rest_time_watch = rest_time.value
                        vel = Velocity(vel_array[0], vel_array[1])
            was_auto = True
            timer = time.perf_counter()
            while auto_req.value == True and not is_rest.value:
                time.sleep(0.1)
                if move_status != 'F':
                    move(vel.forward.normal)
                if shutdown_button.is_pressed:
                    printe("Boton apretado")
                    move(0, 0)
                    is_stopped.value = True
                    time.sleep(3)
                    if shutdown_button.is_pressed:
                        printe('Me voy a apagar')
                        logwriter("Recibi pedido de apagado", id=8)
                        os.system("sudo shutdown now")
                    else:
                        try:
                            bt_connection(prints_enable)
                        except Exception as ex:
                            printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, "Problemas con conexion Bluetooth"+ bcolors.ENDC)
                        printe("Sali de la conexion Bluetooth")


                if ((time.perf_counter() - last_time_rest > wake_time_watch) or is_hot.value):
                    is_rest.value = True
                    move(0, 0)
                    time.sleep(1)
                    is_stopped.value = True
                    last_time_on = time.perf_counter()
                    printe("Voy a descansar")
                    flash_enable.off()
                    logwriter("Empece descanso", id=10)
                    break
                if night_mode_enable:
                    if not night_mode_setted:
                        if not night_mode_reversed and (datetime.time(datetime.now()) > night_mode_start and datetime.time(datetime.now()) < night_mode_end):
                            printe(bcolors.OKBLUE + "Habilitado Night Mode" + bcolors.ENDC)
                            night_mode_setted = True
                            wake_time_watch = night_mode_wake_time
                            rest_time_watch = night_mode_rest_time
                            vel = Velocity(night_mode_vel_array[0], night_mode_vel_array[1])
                        elif night_mode_reversed and (datetime.time(datetime.now()) > night_mode_start or datetime.time(datetime.now()) < night_mode_end):
                            printe(bcolors.OKBLUE + "Habilitado Night Mode" + bcolors.ENDC)
                            night_mode_setted = True
                            wake_time_watch = night_mode_wake_time
                            rest_time_watch = night_mode_rest_time
                            vel = Velocity(night_mode_vel_array[0], night_mode_vel_array[1])
                    else:
                        if not night_mode_reversed and (datetime.time(datetime.now()) < night_mode_start or datetime.time(datetime.now()) > night_mode_end):
                            printe(bcolors.OKBLUE + "Deshabilitado Night Mode" + bcolors.ENDC)
                            night_mode_setted = False
                            wake_time_watch = wake_time.value
                            rest_time_watch = rest_time.value
                            vel = Velocity(vel_array[0], vel_array[1])
                        elif night_mode_reversed and (datetime.time(datetime.now()) < night_mode_start and datetime.time(datetime.now()) > night_mode_end):
                            printe(bcolors.OKBLUE + "Deshabilitado Night Mode" + bcolors.ENC)
                            night_mode_setted = False
                            wake_time_watch = wake_time.value
                            rest_time_watch = rest_time.value
                            vel = Velocity(vel_array[0], vel_array[1])
                if time.perf_counter() - timer < timer_boring.value:
                    if clearance_stuck_flag.value == True:
                        printe("Estoy trabado por clearance", no_repeat = True)
                        printe('Voy a frenar')
                        move(0, 0)
                        time.sleep(1)
                        move_status_before_destuck = move_status
                        if move_status in ['R','L','F']:
                            printe('Estaba yendo hacia delante, voy a retrocedecer a baja velocidad')
                            move(vel.backward.stuck, 0, time_backwards.value)
                        else:
                            printe('Estaba yendo hacia delante, voy a avanzar a baja velocidad')
                            move(vel.forward.stuck, 0, time_backwards.value)
                        if move_status_before_destuck == 'R':
                            printe('Antes de trabarme estaba yendo hacia la derecha, voy a girar a la izquierda')
                            move_sequence('IZQ_STUCK', time_turn.value)
                        elif move_status_before_destuck == 'L':
                            printe('Antes de trabarme estaba yendo hacia la izquierda, voy a girar a la derecha')
                            move_sequence('DER_STUCK', time_turn.value)
                        else:
                            printe('Antes de trabarme esta yendo recto, voy a elegir aleatorio')
                            move_sequence(random.choice(['DER_STUCK','IZQ_STUCK']), time_turn.value)
                            if clearance_stuck_flag.value == True:
                                printe('Sigo trabado, voy a reintentar')
                    elif imu_stuck_flag.value == True:
                        printe("Estoy trabado por IMU", no_repeat = True)
                        printe('Voy a frenar')
                        move(0, 0)
                        time.sleep(1)
                        move_status_before_destuck = move_status
                        if move_status in ['R','L','F']:
                            printe('Estaba yendo hacia delante, voy a retrocedecer a baja velocidad')
                            move(vel.backward.stuck, 0, time_backwards.value)
                        else:
                            printe('Estaba yendo hacia delante, voy a avanzar a baja velocidad')
                            move(vel.forward.stuck, 0, time_backwards.value)
                        if move_status_before_destuck == 'R':
                            printe('Antes de trabarme estaba yendo hacia la derecha, voy a girar a la izquierda')
                            move_sequence('IZQ_STUCK', time_turn.value)
                        elif move_status_before_destuck == 'L':
                            printe('Antes de trabarme estaba yendo hacia la izquierda, voy a girar a la derecha')
                            move_sequence('DER_STUCK', time_turn.value)
                        else:
                            printe('Antes de trabarme esta yendo recto, voy a elegir aleatorio')
                            move_sequence(random.choice(['DER_STUCK','IZQ_STUCK']), time_turn.value)
                            if imu_stuck_flag.value == True:
                                printe('Sigo trabado, voy a reintentar')
                    elif cam_stuck_flag.value == True:
                        printe("Estoy trabado por camara", no_repeat = True)
                        printe('Voy a frenar')
                        move(0, 0)
                        time.sleep(1)
                        move_status_before_destuck = move_status
                        if move_status in ['R','L','F']:
                            printe('Estaba yendo hacia delante, voy a retrocedecer a baja velocidad')
                            move(vel.backward.stuck, 0, time_backwards.value)
                        else:
                            printe('Estaba yendo hacia atras, voy a avanzar a baja velocidad')
                            move(vel.forward.stuck, 0, time_backwards.value)
                        if move_status_before_destuck == 'R':
                            printe('Antes de trabarme estaba yendo hacia la derecha, voy a girar a la izquierda')
                            move_sequence('IZQ_STUCK', time_turn.value)
                        elif move_status_before_destuck == 'L':
                            printe('Antes de trabarme estaba yendo hacia la izquierda, voy a girar a la derecha')
                            move_sequence('DER_STUCK', time_turn.value)
                        else:
                            printe('Antes de trabarme esta yendo recto, voy a elegir aleatorio')
                            move_sequence(random.choice(['DER_STUCK','IZQ_STUCK']), time_turn.value)
                            if cam_stuck_flag.value == True:
                                printe('Sigo trabado, voy a reintentar')


                    while taking_pics.value == True:
                        move(0, 0)
                        time.sleep(1)
                        is_stopped.value = True
                        printe("Stop esperando foto")
                    is_stopped.value = False
                    if bumper_l.is_pressed and not bumper_r.is_pressed: # Choque izquierdo
                        if no_crash:
                            no_crash = False
                            logwriter("Volvi a detectar una colision, pasaron {} segundos", id=25)
                        crash("IZQ")
                    elif bumper_r.is_pressed and not bumper_l.is_pressed: #Choque derecho
                        if no_crash:
                            no_crash = False
                            logwriter("Volvi a detectar una colision, pasaron {} segundos", id=25)
                        crash("DER")
                    elif (bumper_l.is_pressed and bumper_r.is_pressed): #Choque frontal
                        if no_crash:
                            no_crash = False
                            logwriter("Volvi a detectar una colision, pasaron {} segundos", id=25)
                        crash('FRO')

                else:
                    if not no_crash:
                        no_crash = True
                        logwriter("Pasaron {} segundos sin detectar colisiones, atencion".format(timer_boring.value), id=24)
                    printe('No detecte ningun crash, voy marcha atras por las dudas')
                    timer = time.perf_counter()
                    move_sequence(random.choice(['DER', 'IZQ']), time_turn.value)
        time.sleep(1)

def main():

    vel_array = multiprocessing.Array('d', [])
    time_turn = multiprocessing.Value('d', 0)
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
    rest_time = multiprocessing.Value('i', 0)
    wake_time = multiprocessing.Value('i', 0)
    time_backwards = multiprocessing.Value('d', 0)
    crash_timeout = multiprocessing.Value('d', 0)
    last_touch_window_timeout = multiprocessing.Value('i', 0)
    pic_sensibility_in = multiprocessing.Value('d', 0)
    pic_sensibility_out = multiprocessing.Value('d', 0)
    stucks_to_confirm = multiprocessing.Value('i', 0)
    stuck_window = multiprocessing.Value('d', 0)
    x_com = multiprocessing.Value('d', 0)
    z_com = multiprocessing.Value('d', 0)
    is_rest = multiprocessing.Value('b', False)
    is_stuck_confirm = multiprocessing.Value('b', False)
    manager = multiprocessing.Manager()
    bme_state = manager.dict()
    camera_state = manager.dict()
    mlx_state = manager.dict()
    vl53_state = manager.dict()
    imu_state = manager.dict()
    imu_state["status"] = False 
    imu_state["status_str"] = ""
    vl53_state["status"] = False 
    vl53_state["status_str"] = ""
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
    rest_time.value = behavior["rest_time"]
    wake_time.value = behavior["wake_time"]
    time_backwards.value = behavior["time_backwards"]
    day_crash_timeout = behavior["day_crash_timeout"]
    crash_timeout_before = behavior["crash_timeout_before"]
    crash_timeout_after = behavior["crash_timeout_after"]
    last_touch_window_timeout.value = behavior["last_touch_window_timeout"]
    pic_sensibility_in.value = behavior["pic_sensibility_in"]
    pic_sensibility_out.value = behavior["pic_sensibility_out"]
    stucks_to_confirm.value = behavior['stucks_to_confirm']
    stuck_window.value = behavior['stuck_window']
    stuck_watchdog_time = behavior['stuck_watchdog_time']
    vel_array = [[behavior["vel_forward_stuck"], behavior["vel_forward_normal"]], [-behavior["vel_backward_stuck"], -behavior["vel_backward_normal"]], behavior["vel_turn_inner"], behavior["vel_turn_outter"]]
    time_turn.value = behavior["time_turn"]
    teen_day = behavior["teen_day"]
    night_mode_enable = behavior["night_mode_enable"]
    night_mode_start =  datetime.time(datetime.strptime(behavior["night_mode_start"], "%H:%M"))
    night_mode_end =  datetime.time(datetime.strptime(behavior["night_mode_end"], "%H:%M"))
    night_mode_rest_time = behavior["night_mode_rest_time"]
    night_mode_wake_time = behavior["night_mode_wake_time"]
    night_mode_vel_array = [[behavior["night_mode_vel_forward_stuck"], behavior["night_mode_vel_forward_normal"]], [-behavior["night_mode_vel_backward_stuck"], -behavior["night_mode_vel_backward_normal"]], behavior["vel_turn_inner"], behavior["vel_turn_outter"]]
    night_mode_reversed = night_mode_start > night_mode_end
    if breeding_day >= day_crash_timeout:
        crash_timeout.value = crash_timeout_after
        printe("Sensibilidad baja del bumper")
        logwriter("Sensibilidad baja", id=23)
    else:
        printe("Sensibilidad alta del bumper")
        crash_timeout.value = crash_timeout_before
        logwriter("Sensibilidad alta", id=23)
    if breeding_day < teen_day:
        printe("Los pollos son pequeños, se va a a trabajar a velocidad reducida")
        vel_array = [[behavior["baby_vel_forward_stuck"], behavior["baby_vel_forward_normal"]], [-behavior["baby_vel_backward_stuck"], -behavior["baby_vel_backward_normal"]], behavior["vel_turn_inner"], behavior["vel_turn_outter"], ]
        time_turn.value = behavior["baby_time_turn"]
        time_backwards.value = behavior["baby_time_backwards"]

    json_state = {"flash": flash_req.value, "auto": auto_req.value,
                  "camera": cam_req.value, "imu_req": imu_req.value}
    with open('config/actual/state.json', 'w') as outfile:
        json.dump(json_state, outfile)
    command_handler = multiprocessing.Process(target=command, args=(lst, cam_req, camera_rate, auto_req, imu_req, cam_stuck_flag, imu_stuck_flag, flash_req, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, pitch_flag, pitch_counter, timer_temp, timer_log, rest_time, wake_time, time_backwards, timer_boring, crash_timeout, x_com, z_com,))
    savior_handler = multiprocessing.Process(target=savior, args=(imu_req, is_rest, pitch_flag, pitch_counter, clearance, clearance_stuck_flag, imu_stuck_flag, is_stuck_confirm, stuck_watchdog_time, vl53_state, imu_state,))
    auto_handler = multiprocessing.Process(target=auto, args=(auto_req, timer_boring, taking_pics, is_stopped, cam_stuck_flag, imu_stuck_flag, clearance_stuck_flag, is_hot, rest_time, wake_time, time_backwards, crash_timeout, last_touch_window_timeout, flash_req, vel_array, time_turn, x_com, z_com, is_rest, night_mode_enable, night_mode_start, night_mode_end, night_mode_rest_time, night_mode_wake_time, night_mode_vel_array, night_mode_reversed, prints_enable,))
    pitch_handler = multiprocessing.Process(target=pitch, args=(lst, cam_stuck_flag, clearance, cam_req, camera_rate, taking_pics, is_stopped, is_hot, temp_cpu, temp_clock, temp_out, humedad, amoniaco, window_stuck_pic, timer_temp, timer_log, pic_sensibility_in, pic_sensibility_out, stucks_to_confirm, stuck_window, stuck_watchdog_time,is_rest, flash_req, current_date, score_config, zero_date, day_score_config, breeding_day, campaign_id, is_stuck_confirm, camera_state, mlx_state, bme_state, vl53_state, imu_state,))
    # Add 'em to our list
    if not campaign_is_active:
        while True:
            bt_connection(prints_enable)
    else:
        PROCESSES.append(command_handler)
        PROCESSES.append(savior_handler)
        PROCESSES.append(auto_handler)
        PROCESSES.append(pitch_handler)
    
        
    for p in PROCESSES:
        p.start() 
    while True:
        time.sleep(0.1)
        # printe("che")


if __name__ == '__main__':
    prints_enable = multiprocessing.Value('b', False)
    prints_enable.value = True
    wait_to_run = False 
    if len(sys.argv) > 1:
        if sys.argv[1] == "-s":
            # silent mode, no hay prints, es el modo que se pone en produccion
            prints_enable.value = False
        elif sys.argv[1] == "-d":
            # debug mode, hay prints
            prints_enable.value = True
        elif sys.argv[1] == "-nw":
            # no wait mode, no espera 20 segundos para empezar
            wait_to_run = False
        elif sys.argv[1] == "-w":
            # no wait mode, no espera 20 segundos para empezar
            wait_to_run = True
        elif sys.argv[1] == "-snw":
            prints_enable.value = False
            wait_to_run = False
        elif sys.argv[1] == "-dnw":
            prints_enable.value = True
            wait_to_run = False        
        elif sys.argv[1] == "-sw":
            prints_enable.value = False
            wait_to_run = True
        elif sys.argv[1] == "-dw":
            prints_enable.value = True
            wait_to_run = True    
    
    flash_enable = DigitalOutputDevice("BOARD23", active_high=True)
    led_enable = DigitalOutputDevice("BOARD13", active_high=True)
    printe(gethostname())
    if gethostname() == 'AVISense':
        is_milka = True
    else:
        is_milka = False
    ssid= "AVISenseNetwork"
    if is_milka:
        buzz = DigitalOutputDevice("BOARD28", active_high=True)
    else:
        buzz = DigitalOutputDevice("BOARD15", active_high=True)
    buzzer("off")
    enable_peripheral = DigitalOutputDevice("BOARD11", active_high=True)
    enable_peripheral.on()
    #region Address de los dispositivos I2C
    imu_address = 0x68
    bme_address = 0x76
    mlx90614_address = 0x5A
    vl53l0x_address = 0x29
    time.sleep(1)
    #endregion

    global last_string
    last_string = ''
    global last_error
    last_error = ''
    #region Compruebo los modos en los cuales inicia el programa
        
    start_time = time.perf_counter()
    #endregion
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
    except:
        pass
    #endregion
    #region Si no existen carpetas y archivos los creo aca

    #endregion
    while wait_to_run and time.perf_counter()-start_time < 20:
        led_enable.on()
        time.sleep(0.5)
        led_enable.off()
        time.sleep(0.5)
    printe(bcolors.OKGREEN + "AVI-Sense 1.0 APELIE ROBOTICS - 2022" + bcolors.ENDC)
    #region Chequeo los dispositivos I2C
    printe("Comprobando si los dispositivos I2C estan conectados")
    i2c_dev_list = adafruit_extended_bus.ExtendedI2C(4).scan()
    if imu_address in i2c_dev_list:
        printe(bcolors.OKGREEN + "El IMU fue detectado" + bcolors.ENDC)
        imu_detected = True
    else:
        printe(bcolors.FAIL + "El IMU no fue detectado" + bcolors.ENDC)
        errorwriter(error="IMU no detectado", comentario="El IMU no fue detectado en el arranque")
        imu_detected = False
    if bme_address in i2c_dev_list:
        printe(bcolors.OKGREEN + "El BME fue detectado" + bcolors.ENDC)
        bme_detected = True
    else:
        printe(bcolors.FAIL + "El BME no fue detectado" + bcolors.ENDC)
        errorwriter(error="BME no detectado", comentario="El BME no fue detectado en el arranque")
        bme_detected = False
    if mlx90614_address in i2c_dev_list:
        printe(bcolors.OKGREEN + "El MLX90614 fue detectado" + bcolors.ENDC)
        mlx90614_detected = True
    else:
        printe(bcolors.FAIL + "El MLX90614 no fue detectado" + bcolors.ENDC)
        errorwriter(error="MLX90614 no detectado", comentario="El MLX90614 no fue detectado en el arranque")
        mlx90614_detected = False
    if vl53l0x_address in i2c_dev_list:
        printe(bcolors.OKGREEN + "El VL53L0X fue detectado" + bcolors.ENDC)
        vl53l0x_detected = True
    else:
        printe(bcolors.FAIL + "El VL53L0X no fue detectado" + bcolors.ENDC)
        errorwriter(error="VL53L0X no detectado", comentario="El VL53L0X no fue detectado en el arranque")
        vl53l0x_detected = False
    #endregion
    flash_enable.off()

    if os.path.exists("config/actual/actual_behavior.json"):
        behavior = open_json('config/actual/actual_behavior.json')
    else:
        behavior = open_json('config/default/default_behavior.json')


    init_wifi()
    
    if not os.path.exists("config/actual/stuck_count_imu.json"):
        json_stuck_line_imu = {"IMU": 0}
        with open('config/actual/stuck_count_imu.json', 'w') as outfile:
            json.dump(json_stuck_line_imu, outfile)
    if not os.path.exists("config/actual/stuck_count_cam.json"):
        json_stuck_line_cam = {"Cam": 0}
        with open('config/actual/stuck_count_cam.json', 'w') as outfile:
            json.dump(json_stuck_line_cam, outfile)
    if not os.path.exists("config/actual/stuck_count_camconf.json"):
        json_stuck_line_camconf = {"CamConf": 0}
        with open('config/actual/stuck_count_camconf.json', 'w') as outfile:
            json.dump(json_stuck_line_camconf, outfile)
    last_stuck_imu = open_json('config/actual/stuck_count_imu.json')
    last_stuck_cam = open_json('config/actual/stuck_count_cam.json')
    last_stuck_camconf = open_json('config/actual/stuck_count_camconf.json')
    if not os.path.exists("config/actual/last_on.json"):
        last_on()
    last_watch = open_json('config/actual/last_on.json')

    start_log = "Me apague, minutos trabado camara / IMU " 
    minutos_start = str(last_stuck_cam["Cam"]) + "/" + str(last_stuck_imu["IMU"])
    logwriter(start_log, id=6, watch_dog=True, minutos= minutos_start, 
                last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    logwriter("Me apague, minutos trabado camara confirmados", id=22, watch_dog=True, minutos= str(last_stuck_camconf["CamConf"]), 
                last_date=last_watch["Fecha"], last_hour=last_watch["Hora"], last_name=last_watch["Name"])
    # Vamos a tomar la configuracion de camapaña
    if not os.path.exists('config/actual/campaign_status.json'):
        campaing_status_line = {"is_active": 0, "zero_date": "", "end_date": "", "campaign_id": 0, "baby_origin": "", "batch": 0, "shed_number": 0}
        with open('config/actual/campaign_status.json', 'w') as outfile:
            json.dump(campaing_status_line, outfile)
    campaign_status = open_json('config/actual/campaign_status.json')
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
        if os.path.exists("config/actual/actual_config_scoring.csv"):
            try:
                with open ('config/actual/actual_config_scoring.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('config/actual/actual_config_scoring.csv') as f:
                    copy = f.read()
                    with open ('config/actual/actual_config_scoring_backup.csv', 'w') as file:
                        file.write(copy)
                
            except:
                with open ('config/actual/actual_config_scoring_backup.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('config/actual/actual_config_scoring_backup.csv') as f:
                    copy = f.read()
                    with open ('config/actual/actual_config_scoring.csv', 'w') as file:
                        file.write(copy)
        else:  
            try:                         
                with open ('config/default/default_config_scoring.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))
                with open ('config/default/default_config_scoring.csv') as f:
                    copy = f.read()
                    with open ('config/default/default_config_scoring_backup.csv', 'w') as file:
                        file.write(copy)
            except:
                with open ('config/default/default_config_scoring_backup.csv') as f:
                    rows = csv.reader(f)
                
                    headers = next(rows)
                    record = dict(zip())
                    score_config = []
                    for i,row in enumerate(rows):
                        record = dict(zip(headers,row))
                        score_config.append(record)
                        day_list.append(int(record['Dia']))  
                with open ('config/default/default_config_scoring_backup.csv') as f:
                    copy = f.read()
                    with open ('config/default/default_config_scoring.csv', 'w') as file:
                        file.write(copy)   
        breeding_day = (datetime.now() - datetime.strptime(zero_date, "%Y%m%d")).days
        printe("Breeding day:", breeding_day)
        if breeding_day > 60 or breeding_day < 0:
            printe("Fecha invalida")
            raise Exception
        if breeding_day in day_list:
            breeding_day_index = day_list.index(breeding_day)
        else:
            day_list.append(breeding_day)
            day_list.sort()
            breeding_day_index = day_list.index(breeding_day) - 1
        day_score_config = score_config[breeding_day_index]

        logwriter(id=0, event=str(day_score_config))
    except Exception as ex:
        printe(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno , bcolors.ENDC)
        printe("Error in line:", sys.exc_info()[-1].tb_lineno)
        printe("Fallo la carga de configuracion scoring")
    #endregion
    
    current_date = datetime.now().strftime("%Y%m%d")

    # logwriter("Me prendi con esta configuracion: " + str(config)+'/'+str(behavior), id=1)
    last_on()
    for i in range(3):
        flash_enable.on()
        time.sleep(0.5)
        flash_enable.off()
        time.sleep(0.5)
    if behavior["flash_enable"] == True:
        flash_enable.on()
        logwriter("Prendi luces", id=12)
    # motors_enable.on()
    try:
        led_enable.on()
        main()
    except KeyboardInterrupt:
        # motor_1_pwm.off()
        # motor_2_pwm.off()
        led_enable.off()
        buzz.off()
        for p in PROCESSES:
            p.terminate()
    # except Exception as ex:
    #     print(bcolors.FAIL + "Exception:", ex," in line:", sys.exc_info()[-1].tb_lineno, bcolors.ENDC)
    #     # while True:
    #     #     led_enable.on()
    #     #     time.sleep(0.1)
    #     #     led_enable.off()
    #     #     time.sleep(0.1)
