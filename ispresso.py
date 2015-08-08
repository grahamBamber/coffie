#!/usr/bin/python

# Copyright (c) 2013 Chris Synan & Dataworlds LLC
# Portions copyright (c) 2012 Stephen P. Smith
#
# Permission is hereby granted, free of charge, to any person obtaining 
# a copy of this software and associated documentation files 
# (the "Software"), to deal in the Software without restriction, 
# including without limitation the rights to use, copy, modify, 
# merge, publish, distribute, sublicense, and/or sell copies of the Software, 
# and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:

# The software is free for non-commercial uses.  Commercial uses of this software 
# or any derivative must obtain a license from Dataworlds LLC (Austin TX)

# In addition, the above copyright notice and this permission notice shall 
# be included in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS 
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, 
# WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
# IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

import sys, os, time, logging, logging.handlers, traceback
import threading, subprocess, urllib2
from multiprocessing import Process, Pipe, Queue, Value, current_process
from subprocess import Popen, PIPE, call, signal
from datetime import datetime
sys.path.append(os.path.abspath(os.path.dirname(__file__)))

import web, random, json, atexit
from pid import pidpy as PIDController
import RPi.GPIO as GPIO 
#from lcd import lcddriver
import glob
from temp import tsic


# logging.basicConfig()
logger = logging.getLogger('ispresso')


# REMOTE DEBUG -- TODO:  Remove this before going to production
# import rpdb2 
# rpdb2.start_embedded_debugger('funkymonkey', fAllowRemote = True)

gpio_heat = 24
gpio_pump = 23
gpio_btn_steam_sig = 11
gpio_btn_steam_led = 12
gpio_btn_heat_led = 8
gpio_btn_heat_sig = 7
gpio_btn_pump_led = 10
gpio_btn_pump_sig = 9
gpio_tsic = 22

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM) 
GPIO.setup(gpio_btn_heat_sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(gpio_btn_pump_sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN) 
GPIO.setup(gpio_btn_steam_sig, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

GPIO.setup(gpio_heat, GPIO.OUT) 
GPIO.setup(gpio_pump, GPIO.OUT) 

GPIO.setup(gpio_btn_heat_led, GPIO.OUT)
GPIO.setup(gpio_btn_pump_led, GPIO.OUT)
GPIO.setup(gpio_btn_steam_led, GPIO.OUT)


def logger_init():

    logger.setLevel(logging.DEBUG)
    log_file_size = 1024 * 1024 * 1  # 1 MB
    formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(process)d - %(name)s : %(message)s')
    fh = logging.handlers.RotatingFileHandler('/var/log/ispresso.log', maxBytes=log_file_size, backupCount=5)
    fh.setFormatter(formatter)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(formatter)
    logger.addHandler(fh)
    logger.addHandler(sh)
    logger.info('******************************************')
    logger.info('Starting up...')

def initialize():

    settings.load()

   # if setup.check_connected() == False:  # this needs to happen after lcd pipe is set up
   #     logger.warn("WiFi can't connect to internet.  Entering Smart Connect mode.  Connect to iSPRESSO wireless network.")
   #     mem.lcd_connection.send(["COFFGB", "Access Point", 0])
   #     setup.smart_connect()
   # else:
    logger.info("WiFi connection looks ok")
    mem.lcd_connection.send(["COFFGB", "WiFi OK", 3])
    mem.lcd_connection.send(["COFFGB", "", 0])


class mem:  # global class  
    cache_day = None
    cache_start_time = None
    cache_end_time = None
    heat_connection = Pipe()
    lcd_connection = Pipe()
    brew_connection = Pipe()
    flag_pump_on = False
    sched_flag_on = False
    sched_flag_off = False
    time_heat_button_pressed = time.time()
    time_steam_button_pressed = time.time()
    scheduler_enabled = True
    presoak_time = 3
    wait_time = 1.2
    brew_time = 25
    one_wire = None
    tsicObj = None

class param:
    mode = "off"
    cycle_time = 1.0
    duty_cycle = 0.0
    set_point = 99
    k_param = 6  # was 6
    i_param = 120  # was 120
    d_param = 5  # was 5
    steam_cycle = 2.0
    steam_temp = 125
    steam_k = 100
    steam_i = 20
    steam_d = 5
	

def add_global_hook(parent_conn, statusQ, statusQ2):

#    mem.heat_connection = parent_conn    
    g = web.storage({"parent_conn" : parent_conn, "statusQ" : statusQ, "statusQ2" : statusQ2 })
    def _wrapper(handler):
        web.ctx.globals = g
        return handler()
    return _wrapper

class advanced: 
    def __init__(self):
                
        self.mode = param.mode
        self.cycle_time = param.cycle_time
        self.duty_cycle = param.duty_cycle
        self.set_point = param.set_point
        self.k_param = param.k_param
        self.i_param = param.i_param
        self.d_param = param.d_param
        self.steam_temp = param.steam_temp
	self.steam_cycle = param.steam_cycle

    def GET(self):
       
        return render.advanced(self.mode, self.set_point, self.duty_cycle, self.cycle_time, self.k_param, self.i_param, self.d_param)
        
    def POST(self):
        data = web.data()
        datalist = data.split("&")
        for item in datalist:
            datalistkey = item.split("=")
            if datalistkey[0] == "mode":
                self.mode = datalistkey[1]
            if datalistkey[0] == "setpoint":
                self.set_point = float(datalistkey[1])
            if datalistkey[0] == "dutycycle":
                self.duty_cycle = float(datalistkey[1])
            if datalistkey[0] == "cycletime":
                self.cycle_time = float(datalistkey[1])
            if datalistkey[0] == "k":
                self.k_param = float(datalistkey[1])
            if datalistkey[0] == "i":
                self.i_param = float(datalistkey[1])
            if datalistkey[0] == "d":
                self.d_param = float(datalistkey[1])
            if datalistkey[0] == "steam_temp":
                self.d_param = float(datalistkey[1])
            if datalistkey[0] == "steam_cycle":
                self.d_param = float(datalistkey[1])

        param.mode = self.mode 
        param.cycle_time = self.cycle_time
        param.duty_cycle = self.duty_cycle 
        param.set_point = self.set_point
        param.k_param = self.k_param
        param.i_param = self.i_param 
        param.d_param = self.d_param
	param.steam_temp = self.steam_temp
	param.steam_cycle = self.steam_cycle

        settings.save()

        web.ctx.globals.parent_conn.send([self.mode, self.cycle_time, self.duty_cycle, self.set_point, self.k_param, self.i_param, self.d_param, self.steam_temp, self.steam_cycle, False])  

def gettempProc(conn):
    p = current_process()
    logger = logging.getLogger('ispresso').getChild("getTempProc")
    logger.info('Starting:' + p.name + ":" + str(p.pid))
    num = 0.0
    try:
        while (True):
            t = time.time()
            time.sleep(0.5)  # .1+~.83 = ~1.33 seconds
            num = tempdata()
            elapsed = "%.2f" % (time.time() - t)
	    #print num
            conn.send([num, elapsed])
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))


def getonofftime(cycle_time, duty_cycle):
    duty = duty_cycle / 100.0
    on_time = cycle_time * (duty)
    off_time = cycle_time * (1.0 - duty)   
    return [on_time, off_time]

def tellHeatProc(heat_mode=None, flush_cache=None):

    if flush_cache is None:
        flush_cache = False
    
    if heat_mode is not None:
        param.mode = heat_mode

    mem.heat_connection.send([param.mode, param.cycle_time, param.duty_cycle, param.set_point, param.k_param, param.i_param, param.d_param, param.steam_temp, param.steam_cycle, flush_cache])
        
def heatProc(cycle_time, duty_cycle, conn):
    p = current_process()
    logger = logging.getLogger('ispresso').getChild("heatProc")
    logger.info('Starting:' + p.name + ":" + str(p.pid))

    try:    
        while (True):
            while (conn.poll()):  # get last
                cycle_time, duty_cycle = conn.recv()
            conn.send([cycle_time, duty_cycle])  
            if duty_cycle == 0:
                GPIO.output(gpio_heat, GPIO.LOW)
                time.sleep(cycle_time)
            elif duty_cycle == 100:
                GPIO.output(gpio_heat, GPIO.HIGH)
                time.sleep(cycle_time)
            else:
                on_time, off_time = getonofftime(cycle_time, duty_cycle)
                GPIO.output(gpio_heat, GPIO.HIGH)
                time.sleep(on_time)
                GPIO.output(gpio_heat, GPIO.LOW)
                time.sleep(off_time)
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))


def lcdControlProc(lcd_child_conn):
    p = current_process()
    logger = logging.getLogger("ispresso").getChild("lcdControlProc")
    logger.info('Starting:' + p.name + ":" + str(p.pid))

    #lcd = lcddriver.lcd()
    
    last_line1 = ""
    last_line2 = ""

    while (True):
        time.sleep(0.25)
        while lcd_child_conn.poll():
            try:
                line1, line2, duration = lcd_child_conn.recv()
                if line1 is not None: 
                    if last_line1 != line1:
                        #lcd.lcd_display_string(line1.ljust(16), 1)
                        last_line1 = line1
                        time.sleep(duration)
                    
                if line2 is not None:
                    if last_line2 != line2:
                        #lcd.lcd_display_string(line2.ljust(16), 2)
                        last_line2 = line2
                        time.sleep(duration)

            except:
                exc_type, exc_value, exc_traceback = sys.exc_info()
                logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))
                subprocess.call(['i2cdetect', '-y', '1'])
                try:
                    lcd = None
                    time.sleep(0.1)
                    #lcd = lcddriver.lcd()
                    time.sleep(0.1)
                except:  
                    logger.error("Trying to re-initialize the LCD by nulling it out and re-instantiating.  Couldln't pull it off :(")
                continue


def brewControlProc(brew_child_conn):
    p = current_process()
    logger = logging.getLogger("ispresso").getChild("brewControlProc")
    logger.info('Starting:' + p.name + ":" + str(p.pid))

    try:

        mem.flag_pump_on = False
        if (not statusQ2.full()):
            statusQ2.put([mem.flag_pump_on])  # GET request

        button_bounce_threshold_secs = 1
        
        while(True):
            time_button_pushed, brew_plan = brew_child_conn.recv()  # BLOCKS until something shows up
            mem.flag_pump_on = True
            if (not statusQ2.full()):        
                statusQ2.put([mem.flag_pump_on])  # GET request

            for listitem in brew_plan:
                if mem.flag_pump_on == False:
                    while brew_child_conn.poll():  # clear out anything other button presses in the queue
                        brew_child_conn.recv()                              
                    break
                
                action = listitem[0]
                duration = listitem[1]
                counter = 0
                start_time = time.time()
                
                if action.upper() in ("PRESOAK", "BREW"):
                    GPIO.output(gpio_btn_pump_led, GPIO.HIGH)
                    GPIO.output(gpio_pump, GPIO.HIGH)
            
                while ((counter < duration) & mem.flag_pump_on) :  # might not need the check for flag_pump_on here, as its above
                    time.sleep(0.1)
                    if brew_child_conn.poll():  # mem.brew_connection.poll() returns TRUE or FALSE immediately and does NOT block
                        time_button_pushed_again, throwaway_brew_plan = brew_child_conn.recv()  # get item off the list, check how long since time_button_pushed, against button_bounce_threshold_secs.  If too short, clean up and exit this loop
                        if time_button_pushed_again - time_button_pushed > button_bounce_threshold_secs:
                            GPIO.output(gpio_pump, GPIO.LOW)
                            GPIO.output(gpio_btn_pump_led, GPIO.LOW)
                            mem.flag_pump_on = False
                            mem.lcd_connection.send([None, "", 0])
                            break
    
                    if (time.time() - start_time) >= counter:
                        counter = counter + 1
                        message = action + 'ing ' + str(duration - counter) + 's'
                        mem.lcd_connection.send([None, message, 0])
                        logger.debug(message)
                    if (not statusQ2.full()):
                        statusQ2.put([mem.flag_pump_on])  # GET request
            
                GPIO.output(gpio_pump, GPIO.LOW)
                GPIO.output(gpio_btn_pump_led, GPIO.LOW)
                mem.lcd_connection.send([None, '', 0])
    

            while brew_child_conn.poll():  # clear out anything other button presses in the queue
                brew_child_conn.recv()    

            if (not statusQ2.full()):
                statusQ2.put([mem.flag_pump_on])  # GET request

    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))
    finally:
        GPIO.output(gpio_pump, GPIO.LOW)
        GPIO.output(gpio_btn_pump_led, GPIO.LOW)


def tempControlProc(mode, cycle_time, duty_cycle, set_point, k_param, i_param, d_param, steam_temp, steam_cycle, statusQ, conn):  
    p = current_process()
    logger = logging.getLogger('ispresso').getChild("tempControlProc")
    logger.info('Starting:' + p.name + ":" + str(p.pid))

    try:
        parent_conn_temp, child_conn_temp = Pipe()            
        ptemp = Process(name="gettempProc", target=gettempProc, args=(child_conn_temp,))
        ptemp.daemon = True
        ptemp.start()   
        parent_conn_heat, child_conn_heat = Pipe()           
        pheat = Process(name="heatProc", target=heatProc, args=(cycle_time, duty_cycle, child_conn_heat))
        pheat.daemon = True
        pheat.start() 

        pid = PIDController.pidpy(cycle_time, k_param, i_param, d_param)  # init pid
   	pidSteam = PIDController.pidpy(steam_cycle, k_param, i_param, d_param)
        flush_cache = False
        last_temp_C = 0
        
        while (True):
            time.sleep(0.1)

            readytemp = False
            while parent_conn_temp.poll():
                temp_C, elapsed = parent_conn_temp.recv()  # non blocking receive    
                    
                mode = scheduled_mode(mode)  # check to see if scheduler should fire on or off -- MOVING THIS as the OFF doesnt seem to fire..
                if temp_C > 0:  #  the 1-wire sensor sometimes comes back with 0 -- need to fix that by holding on to last value.  
                    last_temp_C = temp_C
                else:
                    temp_C = last_temp_C    
                
                temp_F = (9.0 / 5.0) * temp_C + 32
                temp_C_str = "%3.0f" % temp_C

                #temp_SV_C = (set_point - 32) / 1.8
                temp_SV_str = "%3.0f" % set_point
             

                temp_F_str = "%3.2f" % temp_F
                temp_F_pretty = "%3.0f" % temp_F

		if mode == "steam":
			status = "   Steaming "
                elif mode == "auto":
			status = "    Heating "
		else:
			status = "    Standby"

                if mem.flag_pump_on:
                    mem.lcd_connection.send([ 'pV:' +str(temp_C_str) + '  ' +'sV:' +str(temp_SV_str) , None , 0])
                else:
                    mem.lcd_connection.send([ 'pV:' +str(temp_C_str) + '  ' +'sV:' +str(temp_SV_str) , '' +str(status) , 0])

                readytemp = True
            if readytemp == True:

		if mode == "steam":
		    duty_cycle = pidSteam.calcPID_reg4(temp_C, steam_temp, True)
	            #duty_cycle = 100
		    parent_conn_heat.send([steam_cycle, duty_cycle])
		    GPIO.output(gpio_btn_steam_led, GPIO.HIGH)
		    GPIO.output(gpio_btn_heat_led, GPIO.LOW)
                elif mode == "auto":
                    duty_cycle = pid.calcPID_reg4(temp_C, set_point, True)
                    parent_conn_heat.send([cycle_time, duty_cycle])
                    GPIO.output(gpio_btn_heat_led, GPIO.HIGH) 
		    GPIO.output(gpio_btn_steam_led, GPIO.LOW)
                elif mode == "off":
                    duty_cycle = 0
                    parent_conn_heat.send([cycle_time, duty_cycle])
                    GPIO.output(gpio_btn_heat_led, GPIO.LOW)
		    GPIO.output(gpio_btn_steam_led, GPIO.LOW)
                if (not statusQ.full()):    
                    statusQ.put([temp_C, elapsed, mode, cycle_time, duty_cycle, set_point, k_param, i_param, d_param, steam_temp, steam_cycle])  # GET request
                readytemp == False   
                
            while parent_conn_heat.poll():  # non blocking receive
                cycle_time, duty_cycle = parent_conn_heat.recv()
                     
            while conn.poll():  # POST settings
                mode, cycle_time, duty_cycle_temp, set_point, k_param, i_param, d_param, steam_temp, steam_cycle , flush_cache = conn.recv()
            
            if flush_cache:
                mem.cache_day = None  # this should force cache flush   
                flush_cache = False 

            mode = scheduled_mode(mode)  # check to see if scheduler should fire on or off

    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))


class getstatus:

    def __init__(self):
        pass

    @property
    def GET(self):  # blocking receive
        web.header('Access-Control-Allow-Origin',      '*')
        if (statusQ.full()):  # remove old data
            for i in range(statusQ.qsize()):
                temp, elapsed, mode, cycle_time, duty_cycle, set_point, k_param, i_param, d_param , steam_temp , steam_cycle = web.ctx.globals.statusQ.get() 
        temp, elapsed, mode, cycle_time, duty_cycle, set_point, k_param, i_param, d_param , steam_temp, steam_cycle = web.ctx.globals.statusQ.get() 

	if (web.ctx.globals.statusQ2.empty()):
            flag_pump_on = False
	else:
            flag_pump_on = web.ctx.globals.statusQ2.get_nowait()

        #if (statusQ2.empty()):
	#    flag_pump_on = False
        #elif (statusQ2.full()):  # remove old data
        #    for i in range(statusQ2.qsize()):
        #        flag_pump_on = web.ctx.globals.statusQ2.get()
        #elif flag_pump_on = web.ctx.globals.statusQ2.get_nowait()
        #    if (statusQ2.empty()):
	#	flag_pump_on = False
        #if (flag_pump_on.qsize() > 0):
        #     flag_pump_on = web.ctx.globals.statusQ2.get() 
        #else:
	#     flag_pump_on = False

             
        out = json.dumps({"temp" : temp, "elapsed" : elapsed, "mode" : mode, "cycle_time" : cycle_time, "duty_cycle" : duty_cycle,
                     "set_point" : set_point, "k_param" : k_param, "i_param" : i_param, "d_param" : d_param, "pump" : flag_pump_on
			, "steam_temp" : steam_temp , "steam_cycle" : steam_cycle })
	
        return out

    def POST(self):
        pass

class settings:
    def GET(self):
        with open("settings.json") as f:
            filecontents = json.load(f)
            return render.settings(json.dumps(filecontents))  # a JSON object (string) at this point
    
    def POST(self):
        data = web.data()  # web.input gives back a Storage < > thing
        mydata = json.loads(data)
        for datalistkey in mydata:
            logger.debug("datalistkey = " + str(datalistkey))
            if datalistkey == "temp":
                param.set_point = int(mydata[datalistkey])  
                logger.debug("temp changed to " + str(mydata[datalistkey]))
            if datalistkey == "brewSecs":
                mem.brew_time = int(mydata[datalistkey])
                logger.debug("brew secs changed")
            if datalistkey == "soakSecs":
                mem.presoak_time = int(mydata[datalistkey])
                logger.debug("soak secs changed")
            if datalistkey == "waitSecs":
                mem.wait_time = int(mydata[datalistkey])
                logger.debug("wait secs changed")
        logger.debug("Settings updated:  " + str(mydata))
        settings.save()

    @staticmethod 
    def load():
        with open("settings.json") as loadFile:
            my_settings = json.load(loadFile)      
            mem.brew_time = my_settings["brewSecs"]
            mem.presoak_time = my_settings["soakSecs"]
            mem.wait_time = my_settings["waitSecs"]
            param.set_point = my_settings["temp"]
            param.k_param = my_settings["p_value"]
            param.i_param = my_settings["i_value"]
            param.d_param = my_settings["d_value"]
	    param.steam_temp = my_settings["steam_temp"]
	
	
	
            
    
    @staticmethod
    def save():
        with open("settings.json") as saveFile:
            my_settings = json.load(saveFile)
            my_settings['brewSecs'] = mem.brew_time
            my_settings['soakSecs'] = mem.presoak_time
            my_settings['waitSecs'] = mem.wait_time
            my_settings['temp'] = param.set_point
            my_settings['p_value'] = param.k_param
            my_settings['i_value'] = param.i_param
            my_settings['d_value'] = param.d_param
        logger.debug("About to save settings = " + str(my_settings))
        with open("settings.json", "wb") as output_file:
            json.dump(my_settings, output_file)


class ispresso:

    def GET(self):
	web.header('Access-Control-Allow-Origin',      '*')

        return render.ispresso()

    def POST(self):
        web.header('Access-Control-Allow-Origin',      '*')

        op = ""
        flag = ""
        data = web.data()
        datalist = data.split("&")
        for item in datalist:
            datalistkey = item.split("=")
            if datalistkey[0] == "operation":
                op = datalistkey[1]
            if datalistkey[0] == "flag":
                flag = datalistkey[1]

        
	if str(op).upper() == "STEAM":
 	    if flag == "on":
		tellHeatProc("steam")
	    else:
		tellHeatProc("off")  
        elif str(op).upper() == "HEAT":
            if flag == "on":
                tellHeatProc("auto")
            else:
                tellHeatProc("off")
        elif str(op).upper() == "PUMP":
            time_stamp = time.time()
            brew_plan = [['Presoak', mem.presoak_time], ['Wait', mem.wait_time], ['Brew', mem.brew_time]]
            logger.debug("Caught POST, Pump button.  brewing ... " + str(brew_plan))
            mem.brew_connection.send([time_stamp, brew_plan])

def scheduled_mode(old_mode):

    try:
        now = datetime.now()
        today = datetime.isoweekday(datetime.now())
        if today == 7:
            today = 0
        if mem.cache_day is None or mem.cache_day != today:  # refresh cache, reset flags, turn off heat
            logger.debug("scheduled_mode:  cache flush or new day.  resetting flags, turning off heat.")
            mem.cache_day = today
            mem.sched_flag_off = False
            mem.sched_flag_on = False
            with open("schedule.json") as f:
                my_schedule = json.load(f)  # t= time.strptime("00:05:42.244", "%H:%M:%S")
                mem.cache_start_time = my_schedule['days'][today]['time']['startTime']
                mem.cache_start_time = now.replace(hour=int(mem.cache_start_time.split(":")[0]), minute=int(mem.cache_start_time.split(":")[1]))
                mem.cache_end_time = my_schedule['days'][today]['time']['endTime']
                mem.cache_end_time = now.replace(hour=int(mem.cache_end_time.split(":")[0]), minute=int(mem.cache_end_time.split(":")[1]))
            return "off"
    
        if now < mem.cache_start_time:
            return old_mode
    
        if now > mem.cache_start_time and now < mem.cache_end_time:
            if mem.sched_flag_on:
                return old_mode
            else:  # start flag NOT set
                mem.sched_flag_on = True  # set flag
                logger.debug("scheduled_mode:  going AUTO")
                return "auto"
    
        if now > mem.cache_end_time:
            if mem.sched_flag_off:
                return old_mode
            else:  # end flag NOT set
                mem.sched_flag_off = True  # set end flag
                logger.debug("scheduled_mode:  going OFF")
                return "off"

    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))


class setup:
    def GET(self):
        
        try:
            iwlist_cmd = "iwlist wlan0 scanning | grep ESSID"
            proc = subprocess.Popen(iwlist_cmd, shell=True, stdout=subprocess.PIPE)
            myNwList = []
            while True:
                line = proc.stdout.readline()
                if line != '':
                    line = line[line.find('"') + 1 : len(line) - 2]
                    myNwList.append(line)
                else:
                    break
                
            return render.setup(myNwList)

        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))

    
    def POST(self):  # catch the inputs, put them into a config file, then call a shell script

        try:
            input = web.input()
            protocol = input.protocol
            ssid = input.ssid
            passwd = input.passwd
            if protocol == "personal":
                logger.debug("doing config for WPA personal.  ssid = " + ssid)
                with open('/var/www/setup/interfaces_default', 'r') as file:
                    lines = file.readlines()
                for idx, line in enumerate(lines):
                    if line.find("wpa-ssid") > -1:
                        lines[idx] = '  wpa-ssid "' + ssid + '"\n'
                    if line.find("wpa-psk") > -1:
                        lines[idx] = '  wpa-psk "' + passwd + '"\n'
                    if line.find("pre-up") > -1:
                        lines[idx] = '  # pre-up wpa_supplicant  \n'
                    if line.find("post-down") > -1:
                        lines[idx] = '  # post-down # wpa_supplicant  \n'
                with open('/var/www/setup/interfaces_default', 'w') as file:
                    file.writelines(lines)
                subprocess.call("/var/www/setup/default.sh 2>&1 >> /var/log/smartconnect.log", shell=True)  # , Shell=True
            elif protocol == "enterprise":
                mycert = web.input(ca_cert={})
                filename = ""
                filedir = '/etc/certs/'  # change this to the directory you want to store the file in.
                if 'ca_cert' in mycert:  # to check if the file-object is created
                    filepath = mycert.ca_cert.filename.replace('\\', '/')  # replaces the windows-style slashes with linux ones.
                    filename = filepath.split('/')[-1]  # splits the and chooses the last part (the filename with extension)
                    filename = filedir + filename  # put together with my path
                    fout = open(filename, 'w')  # creates the file where the uploaded file should be stored
                    fout.write(mycert.ca_cert.file.read())  # writes the uploaded file to the newly created file.
                    fout.close()  # closes the file, upload complete.
                    logger.debug("SETUP:  Enterprise - cert file written: " + filename)
                with open ('/var/www/setup/interfaces_default', 'r') as file:
                    lines = file.readlines()
                for idx, line in enumerate(lines):
                    if line.find("wpa-ssid") > -1:
                        lines[idx] = '  wpa-ssid "' + ssid + '"\n'
                    if line.find("wpa-psk") > -1:
                        lines[idx] = '# wpa-psk  \n'      # commenting out the PSK line for Enterprise, we're going to do wpa-supplicant instead
                    if line.find("pre-up") > -1:
                        lines[idx] = '  pre-up wpa_supplicant -B -Dwext -i wlan0 -c/etc/wpa_supplicant/wpa_supplicant.conf -f /var/log/wpa_supplicant.log \n'
                    if line.find("post-down") > -1:
                        lines[idx] = '  post-down killall -q wpa_supplicant \n'
                with open('/var/www/setup/interfaces_default', 'w') as file:
                    file.writelines(lines)
                with open ('/var/www/setup/wpa_supplicant.conf', 'r') as file:
                    lines = file.readlines()
                for idx, line in enumerate(lines):
                    if line.find(" ssid") > -1:         # need the trailing space so it doesnt squash scan_ssid field
                        lines[idx] = '    ssid="' + ssid + '"\n'
                    if line.find("key_mgmt") > -1:
                        lines[idx] = '    key_mgmt=' + input.key_mgmt + '\n'
                    if line.find("pairwise") > -1:
                        lines[idx] = '    pairwise=' + input.pairwise + '\n'
                    if line.find("group") > -1:
                        lines[idx] = '    group=' + input.group + '\n'
                    if line.find("psk") > -1:
                        lines[idx] = '    psk="' + input.psk + '"\n'
                    if line.find("eap") > -1:
                        lines[idx] = '    eap=' + input.eap + '\n'
                    if line.find("identity") > -1:
                        lines[idx] = '    identity="' + input.identity + '"\n'
                    if line.find("password") > -1:
                        lines[idx] = '    password="' + passwd + '"\n'
                    if line.find("ca_cert=") > -1 :         # need the trailing = so it doesn't squash ca_cert2 field
                        lines[idx] = '    ca_cert="' + filename + '"\n'
                with open('/var/www/setup/wpa_supplicant.conf', 'w') as file:
                    file.writelines(lines)                        
                subprocess.call("/var/www/setup/default.sh 2>&1 >> /var/log/smartconnect.log", shell=True)  # , Shell=True

        except:
            exc_type, exc_value, exc_traceback = sys.exc_info()
            logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))

    @staticmethod
    def check_connected():
        try:
            response = urllib2.urlopen('http://google.com', timeout=30)
            return True
        except urllib2.URLError as err: pass
        return False

    @staticmethod
    def smart_connect():
        logger.debug("Calling SmartConnect setup.sh")
        subprocess.call("/var/www/setup/setup.sh 2>&1 >> /var/log/smartconnect.log", shell=True)


class schedule:
    def GET(self):
        with open("schedule.json") as f:
            filecontents = json.load(f)
            return render.schedule(json.dumps(filecontents), str(datetime.now()))  # a JSON object (string) at this point

    def POST(self):
        data = web.data()  # web.input gives back a Storage < > thing
        mydata = json.loads(data)
        with open("schedule.json") as f:
            my_schedule = json.load(f)
            week = {'Sunday':0, 'Monday':1, 'Tuesday':2, 'Wednesday':3, 'Thursday':4, 'Friday':5, 'Saturday':6}
            my_schedule['days'][week[mydata['day']]]['time']['startTime'] = mydata['time']['startTime']
            my_schedule['days'][week[mydata['day']]]['time']['endTime'] = mydata['time']['endTime']
            tellHeatProc(None, True)  # FLUSH the cache so that the other process picks up the changes
        with open("schedule.json", "wb") as output_file:
            json.dump(my_schedule, output_file)

        return json.dumps("OK")

def tempdata():

#    try:
#        one_wire = mem.one_wire  # gets set below, on init      "/sys/bus/w1/devices/28-000004e0badb/w1_slave"
#        pipe = Popen(["cat", one_wire], stdout=PIPE)
#        result = pipe.communicate()[0]
#        result_list = result.split("=")  
  
#        try:
    if not mem.tsicObj:
        mem.tsicObj = tsic.TSIC()
        mem.tsicObj.openTSIC(gpio_tsic)
        
    temp_C = round(float(mem.tsicObj.getDegrees()/1000.00),2)
    #print str(temp_C)
    #        temp_C = float(result_list[-1]) / 1000  # temp in Celcius
    #    except ValueError:  # probably means we can't read the 1-wire sensor
    #        logger.warn('Could not get a value from 1-wire connector.  Using ' + one_wire )
    #        temp_C = 0
    return temp_C

    #except:
    #    exc_type, exc_value, exc_traceback = sys.exc_info()
    #    logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))

def catchButton(btn):  # GPIO

    try:
        time.sleep(0.05)        
        if GPIO.input(btn) != GPIO.HIGH:  # check to see if the input button is still high, protect against EMI false positive
            return
    
        if (GPIO.input(gpio_btn_heat_sig) == GPIO.HIGH & GPIO.input(gpio_btn_pump_sig) == GPIO.HIGH):  # both buttons pressed
            mem.lcd_connection.send(["Live long", "and prosper!", 1])  # easter egg
            mem.lcd_connection.send(["COFFGB", "", 0])  # easter egg
            logger.info("You found an easter egg!")
            return

	if btn == gpio_btn_steam_sig:
		now = time.time()
		if now - mem.time_steam_button_pressed < 1:
			mem.time_steam_button_pressed = now
			return
		mem.time_steam_button_pressed = now
		if param.mode == "off" or param.mode == "auto":
		    GPIO.output(gpio_btn_steam_led, GPIO.HIGH)
		    GPIO.output(gpio_btn_heat_led, GPIO.LOW)

		    logger.debug("cacheButton: telling Heat Proc STEAM (ON)")
		    tellHeatProc("steam")
		else:
		    GPIO.output(gpio_btn_steam_led, GPIO.LOW)
		    GPIO.output(gpio_btn_heat_led, GPIO.LOW)
  		    logger.debug("cacheButton: telling Heat Proc STEAM (OFF)")
		    tellHeatProc("off")

        elif btn == gpio_btn_heat_sig:
            
            now = time.time()
            if now - mem.time_heat_button_pressed < 1:
                mem.time_heat_button_pressed = now
                return
            mem.time_heat_button_pressed = now    
    
            if param.mode == "off":
                GPIO.output(gpio_btn_heat_led, GPIO.HIGH)  # this is a bit of a hack because the temp control also regulates the LED but putting it here gives better user experience.
		GPIO.output(gpio_btn_steam_led, GPIO.LOW)
                logger.debug("catchButton:  telling Heat Proc AUTO (ON) ")
                tellHeatProc("auto")
            else:
                GPIO.output(gpio_btn_heat_led, GPIO.LOW) 
		GPIO.output(gpio_btn_steam_led, GPIO.LOW)
                logger.debug("catchButton:  telling Heat Proc OFF")
                tellHeatProc("off")
    
        elif btn == gpio_btn_pump_sig:
            logger.debug("catchButton:  telling Brew Proc (toggle)")
            time_stamp = time.time()
            brew_plan = [['Presoak', mem.presoak_time], ['Wait', mem.wait_time], ['Brew', mem.brew_time]]
            mem.brew_connection.send([time_stamp, brew_plan])

    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))


class logdisplay:
    def GET(self):
        fp = open('/var/log/ispresso.log', 'rU')  # reading file from file path
        text = fp.read()  # no problem found till this line.
        fp.close()            
        return render.logdisplay(text)  # calling file_display.html 

def cleanUp():
    logger.info("Shutting down...")
    mem.lcd_connection.send(["COFFGB", "Shutting down", 0])
    execfile ('shutdown.py')

if __name__ == '__main__':
    
    try:

	logger_init()

        os.chdir("/var/www")
    
        #call(["modprobe", "w1-gpio"])
        #call(["modprobe", "w1-therm"])
        #call(["modprobe", "i2c-dev"])
        
        #base_dir = '/sys/bus/w1/devices/'
        #tsicObj = tsic.TSIC()
        #tsicObj.openTSIC(gpio_tsic)
        #mem.tsicObj = tsicObj
	#try:
	#    base_dir = glob.glob(base_dir + '28*')[0]
	#except:
	#    logger.error("EPIC FAIL!  1-Wire Temp sensor not found in " + base_dir)

        #mem.one_wire = base_dir + '/w1_slave'
        
        urls = ("/", "ispresso", "/settings", "settings", "/schedule", "schedule", "/advanced", "advanced", "/getstatus", "getstatus", "/logdisplay", "logdisplay", "/setup", "setup")
    
        render = web.template.render("/var/www/templates/")
    
        app = web.application(urls, globals()) 
    
        atexit.register(cleanUp)
    
        statusQ = Queue(2)
	statusQ2 = Queue(2)       
        parent_conn, child_conn = Pipe()
    
        lcd_parent_conn, lcd_child_conn = Pipe()
        mem.lcd_connection = lcd_parent_conn

        initialize()

        brew_parent_conn, brew_child_conn = Pipe()
        mem.brew_connection = brew_parent_conn
        
        GPIO.add_event_detect(gpio_btn_heat_sig, GPIO.RISING, callback=catchButton, bouncetime=250)  
        GPIO.add_event_detect(gpio_btn_pump_sig, GPIO.RISING, callback=catchButton, bouncetime=250)  # was RISING, at one point HIGH. who knows
        GPIO.add_event_detect(gpio_btn_steam_sig, GPIO.RISING, callback=catchButton, bouncetime=250)
        mem.heat_connection = parent_conn
        lcdproc = Process(name="lcdControlProc", target=lcdControlProc, args=(lcd_child_conn,))
        lcdproc.start()
        
        brewproc = Process(name="brewControlProc", target=brewControlProc, args=(brew_child_conn,))
        brewproc.start()

        p = Process(name="tempControlProc", target=tempControlProc, args=(param.mode, param.cycle_time, param.duty_cycle, \
                                            param.set_point, param.k_param, param.i_param, param.d_param, param.steam_temp, param.steam_cycle, statusQ, child_conn))
        p.start()

        app.add_processor(add_global_hook(parent_conn, statusQ, statusQ2))
        app.run()

    except KeyboardInterrupt:
        cleanUp()
        sys.exit()
        
    except:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        logger.error(''.join('!! ' + line for line in traceback.format_exception(exc_type, exc_value, exc_traceback)))

        cleanUp()
        sys.exit()

    if mem.scheduler_enabled:  # if program is just been started, set the mode according to the schedule, assuming schedule is ON
        tellHeatProc("auto") 



