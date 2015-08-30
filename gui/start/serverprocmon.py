#!/usr/bin/python3.4

import sys
import os
import subprocess
import time

import asyncio
import websockets

import _thread
import json
import socket

from collections import deque
from threading import Lock

speakLock = Lock()
def Speak(msg):
  speakLock.acquire(True)
  p = subprocess.Popen("pico2wave -w /tmp/status.wav \"" + msg + "\" && aplay /tmp/status.wav", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  p.wait()
  speakLock.release()

statusSpeakLock = Lock()
def StatusSpeak(msg):
  statusSpeakLock.acquire(True)
  p = subprocess.Popen("pico2wave -w /tmp/nbstatus.wav \"" + msg + "\" && aplay /tmp/nbstatus.wav", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  p.wait()
  statusSpeakLock.release()



def StartStopAll():
  Speak("Mooooooo. moooooo. moooooo. ")
  global errorCount
  if runningCount > 0 or errorCount > 0: 
    # stop all processes
    errorCount = 0
    for proc in config["monitor"]:
      # skip manual processes
      if 'mode' in proc and proc['mode'] == 'manual':
        continue

      if proc["status"] in ["Starting", "Started", "Unknown", "Finished"]:
        StartStop(proc["num"])
      elif proc["status"] in ["Respawning","Queued"]:
        proc["status"] = "Stopped"
  else:
    # start all processes
    autoDelay = 0
    for proc in config["monitor"]:
      # skip manual processes
      if 'mode' in proc and proc['mode'] == 'manual':
        continue

      if proc['status'] in ['', 'Stopped', 'Terminated', 'Respawning']:
        if "delay" in proc:
          proc["status"] = "Queued"
          proc["queuetill"] = time.time() + proc['delay']
        else:
          if autoDelay == 0:
            StartStop(proc['num'])
          else:
            proc["status"] = "Queued"
            proc["queuetill"] = time.time() + autoDelay
          autoDelay = autoDelay + 1
  return


def GetProc(pnum):
  i = 0
  for proc in config["monitor"]:
    i = i + 1
    if i == pnum:
      return proc

  return None

def GetProcByName(name):
  for proc in config["monitor"]:
    if proc['name'] == name:
      return proc;
 
  return None


def AreDependenciesOk(proc):
  ok = True;
  if 'deps' in proc:
    for d in proc['deps']:
      dproc = GetProcByName(d)
      if dproc['pid'] == "":
        ok = False
        break;

  return ok
  

def StartStop(pnum, allow_start=True, allow_stop=True):
  proc = GetProc(pnum)
  StartStopProc(proc, allow_start, allow_stop)

def StartStopProc(proc, allow_start, allow_stop):
  pid = proc['pid']
  # terminate a starting process
  if pid == "" and proc["status"] == "Starting" and 'startedpid' in proc and proc['startedpid'] is not None:
    if allow_stop:
      p = subprocess.Popen("/bin/kill -SIGTERM {0} `pgrep -P {0}`".format(proc['startedpid']), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      proc['status'] = "Stopping"
  # terminate a running process (hard)
  elif pid != "" and proc['status'] == "Stopping":
    if allow_stop:
      p = subprocess.Popen("/bin/kill -SIGKILL {0} `pgrep -P {0}`".format(pid), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  # terminate a running process (nice)
  elif pid != "":
    if allow_stop:
      p = subprocess.Popen("/bin/kill -SIGINT {0} `pgrep -P {0}`".format(pid), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      proc['status'] = "Stopping"
  else:
    if allow_start:
    # start a process
      args = proc["startup"]
      p = subprocess.Popen("stdbuf -oL {0} >> /tmp/procmon.log 2>&1".format(args), shell=True, preexec_fn=os.setsid)
      proc['status'] = "Starting"
      proc['startedpid'] = p.pid

  return 
  
def IsPIDAlive(pid):
  return os.path.exists("/proc/{0}".format(pid))

def GetPID(proc):
  args = proc["pidp"]
  p = subprocess.Popen(args, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  rc = p.wait()
  line = p.stdout.readline().decode('ascii').strip()
  if line != "" and os.path.exists("/proc/{0}".format(line)) == False:
    line = ""
  return line

def PollSystem():
  global runningCount, errorCount
  runningCount = 0
  for proc in config["monitor"]:
    proc['pid'] = GetPID(proc)
    
  
    if "status" in proc:
      # if the process is dead
      if proc['pid'] == "":
        if proc['status'] == "Stopping":
          proc['status'] = "Stopped"
          Speak(proc['name'] + " stopped")

        elif proc['status'] == "Started":
          if 'mode' in proc and proc['mode'] in ['oneshot','manual']:
            proc['status'] = "Finished"
          else:
            proc['status'] = "Terminated"
            errorCount = errorCount + 1
            Speak(proc['name'] + " terminated")

        elif proc['status'] == 'Starting' and IsPIDAlive(proc['startedpid']) == False:
          proc['status'] = "Terminated"
          errorCount = errorCount + 1
          Speak(proc['name'] + " aborted")

        elif respawn and proc['status'] == "Terminated":
          proc['status'] = "Respawning"

        elif proc['status'] == "Respawning":
          StartStop(proc['num'])

        elif proc['status'] == 'Queued' and time.time() > proc['queuetill']:
          StartStop(proc['num'])

      # if the process is alive
      else:
        if proc['status'] == "Starting":
          proc['status'] = "Started"
          Speak(proc['name'] + " started")

        if 'mode' not in proc or proc['mode'] != 'manual':
          runningCount = runningCount + 1
    else:
      if(proc['pid'] == ""):
        proc['status'] = ''
      else:
        proc['status'] = 'Unknown'
        runningCount = runningCount + 1

# --------------------------------------------------------------------------------------------

config = {}
respawn = False
errorCount = 0
runningCount = 0

# get configuration file
confname = "procmon.conf"
if len(sys.argv) == 2:
  confname = sys.argv[1]

# load monitored processes
exec(compile(open(confname).read(), confname, 'exec'), config)
i = 0

# assign process number and button
for proc in config["monitor"]:
  i = i + 1
  proc["num"] = i
  if i < 10:
    proc["key"] = ord('0') + i
  else:
    proc["key"] = ord('a') - 10 + i


# launch status thread
def StatusThread(statuses, checkfuncs):
  time.sleep(1)
  while 1:
    for stat in statuses:
      if AreDependenciesOk(stat):
          args = stat['cmd']
          # print(args)
          p = subprocess.Popen(args, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
          stat['rc'] = p.wait()
          line = p.stdout.readline().decode('ascii').strip()
          stat['line'] = line

          if ('rc' in stat and stat['rc'] == 0 and 'line' in stat and stat['line'] != ''):
              if stat['name'] in checkfuncs:
                  stat['rc'] = 0 if checkfuncs[stat['name']](stat['line']) else 1

          if ('rc' in stat and stat['rc'] != 0 or 'line' not in stat or stat['line'] == '') and ('mode' not in stat or stat['mode'] != 'silent'):
              StatusSpeak(stat['alert'] if 'alert' in stat else stat['name'])

          time.sleep(1)

    time.sleep(1)

if "status" in config:
  _thread.start_new_thread(StatusThread, (config["status"],config['checkfuncs']))

#######################################################

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) # UDP

def UDPHandoff(msg):
    UDP_IP = "127.0.0.1"
    UDP_PORT = 8750
    MESSAGE = json.dumps(msg)

    sock.sendto(bytes(MESSAGE, "utf-8"), (UDP_IP, UDP_PORT))


sensorinfo = {}
mapinfo = {}
log = deque([], 25)

@asyncio.coroutine
def handler(msg):
    if msg == "ping":
        UDPHandoff({'r': 'ping'})
        return json.dumps({'r':'ping'})
    elif msg == "plist":
        return json.dumps({'r':'plist', 'proc': config['monitor']})
    elif msg == "statuslist":
        return json.dumps({'r':'statuslist', 'status': config['status']})
    else:
        obj = json.loads(msg)
        if 'r' in obj:
            if obj['r'] == 'startproc':
                c = obj['key']
                if c >= ord('1') and c <= ord('9'):
                    StartStop(c - ord('0'), allow_stop=False)
                elif c >= ord('a') and c <= ord('m'):
                    StartStop(c - ord('a')+10, allow_stop=False)
                return json.dumps({'r':'plist', 'proc': config['monitor']})
            elif obj['r'] == 'stopproc':
                c = obj['key']
                if c >= ord('1') and c <= ord('9'):
                    StartStop(c - ord('0'), allow_start=False)
                elif c >= ord('a') and c <= ord('m'):
                    StartStop(c - ord('a')+10, allow_start=False)
                return json.dumps({'r':'plist', 'proc': config['monitor']})
            elif obj['r'] == 'startstopallproc':
                StartStopAll()
                return json.dumps({'r':'plist', 'proc': config['monitor']})
            elif obj['r'] in ['drive', 'waypoint']:
                UDPHandoff(obj)
                return json.dumps({'r':'drive', 'linear': 0, 'angular': 0})
            elif obj['r'] == 'sensor':
                return json.dumps({'r':'sensor', 'sensorinfo': sensorinfo})
            elif obj['r'] == 'map':
                return json.dumps({'r':'map', 'mapinfo': mapinfo})
            elif obj['r'] == 'log':
                loginfo = []
                for item in log:
                    loginfo.append(item)
                return json.dumps({'r':'log', 'loginfo': loginfo})


    return None
    #return json.dumps({'r':''})


@asyncio.coroutine
def onconnect(websocket, path):
  while True:
    message = yield from websocket.recv()
    if message is None:
      break
    print("< {}".format(message))

    message = yield from handler(message)
    if not websocket.open:
      break

    if message is not None:
      print("> {}".format(message))
      yield from websocket.send(message)


start_ws_server = websockets.serve(onconnect, None, 8765)

def asynctick(eventloop):
  print("TICK")
  PollSystem()
  eventloop.call_later(5, asynctick, eventloop)


####################

class UDPServer:
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        message = data.decode()
        # print('<< %r' % message)
        obj = json.loads(message)

        if obj['t'] == '/rosout':
            log.append(obj['data'])

        if 'id' in obj and obj['id'] in ['gps', 'path', 'mappos', 'waypoint', 'origin_fix']:
            mapinfo[obj['id']] = obj['data']

        if 'id' in obj and obj['id'] in ['laserscan','barrels','lines','battery','cmdvel','estop']:
            if obj['id'] == 'laserscan':
                sensorinfo['barrels'] = []
                sensorinfo['lines'] = []
            sensorinfo[obj['id']] = obj['data']

        #print('Send %r to %s' % (message, addr))
        #self.transport.sendto(data, addr)

loop = asyncio.get_event_loop()

start_udp_server = loop.create_datagram_endpoint(UDPServer, local_addr=('127.0.0.1', 8751))


loop.call_later(0, asynctick, loop)
loop.run_until_complete(start_ws_server)

loop.run_until_complete(start_udp_server)
loop.run_forever()

