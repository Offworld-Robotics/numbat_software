#!/usr/bin/python

import curses
import sys
import os
import subprocess
import time

def StartStopAll():
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
  

def StartStop(pnum):
  proc = GetProc(pnum)
  StartStopProc(proc)

def StartStopProc(proc):
  pid = proc['pid']
  # terminate a starting process
  if pid == "" and proc["status"] == "Starting" and 'startedpid' in proc and proc['startedpid'] is not None:
    p = subprocess.Popen("/bin/kill -SIGTERM {0} `pgrep -P {0}`".format(proc['startedpid']), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    proc['status'] = "Stopping"
  # terminate a running process (hard)
  elif pid != "" and proc['status'] == "Stopping":
    p = subprocess.Popen("/bin/kill -SIGKILL {0} `pgrep -P {0}`".format(pid), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
  # terminate a running process (nice)
  elif pid != "":
    p = subprocess.Popen("/bin/kill -SIGINT {0} `pgrep -P {0}`".format(pid), shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    proc['status'] = "Stopping"
  else:
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
  line = p.stdout.readline().strip()
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

        elif proc['status'] == "Started":
          if 'mode' in proc and proc['mode'] in ['oneshot','manual']:
            proc['status'] = "Finished"
          else:
            proc['status'] = "Terminated"
            errorCount = errorCount + 1

        elif proc['status'] == 'Starting' and IsPIDAlive(proc['startedpid']) == False:
          proc['status'] = "Terminated"
          errorCount = errorCount + 1

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

        if 'mode' not in proc or proc['mode'] != 'manual':
          runningCount = runningCount + 1
    else:
      if(proc['pid'] == ""):
        proc['status'] = ''
      else:
        proc['status'] = 'Unknown'
        runningCount = runningCount + 1

def Display():
  stdscr.addstr(1, 3, "0)                            ")
  if runningCount == 0:
    stdscr.addstr(1, 7, " Auto System Startup   ", curses.color_pair(1))
  else:
    stdscr.addstr(1, 7, " Auto System Shutdown  ", curses.color_pair(2))

  if errorCount > 0:
    stdscr.addstr(1, 30, " {0} ".format(errorCount), curses.color_pair(3))


  i = 0
  for proc in config["monitor"]:
    i = i + 1
    stdscr.addstr(i + 2, 5, "{0}) {1}".format(chr(proc['key']), proc["name"]))

    if proc['pid'] == "":
      stdscr.addstr(i + 2, 31, " OFF ", curses.color_pair(2))
    else:
      stdscr.addstr(i + 2, 31, "{0}".format(proc['pid']).center(5), curses.color_pair(1) if AreDependenciesOk(proc) else curses.color_pair(4))


    if "status" in proc:
      stdscr.addstr(i + 2, 40, "{0}    ".format(proc['status']))


  stdscr.addstr(i + 4, 3, "q) Quit   r) Respawn")
  stdscr.addstr(i + 4, 24, " Y " if respawn else " N ", curses.color_pair(1 if respawn else 2))

  # draw status
  i = 0
  if "status" in config:
    for stat in config["status"]:
      i = i + 1
      stdscr.addstr(i + 2, 52, "{0}".format(stat['name']))
      if 'rc' in stat and stat['rc'] != 0 or 'line' not in stat or stat['line'] == '':
        stdscr.addstr(i + 2, 65, " ERR ", curses.color_pair(2))
      else:
        stdscr.addstr(i + 2, 65, stat['line'] + "      ")
      
  stdscr.refresh()

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
execfile(confname, config)
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
def StatusThread(statuses):
  while 1:
    for stat in statuses:
      args = stat['cmd']
      p = subprocess.Popen(args, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
      stat['rc'] = p.wait()
      line = p.stdout.readline().strip()
      stat['line'] = line

    time.sleep(5)

import thread
if "status" in config:
  thread.start_new_thread(StatusThread, (config["status"],))





stdscr = curses.initscr()
curses.start_color()
curses.use_default_colors()
curses.noecho()
curses.cbreak()
curses.curs_set(0)

curses.init_pair(1, curses.COLOR_WHITE, curses.COLOR_GREEN)
curses.init_pair(2, curses.COLOR_WHITE, curses.COLOR_RED)
curses.init_pair(3, curses.COLOR_WHITE, curses.COLOR_YELLOW)
curses.init_pair(4, curses.COLOR_WHITE, curses.COLOR_CYAN)


def MainLoop(args):
  global errorCount, respawn
  curses.halfdelay(20)
  while 1:
    PollSystem()
    try:
      Display()
    except curses.error:
      0 # destroy the exception.  we're not interested in display errors

    c = stdscr.getch()
  
    try:

      if c == ord('0'):
        StartStopAll()
      elif c == ord('q'):
        break
      elif c == ord('r'):
        respawn = not respawn
      elif c >= ord('1') and c <= ord('9'):
        StartStop(c - ord('0'))
      elif c >= ord('a') and c <= ord('m'):
        StartStop(c - ord('a')+10)
    except:
      errorCount = errorCount + 1

curses.wrapper(MainLoop)

# if we're using tmux, kill the server
if 'TMUX' in os.environ or 'TMUXPROCMON' in os.environ:
  p = subprocess.Popen("tmux kill-session -t prm2", shell=True)


curses.nocbreak()
stdscr.keypad(0)
curses.echo()
curses.endwin()
