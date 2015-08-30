#! /bin/sh
### BEGIN INIT INFO
# Provides: tomcat
# Required-Start: $remote_fs $syslog
# Required-Stop: $remote_fs $syslog
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
# Short-Description: Tomcat
# Description: This file starts and stops Tomcat server
# 
### END INIT INFO


case "$1" in
 start)
   su bluenuc -c /home/bluenuc/owr_software/rover/start/startup
   ;;
 stop)
   echo "Fail"
   sleep 10
   ;;
 restart)
   su bluenuc -c "pkill ros*"
   su bluenuc -c "pkill tmux"
   su bluenuc -c "pkill owr*"
   sleep 20
   su bluenuc -c /home/bluenuc/owr_software/rover/start/startup
   ;;
 *)
   echo "Usage: procmon {start|stop|restart}" >&2
   exit 3
   ;;
esac
