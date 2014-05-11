#!/bin/sh -x

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

cd "/home/ros/owr/nmea/src/nmea_navsat_driver"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
/usr/bin/env \
    PYTHONPATH="/home/ros/owr/nmea/install/lib/python2.7/dist-packages:/home/ros/owr/nmea/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ros/owr/nmea/build" \
    "/usr/bin/python" \
    "/home/ros/owr/nmea/src/nmea_navsat_driver/setup.py" \
    build --build-base "/home/ros/owr/nmea/build/nmea_navsat_driver" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ros/owr/nmea/install" --install-scripts="/home/ros/owr/nmea/install/bin"
