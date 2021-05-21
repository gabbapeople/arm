#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ubuntu/ARM/ros_arm/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ubuntu/ARM/ros_arm/install/lib/python3/dist-packages:/home/ubuntu/ARM/ros_arm/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ubuntu/ARM/ros_arm/build" \
    "/usr/bin/python3" \
    "/home/ubuntu/ARM/ros_arm/src/rqt_ez_publisher/setup.py" \
    egg_info --egg-base /home/ubuntu/ARM/ros_arm/build/rqt_ez_publisher \
    build --build-base "/home/ubuntu/ARM/ros_arm/build/rqt_ez_publisher" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ubuntu/ARM/ros_arm/install" --install-scripts="/home/ubuntu/ARM/ros_arm/install/bin"
