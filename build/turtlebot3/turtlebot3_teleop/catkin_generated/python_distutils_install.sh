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

echo_and_run cd "/home/ebrahim/TURTLEBOT3_controller/src/turtlebot3/turtlebot3_teleop"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ebrahim/TURTLEBOT3_controller/install/lib/python3/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ebrahim/TURTLEBOT3_controller/install/lib/python3/dist-packages:/home/ebrahim/TURTLEBOT3_controller/build/lib/python3/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ebrahim/TURTLEBOT3_controller/build" \
    "/usr/bin/python3" \
    "/home/ebrahim/TURTLEBOT3_controller/src/turtlebot3/turtlebot3_teleop/setup.py" \
     \
    build --build-base "/home/ebrahim/TURTLEBOT3_controller/build/turtlebot3/turtlebot3_teleop" \
    install \
    --root="${DESTDIR-/}" \
    --install-layout=deb --prefix="/home/ebrahim/TURTLEBOT3_controller/install" --install-scripts="/home/ebrahim/TURTLEBOT3_controller/install/bin"
