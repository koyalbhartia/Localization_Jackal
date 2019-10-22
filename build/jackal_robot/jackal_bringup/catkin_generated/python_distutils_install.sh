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
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/koko/Desktop/Localization_Jackal/src/jackal_robot/jackal_bringup"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/koko/Desktop/Localization_Jackal/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/koko/Desktop/Localization_Jackal/install/lib/python2.7/dist-packages:/home/koko/Desktop/Localization_Jackal/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/koko/Desktop/Localization_Jackal/build" \
    "/usr/bin/python" \
    "/home/koko/Desktop/Localization_Jackal/src/jackal_robot/jackal_bringup/setup.py" \
    build --build-base "/home/koko/Desktop/Localization_Jackal/build/jackal_robot/jackal_bringup" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/koko/Desktop/Localization_Jackal/install" --install-scripts="/home/koko/Desktop/Localization_Jackal/install/bin"
