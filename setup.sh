#!/bin/bash
# https://discuss.px4.io/t/create-custom-model-for-sitl/6700/17
# https://github.com/willinum/SPEC21/wiki/custom-model-for-PX4-sitl-GAZEBO-simulation
RED='\033[1;31m'
YELLOW='\033[1;33m'
NOCOLOR='\033[0m'

if [ -z $1 ] || [ $1 == "--help" ]; then
    echo "Usage: setup.sh PATH_TO_PX4"
    echo "  For example:"
    echo "  setup.sh /home/user/PX4-Autopilot"
elif [ ! -d $1 ]; then
    echo -en "${RED}[ERR]: ${NOCOLOR}"
    echo "Directory $1 not found"
else
    SCRIPT=$(realpath $0)
    SCRIPTPATH=$(dirname $SCRIPT)

    #0 Remove file of the previous setup
    MODELS=$1/Tools/sitl_gazebo/models
    rm -rf $1/build
    rm -rf $MODELS/fpv_cam_new $MODELS/iris_front_fpv
 
    #1 Create a model under Tools/sitl_gazebo/models
    if [ -d $MODELS ]; then
        cp -r $SCRIPTPATH/models/* $MODELS
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $MODELS not found"
    fi

    #3 Create an airframe file under ROMFS/px4fmu_common/init.d-posix/airframes
    AIRFRAMES=$1/ROMFS/px4fmu_common/init.d-posix/airframes
    if [ -d $AIRFRAMES ]; then
        cp $SCRIPTPATH/airframes/* $AIRFRAMES
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "Directory $AIRFRAMES not found"
    fi

    #4 Add the airframe name to the file platforms/posix/cmake/sitl_target.cmake
    FILE_SITL_TARGET=$1/platforms/posix/cmake/sitl_target.cmake
    if [ -f $FILE_SITL_TARGET ]; then
        lineNum=$(grep -n "set(models" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum+1))
        gardDroneOK=$(grep -n "iris_front_fpv" $FILE_SITL_TARGET | head -n 1 | cut -d: -f1)
        if [ -z $gardDroneOK ]; then
            sed -i "${lineNum}i \\\tiris_front_fpv" $FILE_SITL_TARGET
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_front_fpv already contains in sitl_target.cmake"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_SITL_TARGET not found"
    fi

    #5 Add the airframe name to the file ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    FILE_CMAKELISTS=$1/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
    if [ -f $FILE_CMAKELISTS ]; then
        lineNum=$(grep -n "px4_add_romfs_files(" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        lineNum=$((lineNum+1))
        gardDroneOK=$(grep -n "220226_iris_front_fpv" $FILE_CMAKELISTS | head -n 1 | cut -d: -f1)
        if [ -z $gardDroneOK ]; then
            sed -i "${lineNum}i \\\t220226_iris_front_fpv" $FILE_CMAKELISTS
        else
            echo -en "${YELLOW}[WARNING]: ${NOCOLOR}"
            echo "iris_front_fpv already contains in CMakeLists.txt"
        fi
    else
        echo -en "${RED}[ERR]: ${NOCOLOR}"
        echo "File $FILE_CMAKELISTS not found"
    fi

    echo "Setup completed"
fi