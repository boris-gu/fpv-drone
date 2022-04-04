# fpv-drone
Модели дрона с ROS камерой для симулятора  Gazebo

Так как PX4-Autopilot быстро развивается, за основу взята стабильная версия 1.12.3.  
Клонировать:  
```bash
git clone --recurse-submodules --branch v1.12.3 https://github.com/PX4/PX4-Autopilot.git
```

Добавьте модель, запустив `setup.sh <PATH_TO_PX4>`. Этот скрипт внесет все необходимые изменения в PX4-Autopilot.

## Запуск
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd) && \
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_front_fpv
```
или
```bash
roslaunch px4 mavros_posix_sitl.launch vehicle:=iris_bottom_fpv
```


## Настройка
Настройка дрона производится в программе QGroundControl.  
*Максимальная горизонтальная скорость в режимах POSCTL, OFFBOARD, AUTO  
Необходимо снизить для более точного позиционирования*  
**MPC_XY_VEL_MAX = 1 м/с**  

## Данные с дрона
Дрон публикует в топики ROS данные с камеры.

Для просмотра изображения с камеры запустите команду `rqt_image_view` и выберите
/iris_front_fpv/usb_cam/image_raw или /iris_bottom_fpv/usb_cam/image_raw.