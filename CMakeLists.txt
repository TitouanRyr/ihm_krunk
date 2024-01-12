cmake_minimum_required(VERSION 3.0.2)
project(ihm_krunk)

## Trouver les packages nécessaires
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

## Spécifier les packages nécessaires
catkin_package(
  CATKIN_DEPENDS rospy std_msgs
)

## Ajouter les exécutables Python
catkin_install_python(PROGRAMS
  src/launch_IHM.py
  src/Codes/Camera/acquisition_camera.py
  src/Codes/Camera/launch_camera.py
  src/Codes/Lidar/acquisition_lidar.py
  src/Codes/Lidar/launch_lidar.py
  src/Codes/Light/launch_light.py
  src/Codes/Screens/graph.py
  src/Codes/Screens/picture.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Installer les fichiers de lancement
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
