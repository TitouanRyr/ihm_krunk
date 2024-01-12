#!/bin/bash

# Script d'installation pour le package ROS

# Compilation du package
source /opt/ros/$ROS_DISTRO/setup.bash
cd /home/oem/kuhn_ws/
catkin_make

# Configuration de l'environnement ROS
source devel/setup.bash

# Spécifiez le répertoire de base (utilisez le chemin absolu)
BASE_PATH="/home/oem/kuhn_ws/src/ihm_krunk/exe"

# Création du script de lancement
echo "#!/bin/bash" > $BASE_PATH/launch_ihm_exe
echo "" >> $BASE_PATH/launch_ihm_exe
echo "# Script de lancement pour mon IHM ROS" >> $BASE_PATH/launch_ihm_exe
echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> $BASE_PATH/launch_ihm_exe
echo "cd /home/oem/kuhn_ws" >> $BASE_PATH/launch_ihm_exe
echo "source devel/setup.bash" >> $BASE_PATH/launch_ihm_exe
echo "" >> $BASE_PATH/launch_ihm_exe
echo "roslaunch ihm_krunk start_ihm.launch" >> $BASE_PATH/launch_ihm_exe

# Donner les autorisations d'exécution au script de lancement
chmod +x $BASE_PATH/launch_ihm_exe

# Création du fichier .desktop
DESKTOP_PATH="$BASE_PATH/config_launch_ihm_exe.desktop"
echo "[Desktop Entry]" > $DESKTOP_PATH
echo "Name=IHM Krunk" >> $DESKTOP_PATH
echo "Exec=gnome-terminal -- /bin/bash -i -c '$BASE_PATH/launch_ihm_exe; exec bash'" >> $DESKTOP_PATH
echo "Icon=$BASE_PATH/icon.png" >> $DESKTOP_PATH
echo "Type=Application" >> $DESKTOP_PATH
echo "Categories=Utility;" >> $DESKTOP_PATH

# Donner les autorisations d'exécution au fichier .desktop
chmod +x $DESKTOP_PATH

# Copie du fichier .desktop dans le répertoire des applications système
sudo cp $DESKTOP_PATH /usr/share/applications/

echo "Installation terminée. Vous pouvez maintenant utiliser le raccourci dans votre menu d'applications pour démarrer votre IHM ROS."

