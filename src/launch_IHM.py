#!/usr/bin/env python

import subprocess
import sys
import time
import threading
import rospy
from PyQt5.QtWidgets import *
from PyQt5.QtGui import QIcon
from PyQt5.QtCore import QCoreApplication

# Ajoutez le chemin absolu du répertoire Codes à votre PYTHONPATH
sys.path.append('/home/oem/kuhn_ws/src/ihm_krunk/src')

from Codes.Lidar.acquisition_lidar import acquisition_r2000
from Codes.Camera.acquisition_camera import acquisition_camera
from Codes.Lidar.launch_lidar import launch_lidar, kill_lidar
from Codes.Camera.launch_camera import launch_camera, kill_camera
from Codes.Light.launch_light import arduino_init

class InterfaceGraphique(QWidget):
    def __init__(self):
        super().__init__()
        arduino_init(self)
        self.initUI()
        self.t = None

    def initUI(self):
        # Création des boutons
        self.bouton_1 = QPushButton("ON", self)
        self.bouton_1.setIcon(QIcon.fromTheme("system-run"))

        self.bouton_2 = QPushButton("OFF", self)
        self.bouton_2.setIcon(QIcon.fromTheme("system-shutdown"))

        self.bouton_3 = QPushButton("Play", self)
        self.bouton_3.setIcon(QIcon.fromTheme("media-playback-start"))

        self.bouton_4 = QPushButton("Stop", self)
        self.bouton_4.setIcon(QIcon.fromTheme("media-playback-stop"))

        self.bouton_5 = QPushButton("Lidar", self)
        self.bouton_5.setIcon(QIcon.fromTheme("system-search"))

        self.bouton_6 = QPushButton("Camera", self)
        self.bouton_6.setIcon(QIcon.fromTheme("camera-web"))

        self.bouton_7 = QPushButton("Réglage", self)
        self.bouton_7.setIcon(QIcon.fromTheme("applications-system"))

        # Création des blocs pour l'image, le graphique et le texte
        self.text_browser = QTextBrowser()
        self.text_browser.setFixedHeight(50)

        self.graph_block = QLabel("Image Block", self)
        self.graph_block.setStyleSheet("background-color: lightgray; padding: 10px;")

        self.text_block = QLabel("Graph Block", self)
        self.text_block.setStyleSheet("background-color: lightgray; padding: 10px;")

        # Mise en place des layouts
        layout_buttons = QVBoxLayout()
        layout_buttons.addWidget(self.bouton_1)
        layout_buttons.addWidget(self.bouton_2)
        layout_buttons.addWidget(self.bouton_3)
        layout_buttons.addWidget(self.bouton_4)
        layout_buttons.addWidget(self.bouton_5)
        layout_buttons.addWidget(self.bouton_6)
        layout_buttons.addWidget(self.bouton_7)

        layout_blocks = QVBoxLayout()
        layout_blocks.addWidget(self.text_browser)
        layout_blocks.addWidget(self.graph_block)
        layout_blocks.addWidget(self.text_block)

        main_layout = QHBoxLayout()
        main_layout.addLayout(layout_buttons, 1)  # Boutons prennent 1/5
        main_layout.addLayout(layout_blocks, 4)  # Blocks prennent 4/5

        self.setLayout(main_layout)

        # Connexion des boutons aux fonctions d'action
        self.bouton_1.clicked.connect(self.action_bouton_on)
        self.bouton_2.clicked.connect(self.action_bouton_off)
        self.bouton_3.clicked.connect(self.action_bouton_play)
        self.bouton_4.clicked.connect(self.action_bouton_stop)
        self.bouton_5.clicked.connect(self.action_bouton_lidar)
        self.bouton_6.clicked.connect(self.action_bouton_camera)
        self.bouton_7.clicked.connect(self.action_bouton_reglage)

        self.setGeometry(100, 100, 800, 400)
        self.setWindowTitle('IHM_Krunk')
        self.show()

    # Fonctions d'action pour les boutons (à personnaliser selon vos besoins)

    def node_exists(self, node_name):
        time.sleep(1)
        try:
            # Obtenez la liste des sujets publiés
            topics = rospy.get_published_topics()

            # Parcourez la liste des sujets pour vérifier si le nœud existe
            for topic, _ in topics:
                if topic.startswith("/" + node_name):
                    return True

            # Si le nœud n'est pas trouvé
            return False

        except rospy.ROSException:
            return False
    def update_text_browser(self, message):
        self.text_browser.append(message)
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())

    def action_bouton_on(self):
        self.update_text_browser("Lancement Lidar et Caméra : ")
        QCoreApplication.processEvents()
        launch_lidar()
        launch_camera()
        if self.node_exists("r2000_node") and self.node_exists("pylon_camera_node"):
            self.update_text_browser("Lancement ok")
        elif self.node_exists("r2000_node"):
            self.update_text_browser("Lidar ok")
        elif self.node_exists("pylon_camera_node"):
            self.update_text_browser("Camera ok")
        else:
            self.update_text_browser("Echec")

    def action_bouton_off(self):
        self.update_text_browser("Arret Lidar et Caméra : ")
        QCoreApplication.processEvents()
        kill_lidar()
        kill_camera()
        if self.node_exists("r2000_node") and self.node_exists("pylon_camera_node"):
            self.update_text_browser("Echec")
        elif self.node_exists("r2000_node"):
            self.update_text_browser("Echec lidar")
        elif self.node_exists("pylon_camera_node"):
            self.update_text_browser("Echec camera")
        else:
            self.update_text_browser("Arret")

    def action_bouton_play(self):
        self.start_value = True
        t = threading.Thread(target=self.boucle_acquisition)
        t.start()

    def boucle_acquisition(self):
        self.update_text_browser("lancement Acquisition :")
        numero = 1
        while self.start_value:
            self.update_text_browser(f"Acquisition {numero}")
            subprocess.Popen("/opt/ros/noetic/bin/rosbag record --duration 1 -o /home/oem/kuhn_ws/src/ihm_krunk/src/acquisitions/acquisition_globale/ /scan /pylon_camera_node/image_raw", shell=True)
            time.sleep(1)
            numero += 1

    def action_bouton_stop(self):
        self.update_text_browser("Arret acquisition")
        self.start_value = False
        if self.t is not None:
            self.t.join()

    def action_bouton_lidar(self):
        self.update_text_browser("Lancement acquisition lidar")
        acquisition_r2000()

    def action_bouton_camera(self):
        self.update_text_browser("Lancement acquisition camera")
        # launch_light()
        acquisition_camera()
        # kill_light()

    def action_bouton_reglage(self):
        self.update_text_browser("Pas encore en fonctionnement")


if __name__ == '__main__':
    app = QApplication(sys.argv)
    ex = InterfaceGraphique()
    sys.exit(app.exec_())
