import os
import rosbag
import glob
import subprocess
import time
import sys
import serial
import tkinter as tk
import threading
import numpy as np
import cv2
from matplotlib import cm
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
from PyQt5.QtGui import QImage, QPixmap
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from code_graphic_V3 import generate_3d_plot_data
from control_arduino import ArduinoController
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ROSLauncherApp(QWidget):
    def __init__(self):
        super().__init__()

        self.arduino_port = "/dev/ttyACM0"
        self.ser = None  # Initialisation à None pour éviter toute erreur si le port série n'est pas ouvert

        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
        except serial.SerialException as e:
            print(f"Erreur : {e}")
            print("Arduino non connecté. L'application se poursuivra sans communication avec l'Arduino.")

        # Le reste de votre initialisation ici
        self.flash_state = False  # État du flash
        self.setWindowTitle("IHM Krunk")

        # Calcul des dimensions pour un rapport d'aspect 4:3
        screen_width = 800
        screen_height = int(screen_width * 3 / 4)  # 4:3 aspect ratio
        self.setGeometry(100, 100, screen_width, screen_height)

        # Créez une zone de texte pour les instructions
        self.text_browser = QTextBrowser()
        self.text_browser.setFixedHeight(50)

        # Créez une étiquette pour afficher l'image traitée
        self.image_label = QLabel()
        self.image_label.setFixedSize(350, 300)

        # Créez des boutons pour lancer des commandes ROS
        self.launch_button_1 = QPushButton("Play")
        self.launch_button_2 = QPushButton("stop")
        self.launch_button_3 = QPushButton("rviz")
        self.launch_button_4 = QPushButton("r2000")
        self.launch_button_5 = QPushButton("Photo")
        self.launch_button_6 = QPushButton("Lumière")

        # Créez des boutons pour arrêter les commandes ROS
        self.stop_button_1 = QPushButton("Arrêter Photo")
        self.stop_button_2 = QPushButton("Arrêter r2000")
        self.stop_button_3 = QPushButton("Arrêter Lumière")
        self.stop_button_4 = QPushButton("video")

        # Ajoutez un bouton "Clear" pour effacer le contenu de text_browser
        self.clear_button = QPushButton("Clear")
        self.clear_button.setStyleSheet(
            "QPushButton { background-color: #E74C3C; color: white; font-size: 18px; border: 2px solid #E74C3C; border-radius: 10px; }")
        self.clear_button.clicked.connect(self.clear_text_browser)

        # Utilisez un style CSS pour styliser les boutons de lancement
        button_style = "QPushButton { background-color: #4CAF50; color: white; font-size: 18px; border: 2px solid #4CAF50; border-radius: 10px; }"
        self.launch_button_1.setStyleSheet(button_style)
        self.launch_button_2.setStyleSheet(button_style)
        self.launch_button_3.setStyleSheet(button_style)
        self.launch_button_4.setStyleSheet(button_style)
        self.launch_button_5.setStyleSheet(button_style)
        self.launch_button_6.setStyleSheet(button_style)

        # Utilisez un style CSS pour styliser les boutons d'arrêt
        stop_button_style = "QPushButton { background-color: #FF5733; color: white; font-size: 18px; border: 2px solid #FF5733; border-radius: 10px; }"
        self.stop_button_1.setStyleSheet(stop_button_style)
        self.stop_button_2.setStyleSheet(stop_button_style)
        self.stop_button_3.setStyleSheet(stop_button_style)
        self.stop_button_4.setStyleSheet(stop_button_style)

        # Créez le canevas en dehors du bloc try
        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)

        # Layout
        layout = QVBoxLayout()
        affiche_layout = QHBoxLayout()
        button_layout = QHBoxLayout()
        stop_button_layout = QHBoxLayout()

        button_layout.addWidget(self.launch_button_1)
        button_layout.addWidget(self.launch_button_2)
        button_layout.addWidget(self.launch_button_3)
        button_layout.addWidget(self.launch_button_4)
        button_layout.addWidget(self.launch_button_5)
        button_layout.addWidget(self.launch_button_6)

        stop_button_layout.addWidget(self.stop_button_1)
        stop_button_layout.addWidget(self.stop_button_2)
        stop_button_layout.addWidget(self.stop_button_3)
        stop_button_layout.addWidget(self.stop_button_4)

        affiche_layout.addWidget(self.canvas)
        affiche_layout.addWidget(self.image_label)

        layout.addWidget(self.text_browser)  # Ajout de la zone de texte aux instructions
        layout.addWidget(self.clear_button)
        layout.addLayout(affiche_layout)
        layout.addLayout(button_layout)
        layout.addLayout(stop_button_layout)
        layout.setSpacing(20)
        layout.setAlignment(Qt.AlignHCenter)

        # Définir une politique de taille extensible pour les boutons
        for button in [self.launch_button_1, self.launch_button_2, self.launch_button_3, self.launch_button_4,
                       self.launch_button_5, self.launch_button_6]:
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.setLayout(layout)

        # Connexion des boutons aux fonctions
        self.launch_button_1.clicked.connect(self.play_acquisition)
        self.launch_button_2.clicked.connect(self.stop_acquisition)
        self.launch_button_3.clicked.connect(self.acquisition_full)
        self.launch_button_4.clicked.connect(self.launch_rosrun)
        self.launch_button_5.clicked.connect(self.launch_roslaunch_1)
        self.launch_button_6.clicked.connect(self.launch_roslaunch_2)
        self.stop_button_1.clicked.connect(self.stop_roslaunch_1)
        self.stop_button_2.clicked.connect(self.launch_rviz)
        self.stop_button_3.clicked.connect(self.stop_roslaunch_2)
        self.stop_button_4.clicked.connect(self.video)

    def run_command(self, command):
        # Exécutez une commande système
        subprocess.Popen(command, shell=True)

    def clear_text_browser(self):
        # Effacez le contenu de text_browser
        self.text_browser.clear()

    def launch_roslaunch_1(self):
        # Lancer un roslaunch (Personnalisez la commande selon vos besoins)
        # self.text_browser.append("Lancement du mrs 1000")
        # self.run_command("/opt/ros/noetic/bin/rosrun tf static_transform_publisher 0 0 0 0 0 0 1 map cloud 5")
        # self.run_command("/opt/ros/noetic/bin/roslaunch sick_scan sick_mrs_1xxx.launch __ns:=sick")
        self.text_browser.append("Lancement caméra")
        self.run_command("/opt/ros/noetic/bin/roslaunch pylon_camera pylon_camera_node.launch")

    def launch_roslaunch_2(self):
        # Lancer un autre roslaunch
        # self.text_browser.append("Lancement du rplidar")
        # self.run_command("/opt/ros/noetic/bin/roslaunch rplidar_ros rplidar_a3.launch")
        self.ser.write(b'H')  # Envoyer 'H' pour activer le flash

    def launch_rosrun(self):
        # Lancer un rosrun (Personnalisez la commande selon vos besoins)
        self.text_browser.append("Lancement de detection mottes")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.run_command("/opt/ros/noetic/bin/rosrun vision_pkg detection_mottes.py")

    def launch_rviz(self):
        # Lancer un rosrun (Personnalisez la commande selon vos besoins)
        self.text_browser.append("Arret du r2000")
        self.run_command("/opt/ros/noetic/bin/rosnode kill /r2000_node")

    def stop_roslaunch_1(self):
        # Arrêter le roslaunch 1 (Personnalisez la commande selon vos besoins)
        self.text_browser.append("Arret de la caméra")
        self.run_command("/opt/ros/noetic/bin/rosnode kill /pylon_camera_node")

    def stop_roslaunch_2(self):
        # Arrêter le roslaunch 2 (Personnalisez la commande selon vos besoins)
        # self.text_browser.append("Arret du rplidar")
        # self.run_command("/opt/ros/noetic/bin/rosnode kill /rplidarNode")
        self.ser.write(b'L')  # Envoyer 'L' pour désactiver le flash

    def acquisition_rplidar(self):
        print("Lancement de l'acquisition du rplidar")  # Affichez le message dans la console
        self.text_browser.append("Lancement de l'acquisition du rplidar")
        self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        QCoreApplication.processEvents()  # Traitez les événements de l'application
        sys.stdout.flush()  # Videz le tampon de la sortie standard

        self.run_command("/opt/ros/noetic/bin/rosbag record --duration 2 -o /home/icam/acquisition_scan/ /scan")
        time.sleep(3)

        # Lancez le script acquisition_scan.py dans un processus séparé
        process = subprocess.Popen(["/usr/bin/python3", "/home/icam/kuhn_ws/src/tools/acquisition_scan.py"],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                self.text_browser.append(output.strip())
                self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
                QCoreApplication.processEvents()  # Traitez les événements de l'application
        process.communicate()

        # self.run_command("/opt/ros/noetic/bin/rosnode kill /rplidarNode")

    def acquisition_sick(self):
        print("Lancement de l'acquisition du mrs 1000")  # Affichez le message dans la console
        self.text_browser.append("Lancement de l'acquisition du mrs 1000")
        self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        QCoreApplication.processEvents()  # Traitez les événements de l'application

        self.run_command("/opt/ros/noetic/bin/rosbag record --duration 2 -o /home/icam/acquisition_cloud/ /sick/cloud")
        time.sleep(3)

        # Lancez le script acquisition_cloud.py dans un processus séparé
        process = subprocess.Popen(["/usr/bin/python3", "/home/icam/kuhn_ws/src/tools/acquisition_cloud.py"],
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        while True:
            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break
            if output:
                self.text_browser.append(output.strip())
                self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
                QCoreApplication.processEvents()  # Traitez les événements de l'application
        process.communicate()

    def acquisition_full(self):
        # print("Lancement de l'acquisition full")  # Affichez le message dans la console
        # self.text_browser.append("Lancement de l'acquisition full")
        # self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
        # self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        # QCoreApplication.processEvents()  # Traitez les événements de l'application

        # self.run_command("/opt/ros/noetic/bin/rosbag record --duration 4 -o /home/icam/acquisition/ /sick/cloud /scan")
        # time.sleep(6)

        # process = subprocess.Popen(["/usr/bin/python3", "/home/icam/kuhn_ws/src/tools/acquisition_full.py"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        # while True:
        #     output = process.stdout.readline()
        #     if output == '' and process.poll() is not None:
        #         break
        #     if output:
        #         self.text_browser.append(output.strip())
        #         self.text_browser.repaint()  # Forcez la mise à jour de l'interface utilisateur
        #         QCoreApplication.processEvents()  # Traitez les événements de l'application
        # process.communicate()
        self.text_browser.append("rviz")
        self.run_command("rviz rviz")

    def boucle_acquisition(self):
        numero = 1
        compteur = 0
        while self.boucle_active:
            self.text_browser.append(f"Acquisition {numero} compteur : {compteur}")
            self.launch_roslaunch_2()
            self.run_command(
                "/opt/ros/noetic/bin/rosbag record --duration 4 -o /home/icam/acquisition_test/ /pylon_camera_node/image_raw /r2000/scan")
            self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
            numero = numero + 1
            compteur = compteur + 1
            time.sleep(6)
            self.stop_roslaunch_2()
            if compteur == 5:
                self.plot_3d_surface()
                compteur = 0

    # Fonction pour démarrer la boucle
    def play_acquisition(self):
        self.text_browser.append("Lancement de l'acquisition full")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.boucle_active = True
        t = threading.Thread(target=self.boucle_acquisition)
        t.start()

    # Fonction pour arrêter la boucle
    def stop_acquisition(self):
        self.text_browser.append("Arret acquisition full")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.boucle_active = False
        print('fin')

    def plot_3d_surface(self):
        try:
            # Appel à la fonction pour générer les données du tracé 3D
            x_sublists, y_sublists, Z_decalage = generate_3d_plot_data()

            # Récupérer l'instance de Figure et effacer le contenu précédent
            self.figure.clear()

            # Définir les valeurs seuils
            moy_X = np.mean(x_sublists)
            seuil_max = moy_X + 0.01
            masque = x_sublists > seuil_max

            # Créer le tracé 3D
            ax = self.figure.add_subplot(111, projection='3d')
            ax.scatter(y_sublists[masque].flatten(), Z_decalage[masque].flatten(), x_sublists[masque].flatten(),
                       color='red', alpha=1.0)
            ax.plot_trisurf(y_sublists[~masque].flatten(), Z_decalage[~masque].flatten(), x_sublists[~masque].flatten(),
                            color='green', alpha=0.5)

            # Définir les limites sur chaque axe (échelle)
            ax.set_xlim([-0.2, 0.2])
            ax.set_ylim([0, 50])
            ax.set_zlim([-0.3, -0.1])

            # Ajouter des étiquettes et un titre
            ax.set_xlabel('Y')
            ax.set_ylabel('Z')
            ax.set_zlabel('X')
            self.figure.tight_layout()

            self.canvas.draw()
        except Exception as e:
            print(f"Erreur lors de l'exécution : {e}")

    def video(self):
        # Vous devrez utiliser le code approprié pour lire le flux vidéo du rostopic
        # et le convertir en image OpenCV. L'exemple suivant utilise une image statique.
        # Remplacez cette partie par votre propre logique de capture de vidéo en temps réel.

        # Exemple statique (remplacez par la capture de votre flux vidéo)
        static_image = cv2.imread("path/to/your/image.jpg")

        # Convertir l'image OpenCV en format QImage
        height, width, channel = static_image.shape
        bytes_per_line = 3 * width
        q_image = QImage(static_image.data, width, height, bytes_per_line, QImage.Format_RGB888)

        # Convertir l'image QImage en QPixmap
        pixmap = QPixmap.fromImage(q_image)

        # Mettez à jour le QLabel avec la nouvelle image
        self.image_label.setPixmap(pixmap)
        QApplication.processEvents()


boucle_active = False


def main():
    app = QApplication(sys.argv)
    try:
        window = ROSLauncherApp()
        window.show()
        window.plot_3d_surface()  # Appel de la méthode pour afficher le tracé 3D
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Une erreur est survenue : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
