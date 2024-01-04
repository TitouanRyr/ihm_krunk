import os
import glob
import subprocess
import time
import sys
import serial
import threading
import numpy as np
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtWidgets import QMainWindow, QWidget, QVBoxLayout
from matplotlib.figure import Figure
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from code_graphic_V3 import generate_3d_plot_data
from control_arduino import ArduinoController


class ROSLauncherApp(QWidget):
    def __init__(self):
        super().__init__()

        self.arduino_port = "/dev/ttyACM0"
        self.ser = None
        try:
            self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
        except serial.SerialException as e:
            print(f"Erreur : {e}")
            print("Arduino non connecté. L'application se poursuivra sans communication avec l'Arduino.")

        self.flash_state = False
        self.setWindowTitle("IHM Krunk")

        screen_width = 800
        screen_height = int(screen_width * 3 / 4)
        self.setGeometry(100, 100, screen_width, screen_height)

        self.text_browser = QTextBrowser()
        self.text_browser.setFixedHeight(50)

        self.image_label = QLabel()

        self.launch_button_1 = QPushButton("Play")
        self.launch_button_2 = QPushButton("stop")
        self.launch_button_3 = QPushButton("rviz")
        self.launch_button_4 = QPushButton("r2000")
        self.launch_button_5 = QPushButton("Photo")
        self.launch_button_6 = QPushButton("Lumière")

        self.stop_button_1 = QPushButton("Arrêter Photo")
        self.stop_button_2 = QPushButton("Arrêter r2000")
        self.stop_button_3 = QPushButton("Arrêter Lumière")

        self.clear_button = QPushButton("Clear")
        self.clear_button.setStyleSheet(
            "QPushButton { background-color: #E74C3C; color: white; font-size: 18px; border: 2px solid #E74C3C; border-radius: 10px; }")
        self.clear_button.clicked.connect(self.clear_text_browser)

        button_style = "QPushButton { background-color: #4CAF50; color: white; font-size: 18px; border: 2px solid #4CAF50; border-radius: 10px; }"
        self.launch_button_1.setStyleSheet(button_style)
        self.launch_button_2.setStyleSheet(button_style)
        self.launch_button_3.setStyleSheet(button_style)
        self.launch_button_4.setStyleSheet(button_style)
        self.launch_button_5.setStyleSheet(button_style)
        self.launch_button_6.setStyleSheet(button_style)

        stop_button_style = "QPushButton { background-color: #FF5733; color: white; font-size: 18px; border: 2px solid #FF5733; border-radius: 10px; }"
        self.stop_button_1.setStyleSheet(stop_button_style)
        self.stop_button_2.setStyleSheet(stop_button_style)
        self.stop_button_3.setStyleSheet(stop_button_style)

        self.figure = Figure(figsize=(6, 4), dpi=100)
        self.canvas = FigureCanvas(self.figure)

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

        affiche_layout.addWidget(self.canvas)
        affiche_layout.addWidget(self.image_label)

        layout.addWidget(self.text_browser)
        layout.addWidget(self.clear_button)
        layout.addLayout(affiche_layout)
        layout.addLayout(button_layout)
        layout.addLayout(stop_button_layout)
        layout.setSpacing(20)
        layout.setAlignment(Qt.AlignHCenter)

        for button in [self.launch_button_1, self.launch_button_2, self.launch_button_3, self.launch_button_4,
                       self.launch_button_5, self.launch_button_6]:
            button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.setLayout(layout)

        self.launch_button_1.clicked.connect(self.play_acquisition)
        self.launch_button_2.clicked.connect(self.stop_acquisition)
        self.launch_button_3.clicked.connect(self.acquisition_full)
        self.launch_button_4.clicked.connect(self.launch_rosrun)
        self.launch_button_5.clicked.connect(self.launch_roslaunch_1)
        self.launch_button_6.clicked.connect(self.launch_roslaunch_2)
        self.stop_button_1.clicked.connect(self.stop_roslaunch_1)
        self.stop_button_2.clicked.connect(self.launch_rviz)
        self.stop_button_3.clicked.connect(self.stop_roslaunch_2)

    def run_command(self, command):
        subprocess.Popen(command, shell=True)

    def clear_text_browser(self):
        self.text_browser.clear()

    def launch_roslaunch_1(self):
        self.text_browser.append("Lancement caméra")
        self.run_command("/opt/ros/noetic/bin/roslaunch pylon_camera pylon_camera_node.launch")

    def launch_roslaunch_2(self):
        self.ser.write(b'H')

    def launch_rosrun(self):
        self.text_browser.append("Lancement du r2000")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.run_command("/opt/ros/noetic/bin/roslaunch pf_driver r2000.launch")

    def launch_rviz(self):
        self.text_browser.append("Arret du r2000")
        self.run_command("/opt/ros/noetic/bin/rosnode kill /r2000_node")

    def stop_roslaunch_1(self):
        self.text_browser.append("Arret de la caméra")
        self.run_command("/opt/ros/noetic/bin/rosnode kill /pylon_camera_node")

    def stop_roslaunch_2(self):
        self.ser.write(b'L')

    def acquisition_full(self):
        self.text_browser.append("rviz")
        self.run_command("rviz rviz")

    def boucle_acquisition(self):
        numero = 1

        while self.boucle_active:
            self.text_browser.append(f"Acquisition {numero}")
            self.launch_roslaunch_2()
            self.run_command(
                "/opt/ros/noetic/bin/rosbag record --duration 6 -o /home/icam/acquisition_test/ /pylon_camera_node/image_raw /r2000/scan")
            self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
            numero = numero + 1

            time.sleep(6)
            self.stop_roslaunch_2()

    def play_acquisition(self):
        self.text_browser.append("Lancement de l'acquisition full")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.boucle_active = True
        t = threading.Thread(target=self.boucle_acquisition)
        t.start()

    def stop_acquisition(self):
        self.text_browser.append("Arret acquisition full")
        self.text_browser.verticalScrollBar().setValue(self.text_browser.verticalScrollBar().maximum())
        self.boucle_active = False
        print('fin')

    def plot_3d_surface(self):
        try:
            x_sublists, y_sublists, Z_decalage = generate_3d_plot_data()
            self.figure.clear()

            moy_X = np.mean(x_sublists)
            seuil_max = moy_X + 0.01
            masque = x_sublists > seuil_max

            ax = self.figure.add_subplot(111, projection='3d')
            ax.scatter(y_sublists[masque].flatten(), Z_decalage[masque].flatten(), x_sublists[masque].flatten(),
                       color='red', alpha=1.0)
            ax.plot_trisurf(y_sublists[~masque].flatten(), Z_decalage[~masque].flatten(), x_sublists[~masque].flatten(),
                            color='green', alpha=0.5)

            ax.set_xlim([-0.2, 0.2])
            ax.set_ylim([0, 50])
            ax.set_zlim([-0.3, -0.1])

            ax.set_xlabel('Y')
            ax.set_ylabel('Z')
            ax.set_zlabel('X')
            self.figure.tight_layout()

            self.canvas.draw()
        except Exception as e:
            print(f"Erreur lors de l'exécution : {e}")


def main():
    app = QApplication(sys.argv)
    try:
        window = ROSLauncherApp()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Une erreur est survenue : {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
