import serial

def arduino_init(self):
    self.arduino_port = "/dev/ttyACM0"
    self.ser = None  # Initialisation à None pour éviter toute erreur si le port série n'est pas ouvert

    try:
        self.ser = serial.Serial(self.arduino_port, 9600, timeout=1)
    except serial.SerialException as e:
        print(f"Erreur : {e}")
        print("Arduino non connecté. L'application se poursuivra sans communication avec l'Arduino.")

def launch_light(self):
    # Envoi de la commande HIGH à l'Arduino pour activer le flash
    # self.text_browser.append("Lancement du rplidar")
    # self.run_command("/opt/ros/noetic/bin/roslaunch rplidar_ros rplidar_a3.launch")
    self.ser.write(b'H')  # Envoyer 'H' pour activer le flash

def kill_light(self):
    # Arrêt du flash
    # self.text_browser.append("Arret du rplidar")
    # self.run_command("/opt/ros/noetic/bin/rosnode kill /rplidarNode")
    self.ser.write(b'L')  # Envoyer 'L' pour désactiver le flash
