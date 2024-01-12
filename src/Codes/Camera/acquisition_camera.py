import subprocess


def acquisition_camera():
    print("Lancement de l'acquisition de la camera")  # Affichez le message dans la console
    subprocess.Popen("/opt/ros/noetic/bin/rosbag record --duration 1 -o /home/oem/kuhn_ws/src/ihm_krunk/src/acquisitions/acquisition_camera/ /pylon_camera_node/image_raw", shell=True)