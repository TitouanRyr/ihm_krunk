import subprocess


def acquisition_rplidar():
    print("Lancement de l'acquisition du rplidar")  # Affichez le message dans la console
    subprocess.Popen("/opt/ros/noetic/bin/rosbag record --duration 1 -o /home/icam/kuhn_ws/src/ihm_krunk/src/acquisitions/acquisition_rplidar/ /scan", shell=True)
