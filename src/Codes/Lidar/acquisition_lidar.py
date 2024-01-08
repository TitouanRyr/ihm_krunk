import subprocess


def acquisition_r2000():
    print("Lancement de l'acquisition du r2000")  # Affichez le message dans la console
    subprocess.Popen("/opt/ros/noetic/bin/rosbag record --duration 1 -o /home/icam/kuhn_ws/src/ihm_krunk/src/acquisitions/acquisition_r2000/ /scan", shell=True)
