import subprocess

def kill_lidar():
    print("Arret lidar")
    command_to_run = ["/opt/ros/noetic/bin/rosnode", "kill", "/r2000_node"]
    subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def launch_lidar():
    print("Lancement lidar")
    command_to_run = ["/opt/ros/noetic/bin/roslaunch", "pf_driver", "r2000.launch"]
    subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

