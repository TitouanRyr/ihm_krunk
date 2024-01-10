import subprocess

def kill_camera():
    print("Arret camera")
    command_to_run = ["/opt/ros/noetic/bin/rosnode", "kill", "/pylon_camera_node"]
    subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def launch_camera():
    print("Lancement camera")
    command_to_run = ["/opt/ros/noetic/bin/roslaunch", "pylon_camera", "pylon_camera_node.launch"]
    subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)