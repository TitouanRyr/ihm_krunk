import subprocess
import time

camera_process = None  # Variable pour stocker l'objet du processus

def topic_available(topic_name, timeout=30):
    """
    Vérifie si le topic ROS est disponible.
    """
    start_time = time.time()
    while time.time() - start_time < timeout:
        topics = subprocess.check_output(['rostopic', 'list']).decode('utf-8').split('\n')
        if topic_name in topics:
            return True
        time.sleep(1)

    return False

def kill_camera():
    global camera_process
    if camera_process:
        try:
            # Terminer le processus du roslaunch
            camera_process.terminate()
            camera_process.wait()  # Attendre la fin du processus
            print("Le processus roslaunch a été terminé.")
        except Exception as e:
            print(f"Erreur lors de la fermeture du processus roslaunch : {e}")

def launch_camera():
    global camera_process
    command_to_run = ["/opt/ros/noetic/bin/roslaunch on_camera pylon_camera_node.launch"]

    try:
        # Lancer la commande dans un processus séparé
        camera_process = subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Commande en cours d'exécution en arrière-plan.")

        # Attendre que le Lidar soit prêt
        time.sleep(5)

        camera_topic = "/pylon_camera_node/image_raw"
        if topic_available(camera_topic):
            error_message = f"Le topic {camera_topic} est disponible. Le ROS node a été lancé avec succès."
            print(error_message)
            return True, error_message
        else:
            error_message = f"Le topic {camera_topic} n'est pas disponible. Il pourrait y avoir un problème avec le ROS node."
            print(error_message)
            return False, error_message

    except Exception as e:
        error_message = f"Erreur lors de l'exécution de la commande roslaunch : {e}"
        print(error_message)
        return False, error_message

if __name__ == "__main__":
    camera_result, error_message = launch_camera()

    if lidar_result:
        print("Le lidar fonctionne.")
    else:
        print(f"Il y a eu un problème avec le lidar. Erreur : {error_message}")
