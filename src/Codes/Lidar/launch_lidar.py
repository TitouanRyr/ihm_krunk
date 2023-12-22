import subprocess
import time

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

def launch_lidar():
    command_to_run = "/opt/ros/noetic/bin/roslaunch pf_driver r2000.launch"
    try:
        subprocess.run(command_to_run, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Commande exécutée avec succés.")

        time.sleep(5)

        lidar_topic = "/scan"
        if topic_available(lidar_topic):
            error_message = f"Le topic {lidar_topic} est disponible. Le ROS node a été lancé avec succès."
            return True, error_message
        else:
            error_message = f"Le topic {lidar_topic} n'est pas disponible. Il pourrait y avoir un problème avec le ROS node."
            print(error_message)
            return False, error_message

    except Exception as e:
        error_message = f"Erreur lors de l'exécution de la commande : {e}"
        print(error_message)
        return False, error_message

if __name__ == "__main__":
    lidar_result, error_message = launch_lidar()

    if lidar_result:
        print("Le lidar fonctionne.")
    else:
        print(f"Il y a eu un problème avec le lidar. Erreur : {error_message}")
