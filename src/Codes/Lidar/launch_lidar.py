import subprocess
import time

lidar_process = None  # Variable pour stocker l'objet du processus

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

def kill_lidar():
    global lidar_process
    if lidar_process:
        try:
            # Terminer le processus du roslaunch
            lidar_process.terminate()
            lidar_process.wait()  # Attendre la fin du processus
            print("Le processus roslaunch a été terminé.")
        except Exception as e:
            print(f"Erreur lors de la fermeture du processus roslaunch : {e}")

def launch_lidar():
    global lidar_process
    command_to_run = ["/opt/ros/noetic/bin/roslaunch", "pf_driver", "r2000.launch"]

    try:
        # Lancer la commande dans un processus séparé
        lidar_process = subprocess.Popen(command_to_run, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("Commande en cours d'exécution en arrière-plan.")

        # Attendre que le Lidar soit prêt
        time.sleep(5)

        lidar_topic = "/scan"
        if topic_available(lidar_topic):
            error_message = f"Le topic {lidar_topic} est disponible. Le ROS node a été lancé avec succès."
            print(error_message)
            return True, error_message
        else:
            error_message = f"Le topic {lidar_topic} n'est pas disponible. Il pourrait y avoir un problème avec le ROS node."
            print(error_message)
            return False, error_message

    except Exception as e:
        error_message = f"Erreur lors de l'exécution de la commande roslaunch : {e}"
        print(error_message)
        return False, error_message

if __name__ == "__main__":
    lidar_result, error_message = launch_lidar()

    if lidar_result:
        print("Le lidar fonctionne.")
    else:
        print(f"Il y a eu un problème avec le lidar. Erreur : {error_message}")
