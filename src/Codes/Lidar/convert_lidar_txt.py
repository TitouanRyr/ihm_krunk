import os
import glob
import rosbag
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from datetime import datetime
import math

# Définissez le répertoire où se trouvent les fichiers ROS bag
repertoire_rosbag = '/home/icam/acquisition_test'

# Définissez le répertoire où vous souhaitez enregistrer les fichiers texte
repertoire_txt = '/home/icam/acquisition_test/txt'

# Recherchez tous les fichiers ROS bag dans le répertoire
liste_fichiers_bag = glob.glob(os.path.join(repertoire_rosbag, '*.bag'))

# Triez la liste des fichiers par date de création (du plus récent au plus ancien)
liste_fichiers_bag.sort(key=os.path.getctime, reverse=True)

if liste_fichiers_bag:
    for fichier_bag in liste_fichiers_bag:
        # Spécifiez le topic que vous souhaitez extraire
        topic = '/r2000/scan'

        # Obtenez le nom de base du fichier ROS bag sans extension
        nom_base = os.path.splitext(os.path.basename(fichier_bag))[0]

        # Générez le nom de fichier texte basé sur le nom du fichier ROS bag
        fichier_txt = f'{nom_base}.txt'

        # Chemin complet du fichier texte
        chemin_fichier_txt = os.path.join(repertoire_txt, fichier_txt)

        # Créez un fichier PointCloud pour stocker les données projetées
        point_cloud = PointCloud()

        # Ouvrez le fichier ROS bag en lecture
        with rosbag.Bag(fichier_bag, 'r') as bag:
            with open(chemin_fichier_txt, 'w') as f:
                for topic, msg, t in bag.read_messages(topics=[topic]):
                    if topic == '/r2000/scan':
                        # Accédez aux données du LaserScan
                        ranges = msg.ranges

                        for i, range_value in enumerate(ranges):
                            if not math.isnan(range_value):  # Assurez-vous que la valeur n'est pas NaN
                                angle = msg.angle_min + i * msg.angle_increment
                                x = range_value * math.cos(angle)
                                y = range_value * math.sin(angle)

                                # Créez un point dans le cloud de points
                                point = Point32()
                                point.x = x
                                point.y = y
                                point.z = 0.0  # Pour un nuage de points 2D, z est généralement à 0

                                # Ajoutez le point au PointCloud
                                point_cloud.points.append(point)

                                # Écrivez les données dans le fichier texte
                                f.write(f'x: {point.x}, y: {point.y}, z: {point.z}\n')

        print(f"Les données du LaserScan de {fichier_bag} ont été transformées en un cloud de points et enregistrées dans {fichier_txt}")
else:
    print("Aucun fichier ROS bag trouvé dans le répertoire spécifié.")
