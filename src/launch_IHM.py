import tkinter as tk
from Codes.Lidar.launch_lidar import launch_lidar, kill_lidar

def lancer_lidar():
    launch_lidar()

def arreter_lidar():
    kill_lidar()

# Création de la fenêtre principale
fenetre = tk.Tk()
fenetre.title("Lancer/Arrêter Lidar")

# Création d'un bouton pour lancer le Lidar
bouton_lancer = tk.Button(fenetre, text="Lancer le Lidar", command=lancer_lidar)
bouton_lancer.pack(pady=10)

# Création d'un bouton pour arrêter le Lidar
bouton_arreter = tk.Button(fenetre, text="Arrêter le Lidar", command=arreter_lidar)
bouton_arreter.pack(pady=10)

# Lancement de la boucle principale de la fenêtre
fenetre.mainloop()
