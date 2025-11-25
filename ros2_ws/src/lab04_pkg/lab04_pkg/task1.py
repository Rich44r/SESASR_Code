import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py
import matplotlib.pyplot as plt
import numpy as np
import math

BAG_PATH = '/home/luke_skywalker/ros2_ws/rosbag2_2025_11_22-11_01_01' 

# List of topics we want to read
TOPICS = ['/odom', '/ground_truth', '/ekf']

def read_rosbag(bag_path):
    """
    Read a Rosbag and returns dictionaries 
    """
    #Read data in temporal sequence
    reader = rosbag2_py.SequentialReader()
    
    # Settings for the rosbag data configuration
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        #cdr: format used by rosbag
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    
    reader.open(storage_options, converter_options)

    # get type of message for each topic (es. nav_msgs/msg/Odometry)
    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    # dictionaires to save data: {topic: {'t': [], 'x': [], 'y': []}}
    data = {topic: {'t': [], 'x': [], 'y': [], 'theta': []} for topic in TOPICS}
    
    #lettura rosbag

    while reader.has_next():
        (topic, data_serialized, t_ns) = reader.read_next()

        if topic in TOPICS:
            # 1. Ottieni il tipo di messaggio (es. nav_msgs/msg/Odometry)
            msg_type_name = type_map[topic]
            msg_class = get_message(msg_type_name)

            # 2. Deserializza: converte i byte in un oggetto Python leggibile
            msg = deserialize_message(data_serialized, msg_class)

            # 3. Estrai i dati (Assumiamo che siano tutti Odometry)
            pos_x = msg.pose.pose.position.x
            pos_y = msg.pose.pose.position.y
            
            # Estrai l'orientamento (da Quaternione a Yaw)
            q = msg.pose.pose.orientation
            # Calcolo semplificato di Yaw da quaternione (se roll/pitch sono 0)
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            # Salva i dati nelle liste
            data[topic]['t'].append(t_ns / 1e9) # Converti nanosecondi in secondi
            data[topic]['x'].append(pos_x)
            data[topic]['y'].append(pos_y)
            data[topic]['theta'].append(theta)

    return data

def main():
    # 1. Leggi i dati
    try:
        bag_data = read_rosbag(BAG_PATH)
    except Exception as e:
        print(f"Errore nella lettura del rosbag: {e}")
        print("Assicurati che il percorso sia corretto e di aver 'sorgentato' ROS 2.")
        return

    # 2. Esempio di Plot: Traiettoria XY
    plt.figure(figsize=(10, 6))
    
    # Plot Ground Truth
    if bag_data['/ground_truth']['x']:
        plt.plot(bag_data['/ground_truth']['x'], bag_data['/ground_truth']['y'], 
                 label='Ground Truth', color='black', linestyle='-', linewidth=2)

    # Plot Odometria
    if bag_data['/odom']['x']:
        plt.plot(bag_data['/odom']['x'], bag_data['/odom']['y'], 
                 label='Odometry (Drift)', color='blue', linestyle='--')

    # Plot EKF
    if bag_data['/ekf']['x']:
        plt.plot(bag_data['/ekf']['x'], bag_data['/ekf']['y'], 
                 label='EKF Estimate', color='red', linestyle='-.')

    plt.title('Trajectories in 2D')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    print("Generazione grafico...")
    plt.show()

if __name__ == "__main__":
    main()