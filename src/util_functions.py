from pyproj import Transformer # For coordinate transforming
from src.mavlink_communication import get_gps
from time import localtime, strftime
import matplotlib.pyplot as plt
import ruamel.yaml # For Yaml
import numpy as np
import csv
import os

# The plot_path function will be replaced by a more polished version
def plot_path(grid, path, title=None, savepath="./data/%s.png"%'astar_path'):
    plt.figure('Path')
    plt.title(title)

    background =  np.rot90(np.flip(np.array(grid),0),3) # Flipping & Rotating the map 270* to make it match the  coordinates system
    plt.imshow(background, cmap='Greys')
    
    plt.xlim(0,1000)
    plt.ylim(0,1000)

    x = []
    y = []
    for waypoint in path:
        x.append(waypoint[0])
        y.append(waypoint[1])
    plt.scatter(x,y,s=0.5,color='red')
    plt.savefig(savepath, dpi=300, bbox_inches='tight', pad_inches=0.2)
       

def configure_enc(yaml_path, center, size):
    """ Changes configuration values in the ENC's .yaml file:
        Input:
            - yaml_path: Filepath to yaml file
            - center: New centervalue (in practice "cornervalue")
            - size: New sizevalue
        """
    
    # Loading configuration values:
    yaml = ruamel.yaml.YAML()
    with open (yaml_path) as yamlfile:
        content = yaml.load(yamlfile)

    # Replace current values with the new ones:
    content['enc']['center'][0] = center[0]
    content['enc']['center'][1] = center[1]
    content['enc']['size'][0]   = size[0]
    content['enc']['size'][1]   = size[1]
    
    # Write changes to .yaml file
    with open(yaml_path, "w") as yamlfile:
        yaml.dump(content, yamlfile)


def log_mission_items(identifier, logpath, mission_items, the_connection):
    starting_position = get_gps(the_connection)
    with open(f'{logpath}/mission_log_waypoints_{identifier}.csv', 'a', newline='\n') as file:
        writer = csv.writer(file, delimiter=';')
        writer.writerow([strftime("%d/%m/%Y_%H:%M:%S", localtime()) , 0, starting_position.lat/10**7 , starting_position.lon/10**7])
        for item in mission_items:
            writer.writerow([strftime("%d/%m/%Y_%H:%M:%S", localtime()) , item.seq+1, item.x, item.y])

def log_enc_config(identifier,logpath, size, center, target, algorithm, note): 
    with open(f'{logpath}/mission_log_config_{identifier}.csv', 'a', newline='\n') as file:
        writer = csv.writer(file, delimiter=';')
        writer.writerow([size[0], size[1], center[0], center[1], target[0], target[1], algorithm, note])


if __name__ == '__main__':
    """ Testing corner"""
    absolute_path = os.path.dirname(__file__)
    relative_path = r'./test.yaml'
    full_path = os.path.join(absolute_path)
    print(os.listdir(full_path))

    a = [1004,1040]
    b = [2040,2004]

    configure_enc(f'{absolute_path}/test.yaml', center=a, size=b)
    