import ruamel.yaml # For Yaml
from pyproj import Transformer # For coordinate transforming
import csv
from time import localtime, strftime
import numpy as np
import matplotlib.pyplot as plt
import os


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
            - center: New centervalue (in practice "cornervalue") TODO: Add offset
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


def log_mission_items(logpath, mission_items):
    identifier = strftime("%d_%m_%Y_%H_%M_%S", localtime())
    with open(f'{logpath}/mission_log_position_{identifier}.csv', 'a', newline='\n') as file:
        writer = csv.writer(file, delimiter=';')
        for item in mission_items:
            writer.writerow([strftime("%d/%m/%Y_%H:%M:%S", localtime()) , item.seq, item.lat / 10**7, item.lon / 10**7])


if __name__ == '__main__':
    """ Testing corner"""
    absolute_path = os.path.dirname(__file__)
    relative_path = r'./test.yaml'
    full_path = os.path.join(absolute_path)
    print(os.listdir(full_path))

    a = [1004,1040]
    b = [2040,2004]

    configure_enc(f'{absolute_path}/test.yaml', center=a, size=b)
    