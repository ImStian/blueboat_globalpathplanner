from shapely.geometry import Polygon, MultiPolygon
from scipy.ndimage import binary_dilation
import matplotlib.pyplot as plt
from seacharts.enc import ENC

import time
import cv2
import sys
import numpy as np
np.set_printoptions(threshold=sys.maxsize)
import csv
csv.field_size_limit(sys.maxsize) # ups the writing limit for csvfiles

'''
    This map preparation code contains modified codesnippets provided by Melih Ǎkdag. 
    In addition to being modified, the code has also been ported to Seacharts v3.0.0.
    The overall structure of the code however, remains the same.
'''

def extract_map_data(path, center, new_data=False):
    '''
        Extracts land data from .gdb file
        Dependencies: None
        Uses: prepare_map_polygons(), occupancy_grid_map()
    '''
      # ENC config
    enc = ENC(config='map_settings.yaml')
    if new_data:
        enc.update() # Updating enc
    algo_start = time.time()
    print('Map Function - Extracting map data.')
    # Land polygons
    land_polygon_coords = []
    if type(enc.land.mapping['coordinates']) == list:
        for n in range(len(enc.land.mapping['coordinates'])):
            for i in range(len(enc.land.mapping['coordinates'][n])):
                pol = list(enc.land.mapping['coordinates'][n][i])
                poly = Polygon(pol)
                x, y = poly.exterior.xy
                x = [each - center[0] for each in x]
                y = [each - center[1] for each in y]
                land_polygon_coords.append([list(x), list(y)])
    else:
        for i in range(len(enc.land.mapping['coordinates'])):
            pol = list(enc.land.mapping['coordinates'][i])
            poly = Polygon(pol)
            x, y = poly.exterior.xy
            x = [each-center[0] for each in x]
            y = [each-center[1] for each in y]
            land_polygon_coords.append([list(x), list(y)])
    with open(path+"/land_polygon_coords.csv", "w") as f:
        wr = csv.writer(f)
        wr.writerows(land_polygon_coords)


    # Shore polygons
    shore_polygon_coords = []
    if type(enc.shore.mapping['coordinates']) == list:
        for n in range(len(enc.shore.mapping['coordinates'])):
            for i in range(len(enc.shore.mapping['coordinates'][n])):
                    pol = list(enc.shore.mapping['coordinates'][n][i])
                    poly = Polygon(pol)
                    x, y = poly.exterior.xy
                    x = [each-center[0] for each in x]
                    y = [each-center[1] for each in y]
                    shore_polygon_coords.append([list(x), list(y)])
    else:
        for i in range(len(enc.shore.mapping['coordinates'])):
            pol = list(enc.shore.mapping['coordinates'][i])
            poly = Polygon(pol)
            x, y = poly.exterior.xy
            x = [each-center[0] for each in x]
            y = [each-center[1] for each in y]
            shore_polygon_coords.append([list(x), list(y)])
    with open(f"{path}/shore_polygon_coords.csv", "w") as f:
        wr = csv.writer(f)
        wr.writerows(shore_polygon_coords)



def prepare_map_polygons_coords(path):
    '''
        Reads csv file containing map data
        and converts it into lists of coordinates
        and Shapely Polygon/MultiPolygon objects
        Dependencies: extract_map_data()
        Uses: map_visualization()
    '''
    algo_start = time.time()
    print('Map Function - Generating polygons and coordinates form map data.')
    # LAND POLYGONS
    with open(path+'land_polygon_coords.csv', 'r', encoding='UTF-8') as file:
        poly_list = csv.reader(file)
        # Convert string in csv to float
        land_polygon_coords = []
        land_polygons = []
        for row in poly_list:
            # Extract the elements of the list from the string
            x = list(map(float, row[0][1:-1].split(',')))
            y = list(map(float, row[1][1:-1].split(',')))
            land_polygon_coords.append([x, y])
            coords = []
            for (each_x, each_y) in zip(x,y):
                coords.append((each_x, each_y))
            poly = Polygon(coords)
            land_polygons.append(poly)
        land_multipol = MultiPolygon(land_polygons)
    # SHORE POLYGONS
    with open(path+"shore_polygon_coords.csv", 'r', encoding='UTF-8') as file:
        poly_list = csv.reader(file)
        # Convert string in csv to float
        shore_polygon_coords = []
        shore_polygons = []
        for row in poly_list:
            # Extract the elements of the list from the string
            x = list(map(float, row[0][1:-1].split(',')))
            y = list(map(float, row[1][1:-1].split(',')))
            shore_polygon_coords.append([x, y])
            coords = []
            for (each_x, each_y) in zip(x,y):
                coords.append((each_x, each_y))
            poly = Polygon(coords)
            shore_polygons.append(poly)
        shore_multipol = MultiPolygon(shore_polygons)
        

    polygons = [shore_multipol, land_multipol]
    polygons_coords = [shore_polygon_coords, land_polygon_coords]
    
    algo_end = time.time()
    time_elapsed = round(algo_end - algo_start, 4)
    print(f'Map functions - Polygons and coordinates are generated. Time: {time_elapsed} s.')

    return polygons, polygons_coords
    

def map_visualization(polygon_coords, saveas=None):
    '''
        Visualizes and saves the map area as a png image
    '''
    plt.figure().clear() # redundancy to make sure the  figure is cleared before plotting
    plt.figure(figsize=(10,10))
    sea_polygons = polygon_coords[2:]
    sea_polygons.reverse()
    blue_shades = plt.get_cmap('Blues')(np.linspace(0.9, 0.2, len(sea_polygons)))

    for poly, shade in zip(sea_polygons, np.flip(blue_shades, axis=0)):
        for each in poly:
            plt.fill(each[0], each[1], c=shade)
    for each in polygon_coords[0]:
        plt.fill(each[0], each[1], c='grey')
    for each in polygon_coords[1]:
        plt.fill(each[0], each[1], c='black')

    plt.grid(alpha=0.2)

    if saveas is not None:
        plt.savefig(f'./data/{saveas}.png', dpi=300, bbox_inches='tight', pad_inches=0)
        plt.close()
        #plt.show()


def occupancy_grid_map(path, size, buffer_size=None, visualize=False, saveas=None):
    '''
    Creates an occupancy grid map given a set of land polygons
    '''
    algo_start = time.time()
    print('Map functions - Creating occupancy grid map.')
    occupancy_grid = np.zeros((size[0], size[1]), dtype=np.uint8)
    with open(f'{path}/shore_polygon_coords.csv', 'r', encoding='UTF-8') as file:
        poly_list = csv.reader(file)
        coords = []
        # Convert string in csv to float
        for row in poly_list:
            # Extract the elements of the list from the string
            x = list(map(float, row[0][1:-1].split(',')))
            y = list(map(float, row[1][1:-1].split(',')))
            polygon = [(int(each_x), int(each_y)) for each_x, each_y in zip(x,y)]
            coords.append(polygon)
    for polygon in coords:
        # Flip the polygon vertically to match OpenCV's coordinate system
        polygon = [(x, size[1] - y) for x, y in polygon]

        pts = np.array(polygon, np.int32)
        cv2.fillPoly(occupancy_grid, [pts], 1)


    unique, counts = np.unique(occupancy_grid, return_counts=True)
    print(dict(zip(unique, counts)))


    # Buffer zone around obstacles
    if buffer_size is not None:
        buffer_size=buffer_size
        structuring_element = np.ones((2 * buffer_size + 1, 2 * buffer_size + 1))
        buffered_map = binary_dilation(occupancy_grid, structure=structuring_element)
        occupancy_grid = buffered_map.astype(int)

    occupancy_grid = np.rot90(np.flip(np.array(occupancy_grid),0), -1)


    if visualize:
        plt.figure(figsize=(10,10))
        plt.imshow(occupancy_grid, cmap='terrain')
        if saveas is not None:
            plt.savefig("./data/%s.png"%saveas, dpi=300, bbox_inches='tight', pad_inches=0)
            plt.close()

    algo_end = time.time()
    time_elapsed = round(algo_end - algo_start, 4)
    print(f'Map functions - Occupancy grid map is generated. Time: {time_elapsed} s.')

    return occupancy_grid, coords

