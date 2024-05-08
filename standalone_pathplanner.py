import src.coordinate_conversion as cc
import src.map_preparation as mp
import src.util_functions as uf
import src.rrtstar as rrtstar
import src.astar as astar
import numpy as np
import time
import os

algorithm = 'rrtstar'

# Filepath for settings and mapsaves:
mapdata_path = f'{os.getcwd()}/map_data/'
setting_path = f'{os.getcwd()}/map_settings.yaml'
logging_path = f'{os.getcwd()}/mission_logs/'


# Defining map size and center
size   = [7500, 7500]
center = [251459.72431460491, 7030336.552629814] # Centered on current position
center_enc =  [center[0] + size[0]/2, center[1] + size[1]/2] # Centered on current position
uf.configure_enc(setting_path, center=center_enc, size=size) # Updating .yaml file

utm33_current_position = (255154.05073167084, 7034111.246294714)
utm33_home_position = (253500.0412661848, 7036556.63955146)



# Define pathplanning start/end (grid coordinates)           
grid_current_position = cc.utm33_to_grid(utm33_current_position[0], utm33_current_position[1], size, center)
grid_home_location = cc.utm33_to_grid(utm33_home_position[0], utm33_home_position[1], size, center)


# Define pathplanning start/end (grid coordinates)           
#if grid_current_position[0] > size[0] or grid_current_position[0] < 0 or grid_current_position[1] > size[1] or grid_current_position[1] < 0:
#    print('Map Function - ERROR: Location out of bounds!')
#    quit()

start_coordinates = (grid_current_position[0], grid_current_position[1])
end_coordinates   = (grid_home_location[0], grid_home_location[1])

# Extracting map data using SeaCharts:
mp.extract_map_data(mapdata_path, center, new_data=True)
#polygons, polygons_coords = mp.prepare_map_polygons_coords(mapdata_path)
#mp.map_visualization(polygons_coords, saveas='map_visualization')

# Generating occupancy grid:
occupancy_grid, coords = mp.occupancy_grid_map(mapdata_path, size, buffer_size=15, visualize=True, saveas='occupancy_grid')


# Path planning:
start_time = time.time()
if algorithm == 'astar':
    print('Pathplanning - Running A*')
    path = astar.astar(occupancy_grid,start_coordinates,end_coordinates, size, timeout=120)
elif algorithm == 'rrtstar':
    print('Pathplanning - Running RRT*')
    path = rrtstar.rrtstar(np.rot90(np.flip(np.array(occupancy_grid),0),3), start_coordinates, end_coordinates)

print('Pathplanning - Path found! : ', len(path), ' waypoints')
algtime = time.time() - start_time
print(algtime, ' s')