import src.coordinate_conversion as cc
import src.map_preparation as mp
import src.util_functions as uf
import matplotlib.pyplot as plt
import src.plot as pl
import numpy as np
import os

identifier = input('Enter identifier: ')
mapdata_path = f'{os.getcwd()}/map_data/'
setting_path = f'{os.getcwd()}/map_settings.yaml'
logging_path = f'{os.getcwd()}/mission_logs/'

# Reading Configuration Mission Log
size, center, algo,  target, note =  pl.read_config(identifier, logging_path)
center_enc = [center[0] + size[0]/2, center[1] + size[1]/2]

# Reading Position Mission Log
position = pl.read_position(identifier, logging_path)

grid_position_x = []
grid_position_y = []
for i in range(len(position['x'])):
    utm_coordinate  = [position['x'][i], position['y'][i]]
    grid_coordinate = cc.utm33_to_grid(utm_coordinate[0], utm_coordinate[1], size, center)
    grid_position_x.append(grid_coordinate[0])
    grid_position_y.append(grid_coordinate[1])

# Sorting data to ensure no weird backtracking when plotting

data = list(zip(grid_position_x, grid_position_y))
data.sort()  # Sort by the first element of each tuple (x-coordinate)
grid_position_x, grid_position_y = zip(*data)

# Reading Waypoint Mission Log
waypoints = pl.read_waypoints(identifier, logging_path)
grid_waypoints_x = []
grid_waypoints_y = []


print(len(waypoints['seq']))
for i in range(len(waypoints['seq'])):
    if waypoints['seq'][i] == 1:
        continue
    utm_coordinate = [waypoints['x'][i], waypoints['y'][i]]
    grid_coordinate = cc.utm33_to_grid(utm_coordinate[0], utm_coordinate[1], size, center)
    grid_waypoints_x.append(grid_coordinate[0])
    grid_waypoints_y.append(grid_coordinate[1])
# Sorting data to ensure no weird backtracking when plotting
data = list(zip(grid_waypoints_x, grid_waypoints_y))
data.sort()  # Sort by the first element of each tuple (x-coordinate)
grid_waypoints_x, grid_waypoints_y = zip(*data)


# Extracting ENC data to get accurate background
uf.configure_enc(setting_path, center_enc, size)
mp.extract_map_data(mapdata_path, center, new_data=True)
land_grid, coords  = mp.occupancy_grid_map(mapdata_path, size, buffer_size=1, visualize=False, saveas='occupancy_grid', landshore='land')
shore_grid, coords = mp.occupancy_grid_map(mapdata_path, size, buffer_size=1, visualize=False, saveas='occupancy_grid', landshore='shore')

# It is necessary to rotate grids to match up with reality
land_grid_rotated = np.rot90(np.flip(np.array(land_grid),0), 3)
shore_grid_rotated = np.rot90(np.flip(np.array(shore_grid), 0), 3)

# Defining the occupancygrids as the background image
backdrop = np.add(land_grid_rotated, shore_grid_rotated)
color_map = {0:'steelblue', 1:'skyblue', 2:'black'}
plt.imshow(backdrop, cmap=plt.cm.colors.ListedColormap([color_map[i] for i in sorted(color_map.keys())]))

# Drawing a line
plt.plot(grid_waypoints_x, grid_waypoints_y, markersize=12, linewidth=2, c='orange', label='Path', linestyle='--')
plt.plot(grid_position_x, grid_position_y, markersize=12, linewidth=3, c='purple',  alpha=0.75, label='Position')

# Highlighting all waypoints/position readings
#plt.scatter(grid_position_x, grid_position_y, s=5, c='purple')
plt.scatter(grid_waypoints_x, grid_waypoints_y, s=40, c='orange',  edgecolors='black', linewidths=1, label='Waypoints', zorder=2)

# MARKING START AND END
plt.scatter(grid_waypoints_x[0], grid_waypoints_y[0], s=50, c='limegreen', edgecolors='black', linewidths=1, label='Start Location', zorder=3) # Start
plt.scatter(grid_waypoints_x[-1], grid_waypoints_y[-1], s=50, c='red', edgecolors='black', linewidths=1, label='Target Location', zorder=4) # Start


plt.legend()
algo_text = 'A*' if algo == 'astar' else 'RRT*'
plt.title(f'Global Path Planner: {algo_text}')

plt.xlabel('Meters from ENC center')
plt.ylabel('Meters from ENC center')


plt.show()
plt.savefig("./data/%s.png"%'plot', dpi=300, bbox_inches='tight', pad_inches=0)
plt.close()
