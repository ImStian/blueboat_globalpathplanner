import src.mavlink_communication as mav
import src.coordinate_conversion as cc
from time import localtime, strftime
import src.map_preparation as mp
import src.util_functions as uf
import src.rrtstar as rrtstar
import src.astar as astar
import numpy as np
import argparse
import time
import os

if __name__ == '__main__':
    # Parser-arguments for running the Python file:
    parser = argparse.ArgumentParser(description = 'Global path planner parameters:')
    parser.add_argument('-size', type=int, default=2500, metavar='map_size', action='store', 
                        dest='size_single', help='Size of map extraction (Default=2500). A smaller map will limit path length.')
    parser.add_argument('-algo', type=str, default='astar', metavar='pathplanning_algorithm', action='store', 
                        dest='algorithm', help='Selection of which path planning algorithm to be used. Valid entries:  astar  ,  rrtstar')
    args = parser.parse_args()


    # Filepath for settings and mapsaves:
    mapdata_path = f'{os.getcwd()}/map_data/'
    setting_path = f'{os.getcwd()}/map_settings.yaml'
    logging_path = f'{os.getcwd()}/mission_logs/'


    # Initializing connection via MAVLink
    the_connection = mav.establish_heartbeat('udpin:192.168.2.1:14770')

    mav.clear_mission(the_connection) # Clearing previous mission

    # Declaring variables used for starting path planning when new home is detected
    previoushome = [0,0]
    start_pathplanning = False
    first_run = True

    print('General - Click "Pathfind to location" (change home) in QGC to start.')
    while True:
        # Adding extra check to prevent pathplanning from starting instantly upon running script:
        if start_pathplanning and first_run: # If home has been updated
            print('Path planning - Destination request received.')
            start_pathplanning = False # Setting start_pathplanning variable to false

            print('Path planning - ERROR! To prevent unintentional behaviour upon startup, please identify desired end-point again.')
            first_run = False


        if start_pathplanning and not first_run: # If home has been updated
            print('Path planning - Destination request received.')
            start_pathplanning = False # Setting start_pathplanning variable to false

            identifier = strftime("%d_%m_%Y_%H_%M_%S", localtime()) # Mission identifier (Used for log files)

            # Fetching current position of vehicle
            gps = mav.wait_for_gps(the_connection)
            print(f'GPS - Current boat position {gps}')

            # Converting positions from WGS84 to UTM33:
            utm33_current_position = cc.utm33_to_wgs84(gps.lat / 10**7, gps.lon / 10**7, inverse=True)
            utm33_home_position    = cc.utm33_to_wgs84(home[0] / 10**7, home[1] / 10**7, inverse=True)



            # Defining map size and center
            size   = [args.size_single, args.size_single]
            center =  [utm33_current_position[0] - size[0]/2, utm33_current_position[1] - size[1]/2] # Centered on current position
            center_enc = [utm33_current_position[0] , utm33_current_position[1]] # Centered on current position
            uf.configure_enc(setting_path, center=center_enc, size=size) # Updating .yaml file
            uf.log_enc_config(identifier, logging_path, size, center)


            # Define pathplanning start/end (grid coordinates)           
            grid_current_position = cc.utm33_to_grid(utm33_current_position[0], utm33_current_position[1], size, center)
            grid_home_location = cc.utm33_to_grid(utm33_home_position[0], utm33_home_position[1], size, center)


            # Define pathplanning start/end (grid coordinates)           
            if grid_current_position[0] > size[0] or grid_current_position[0] < 0 or grid_current_position[1] > size[1] or grid_current_position[1] < 0:
                print('Map Function - ERROR: Location out of bounds!')
                quit()

            start_coordinates = (grid_current_position[0], grid_current_position[1])
            end_coordinates   = (grid_home_location[0], grid_home_location[1])

            # Extracting map data using SeaCharts:
            mp.extract_map_data(mapdata_path, center, new_data=True)
            #polygons, polygons_coords = mp.prepare_map_polygons_coords(mapdata_path)
            #mp.map_visualization(polygons_coords, saveas='map_visualization')

            # Generating occupancy grid:
            occupancy_grid, coords = mp.occupancy_grid_map(mapdata_path, size, buffer_size=2, visualize=True, saveas='occupancy_grid')


            # Path planning:
            if args.algorithm == 'astar':
                print('Pathplanning - Running A*')
                path = astar.astar(occupancy_grid,start_coordinates,end_coordinates, size, timeout=2000000000)
            elif args.algorithm == 'rrtstar':
                print('Pathplanning - Running RRT*')
                path = rrtstar.rrtstar(np.rot90(np.flip(np.array(occupancy_grid),0),3), start_coordinates, end_coordinates)

            print('Pathplanning - Path found! : ', len(path), ' waypoints')
            uf.plot_path(occupancy_grid, path, 'Path planning')

            # Converting coordinates to WGS84 (latitude/longitude):
            print('Coordinate Conversion - Converting from Occupancy Grid-coordinates to WGS84.')
            wgs84_waypoints = [
            cc.grid_to_wgs84(waypoint[0], waypoint[1], size, center)
                for waypoint in path
            ]
            print('Coordinate Conversion - Conversion Done!')

            # Uploading waypoints to vehicle via MAVLink:
            print('MAVLINK - Uploading Waypoints via mavlink')
            mission_waypoints = []
            for waypoint in wgs84_waypoints:
                item = mav.mission_item(wgs84_waypoints.index(waypoint), 0, 1, waypoint[1], waypoint[0])
                mission_waypoints.append(item)
            mav.upload_mission(the_connection, mission_waypoints)
            print('MAVLINK - Upload concluded!')

            # Starting mission:
            print('MAVLINK - Starting Mission')
            mav.set_mode(the_connection, 220) # Changing mode to Guided
            mav.arm_disarm(the_connection, 1) # Arming
            mav.takeoff(the_connection)
            mav.set_mission_current(the_connection, 0)
            mav.start_mission(the_connection)

            mission_items = mav.print_mission_items(the_connection)
            uf.log_mission_items(identifier,logging_path, mission_items)
            mav.log_until_completion(the_connection, identifier, len(mission_waypoints), logging_path) # Need to be replaced with "log_position_until_completion(the_connection, len(mission_waypoints))", which stores data within a csv file to be used for plotting and analysis of actual path compared to planned path.
            print('Mission was completed!')

            #mav.arm_disarm(the_connection, 0) # Disarming
            mav.land(the_connection)
            mav.set_mode(the_connection, 192) # Changing mode to manual            
            mav.clear_mission(the_connection)




        else:

           home = mav.read_homelocation(the_connection)  # Fetching desired position of vehicle (home position)
           if home != previoushome: 
                start_pathplanning = True
                previoushome = home

           time.sleep(0.2)


