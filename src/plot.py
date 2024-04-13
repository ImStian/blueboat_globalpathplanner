
import src.coordinate_conversion as cc
import src.util_functions as uf
from seacharts import ENC
import csv

def read_config(identifier, logpath):
    '''Reads the mission log config file and returns the values as a dictionary. (Used in plot_mission())
        params:
            identifier = unique identifier tied to the logs (datetime in filename)
            logpath     = path to the mission logfile directory
    '''



    with open(f'{logpath}/mission_log_config_{identifier}.csv', 'r', newline='\n') as configfile:
        reader = csv.reader(configfile, delimiter=';')
        for row in reader:
            size0, size1, center0, center1 = row
            size = (int(size0), int(size1))
            center = (float(center0), float(center1))    
    return size, center


def read_position(identifier, logpath):
    '''Reads the mission log position file and returns the values as a dictionary. (Used in plot_mission())
        params:
            identifier = unique identifier tied to the logs (datetime in filename)
            logpath     = path to the mission logfile directory
    '''

    position = {
        'time'      : [],
        'latitude'  : [],
        'longitude' : [],
    }
    
    with open(f'{logpath}/mission_log_position_{identifier}.csv', 'r', newline='\n') as positionfile:
        reader = csv.reader(positionfile, delimiter=';')
        for row in reader:
            time, latitude, longitude = row
            position['time'].append(time)
            position['latitude'].append(latitude)
            position['longitude'].append(longitude)
    return position


def read_waypoints(identifier, logpath):
    '''Reads the mission log waypoints file and returns the values as a dictionary. (Used in plot_mission())
        params:
            identifier = unique identifier tied to the logs (datetime in filename)
            logpath     = path to the mission logfile directory
    '''

    waypoints = {
        'time'      : [],
        'seq'       : [],
        'x'  : [],
        'y' : [],
    }
    
    with open(f'{logpath}/mission_log_waypoints_{identifier}.csv', 'r', newline='\n') as positionfile:
        reader = csv.reader(positionfile, delimiter=';')
        for row in reader:
            time, seq, latitude, longitude = row
            xy = cc.utm33_to_wgs84(latitude, longitude, inverse=True)

            waypoints['time'].append(time)
            waypoints['seq'].append(int(seq))
            waypoints['x'].append(float(xy[0]))
            waypoints['y'].append(float(xy[1]))
    return waypoints





def plot_mission(enc_settings, identifier, logpath):
    ''' Uses the mission logs (both position and waypoints) to plot the planned 
        path and the actual path taken during the mission.
        params:
            identifier = unique identifier tied to the logs (datetime in filename)
            config     = path to the config log file
            waypoints  = path to the waypoint log file
            positions  = path to the position log file
    '''

    # Reading the mission logs
    size, center    = read_config(identifier, logpath)
    waypoints = read_waypoints(identifier, logpath)
    position  = read_position(identifier, logpath)

    # Configuring ENC to be identical to the mission
    uf.configure_enc(enc_settings, (272338.26,7043905.93), size)
    enc = ENC(config='map_settings.yaml')
    enc.update()

    # Drawing waypoint with arrows between:
    previous_waypoint_coordinates = 0
    for index in waypoints['seq']:
        waypoint_coordinates = (waypoints['x'][index], waypoints['y'][index])
        print(waypoint_coordinates)
        enc.display.draw_circle(center=waypoint_coordinates,
                                radius=4.0,
                                color='white',
                                fill=True,
                                )

        if (previous_waypoint_coordinates != 0):
            enc.display.draw_arrow(start=previous_waypoint_coordinates,
                                end=waypoint_coordinates,
                                color='white',
                                width=0.25,
                                fill=True)
    
        previous_waypoint_coordinates = waypoint_coordinates
    

    enc.display.show()




#center = (271794.3121781586, 7042820.867264936)
#start = center
#end = (271669.0,7043912.5)



#enc = ENC('map_settings.yaml')
#enc.display.draw_arrow(start, end,color='red')
#enc.display.show()