
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
        'x'  : [],
        'y' : [],
    }
    
    with open(f'{logpath}/mission_log_position_{identifier}.csv', 'r', newline='\n') as positionfile:
        reader = csv.reader(positionfile, delimiter=';')
        for row in reader:
            time, latitude, longitude = row
            xy = cc.utm33_to_wgs84(latitude, longitude, inverse=True)
            position['time'].append(time)
            position['x'].append(xy[0])
            position['y'].append(xy[1])
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
    size, startpos    = read_config(identifier, logpath)
    center = center =  [startpos[0] + size[0]/2, startpos[1] + size[1]/2]
    waypoints = read_waypoints(identifier, logpath)
    position  = read_position(identifier, logpath)

    # Configuring ENC to be identical to the mission
    uf.configure_enc(enc_settings, center, size)
    enc = ENC(config='map_settings.yaml')
    enc.update()

    # Drawing waypoint with lines between:
    waypoints_all_positions = []
    for index in range(len(position['x'])):
        position_coordinates = (position['x'][index], position['y'][index])
        waypoints_all_positions.append(position_coordinates)
    enc.display.draw_line(
        points=waypoints_all_positions,
        color = 'white',
        width=1,    
        )

    # Adding the waypointt gots
    for index in waypoints['seq']:
        waypoint_coordinates = (waypoints['x'][index], waypoints['y'][index])

        if index == 0:
            enc.display.draw_circle(center=waypoint_coordinates,
                                    radius=4.0,
                                    color='green',
                                    fill=True,
                                    )
        elif index == len(waypoints['seq']) - 1:
            enc.display.draw_circle(center=waypoint_coordinates,
                                    radius=4.0,
                                    color='red',
                                    fill=True,
                                    )        
        else:
            enc.display.draw_circle(center=waypoint_coordinates,
                                    radius=4.0,
                                    color='white',
                                    fill=True,
                                    )
    
    # Draw actual path
    gps_all_positions = []
    for index in range(len(position['x'])):
        position_coordinates = (position['x'][index], position['y'][index])
        gps_all_positions.append(position_coordinates)

    enc.display.draw_line(points=gps_all_positions,
                            color='blue',
                            width=0.25,
                            )


    enc.display.show()
    




#center = (271794.3121781586, 7042820.867264936)
#start = center
#end = (271669.0,7043912.5)



#enc = ENC('map_settings.yaml')
#enc.display.draw_arrow(start, end,color='red')
#enc.display.show()