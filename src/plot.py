from seacharts import ENC
import src.util_functions as uf
import numpy as np
'''
    Plotting functionality is not yet fully implemented, will come soon. :)
'''

#center = (271794.3121781586, 7042820.867264936)
#start = center
#end = (271669.0,7043912.5)



#enc = ENC('map_settings.yaml')
#enc.display.draw_arrow(start, end,color='red')
#enc.display.show()


def plot_mission(identifier, logpath, ):
    ''' Uses the mission logs (both position and waypoints) to plot the planned 
        path and the actual path taken during the mission.
        params:
            identifier = unique identifier tied to the logs (datetime in filename)
            config     = path to the config log file
            waypoints  = path to the waypoint log file
            positions  = path to the position log file
    '''
    size, center = np.loadtxt(f'{logpath}/mission_log_config_{identifier}.csv', 
                              unpack=True, 
                              skiprows=0,
                              delimiter=';')
    
    print('size: ', size)
    print('center: ', center)

