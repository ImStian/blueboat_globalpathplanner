from pymavlink import mavutil
import time
import math

##########################################################################################
''' Code written by Stian Bernhardsen (2024)'''
''' Thanks to Aaron Porter for assistance related to structuring some of the functions.'''
##########################################################################################

# The first section is dedicated to defining generic MAVLINK commands as Python functions

def ack(the_connection, keyword):
    ''' Informs if messages are recieved and acted upon.
        params:
            the_connection = connection object
            keyword        = what do you want to ack?
    '''
    print(f'\t MAVLINK - ACK - Message read: {str(the_connection.recv_match(type=keyword, blocking=True, timeout=1))}')


def establish_heartbeat(udp):
    # Start a connection listening to a UDP port
    print('MAVLINK - Trying to establish connection...')
    the_connection = mavutil.mavlink_connection(udp)

    # Wait for the first heartbeat
    #  This sets the system and component ID of remote systemf or the link
    the_connection.wait_heartbeat()
    print('MAVLINK - Heartbeat from the system (system %u component %u)' % 
        (the_connection.target_system, the_connection.target_component))
    return the_connection


def arm_disarm(the_connection, param):
    ''' Arms or disarms the boat the boat. 
        params:
            the_connection = connection object
            param          =  'ARM' (1) or 'DISARM (0)
    '''
    print('MAVLINK - Arming USV' if param else 'MAVLINK - Disarming USV')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                      mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, param, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")


def get_gps(the_connection):
    ''' Returns the latest GPS reading.'''
    return the_connection.recv_match(type='GLOBAL_POSITION_INT',blocking=True, timeout=1)


def wait_for_gps(the_connection):
    ''' Loops until GPS message has been read successfully.
        Considers (latitude = 0, longitude = 0) as invalid coordinates.
        Returns GPS-message
    '''
    # Fetching GPS coordinates from MAVLink
    gps = None
    while gps == None or (gps.lat == 0) or (gps.lon == 0):
        gps = the_connection.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
    return gps


def set_mode(the_connection, MAV_MODE):
    ''' Sets the mode using the MAV_MODE enum
        parms:
            the_connection = connection object
            MAV_MODE = See https://mavlink.io/en/messages/common.html#MAV_MODE for overview'''
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, MAV_MODE, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")


def takeoff(the_connection):
    ''' Runs takeoff command in MAVLINK'''
    print('MAVLINK - Running Takeoff command')

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, 0,
                                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0) # If you pass lat/long as 0 it uses the current position
    ack(the_connection, "COMMAND_ACK")



def set_home(the_connection, home_location, altitude):
    ''' Sets home location.
        params:
            the_connection = connection object
            home_location  = [latitude, longitude] (If set to 0,0 current location will be used)
            altitude       = Altitude above sea level in meters
    '''
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_HOME, 1,0,0, 0, 0,
                                         home_location[0], # Latitude
                                         home_location[1], # Longitude
                                         altitude)         # Altitude
    ack(the_connection, "COMMAND_ACK")


def go_home(the_connection):
    ''' Goes to Home position.'''
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 242, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")
    home_location = the_connection.recv_match(type=['HOME_POSITION'],blocking=True, timeout=1)
    latitude  = home_location.latitude  # Latitude 10^7
    longitude = home_location.longitude # Longitude 10^7
    altitude  = home_location.altitude  # Altitude 10^7
    the_connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, the_connection.target_system,
                                                                                           the_connection.target_component,
                                                                                           0, # MAV_FRAME_GLOBAL = 0
                                                                                           int(0b11011111100), # POSITION_TARGET_TYPEMASK
                                                                                           latitude,
                                                                                           longitude,
                                                                                           altitude,
                                                                                           0, # x_velocity in NED frame
                                                                                           0, # y_velocity in NED frame
                                                                                           0, # z_velocity in NED frame
                                                                                           0, # x_acceleration in NED frame
                                                                                           0, # y_acceleration in NED frame
                                                                                           0, # z_acceleration in NED frame
                                                                                           1.57, # yaw setpoint
                                                                                           0.5)) # yaw rate setpoint

def read_homelocation(the_connection):
    ''' Reads the current home position of the vehicle.
        Returns the WGS84-coordinates [latitude,longitude]
    '''
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0, 242, 0, 0, 0, 0, 0, 0)
    #ack(the_connection, "COMMAND_ACK") Enable for debugging of homelocation check, otherwise leave commented out
    home_location = the_connection.recv_match(type=['HOME_POSITION'],blocking=True, timeout=1)
    latitude  = home_location.latitude  # Latitude 10^7
    longitude = home_location.longitude # Longitude 10^7
    #altitude  = home_location.altitude  # Altitude 10^7 REDUNDANT FOR USVs
    return [latitude, longitude]



# The second section is dedicated to defining mission  related commands as Python functions

class mission_item():
    def __init__(self, seq, current, autocontinue, x, y):
        self.seq          = seq # Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4).
        self.frame        = mavutil.mavlink.MAV_FRAME_GLOBAL # MAV_FRAME_GLOBAL : Global (WGS84) coordinate frame + altitude relative to mean sea level (MSL).
        self.current      = current # false:0, true:1
        self.autocontinue = autocontinue # Autocontinue to next waypoint. 0: false, 1: true. Set false to pause mission after the item completes.
        self.command      = mavutil.mavlink.MAV_CMD_NAV_WAYPOINT # The scheduled action for the waypoint.
        self.param1       = 0 # Hold time [s]
        self.param2       = 2 # Accept Radius [m]
        self.param3       = 15 # Pass radius [m]
        self.param4       = math.nan # Yaw [deg] (NaN to use the current system yaw heading mode)
        self.latitude     = int(x*10**7) # global: latitude in degrees * 10^7
        self.longitude    = int(y*10**7) # global: longitude in degrees *10^7
        self.altitude     = 0 # global: altitude in meters (relative or absolute, depending on frame.
        self.mission_type = 0 # MAV_MISSION_TYPE assigned to main mission



def upload_mission(the_connection, mission_items):
    ''' Uploads a list of mission items where the elements are defined as mission_item objects.
        params:
            the_connection = connection object
            mission_items  = List of mission_item objects
    
    '''
    n = len(mission_items)
    print('MAVLINK - Uploading Mission Items...')

    # Informing MavLink that we are sending n mission items.
    the_connection.mav.mission_count_send(the_connection.target_system, the_connection.target_component, n, 0)

    ack(the_connection, "MISSION_REQUEST")

    for waypoint in mission_items:
        print(f'\t MAVLINK - Sending mission item ({mission_items.index(waypoint)+1}/{n})')
        the_connection.mav.mission_item_int_send(the_connection.target_system,
                                                 the_connection.target_component,
                                                 waypoint.seq,
                                                 waypoint.frame,
                                                 waypoint.command,
                                                 waypoint.current,
                                                 waypoint.autocontinue,
                                                 waypoint.param1,
                                                 waypoint.param2,
                                                 waypoint.param3,
                                                 waypoint.param4,
                                                 waypoint.latitude,
                                                 waypoint.longitude,
                                                 waypoint.altitude,
                                                 waypoint.mission_type)

    if waypoint != mission_items[n-1]:
        ack(the_connection, "MISSION_REQUEST")

    ack(the_connection, "MISSION_ACK")


def start_mission(the_connection):
    '''Starts mission'''
    print('MAVLINK - Starting Mission')
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_MISSION_START, 0, 0, 0, 0, 0, 0, 0, 0)
    ack(the_connection, "COMMAND_ACK")

def clear_mission(the_connection):
    '''Deletes all mission items at once'''
    print('MAVLINK - Deleting ALL mission items')
    the_connection.waypoint_clear_all_send()
    ack(the_connection, "MISSION_ACK")


def print_mission_items(the_connection):
    '''Prints mission items.'''
    print('MAVLINK - Printing mission items:')
    the_connection.waypoint_request_list_send() # Warning: got MISSION_REQUEST; use MISSION_REQUEST_INT!
    waypoint_count = 0
    msg = the_connection.recv_match(type=['MISSION_COUNT'], blocking=True, timeout=1)
    waypoint_count = msg.count
    print(f'\t MAVLINK - Counted {waypoint_count} waypoints')

    sequence = []
    for i in range(waypoint_count):
        the_connection.waypoint_request_send(i)
        msg = the_connection.recv_match(type=['MISSION_ITEM'],blocking=True, timeout=1)
        sequence.append(msg.seq)
        print(f'\t MAVLINK - Mission item {msg.seq}: [Latitude:{round((msg.x),10)}, Longitude:{round((msg.y),10)}] (Values are 10^7)')
    
    for  i, waypoint_number in enumerate(sequence):
        if (waypoint_number != sequence[-1]) and (waypoint_number > sequence[i+1]):
            MAV_RESULT = 4 # If the sequence is out of order give error
        else:
            MAV_RESULT = 0
    #the_connection.mav.mission_ack_send(the_connection.target_system, the_connection.target_component, MAV_RESULT) # ACK causes errors.


def wait_for_mission_completion(the_connection, n, mission_timeout=999999999999):
    '''Checks if the mission is complete
        params:
            the_connection  = connection object
            n               = number of mission items
            mission_timeout = seconds before mission throws out an error.
    '''
    while time.time() < mission_timeout:
        msg = the_connection.recv_match(type=['MISSION_ITEM_REACHED'], blocking=True, timeout=2)
        if msg != None:
            #print(msg) # Prints MISSION_ITEM_REACHED message (Used for Debugging)
            if msg.seq == (n-1):
                print('MAVLINK - Mission Completed')
                return True        
    print(f'MAVLINK - ERROR - Mission NOT COMPLETED within specified timeframe!')
    return False


if __name__ == '__main__':
    # Start a connection listening to a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

    # Wait for the first heartbeat
    #  This sets the system and component ID of remote systemf or the link
    the_connection.wait_heartbeat()
    print('MAVLINK - Heartbeat from the system (system %u component %u)' % 
        (the_connection.target_system, the_connection.target_component))


    points1 = [(63.435052284788725, 10.387699667907834), 
              (63.44410805393822, 10.378534730141928),
              (63.44575555220531, 10.408625207682311),
              (63.44046889595942, 10.411445188740531)]
    
    points2 = [(63.45309582272496, 10.382939373976727),
               (63.455148870536945, 10.385791305113983),
               (63.454400937090455, 10.38739658388185),
               (63.45296988750512, 10.387678361504705),
               (63.451744852227, 10.384553191505796)]

    
    mission_waypoints = []
    for waypoint in points2:
        item = mission_item(points2.index(waypoint), 0, 1, waypoint[0], waypoint[1])
        mission_waypoints.append(item)
    
    
    
    # These two are redundant. Use go to location in QGC instead
    #set_home(the_connection, (63.45033458273741, 10.378879055844461), 0)
    #go_home(the_connection)

    set_mode(the_connection, 192) # Changing mode to Preflight (Does not do anything)
    clear_mission(the_connection)
    set_mode(the_connection, 216) # Changing mode to Guided
    arm_disarm(the_connection, 1)
    takeoff(the_connection)

    go_home(the_connection)
    #upload_mission(the_connection, mission_waypoints)
    #start_mission(the_connection)

    print_mission_items(the_connection)
    wait_for_mission_completion(the_connection, len(mission_waypoints))
    print('Mission was completed!')
    clear_mission(the_connection)
    #arm_disarm(the_connection, 0)
