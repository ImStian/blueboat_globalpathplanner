from pymavlink import mavutil

'''
      This script reads the GLOBAL_POSITION_INT (GPS) message from the MAVLink connection
      and prints it to the console. UDP-connection pre-configured to work with SITL.
      Requires "--out=127.0.0.1:14541" as an argument to the SITL instance.
'''


# Start a connection listening to a UDP port
the_connection = mavutil.mavlink_connection('udpin:localhost:14541')

# Wait for the first heartbeat
#  This sets the system and component ID of remote systemf or the link
the_connection.wait_heartbeat()
print('Heartbeat from the system (system %u component %u)' % 
      (the_connection.target_system, the_connection.target_component))

while 1:
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
    print(msg)