from src.plot import plot_mission
import os

'''
    This is a temporary script to test the plotting functionality.
    It is not part of the main program. (Does not work right now)
'''

setting_path = f'{os.getcwd()}/map_settings.yaml'
logging_path = f'{os.getcwd()}/mission_logs/'
identifier = '13_04_2024_15_29_34'

identifier = input('Enter identifier: ')

plot_mission(setting_path, identifier, logging_path)
