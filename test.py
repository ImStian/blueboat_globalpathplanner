import src.plot as plt
import os

'''
    This is a temporary script to test the plotting functionality.
    It is not part of the main program. (Does not work right now)
'''


logging_path = f'{os.getcwd()}/mission_logs/'
identifier = '12_04_2024_17_40_49'

plt.plot_mission(identifier, logging_path)