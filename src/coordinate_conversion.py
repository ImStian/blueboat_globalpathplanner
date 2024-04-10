import numpy as np
from pyproj import Transformer # For coordinate transforming


def utm33_to_grid(coordinate1, coordinate2, size, center, inverse=False):
    """ Converts from UTM Zone 33N- to occupancy grid-coordinates (and  vice versa).
        Returns a tuple containing the converted coordinate.
        Input:
            - coordinate1: in either system
            - coordinate2: in either system
            - size: 
            - inverse: If "False", the  function converts from UTM to grid-coordinates (default option) 
                       if "True" it does the opposite.
        ("forward-mode" is based of code provided by Melih Ǎkdag)
    """
    if inverse:
        easting = coordinate1 + center[0]
        northing = size[1] - coordinate2 + center[1]
        return np.array([easting, northing])
    else: # This one does not work
        x = coordinate1 - center[0]
        y = coordinate2 - center[1]
        return np.array([x, size[1] - y]).astype(int)
            


def utm33_to_wgs84(coordinate1, coordinate2, inverse=False):
    """ Converts from UTM Zone 33N- to WGS84-coordinates (and  vice versa).
        Returns a tuple containing the converted coordinate.
        Input:
            - coordinate1: either latitude or x-value
            - coordinate2: either longitude or y-value
            - inverse: If "False", the  function converts from UTM to WGS84, 
                       if "True" it does the opposite. Default is "False".
    """
    # EPSG codes for UTM and WGS84
    WGS84  = 'EPSG:4326' # Latitude / Longitude
    UTM33  = 'EPSG:25833' # UTM Zone 33N

    if inverse:   
        transformer = Transformer.from_crs(WGS84, UTM33)
        return transformer.transform(coordinate1, coordinate2) # Necessary arg order
    else:
        transformer = Transformer.from_crs(UTM33, WGS84)
        converted_coordinates = transformer.transform(coordinate1, coordinate2) # Necessary arg order
        # transform returns reverse order (longitude, latitude), reversing to get (latitude, longitdue)
        return (converted_coordinates[1], converted_coordinates[0])
    

def grid_to_wgs84(coordinate1, coordinate2, size, center, inverse=False):
    """
        Combines the functions utm33_to_wgs84() and utm33_to_grid().
    """

    utm33 = utm33_to_grid(coordinate1, coordinate2, size, center, inverse=True)
    return  utm33_to_wgs84(utm33[0], utm33[1])


