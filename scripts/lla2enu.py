#! /usr/bin/env python
import pymap3d as pm

if __name__ == '__main__':

    # The local coordinate origin (Zermatt, Switzerland)
    lat0 = 22.99665875 # deg
    lon0 = 120.222584889 # deg
    h0 = 98.211     # meters

    # The point of interest
    # lat = 22.996538  # deg
    # lon = 120.2226862   # deg
    # h = 92.771      # meters

    lat = 22  # deg
    lon = 120   # deg
    h =  98      # meters

    result = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    print(result)