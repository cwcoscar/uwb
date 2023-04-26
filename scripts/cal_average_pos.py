#! /usr/bin/env python
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import sys
import csv
import pymap3d as pm

dir = "/home/mec/Desktop/cwc/uwb/20230328test_frontbuilding/tag_location/"
filename = "tag04_fix"

def cal_bias_std(df):
    lat = df['field.latitude'].mean(axis = 0)
    lon = df['field.longitude'].mean(axis = 0)
    h = df['field.altitude'].mean(axis = 0)
    print("LLA : "+str(lat) + ", " + str(lon) + ", " + str(h))
    return lat, lon, h


if __name__ == '__main__':
    df = pd.read_csv(dir + filename + ".csv")
    (lat, lon, h) = cal_bias_std(df)
    print("--------------------------------")
    # The local coordinate origin (ee building)
    lat0 = 22.99665875 # deg
    lon0 = 120.222584889 # deg
    h0 = 98.211     # meters
    result = pm.geodetic2enu(lat, lon, h, lat0, lon0, h0)
    print(result)

