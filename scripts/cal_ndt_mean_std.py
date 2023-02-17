#! /usr/bin/env python
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import sys
import csv

dir = "/home/meclab/Desktop/Bag/test_uwb/20221024/"
filename = "_1180cm"
tag_id = 0

def cal_bias_std(df, filename):
    mean = df.mean(axis = 0)
    std = df.std(axis = 0)

    print(filename + ": ")
    print("Mean range:")
    print(mean)
    print("")
    print("STD")
    print(std)

    return filename[0:2], mean, std


if __name__ == '__main__':
    df = pd.read_csv(dir + "T" + str(tag_id) + filename + ".csv")
    (filename, mean, std) = cal_bias_std(df, "T" + str(tag_id) + filename)
    print("--------------------------------")