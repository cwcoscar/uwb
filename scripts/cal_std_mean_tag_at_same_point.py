#! /usr/bin/env python
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import sys
import csv

test_range = ["30"]
dir = "/home/meclab/catkin_ws/src/uwb_YCHIOT/uwb_raw_data/20221006/csv/preprocessed/"
tag_id = 3


def cal_bias_std_csv(df, filename, true_range, writer):
    mean = df.mean(axis = 0)
    tr = int(true_range)
    bias = mean - tr
    std = df.std(axis = 0)

    print(filename + ": ")
    print("True range: "+ str(true_range))
    print("")
    print("Bias")
    print(bias)
    print("")
    print("STD")
    print(std)

    if filename[0:2] == "T0":
        data = [tr, mean[0], bias[0], std[0]]
        writer.writerow(data)

    if filename[0:2] == "T3":
        data = [tr, mean[0], bias[0], std[0]]
        writer.writerow(data)

    return filename[0:2], tr, mean, bias, std

def cal_bias_std(df, filename, true_range):
    mean = df.mean(axis = 0)
    tr = int(true_range)
    bias = mean - tr
    std = df.std(axis = 0)

    print(filename + ": ")
    print("True range: "+ str(true_range))
    print("")
    print("Bias")
    print(bias)
    print("")
    print("STD")
    print(std)

    return filename[0:2], tr, mean, bias, std

def calmeanbias(history, num_anchor, num_tag):
    bias = []
    mean_bias = 0
    for i in history:
        if i[0] == "T" + str(num_tag):
            bias.append(i[3][num_anchor])
    
    for i in range(len(bias)):
        mean_bias = mean_bias + bias[i]
    mean_bias = mean_bias / len(bias)
    return mean_bias


if __name__ == '__main__':
    history = []
    #####################################################
    # For saving mean measurement, bias and std in .csv #
    #####################################################
    T = open(dir + "T" + str(tag_id) + "_same_place.csv", 'w')
    T_writer = csv.writer(T)
    data = ["TR", "Mean", "Bias", "STD"]
    T_writer.writerow(data)
    #####################################################

    #############################################################################
    # Calculate mean measurement, bias, std of 2 closed tags at a certain range #
    #############################################################################
    for i in range(len(test_range)):
        df = pd.read_csv(dir + "T" + str(tag_id) + "_" + test_range[i] + "m_same_pos.csv")
        (filename, true_range, mean, bias, std) = cal_bias_std_csv(df,  "T" + str(tag_id) + "_" + test_range[i], test_range[i], T_writer)
        # For saving mean measurement, bias and std in .csv #
        # (filename, true_range, mean, bias, std) = cal_bias_std(df,  "T" + str(tag_id) + "_" + test_range[i], test_range[i])
        print("--------------------------------")
        history.append([filename, true_range, mean, bias, std])
    #############################################################################

    print("Same point @ 30m")
    print("T" + str(tag_id) + " Mean Bias: ", calmeanbias(history, 0, tag_id))
    sys.exit(0)



    