#! /usr/bin/env python
import pandas as pd
import numpy as np
import os
import matplotlib.pyplot as plt
import sys
import csv

test_ranges = ["60", "55", "50", "45", "40", "35", "30", "27", "23", "20", "17", "13", "10", "7", "3"]
T0_test_ranges = ["60", "55", "50", "45", "40", "35", "30", "27", "23", "17", "13", "10", "7", "3"]
T3_test_ranges = ["60", "55", "50", "45", "30", "27", "23", "20", "13", "10", "7", "3"]
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

    if filename[0:2] == "T"+str(tag_id):
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

def display_history(history, num_anchor, num_tag):
    true_range = []
    mean_range = []
    bias = []
    std = []

    for i in history:
        if i[0] == "T" + str(num_tag):
            true_range.append(i[1])
            mean_range.append(i[2][num_anchor])
            bias.append(i[3][num_anchor])
            std.append(i[4][num_anchor])

    pic = plt.plot(mean_range, bias,':', label='T'+ str(num_tag) + '_A' + str(num_anchor))
    plt.title('T' + str(num_tag) + '_bias')
    plt.legend(loc='upper left', shadow=False, fontsize='medium')
    plt.show()

    pic = plt.plot(true_range, std,':', label='T'+ str(num_tag) + '_A' + str(num_anchor))
    plt.title('T' + str(num_tag) + '_std')
    plt.legend(loc='upper left', shadow=False, fontsize='medium')
    plt.show()

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
    historys = []
    ####################################################
    # For saving mean measurement, bias and std in .csv#
    ####################################################
    # T = open(dir + "T" + str(tag_id) + "_from_3m_60m.csv", 'w')
    # T_writer = csv.writer(T)
    # data = ["TR", "Mean", "Bias", "STD"]
    # T_writer.writerow(data)

    ######################################################
    # Calculate mean measurement, bias, std with all data#
    ######################################################
    for i in range(len(test_ranges)):
        df = pd.read_csv(dir + "T" + str(tag_id) + "_" + test_ranges[i] + "m.csv")
        # (filename, true_range, mean, bias, std) = cal_bias_std_csv(df, "T" + str(tag_id) + "_" + test_ranges[i], test_ranges[i], T_writer) 
        # For saving mean measurement, bias and std in .csv
        (filename, true_range, mean, bias, std) = cal_bias_std(df, "T" + str(tag_id) + "_" + test_ranges[i], test_ranges[i])
        print("--------------------------------")
        historys.append([filename, true_range, mean, bias, std])
    ######################################################

    #########################################################
    # Calculate mean measurement, bias, std without outliers#
    #########################################################
    # if tag_id == 1:
    #     test_ranges = T0_test_ranges
    # elif tag_id == 3:
    #     test_ranges = T3_test_ranges

    # for i in range(len(test_ranges)):
    #     df = pd.read_csv(dir + "T" + str(tag_id) + "_" + test_ranges[i] + "m.csv")
    #     (filename, true_range, mean, bias, std) = cal_bias_std(df, "T" + str(tag_id) + "_" + test_ranges[i], test_ranges[i])
    #     print("--------------------------------")
    #     historys.append([filename, true_range, mean, bias, std])
    #########################################################

    display_history(historys, 0, tag_id)

    print("closed point from 3m ~ 60m")
    print("T" + str(tag_id) + " Mean Bias: ", calmeanbias(historys, 0, tag_id))

    sys.exit(0)



    