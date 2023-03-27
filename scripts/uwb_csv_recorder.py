#! /usr/bin/env python
import rospy
import csv
from uwb_YCHIOT.msg import uwb_raw
import os
import signal
import sys

row_T0 = 1
row_T1 = 1
row_T2 = 1
row_T3 = 1
dir = '/home/mec/Desktop/cwc/20230311/'
# distance = '32m'
distance = '20230312_uwbraw_T2T3_60m'

def signal_handler(sig, frame):
    global row_T0
    global row_T1
    global row_T2
    global row_T3
    file_name0 = dir + 'T0_' + distance + '.csv'
    file_name1 = dir + 'T1_' + distance + '.csv'
    file_name2 = dir + 'T2_' + distance + '.csv'
    file_name3 = dir + 'T3_' + distance + '.csv'

    if row_T0 == 1:
        os.remove(file_name0)
    if row_T1 == 1:
        os.remove(file_name1)
    if row_T2 == 1:
        os.remove(file_name2)
    if row_T3 == 1:
        os.remove(file_name3)

    print('Press Ctrl+C')
    sys.exit(0)

def check_dir(file_name):
    if os.path.isfile(file_name):
        print(file_name)
        print("File exists! Check the file name!")
        return True
    else:
        return False

def initialize():
    file_name0 = dir + 'T0_' + distance + '.csv'
    file_name1 = dir + 'T1_' + distance + '.csv'
    file_name2 = dir + 'T2_' + distance + '.csv'
    file_name3 = dir + 'T3_' + distance + '.csv'
    if check_dir(file_name0) or check_dir(file_name1) or check_dir(file_name2) or check_dir(file_name3):
        return False
    
    T0 = open(file_name0, 'w')
    T0_writer = csv.writer(T0)
    T1 = open(file_name1, 'w')
    T1_writer = csv.writer(T1)
    T2 = open(file_name2, 'w')
    T2_writer = csv.writer(T2)
    T3 = open(file_name3, 'w')
    T3_writer = csv.writer(T3)

    writer_list = [T0_writer, T1_writer, T2_writer, T3_writer]

    for i in range(len(writer_list)):
        row = ["A0", "A1", "A2", "A3"]
        writer_list[i].writerow(row)

    return writer_list

def write_csv(writer, raw_data, row_number):
    num_anchor = 4
    row = []
    if raw_data.A0:
        row.append(raw_data.distance_to_A0)
    else:
        row.append(0)
    if raw_data.A1:
        row.append(raw_data.distance_to_A1)
    else:
        row.append(0)
    if raw_data.A2:
        row.append(raw_data.distance_to_A2)
    else:
        row.append(0)
    if raw_data.A3:
        row.append(raw_data.distance_to_A3)
    else:
        row.append(0)

    writer.writerow(row)
    print(row_number, row)

def Uwbrawcallback(data, arg):
    global row_T0
    global row_T1
    global row_T2
    global row_T3
    if data.Tag_id == 0:
        write_csv(arg[0],data,row_T0)
        row_T0 = row_T0 + 1
    if data.Tag_id == 1:
        write_csv(arg[1],data,row_T1)
        row_T1 = row_T1 + 1
    if data.Tag_id == 2:
        write_csv(arg[2],data,row_T2)
        row_T2 = row_T2 + 1
    if data.Tag_id == 3:
        write_csv(arg[3],data,row_T3)
        row_T3 = row_T3 + 1


if __name__ == '__main__':
    writer_list = initialize()
    signal.signal(signal.SIGINT, signal_handler)
    if writer_list :
        rospy.init_node('uwb_record_csv', anonymous=True)
        rospy.Subscriber("uwb_raw", uwb_raw, Uwbrawcallback, writer_list)
        rospy.spin()
    sys.exit(0)


