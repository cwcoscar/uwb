#! /usr/bin/env python
import rospy
import csv
from uwb.msg import uwb_raw
import os

ur_data = uwb_raw()
row_T0 = 1

def check_dir(file_name):
    directory = os.path.isfile(file_name)

def initialize():
    file_name = '/home/meclab/catkin_ws/src/uwb_YCHIOT/uwb_raw_data/20221004/csv/raw/T0_25m.csv'
    if os.path.isfile(file_name):
        print(file_name)
        print("File exists! Check the file name!")
        return False
    
    T0 = open(file_name, 'w')
    T0_writer = csv.writer(T0)

    # T1 = open('/home/meclab/cwc/uwb_raw_data/20221004/csv/T1_3m.csv', 'w')
    # T1_writer = csv.writer(T1)
    # writer_list = [T0_writer, T1_writer]
    writer_list = [T0_writer]

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
    write_csv(arg[0],data,row_T0)
    row_T0 = row_T0 + 1


if __name__ == '__main__':
    writer_list = initialize()
    if writer_list :
        rospy.init_node('uwb_record_csv', anonymous=True)
        rospy.Subscriber("uwb_raw", uwb_raw, Uwbrawcallback, writer_list)
        rospy.spin()


