#! /usr/bin/env python
import csv
import pandas as pd
import os

def write_csv(writer, num):
    row = [num[0], num[1], num[2], num[3]]
    writer.writerow(row)


def value_validation(df, writer): ## Check if value of valid data == 0, malfunction
    for i in range(df.shape[0]):
        if df.iloc[i][0] <= 0:
            continue
        else:
            write_csv(writer, df.iloc[i])


dir = "/home/meclab/catkin_ws/src/uwb_YCHIOT/uwb_raw_data/20221004/csv/"
filename = os.listdir(dir + "raw")

for i in range(len(filename)):
    print(filename[i])
    df = pd.read_csv(dir + "raw/" + filename[i])
    
    if not (os.path.exists(dir + 'preprocessed/')):
        os.mkdir(dir + 'preprocessed/')

    f =  open(dir + 'preprocessed/' + filename[i], 'w')
    writer = csv.writer(f)
    row = ["A0", "A1", "A2", "A3"]
    writer.writerow(row)
    value_validation(df, writer)