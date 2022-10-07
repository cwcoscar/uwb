#! /usr/bin/env python
import csv
import pandas as pd
import os
import sys

def write_csv(writer, num):
    row = [num[0], num[1], num[2], num[3]]
    writer.writerow(row)


def value_validation(df, writer, tr): ## Check if value of valid data == 0, malfunction
    for i in range(df.shape[0]):
        if df.iloc[i][0] <= 0 or df.iloc[i][0] >= tr+1 or df.iloc[i][0] <= tr-1:
            continue
        else:
            write_csv(writer, df.iloc[i])

if __name__ == '__main__':
    dir = "/home/meclab/catkin_ws/src/uwb_YCHIOT/uwb_raw_data/20221006/csv/"
    filename = os.listdir(dir + "raw")

    for i in range(len(filename)):
        print(filename[i])
        df = pd.read_csv(dir + "raw/" + filename[i])
        true_range = int(filename[i][:len(filename[i])-4].split("_")[1].replace("m",""))
        
        if not (os.path.exists(dir + 'preprocessed/')):
            os.mkdir(dir + 'preprocessed/')

        f =  open(dir + 'preprocessed/' + filename[i], 'w')
        writer = csv.writer(f)
        row = ["A0", "A1", "A2", "A3"]
        writer.writerow(row)
        value_validation(df, writer,true_range)
    sys.exit(0)