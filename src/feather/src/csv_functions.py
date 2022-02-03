import os
import time
import csv
from pathlib import Path
from sensor_msgs.msg import PointCloud

BASE_DIR = Path(__file__).resolve().parent.parent

def generate_file_name():
    year = time.localtime()[0]
    month =  time.localtime()[1]
    day =  time.localtime()[2]
    hour =  time.localtime()[3]
    mins =  time.localtime()[4]
    sec =  time.localtime()[5]
    name = F"{year}_{month}_{day}_{hour}_{mins}_{sec}"
    return name

def save_csv_data(file_name, data):
    print(F"Saving file to")
    file_path = F"{BASE_DIR}/data/{file_name}_val.csv"
    print(file_path)
    file = open(file_path, "w+")
    file.write(F"x,y\n")
    file.close()
    file = open(file_path, "a")
    for point in data.points:
        file.write(F"{point.x},{point.y}\n")
    file.close()
    file_path = F"{BASE_DIR}/data/{file_name}.txt"
    file = open(file_path, "w+")
    str_dict = repr(data)
    file.write(str_dict)

def read_csv_data(file_name):
    file_handle = open(file_name, 'r', encoding='utf-8')
    csv_reader = csv.DictReader(file_handle)
    print(csv_reader)
    point_cloud = []
    for row in csv_reader:
        point_cloud.append(Point(row['x'], row[' y']))

    file_handle.close()
    return point_cloud

class Point():
    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)
