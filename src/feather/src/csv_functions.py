import os
import time
import csv
from pathlib import Path

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

def save_csv_data(file_name, points):
    print(F"Saving file to")
    file_path = F"{BASE_DIR}/data/{file_name}.csv"
    print(file_path)
    os.system(F"touch {file_path}")
    file = open(file_path, "w+")
    file.write(F"x, y\n")
    file.close()
    file = open(file_path, "a")
    for point in points:
        file.write(F"{point.x}, {point.y}\n")
    file.close()
