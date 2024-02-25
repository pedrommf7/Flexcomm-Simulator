#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt

# Function to parse the file and extract CPU and Consumption data
def parse_file(file_path):
    cpu_data = []
    consumption_data = []

    with open(file_path, 'r') as file:
        for line in file:
            cpu_match = re.match(r"\[CPU\] ([\d.]+) ([\d.]+)", line)
            consumption_match = re.match(r"\[Consumption\] ([\d.]+) ([\d.]+)", line)

            if cpu_match:
                x, y = map(float, cpu_match.groups())
                cpu_data.append((x, y))
            elif consumption_match:
                x, y = map(float, consumption_match.groups())
                consumption_data.append((x, y))

    return cpu_data, consumption_data

# Function to plot the data
def plot_graph(cpu_data, consumption_data):
    cpu_x, cpu_y = zip(*cpu_data)
    consumption_x, consumption_y = zip(*consumption_data)

    plt.plot(cpu_x, cpu_y, label='CPU')
    plt.plot(consumption_x, consumption_y, label='Consumption')
    
    plt.xlabel("Time")
    plt.ylabel('Values')
    plt.legend()
    plt.show()

# Check if a filename is provided as a command-line argument
if len(sys.argv) != 2:
    print("Usage: python script.py <filename>")
    sys.exit(1)

# Get the filename from the command-line argument
file_path = sys.argv[1]
print(f'File Path: {file_path}')

cpu_data, consumption_data = parse_file(file_path)
print(f'CPU Data: {cpu_data}')
print(f'Consumption Data: {consumption_data}')

plot_graph(cpu_data, consumption_data)
