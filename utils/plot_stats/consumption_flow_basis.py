#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt
import pandas as pd


from common import colors, markers

# Function to parse the file and extract MinMax data
def parse_file(file_path):
    min_max_data = {}

    with open(file_path, 'r') as file:
        for line in file:
            minmax_match = re.match(r"\[MinMax\] ([\w\d]+) ([\d.]+) ([\d.]+)", line)

            if minmax_match:
                x, y = map(float, minmax_match.groups()[1:])
                min_max_data[minmax_match.groups()[0]] = (x, y)

    return min_max_data

def read_csv_files(consumption_file_path):
    #cpu_data = pd.read_csv(cpu_file_path, delimiter=';')
    consumption_data = pd.read_csv(consumption_file_path, delimiter=';')

    switches = consumption_data['NodeName'].unique()
    
    #sum all the consumption of the switches with the same time
    data_consumption = consumption_data.groupby('Time')['Consumption'].sum().reset_index()
    #data_cpu = cpu_data.groupby('Time')['CPU_Usage'].mean().reset_index()

    return data_consumption, switches

# Function to plot the data
def plot_graph(trace_folders, cpuMaxCapacity):
    plt.title(f'Switch CPU Capacity: {cpuMaxCapacity}Mbps')
    plt.ylabel('Consumption (Wh)')
    plt.xlabel("Time (s)")
    plt.ylim(bottom=0)
    max_y_global = 0
    color_index = 0
    for trace_folder in trace_folders:
        consumption_data, switchs = read_csv_files(f'{trace_folder}/ecofen-trace.csv')

        # print("Consumption Data\n", consumption_data)
        # print("Switchs\n", switchs)
        switchsNr = len(switchs)

        # print("Cpu Data\n", cpu_data)
        # print("Consumption Data\n", consumption_data)
        # cpu_x, cpu_y = cpu_data['Time'].tolist(), cpu_data['CPU_Usage'].tolist()
        consumption_x, consumption_y = consumption_data['Time'].tolist(), consumption_data['Consumption'].tolist()
        consumption_x = [round(elem, 3) for elem in consumption_x]
        consumption_y = [round(elem, 3) for elem in consumption_y]

        # print("CPU X\n", cpu_x)
        # print("CPU Y\n", cpu_y)
        print(switchsNr-1, "Jump Consumption Y\n", consumption_y)

        col = colors[color_index % len(colors)]
        max_y = max(consumption_y)
        plt.plot(consumption_x, consumption_y, label=f'Jumps:{switchsNr-1}', color=col)
        plt.axhline(y=max_y, color=col, linestyle='--', label=f'Max: {max_y:.2f}')

        color_index += 1
        max_y *= 1.05
        if max_y > max_y_global:
            max_y_global = max_y

        
    plt.ylim(top=max_y_global)
    plt.legend()
    plt.show()

# Check if a filename is provided as a command-line argument
if len(sys.argv) < 2:
    print("Usage: python consumption_flow_basis.py <trace_folder1> <trace_folder2> ... <trace_folderN")
    sys.exit(1)

print("[WARNING] This scrip can only be aplyed to topologies that contain 1 line connection between switches.")

# Get the filename from the command-line argument
trace_folders = sys.argv[1:]

cpuMaxCapacity = input("Enter the CPU Max Capacity in Mbps: ")

plot_graph(trace_folders, cpuMaxCapacity)


