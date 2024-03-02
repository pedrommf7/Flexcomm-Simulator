#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt
import pandas as pd

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

def read_csv_files(cpu_file_path, consumption_file_path, switch_name):
    print("CPU File Path", cpu_file_path)
    print("Consumption File Path", consumption_file_path)
    cpu_data = pd.read_csv(cpu_file_path, delimiter=';')
    consumption_data = pd.read_csv(consumption_file_path, delimiter=';')

    cpu_data = cpu_data[cpu_data['NodeName'] == switch_name]
    consumption_data = consumption_data[consumption_data['NodeName'] == switch_name]

    return cpu_data, consumption_data

# Function to plot the data
def plot_graph(trace_folder, min_max_data, switch_name):
    min_value, max_value = None, None
    if(min_max_data is not None):
        min_value, max_value = min_max_data[switch_name]

    cpu_data, consumption_data = read_csv_files(f'{trace_folder}/switch-stats.csv', f'{trace_folder}/ecofen-trace.csv', switch_name)

    # print("Cpu Data\n", cpu_data)
    # print("Consumption Data\n", consumption_data)
    cpu_x, cpu_y = cpu_data['Time'].tolist(), cpu_data['CPU_Usage'].tolist()
    consumption_x, consumption_y = consumption_data['Time'].tolist(), consumption_data['Consumption'].tolist()
    cpu_x = [round(elem, 3) for elem in cpu_x]
    cpu_y = [round(elem, 3) for elem in cpu_y]
    consumption_x = [round(elem, 3) for elem in consumption_x]
    consumption_y = [round(elem, 3) for elem in consumption_y]

    # print("CPU X\n", cpu_x)
    print("CPU Y\n", cpu_y)
    # print("Consumption X\n", consumption_x)
    print("Consumption Y\n", consumption_y)

    fig, ax1 = plt.subplots()
    ax1.set_title(f"{trace_folder} -> {switch_name}")

    color = 'tab:red'
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel('Consumption (Wh)', color=color)
    ax1.plot(consumption_x, consumption_y, color=color)
    ax1.tick_params(axis='y', labelcolor=color)
    if min_value is not None and max_value is not None:
        ax1.set_ylim(min_value*0.95, max_value*1.05)

    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis

    color = 'tab:blue'
    ax2.set_ylabel('Usage (%)', color=color)  # we already handled the x-label with ax1
    ax2.plot(cpu_x, cpu_y, color=color)
    ax2.tick_params(axis='y', labelcolor=color)
    ax2.set_ylim(-0.01, 1.01)

    fig.tight_layout()  # otherwise the right y-label is slightly clipped
    plt.show()

# Check if a filename is provided as a command-line argument
if len(sys.argv) != 2 and len(sys.argv) != 3:
    print("Usage: python stats_to_graph.py <trace_folder> [<output_file>]")
    sys.exit(1)

# Get the filename from the command-line argument
trace_folder = sys.argv[1]
min_max_data = None
all_switchs = ['Unknown: to know available switchs, run the script with the output file']

if len(sys.argv) == 3:
    output_file = sys.argv[2]
    min_max_data = parse_file(output_file)
    all_switchs = list(min_max_data.keys())

switch_name = input(f"Available switchs names: {all_switchs} \nInsert the switch name: ")

plot_graph(trace_folder, min_max_data, switch_name)


