#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt
import pandas as pd

from common import colors, markers

def parse_weights_file(filename):
    data = {}
    with open(filename, 'r') as file:
        current_time = None
        current_edges = {}
        for line in file:
            if line.startswith("Time(seconds):"):
                if current_time is not None:
                    data[current_time] = current_edges
                current_time = int(line.split(":")[1].strip())
                current_edges = {}
            elif line.startswith("Edge:"):
                edge_id = line.split(" | ")[0].split("Edge: ")[1].strip()
                weight = int(line.split("|")[1].split("Weight: ")[1].strip())
                current_edges[edge_id] = weight

        # Adding the last entry
        if current_time is not None:
            data[current_time] = current_edges
    return data

def parse_links_file(filename):
    data = {}
    with open(filename, 'r') as file:
        # Time;LinkName;LinkUsage
        for line in file:
            if line.startswith("Time;LinkName;LinkUsage"):
                continue
            aux = line.split(";")
            if (len(aux) != 3):
                continue

            time = int(float(aux[0]))
            link = aux[1]
            usage = float(aux[2])
            if time not in data:
                data[time] = {}
            data[time][link] = usage
    
    return data

def createPoints(infoWeights, infoLinks, pathsToTrack, links):
    # print("infoWeights:", infoWeights)
    # print()
    # print()
    # print("infoLinks:", infoLinks)

    time = list(infoWeights.keys())
    points_x = {}
    points_y = {}
    for t in time:
        info1 = []
        for path in pathsToTrack:
            sumWeight = 0
            for jump in path:
                sumWeight += infoWeights[t][jump]
            info1.append(sumWeight)
        points_y[t] = info1

        info2 = []
        for path in links:
            sumTraffic = 0
            for link in path:
                try:
                    sumTraffic += infoLinks[t][link] * 1000000000 # alterar aqui para o datarate do link
                except:
                    print(f"Link {link} not found in time {t}s")
            info2.append(sumTraffic)
        points_x[t] = info2

    points_x = points_x.values()
    points_y = points_y.values()
    
    separated_x = []
    separated_y = []
    for i in range(len(pathsToTrack)):
        info_x = []
        for px in points_x:
            info_x.append(px[i])
        separated_x.append(info_x)

        info_y = []
        for py in points_y:
            info_y.append(py[i])
        separated_y.append(info_y)
    
    return separated_x, separated_y

def plot_graph(title, filename, points_x, points_y, names):
    # print("Title: \n\t", title)
    # print("label_y: \n\t", label_y)

    # for i in range(len(points_x)):
    #     print(f"points_x[{i}]:", points_x[i])
    #     for j in range(len(points_y)):
    #         print(f"\tpoints_y[{j}][{i}]:", points_y[j][i])
    # print("\n")



    print("points_x: \n\t", points_x)
    print("points_y: \n\t", points_y)
    print("names: \n\t", names)

    plt.figure(filename)
    plt.title(title)
    plt.ylabel("Cost of the path")
    plt.xlabel("Amount of traffic of the path (bps)")

    color_index = 0
    # print(f'\t\t---->   Scenario:{title}:')
    for i in range(len(names)):
        
        col = colors[color_index % len(colors)]
        mrk = markers[color_index % len(markers)]
        # print(f'---->   cost nr{i+1}: {(points_y[i])}')
        
        p_x = points_x[i]
        # print("points_x:", p_x)
        # print()
        p_y = points_y[i]
        # print("points_y:", p_y)
        # print()

        plt.plot(p_x, p_y, label=names[i], color=col, marker=mrk)
        max_y = max(p_y)
        plt.axhline(y=max_y, color=col, linestyle='--', label=f'Max: {max_y:.2f}')

        color_index += 1

    plt.legend()
    plt.show()


if (len(sys.argv) != 3):
    if(sys.argv[0].startswith("./")):
        print("Usage:", sys.argv[0], "weights_file link_stats_file")
    else:
        print("Usage: python", sys.argv[0], "weights_file link_stats_file")
    sys.exit(1)

weights_file = sys.argv[1]
link_stats_file = sys.argv[2]

info_weight = parse_weights_file(weights_file)
info_links = parse_links_file(link_stats_file)

names = [
    "H1 -> S1 -> S3 -> S6 -> S9 -> H5",
    "H12 -> S5 -> S2 -> S1 -> H3",
    "H14 -> S7 -> S4 -> S1 -> H4",
    "H9 -> S5 -> S9 -> S8 -> S7 -> H14",
    "H14 -> S7 -> S8 -> S9 -> H8",
    "H13 -> S7 -> S4 -> S1 -> S2 -> S5 -> H11",
    "H1 -> S1 -> S4 -> S7 -> S8 -> S9 -> S5 -> H12"
]

pathsToTrack = [
    ["16 -> 18", "18 -> 21", "21 -> 24"],
    ["20 -> 17", "17 -> 16"],
    ["22 -> 19", "19 -> 16"],
    ["20 -> 24", "24 -> 23", "23 -> 22"],
    ["22 -> 23", "23 -> 24"],
    ["22 -> 19", "19 -> 16", "16 -> 17", "17 -> 20"],
    ["16 -> 19", "19 -> 22", "22 -> 23", "23 -> 24", "24 -> 20"]
]

links = [
    ["link18", "link23", "link27"],
    ["link20", "link17"],
    ["link25", "link19"],
    ["link26", "link29", "link28"],
    ["link28", "link29"],
    ["link25", "link19", "link17", "link20"],
    ["link19", "link25", "link28", "link29", "link26"]
]


points_x, points_y = createPoints(info_weight, info_links, pathsToTrack, links)

plot_graph("Cost of the path vs Amount of traffic of the path", weights_file, points_x, points_y, names)