#!/usr/bin/env python3

import matplotlib.pylab as plt
import regex as re
import argparse
from tabulate import tabulate
from sortedcontainers import SortedSet

colors = [
    "gray",
    "red",
    "sienna",
    "sandybrown",
    "y",
    "g",
    "c",
    "blue",
    "purple",
    "violet",
    "pink",
]
markers = ["o", "v", "^", "<", ">", "s", "D", "h", "p", "X", "*"]

# Define arguments
parser = argparse.ArgumentParser()
parser.add_argument(
    "file", metavar="PATH/TO/FILE", type=argparse.FileType("r")
)
parser.add_argument("--plot", "-p", dest="drawPlot", action="store_true")
parser.set_defaults(drawPlot=False)
args = parser.parse_args()

# Read file
# FIXME: This is not ideal
lines = args.file.readlines()

x = SortedSet()
cpuUsages = {}
nPackets = {}
nDroppedPackets = {}
nBytes = {}
for line in lines:
    # time nodeName cpuUsage nPackets nDropped nBytes
    match = re.match(
        r"(\d+(.\d+)?) (\w+) (\d+(.\d+)?) (\d+) (\d+) (\d+)", line
    )
    if match is not None:
        node = match.group(3)
        time = float(match.group(1))
        cpu = float(match.group(4))
        packets = int(match.group(6))
        droppedPackets = int(match.group(7))
        bytes = int(match.group(8))
        x.add(time)

        if node not in cpuUsages:
            cpuUsages[node] = [cpu]
            nPackets[node] = [packets]
            nDroppedPackets[node] = [droppedPackets]
            nBytes[node] = [bytes]
        else:
            cpuUsages[node].append(cpu)
            nPackets[node].append(packets)
            nDroppedPackets[node].append(droppedPackets)
            nBytes[node].append(bytes)

table = []
for switch in cpuUsages:
    usages = cpuUsages[switch]
    table.append([switch, max(usages), sum(usages) / len(usages)])
print("CPU Usage:")
print(
    tabulate(
        table,
        headers=["Switch", "Max (%)", "Average (%)"],
        tablefmt="pretty",
    )
)
print("")

table = []
for switch in nPackets:
    packets = nPackets[switch]
    table.append(
        [
            switch,
            max(packets),
            sum(packets) / len(packets),
            sum(packets),
        ]
    )
print("Packets processed:")
print(
    tabulate(
        table,
        headers=["Switch", "Max", "Average", "Total"],
        tablefmt="pretty",
    )
)
print("")

table = []
for switch in nDroppedPackets:
    packets = nDroppedPackets[switch]
    table.append(
        [
            switch,
            max(packets),
            sum(packets) / len(packets),
            sum(packets),
        ]
    )
print("Dropped Packets:")
print(
    tabulate(
        table,
        headers=["Switch", "Max", "Average", "Total"],
        tablefmt="pretty",
    )
)
print("")

table = []
for switch in nBytes:
    bytes = nBytes[switch]
    table.append(
        [
            switch,
            max(bytes),
            sum(bytes) / len(bytes),
            sum(bytes),
        ]
    )
print("Bytes processed:")
print(
    tabulate(
        table,
        headers=["Switch", "Max", "Average", "Total"],
        tablefmt="pretty",
    )
)
print("")

if args.drawPlot:
    # CpuUsage graph
    color_index = 0
    for node in cpuUsages:
        plt.plot(
            x,
            cpuUsages[node],
            label=node,
            color=colors[color_index % len(colors)],
            marker=markers[color_index % len(markers)],
            linestyle="none",
        )
        color_index += 1

    # Show plot
    plt.legend()
    plt.xlabel("Time Interval (s)")
    plt.ylabel("Cpu Usage (%)")
    plt.show()

    ################################################################

    # nPackets graph
    color_index = 0
    for node in nPackets:
        plt.plot(
            x,
            nPackets[node],
            label=node,
            color=colors[color_index % len(colors)],
            marker=markers[color_index % len(markers)],
            linestyle="none",
        )
        color_index += 1

    # Show plot
    plt.legend()
    plt.xlabel("Time Interval (s)")
    plt.ylabel("Nº Packets")
    plt.show()

    ################################################################

    # nDroppedPackets graph
    color_index = 0
    for node in nDroppedPackets:
        plt.plot(
            x,
            nDroppedPackets[node],
            label=node,
            color=colors[color_index % len(colors)],
            marker=markers[color_index % len(markers)],
            linestyle="none",
        )
        color_index += 1

    # Show plot
    plt.legend()
    plt.xlabel("Time Interval (s)")
    plt.ylabel("Nº Packets")
    plt.show()

    ################################################################

    # nBytes graph
    color_index = 0
    for node in nBytes:
        plt.plot(
            x,
            nBytes[node],
            color=colors[color_index % len(colors)],
            marker=markers[color_index % len(markers)],
            linestyle="none",
            label=node,
        )
        color_index += 1

    # Show plot
    plt.legend()
    plt.xlabel("Time Interval (s)")
    plt.ylabel("Nº Bytes")
    plt.show()
