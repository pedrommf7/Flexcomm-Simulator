#!/usr/bin/env python3

import sys
import re
import matplotlib.pyplot as plt
import pandas as pd

from common import colors, markers


# link-stats-logger.cc
# void
# LinkStatsLogger::ComputeStatsLog (Time interval, Time stop, ChannelContainer c, std::string path)
# *stream << "Time;LinkName;LinkUsage;Free;Source;Destiny\n";

# link-stats.cc (no .h includes: #include "ns3/node.h" #include "ns3/channel-container.h")
# void
# LinkStats::LogStatsInternal (Ptr<OutputStreamWrapper> streamWrapper)
# {
#   std::string linkName = Names::FindName (m_channel);

#   uint32_t channel_id = m_channel->GetId ();
#   Ptr<Node> src, dst;
#   ChannelContainer c = ChannelContainer::GetSwitch2Switch ();
#   for (ChannelContainer::Iterator i = c.Begin (); i != c.End (); ++i)
#     {
#       if ((*i)->GetId () == channel_id)
#         {
#           src = (*i)->GetDevice (0)->GetNode ();
#           dst = (*i)->GetDevice (1)->GetNode ();
#         }
#     }

#   std::ostream *stream = streamWrapper->GetStream ();
#   double usage = m_channel->GetChannelUsage ();
#   if(usage>=1)
#     usage = 1; 
#   *stream << std::fixed << Simulator::Now ().GetSeconds () << ";" << linkName << ";"
#           << m_channel->GetChannelUsage ()
#           << ";" << m_channel->GetDataRate().GetBitRate() * (1-usage)
#           << ";" << Names::FindName (src) << ";" << Names::FindName (dst)
#           << "\n";
# }


# Function to parse the file and extract MinMax data
def find_reference_bw(file_path):

    with open(file_path, 'r') as file:
        for line in file:
            referenceBandwidth_match = re.match(r"Reference Bandwidth Value: (\d+)", line)

            if referenceBandwidth_match:
                return int(referenceBandwidth_match.groups()[0])

    return -1

def collect_summed_consumption_data(file_path_consumption):
    consumption_data = pd.read_csv(file_path_consumption, delimiter=';')

    #sum all the consumption of the switches with the same time
    data = consumption_data.groupby('Time')['Consumption'].sum().reset_index()
    
    return data

def collect_linkCosts_data(file_path_consumption, file_path_links):
    consumption_data = pd.read_csv(file_path_consumption, delimiter=';')

    links_data = pd.read_csv(file_path_links, delimiter=';')

    costsDict1, costsDict2, costsDict3, costsDict4, costsDict5 = {}, {}, {}, {}, {}
    for row in links_data.iterrows():
        time = row[1]['Time']
        if time not in costsDict1.keys():
            costsDict1[time] = 0
            costsDict2[time] = 0
            costsDict3[time] = 0
            costsDict4[time] = 0
            costsDict5[time] = 0
        dst = row[1]['Destiny']

        consoB = consumption_data[(consumption_data['Time'] == time) & (consumption_data['NodeName'] == dst)]['Consumption'].values[0]
        freeBW = row[1]['Free']
        dataRate = row[1]['DataRate']
        usage = row[1]['LinkUsage']
        if(freeBW <= 0):
            freeBW = 1

        modelAt0 = 5
        modelAt1 = 120

        costsDict1[time] += (dataRate/freeBW) * consoB
        costsDict2[time] += usage * consoB
        costsDict3[time] += usage * (consoB/modelAt0)
        costsDict4[time] += usage * ((2*consoB)/modelAt0)
        costsDict5[time] += usage * (consoB/modelAt1)

    
    costs1 = list(costsDict1.values())
    costs1 = [round(elem, 3) for elem in costs1]

    costs2 = list(costsDict2.values())
    costs2 = [round(elem, 3) for elem in costs2]

    costs3 = list(costsDict3.values())
    costs3 = [round(elem, 3) for elem in costs3]

    costs4 = list(costsDict4.values())
    costs4 = [round(elem, 3) for elem in costs4]

    costs5 = list(costsDict5.values())
    costs5 = [round(elem, 3) for elem in costs5]
    
    return costs1, costs2, costs3, costs4, costs5

# Function to plot the data
def plot_graph(title, label_y, points_x, points_y, names, n, justLastValues):
    print("Title: \n\t", title)
    # print("label_y: \n\t", label_y)

    for i in range(len(points_x)):
        print(f"points_x[{i}]:", points_x[i])
        for j in range(len(points_y)):
            print(f"\tpoints_y[{j}][{i}]:", points_y[j][i])
    print("\n")



    # print("points_x: \n\t", points_x)
    # print("points_y: \n\t", points_y)
    print("names: \n\t", names)

    plt.figure(title)
    plt.title(title)
    plt.ylabel(label_y)
    plt.xlabel("Time (s)")

    color_index = 0
    # print(f'\t\t---->   Scenario:{title}:')
    for i in range(len(names)):
        
        col = colors[color_index % len(colors)]
        mrk = markers[color_index % len(markers)]
        # print(f'---->   cost nr{i+1}: {(points_y[i])}')
        
        if(n>0):
            if justLastValues:
                p_x = points_x[-n:]
                p_y = points_y[i][-n:]
            else:
                p_x = points_x[:-n]
                p_y = points_y[i][:-n]
        else:
            p_x = points_x
            p_y = points_y[i]


        plt.plot(p_x, p_y, label=names[i], color=col, marker=mrk)
        max_y = max(p_y)
        plt.axhline(y=max_y, color=col, linestyle='--', label=f'Max: {max_y:.2f}')

        color_index += 1

    plt.legend()

def plot_graphs(points_x, sum_conso, sum_costs1, sum_costs2, sum_costs3, sum_costs4, sum_costs5, names):
    # number of last points to separate 
    n = 0

    # for i in range(len(sum_conso)):
    #     conso = sum_conso[i]
    #     costs = sum_costs[i]
    #     name = names[i]
    #     if(n>0):
    #         plot_graph(f"{names[i]} Soma dos consumos do caminho (sem ultimos {n} valores)", 'Consumption of the Path', points_x, [conso], [name], n, False)
    #         # plot_graph(f"{names[i]} Soma dos consumos do caminho (só ultimos {n} valores)", 'Consumption of the Path', points_x, [conso], [name], n, True)
    #         plot_graph(f"{names[i]} Soma dos custos do caminho (sem ultimos {n} valores)", 'Custom cost formula of the Path', points_x, [costs], [name], n, False)
    #         # plot_graph(f"{names[i]} Soma dos custos do caminho (só ultimos {n} valores)", 'Custom cost formula of the Path', points_x, [costs], [name], n, True)
    #     else:
    #         plot_graph(f"{names[i]} Soma dos consumos do caminho", 'Comsumption of the Path', points_x, [conso], [name], n, False)
    #         plot_graph(f"{names[i]} Soma dos custos do caminho", 'Custom cost formula of the Path', points_x, [costs], [name], n, False)

    if(n>0):
        plot_graph(f"Soma dos consumos do caminho (sem ultimos {n} valores)", 'Consumption of the Path', points_x, sum_conso, names, n, False)
        # plot_graph(f"TODOS - Soma dos consumos do caminho (só ultimos {n} valores)", 'Consumption of the Path', points_x, sum_conso, names, n, True)
        plot_graph(f"Soma dos custos 1 do caminho (sem ultimos {n} valores)", 'Custom cost formula of the Path', points_x, sum_costs1, names, n, False)
        plot_graph(f"Soma dos custos 2 do caminho (sem ultimos {n} valores)", 'Custom cost formula of the Path', points_x, sum_costs2, names, n, False)
        # plot_graph(f"TODOS - Soma dos custos do caminho (só ultimos {n} valores)", 'Custom cost formula of the Path', points_x, sum_costs, names, n, True)
    else:
        # plot_graph("TODOS - Soma dos consumos do caminho", 'Comsumption of the Path', points_x, sum_conso, names, n, False)
        # plot_graph("TODOS - Soma dos custos 1 do caminho", 'Custom cost formula of the Path', points_x, sum_costs1, names, n, False)
        plot_graph("TODOS - Soma dos custos 2 do caminho", 'Custom cost formula of the Path', points_x, sum_costs2, names, n, False)
        plot_graph("TODOS - Soma dos custos 3 do caminho", 'Custom cost formula of the Path', points_x, sum_costs3, names, n, False)
        plot_graph("TODOS - Soma dos custos 4 do caminho", 'Custom cost formula of the Path', points_x, sum_costs4, names, n, False)
        plot_graph("TODOS - Soma dos custos 5 do caminho", 'Custom cost formula of the Path', points_x, sum_costs5, names, n, False)


    plt.show()

def collect_data(traceFolders):
    sum_costs1, sum_costs2, sum_costs3, sum_costs4, sum_costs5, sum_conso, names = [], [], [], [], [], [], []

    for i in range(0, len(traceFolders), 1):
        c1 = collect_summed_consumption_data(f'{traceFolders[i]}/ecofen-trace.csv')

        # reference_bw = find_reference_bw(traceFolders[i+1])
        c2, c3, c4, c5, c6 = collect_linkCosts_data(f'{traceFolders[i]}/ecofen-trace.csv', 
                                    f'{traceFolders[i]}/link-stats.csv')
        
        name = traceFolders[i].split('/')[-1]
        
        sum_conso.append(c1)
        sum_costs1.append(c2)
        sum_costs2.append(c3)
        sum_costs3.append(c4)
        sum_costs4.append(c5)
        sum_costs5.append(c6)

        names.append(name)

    points_x = sum_conso[0]['Time'].tolist()

    for i in range(len(sum_conso)):
        sum_conso[i] = sum_conso[i]['Consumption'].tolist()

    plot_graphs(points_x, sum_conso, sum_costs1, sum_costs2, sum_costs3, sum_costs4, sum_costs5, names)


if (len(sys.argv) < 2):
    if(sys.argv[0].startswith("./")):
        print("Usage:", sys.argv[0], "trace_folder1 trace_folder2 ... trace_folderN")
    else:
        print("Usage: python", sys.argv[0], "trace_folder1 trace_folder2 ... trace_folderN")
    sys.exit(1)

traceFolders = sys.argv[1:]

collect_data(traceFolders)


