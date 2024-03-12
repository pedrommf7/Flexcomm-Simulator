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
#   ChannelContainer c = ChannelContainer::GetSwitch2Switch ();
#   Ptr<Node> src, dst;
#   for (ChannelContainer::Iterator i = c.Begin (); i != c.End (); ++i)
#     {
#       if ((*i)->GetId () == channel_id)
#         {
#           src = (*i)->GetDevice (0)->GetNode ();
#           dst = (*i)->GetDevice (1)->GetNode ();
#         }
#     }


#   std::ostream *stream = streamWrapper->GetStream ();
#   *stream << std::fixed << Simulator::Now ().GetSeconds () << ";" << linkName << ";"
#           << m_channel->GetChannelUsage ()
#           << ";" << m_channel->GetDataRate().GetBitRate() * (1-m_channel->GetChannelUsage ())
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

def collect_link_stats_data(file_path_links, file_path_switchs, file_path_consumption, reference_bw):
    links_data = pd.read_csv(file_path_links, delimiter=';')
    switchs_data = pd.read_csv(file_path_switchs, delimiter=';')
    consumption_data = pd.read_csv(file_path_consumption, delimiter=';')

    merged_data = pd.merge(switchs_data, consumption_data, on=['Time', 'NodeName'])
    merged_data = merged_data.drop(columns=['NrProcessedPackets','NrDroppedPackets','ProcessedBytes'])
    # print("Merged Data\n", merged_data)

    # Formula1: (referenceBW/freeBW) (CPU_A+CPU_B)*(Consumption_A+Consumption_B)
    # Formula2: (referenceBW/freeBW) (CPU_A+CPU_B)/2*(Consumption_A+Consumption_B)/2
    # Formula3: (referenceBW/freeBW) (CPU_A*Consumption_A)+(CPU_B*Consumption_B)
    # Formula4: (referenceBW/freeBW) ((CPU_A*Consumption_A)+(CPU_B*Consumption_B))/2

    costs1, costs2, costs3, costs4 = [], [], [], []
    for row in links_data.iterrows():
        #print row Source and Destination
        time = row[1]['Time']
        src = row[1]['Source']
        dst = row[1]['Destiny']
        # cpuA = from merged data, where 'NodeName' == src and 'Time' == time, get 'CPU_Usage'
        cpuA = merged_data[(merged_data['NodeName'] == src) & (merged_data['Time'] == time)]['CPU_Usage'].values[0]
        cpuB = merged_data[(merged_data['NodeName'] == dst) & (merged_data['Time'] == time)]['CPU_Usage'].values[0]
        consoA = merged_data[(merged_data['NodeName'] == src) & (merged_data['Time'] == time)]['Consumption'].values[0]
        consoB = merged_data[(merged_data['NodeName'] == dst) & (merged_data['Time'] == time)]['Consumption'].values[0]
        freeBW = row[1]['Free']
        if(freeBW <= 0):
            freeBW = 1#reference_bw*0.05

        costs1.append( (reference_bw/freeBW) * (consoA+consoB) )
        costs2.append( (reference_bw/freeBW) * ((cpuA+cpuB)/2)*(consoA+consoB) )
        # costs3.append( (reference_bw/freeBW) * ((cpuA*consoA)+(cpuB*consoB)))
        # costs4.append( (reference_bw/freeBW) * ((cpuA*consoA)+(cpuB*consoB))/2)
    
    links_data['Cost1'] = costs1
    links_data['Cost2'] = costs2
    # links_data['Cost3'] = costs3
    # links_data['Cost4'] = costs4
    # print("Links Data\n", links_data)

    c1 = links_data.groupby('Time')['Cost1'].sum().reset_index()
    c2 = links_data.groupby('Time')['Cost2'].sum().reset_index()
    # c3 = links_data.groupby('Time')['Cost3'].sum().reset_index()
    # c4 = links_data.groupby('Time')['Cost4'].sum().reset_index()

    return c1, c2#, c3, c4

# Function to plot the data
def plot_graph(title, names, points_x, costs, n, justLastValues):

    plt.figure(title)
    plt.title(title)
    plt.ylabel('Custom cost formula of the Path')
    plt.xlabel("Time (s)")

    color_index = 0
    print(f'\t\t---->   Scenario:{title}:')
    for i in range(len(costs)):
        
        col = colors[color_index % len(colors)]
        mrk = markers[color_index % len(markers)]
        print(f'---->   cost nr{i+1}: {(costs[i])}')
        
        if(n>0):

            if justLastValues:
                p_x = points_x[-n:]
                p_y = costs[i][-n:]
            else:
                p_x = points_x[:-n]
                p_y = costs[i][:-n]
        else:
            p_x = points_x
            p_y = costs[i]


        plt.plot(p_x, p_y, label=names[i], color=col, marker=mrk)
        max_y = max(p_y)
        plt.axhline(y=max_y, color=col, linestyle='--', label=f'Max: {max_y:.2f}')

        color_index += 1

    plt.legend()

def plot_graphs(points_x, names, costs1, costs2):
    # number of last points to separate 
    n = 6

    if(n>0):
        plot_graph(f"Soma dos consumos entre switchs de cada link (sem ultimos {n} valores)", names, points_x, costs1, n, False)
        plot_graph(f"Soma dos consumos entre switchs de cada link (só ultimos {n} valores)", names, points_x, costs1, n, True)
        plot_graph(f"Soma dos consumos e média de CPU Ratio entre switchs de cada link (sem ultimos {n} valores)", names, points_x, costs2, n, False)
        plot_graph(f"Soma dos consumos e média de CPU Ratio entre switchs de cada link (só ultimos {n} valores)", names, points_x, costs2, n, True)
    else:
        plot_graph("Soma dos consumos entre switchs de cada link ", names, points_x, costs1, n, False)
        plot_graph("Soma dos consumos e média de CPU Ratio entre switchs de cada link", names, points_x, costs2, n, False)

    plt.show()

def collect_data(args, idx):
    formula1, formula2, names = [], [], []
    for i in range(idx, len(args), 2):
        trace_folder = args[i]
        output_file = args[i+1]

        reference_bw = find_reference_bw(output_file)
        c1, c2 = collect_link_stats_data(f'{trace_folder}/link-stats.csv', 
                                         f'{trace_folder}/switch-stats.csv', 
                                         f'{trace_folder}/ecofen-trace.csv', 
                                         reference_bw)
        
        formula1.append(c1)
        formula2.append(c2)
        names.append(trace_folder)

    points_x = formula1[0]['Time'].tolist()
    costs1 = []
    costs2 = []
    for i in range(len(names)):
        collected = formula1[i]['Cost1'].tolist() #[int(elem*100)/100 for elem in 
        costs1.append(collected)
        collected = formula2[i]['Cost2'].tolist() #[int(elem*100)/100 for elem in
        costs2.append(collected)
    
    plot_graphs(points_x, names, costs1, costs2)


if ((len(sys.argv) < 3) or (len(sys.argv) % 2) == 0):
    print("Usage: python customFormula_flow_basis.py trace_folder output_file1 nr_of_scenarios2 trace_folder output_file2 ... trace_folderN output_fileN")
    sys.exit(1)

print("[WARNING] these script only works with topologies with the same link bandwidth across all links, and logInterval equal values across links and ecofen stats")

idx = 1#point to the trace folder

collect_data(sys.argv, idx)


