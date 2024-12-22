#!/bin/bash

#exit if nr of arguments is different from 2
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters"
    echo "Usage: ./printParsedInfo.sh <topology> <topology_output_folder>"
    exit 1
fi

topo=$1
output_folder=$2

echo '
------------> [COMMAND] ./utils/topo-vis.py --topology='$topo' --showHosts
'
./utils/topo-vis.py --topology=$topo --showHosts &

echo '
------------> [COMMAND] ./utils/flowmo-parser.py '$output_folder'/flow-monitor.xml
'
./utils/flowmo-parser.py $output_folder/flow-monitor.xml

echo '

------------> [COMMAND] ./utils/ecofen-parser.py '$output_folder'/ecofen-trace.csv --plot
'
./utils/ecofen-parser.py $output_folder/ecofen-trace.csv --plot &

echo '

------------> [COMMAND] ./utils/switch-stats-parser.py '$output_folder'/switch-stats.csv --plot
'
./utils/switch-stats-parser.py $output_folder/switch-stats.csv --plot &

echo '

------------> [COMMAND] ./utils/link-stats-parser.py '$output_folder'/link-stats.csv --plot
'
./utils/link-stats-parser.py $output_folder/link-stats.csv --plot &
