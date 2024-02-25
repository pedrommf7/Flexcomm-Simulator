#!/bin/bash

echo 'Insert the topology folder name:'
read topo

echo '
------------> [COMMAND] ./utils/topo-vis.py --topology='$topo'
'
./utils/topo-vis.py --topology=$topo --showHosts &

echo '
------------> [COMMAND] ./utils/flowmo-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/flowmo-parser.py outputs/$topo/flow-monitor.xml

echo '

------------> [COMMAND] ./utils/ecofen-parser.py outputs/'$topo'/ecofen-trace.csv
'
./utils/ecofen-parser.py outputs/$topo/ecofen-trace.csv

echo '

------------> [COMMAND] ././utils/switch-stats-parser.py outputs/'$topo'/switch-stats.csv
'
./utils/switch-stats-parser.py outputs/$topo/switch-stats.csv

echo '

------------> [COMMAND] ././utils/link-stats-parser.py outputs/'$topo'/link-stats.csv
'
./utils/link-stats-parser.py outputs/$topo/link-stats.csv