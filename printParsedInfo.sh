#!/bin/bash

echo 'Insert the topology folder name:'
read topo

echo '
------------> [COMMAND] ./utils/topo-vis.py --topology='$topo' --showHosts
'
./utils/topo-vis.py --topology=$topo --showHosts &

echo '
------------> [COMMAND] ./utils/flowmo-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/flowmo-parser.py outputs/$topo/flow-monitor.xml

echo '

------------> [COMMAND] ./utils/ecofen-parser.py outputs/'$topo'/ecofen-trace.csv --plot
'
./utils/ecofen-parser.py outputs/$topo/ecofen-trace.csv --plot &

echo '

------------> [COMMAND] ././utils/switch-stats-parser.py outputs/'$topo'/switch-stats.csv --plot
'
./utils/switch-stats-parser.py outputs/$topo/switch-stats.csv --plot &

echo '

------------> [COMMAND] ././utils/link-stats-parser.py outputs/'$topo'/link-stats.csv --plot
'
./utils/link-stats-parser.py outputs/$topo/link-stats.csv --plot &
