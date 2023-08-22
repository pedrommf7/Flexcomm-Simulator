#!/bin/bash

echo 'Insert the topology folder name:'
read topo

echo '
------------> [COMMAND] python ./utils/topo-vis.py --topology='$topo'
'
./utils/topo-vis.py --topology=$topo --showHosts &

echo '
------------> [COMMAND] python ./utils/flowmo-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/flowmo-parser.py outputs/$topo/flow-monitor.xml

echo '

------------> [COMMAND] python ./utils/ecofen-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/ecofen-parser.py outputs/$topo/ecofen-trace

echo '

------------> [COMMAND] python ./utils/switch-stats-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/switch-stats-parser.py outputs/$topo/switch-stats

echo '

------------> [COMMAND] python ./utils/link-stats-parser.py outputs/'$topo'/flow-monitor.xml
'
./utils/link-stats-parser.py outputs/$topo/link-stats