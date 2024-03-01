#!/bin/bash

topos=("jump1" "jump2" "jump3" "jump4" "jump5")

for topo in "${topos[@]}"
do
    make run CONTROLLER=ns3::OspfController TOPO=increase_flow_step_$topo OUTPUTS=$topo > outputs/output_$topo.txt &
done
