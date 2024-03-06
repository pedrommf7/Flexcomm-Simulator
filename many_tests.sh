#!/bin/bash

# topos=("jump1" "jump2" "jump3" "jump4" "jump5")

# for topo in "${topos[@]}"
# do
#     make run CONTROLLER=ns3::OspfController TOPO=increase_flow_step_$topo OUTPUTS=$topo > outputs/output_$topo.txt &
# done


link_capacitys=("1Gb" "2,4Gb" "10Gb") # 1Gb, 2,4Gb, 10Gb

for link_capacity in "${link_capacitys[@]}"
do
    make run CONTROLLER=ns3::OspfController TOPO=jump5_$link_capacity OUTPUTS=$link_capacity > outputs/output_jump5_$link_capacity.txt &
done

