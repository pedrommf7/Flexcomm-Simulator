#!/bin/bash

# topos=("jump1" "jump2" "jump3" "jump4" "jump5")

# for topo in "${topos[@]}"
# do
#     make run CONTROLLER=ns3::OspfController TOPO=increase_flow_step_$topo OUTPUTS=$topo > outputs/output_$topo.txt &
# done


#link_capacitys=("100Mb" "240Mb" "1000Mb" "1Gb" "2,4Gb" "10Gb")
#link_capacitys=("100Mb" "240Mb" "1000Mb")
link_capacitys=("1Gb" "2,4Gb" "10Gb")
#link_capacitys=("11Gb" "15Gb" "20Gb")

for link_capacity in "${link_capacitys[@]}"
do
    make run CONTROLLER=ns3::OspfController TOPO=jump5_$link_capacity > outputs/output_$link_capacity.txt &
    echo "Running $link_capacity topology..."
    sleep 1
done

