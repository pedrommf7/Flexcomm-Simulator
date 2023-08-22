/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * The GPLv2 License (GPLv2)
 *
 * Copyright (c) 2023 Pedro Miguel Marques Ferreira
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http: //www.gnu.org/licenses/>.
 *
 * Author: Pedro Miguel Marques Ferreira <pedro.m.marques@inesctec.pt>
 */

#include <cstdint>
#include <cstdlib>

#include "my-controller.h"

#include "ns3/log.h"
#include "ns3/parser.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/ofswitch13-module.h"
#include "ns3/topology-module.h"
#include "ns3/energy-api-module.h"
#include "ns3/point-to-point-ethernet-net-device.h"

NS_LOG_COMPONENT_DEFINE ("MyController");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MyController);

std::map<Edge, uint64_t> edg_to_weight = std::map<Edge, uint64_t> ();
// if not initializade at 0 it will be the bandwidth reference value used to calculate the weights
uint64_t MyController::referenceBandwidthValue = 0;

MyController::MyController ()
{
  NS_LOG_FUNCTION (this); 
  m_isFirstUpdate = true;
}

MyController::~MyController ()
{
  NS_LOG_FUNCTION (this);
}

TypeId MyController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::MyController")
                          .SetParent<OFSwitch13Controller> ()
                          .SetGroupName ("OFSwitch13")
                          .AddConstructor<MyController> ();
  return tid;
}

void MyController::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  OFSwitch13Controller::DoDispose ();
}

void
MyController::SaveInfo(Edge ed, uint64_t weight){
  edg_to_weight[ed] = weight;
}

void 
MyController::SaveDataRateInfos(){
  Graph topo = Topology::GetGraph ();
  
  if(!referenceBandwidthValue){
    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    for (boost::tie (edgeIt, edgeEnd) = boost::edges (topo); edgeIt != edgeEnd; ++edgeIt)
      {
        Edge ed = *edgeIt;
        Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
        Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);

        if (n1->IsSwitch () && n2->IsSwitch ())
          {
            Ptr<Channel> chnl = Topology::GetChannel(n1, n2);

            uint64_t bitRate = DynamicCast<PointToPointEthernetNetDevice> (chnl->GetDevice (0))->GetDataRate(). GetBitRate();
            if (bitRate > referenceBandwidthValue)
              referenceBandwidthValue = bitRate;

            // ver se guardar só os que sao switch ou todos !!!
            SaveInfo(ed, bitRate);
          }
      }
  }
}

void MyController::SetWeightsBandwidthBased(){
  cout << "Reference Bandwidth Value: " << referenceBandwidthValue << endl;
  if (referenceBandwidthValue)//preventing division by 0
    {
      for(std::map<Edge,uint64_t>::iterator iter = edg_to_weight.begin(); iter != edg_to_weight.end(); ++iter)
        {
          Edge ed =  iter->first;
          cout << "BitRate Base: " << iter->second << endl;
          uint64_t bitRate = referenceBandwidthValue / iter->second; // ver melhor isto !!
          cout << "BitRate: " << bitRate << endl;

          SaveInfo(ed, bitRate);
          cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << bitRate << endl;

          Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
          Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);

          Topology::UpdateEdgeWeight(n1, n2, int (bitRate));
        }
      cout << "-----------------------------" << endl;
    } 
}

void
MyController::UpdateWeights ()
{
  for(std::map<Edge,uint64_t>::iterator iter = edg_to_weight.begin(); iter != edg_to_weight.end(); ++iter)
    {
      Edge ed =  iter->first;
      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);
      uint64_t weight = iter->second;

      int index = int (Simulator::Now ().GetMinutes ()) % 60;
      float flex1 = EnergyAPI::GetFlexArray (Names::FindName (n1)).at (index);
      float flex2 = EnergyAPI::GetFlexArray (Names::FindName (n2)).at (index);

      // alterar aqui a expressão que se pretende usar para calcular o impacto da flexibilidade
      int new_weight = int (weight - (flex1 + flex2)/2);

      Topology::UpdateEdgeWeight (n1, n2, new_weight);
      cout << "Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << new_weight << endl;
    }
  cout << "-----------------------------" << endl;
}

//copiada do simple-controller.cc porue lá está protegida, como fazer??
void
MyController::ApplyRouting (uint64_t swDpId)
{
  uint32_t swId = DpId2Id (swDpId);
  Ptr<Node> sw = NodeContainer::GetGlobal ().Get (swId);
  Ptr<OFSwitch13Device> ofDevice = sw->GetObject<OFSwitch13Device> ();
  NodeContainer hosts = NodeContainer::GetGlobalHosts ();

  for (NodeContainer::Iterator i = hosts.Begin (); i != hosts.End (); i++)
    {
      Ipv4Address remoteAddr = (*i)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
      std::vector<Ptr<Node>> path = Topology::DijkstraShortestPath (sw, *i);

      uint32_t port = ofDevice->GetPortNoConnectedTo (path.at (1));

      std::ostringstream cmd;
      cmd << "flow-mod cmd=add,table=0 eth_type=0x800,ip_dst=" << remoteAddr
          << " apply:output=" << port;

      NS_LOG_DEBUG ("[" << swDpId << "]: " << cmd.str ());

      DpctlExecute (swDpId, cmd.str ());
    }
}

void
MyController::UpdateRouting ()
{
  UpdateWeights ();
  NodeContainer switches = NodeContainer::GetGlobalSwitches ();
  for (auto sw = switches.Begin (); sw != switches.End (); sw++)
    ApplyRouting (Id2DpId ((*sw)->GetId ()));

  Simulator::Schedule (Minutes (1), &MyController::UpdateRouting, this);
}

void
MyController::HandshakeSuccessful (Ptr<const RemoteSwitch> sw)
{
  NS_LOG_FUNCTION (this << sw);

  uint64_t swDpId = sw->GetDpId ();

  // Default rules
  DpctlExecute (swDpId, "flow-mod cmd=add,table=0,prio=0 "
                        "apply:output=ctrl:128");
  DpctlExecute (swDpId, "set-config miss=128");

  if (m_isFirstUpdate)
    {
      SaveDataRateInfos();
      SetWeightsBandwidthBased();
      UpdateWeights ();
      m_isFirstUpdate = false;
      Simulator::Schedule (Minutes (1), &MyController::UpdateRouting, this);
    }

  ApplyRouting (swDpId);
}

} // namespace ns3

