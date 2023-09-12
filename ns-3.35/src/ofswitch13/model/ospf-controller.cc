/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * The GPLv2 License (GPLv2)
 *
 * Copyright (c) 2023 Pedro M. Ferreira
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
 * Author: Pedro M. Ferreira <pedro.m.marques@inesctec.pt>
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

NS_LOG_COMPONENT_DEFINE ("MyController");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (MyController);

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
MyController::FindReferenceBandwidth(){
  base_graph = Topology::GetGraph ();

  bool findMax = !referenceBandwidthValue; 
  
  std::list<Edge> edgesToRemove;

  boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
  for (boost::tie (edgeIt, edgeEnd) = boost::edges (base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);

      if (n1->IsSwitch () && n2->IsSwitch ())
        {
          Ptr<Channel> chnl = Topology::GetChannel (n1, n2);
          uint64_t bitRate = chnl->GetDataRate(). GetBitRate();
          boost::put(edge_weight_t (), base_graph, ed, bitRate);
              
          if(findMax)
            {
              referenceBandwidthValue = bitRate>referenceBandwidthValue ? bitRate : referenceBandwidthValue;
            }
        }
      else
        {
          edgesToRemove.push_back(ed);
          Topology::UpdateEdgeWeight(n1, n2, 0); // rever!!!!
        }
    }
  for (auto ed : edgesToRemove)
    {
      boost::remove_edge(ed, base_graph);
    }
}

void MyController::SetWeightsBandwidthBased(){
  cout << "Reference Bandwidth Value: " << referenceBandwidthValue << endl;
  if (referenceBandwidthValue)//preventing division by 0, if there is no switchs
    {
      boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
      for (boost::tie (edgeIt, edgeEnd) = boost::edges (base_graph); edgeIt != edgeEnd; ++edgeIt)
        {
          Edge ed = *edgeIt;
          uint64_t bitRate = boost::get (edge_weight_t (), base_graph, ed); // ver se preciso de fazer como est ano topology a separar pelos Nodes

          uint64_t weight = bitRate>=referenceBandwidthValue ? 1 : referenceBandwidthValue / bitRate;

          boost::put (edge_weight_t (), base_graph, ed, weight);
          cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << weight << endl;

          Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
          Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);
          Topology::UpdateEdgeWeight(n1, n2, weight);
        }
      cout << "-----------------------------" << endl;
    } 
}

bool
MyController::UpdateWeights ()
{
  bool updated = false;
  boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
  for (boost::tie (edgeIt, edgeEnd) = boost::edges (base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);
      uint64_t weight = boost::get (edge_weight_t (), base_graph, ed); // ver se preciso de fazer como est ano topology a separar pelos Nodes

      int index = int (Simulator::Now ().GetMinutes ()) % 60;
      float flex1 = EnergyAPI::GetFlexArray (Names::FindName (n1)).at (index);
      float flex2 = EnergyAPI::GetFlexArray (Names::FindName (n2)).at (index);

      // tambem podemos usar a taxa de utilizacao da do link para a flexibilidade impactar + ou - no peso
      // Ptr<Channel> chnl = Topology::GetChannel(n1, n2);
      // double usagePercentage =  chnl->GetChannelUsage(); //valor em percentagem !!!
      // int new_weight = int (weight - (flex1 + flex2)/2 * (1-usagePercentage)); // quando maior for a utilizacao menor é o impacto da flexibilidade

      // ponderar se vemos a flexibildade como um valor unico e nao é preciso guardar a informacao inicial
      // pode ser bom caso os switches sejam iguais e tenham o mesmo consumo, a formula de cima seria boa???? 
      // mas podia escaxar porque calcula o min e nao o max, o djikstra so lida com pesos positivos, volta à situação??
      // 
      
      // alterar aqui a expressão que se pretende usar para calcular o impacto da flexibilidade
      int new_weight = int (weight - (flex1 + flex2)/2);
      new_weight = new_weight<0 ? 0 : new_weight;
      //expressao produz nrs negativos, mas depois insere 0 no maximo

      if (Topology::GetEdgeWeight(n1, n2) != new_weight)
        {
          Topology::UpdateEdgeWeight (n1, n2, new_weight);
          updated = true;
          // mudar isto para ajudar o controlador a saber que aresta mudou e não ter de computar os caminhos todos de novo !!!
        }
    }
  return updated;
}

//copiada do simple-controller.cc porque lá está protegida, como fazer??
void
MyController::ApplyRouting (uint64_t swDpId)
{
  uint32_t swId = DpId2Id (swDpId);
  Ptr<Node> sw = NodeContainer::GetGlobal ().Get (swId);

  // assim impede que switchs tenham camminhos de "reserva", 
  // mas diminui o número de vezes que o algoritmo de descoberta de melhor caminho é chamado
  if (sw->IsSwitch())
    {
      Ptr<OFSwitch13Device> ofDevice = sw->GetObject<OFSwitch13Device> ();
      NodeContainer hosts = NodeContainer::GetGlobalHosts ();

      for (NodeContainer::Iterator i = hosts.Begin (); i != hosts.End (); i++)
        {
          Ipv4Address remoteAddr = (*i)->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
          // ideia rui usar o djikstra apenas no inicio e depois usar outro algoritmo para ir computando os melhores caminhos
          std::vector<Ptr<Node>> path = Topology::DijkstraShortestPath (sw, *i);
          
          uint32_t port = ofDevice->GetPortNoConnectedTo (path.at (1));

          std::ostringstream cmd;
          cmd << "flow-mod cmd=add,table=0 eth_type=0x800,ip_dst=" << remoteAddr
              << " apply:output=" << port;

          NS_LOG_DEBUG ("[" << swDpId << "]: " << cmd.str ());

          DpctlExecute (swDpId, cmd.str ());
        }

    }


}

void
MyController::UpdateRouting ()
{
  if (UpdateWeights ())
  {
    NodeContainer switches = NodeContainer::GetGlobalSwitches ();
    for (auto sw = switches.Begin (); sw != switches.End (); sw++)
      ApplyRouting (Id2DpId ((*sw)->GetId ()));
  }

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

  // restringir logo aqui os switchs para nao fazerem schedulings de nada ??????? REVER

  if (m_isFirstUpdate)
    {
      FindReferenceBandwidth();
      SetWeightsBandwidthBased();
      UpdateWeights ();
      m_isFirstUpdate = false;
      Simulator::Schedule (Minutes (1), &MyController::UpdateRouting, this);
    }

  ApplyRouting (swDpId);
}

} // namespace ns3

