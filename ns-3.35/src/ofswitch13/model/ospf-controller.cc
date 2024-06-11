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

#include "ospf-controller.h"

#include "ns3/log.h"
#include "ns3/parser.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/ofswitch13-module.h"
#include "ns3/topology-module.h"
#include "ns3/energy-api-module.h"
#include "ns3/cpu-load-based-energy-model.h"

NS_LOG_COMPONENT_DEFINE ("OspfController");

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (OspfController);

std::vector<std::pair<std::pair< // source e destiny
                          Ptr<Node>, Ptr<Node>>,
                      std::vector< // todos caminhos existentes
                          std::pair<std::vector<Ptr<Node>>, // 1 caminho
                                    int // distancia
                                    >>>>
    equalCostPaths;

// if not initializade at 0 it will be the bandwidth reference value used to calculate the weights
uint64_t OspfController::referenceBandwidthValue = 0;
Time lastUpdate;

OspfController::OspfController ()
{
  NS_LOG_FUNCTION (this);
  m_isFirstUpdate = true;
}

OspfController::~OspfController ()
{
  NS_LOG_FUNCTION (this);
}

TypeId
OspfController::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::OspfController")
                          .SetParent<OFSwitch13Controller> ()
                          .SetGroupName ("OFSwitch13")
                          .AddConstructor<OspfController> ();
  return tid;
}

void
OspfController::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  OFSwitch13Controller::DoDispose ();
}

// ---------------------- Stored Paths ----------------------
void
OspfController::SortStoredPathsAscending ()
{
  for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;

      std::sort (paths_.begin (), paths_.end (), [] (const auto &lhs, const auto &rhs) {
        if (lhs.second == rhs.second)
          return lhs.first.size () < rhs.first.size ();
        else
          return lhs.second < rhs.second;
      });

      ecp.second = paths_;
    }
}

std::pair<Ptr<Node>, Ptr<Node>>
OspfController::AddHostsKey (Ptr<Node> source, Ptr<Node> destination)
{
  std::pair<Ptr<Node>, Ptr<Node>> key = std::make_pair (source, destination);

  if (std::find_if (equalCostPaths.begin (), equalCostPaths.end (),
                    [key] (const auto &ecp) { return ecp.first == key; }) == equalCostPaths.end ())
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = {};
      equalCostPaths.emplace_back (std::move (key), std::move (paths_));
    }

  return key;
}

bool
OspfController::CheckExistsPath (std::vector<Ptr<Node>> path,
                                 std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_)
{
  auto it = std::find_if (paths_.begin (), paths_.end (),
                          [path] (const auto &p) { return p.first == path; });

  return it != paths_.end ();
}

std::vector<std::pair<std::vector<Ptr<Node>>, int>>
OspfController::GetPaths (std::pair<Ptr<Node>, Ptr<Node>> key)
{
  auto it = std::find_if (equalCostPaths.begin (), equalCostPaths.end (),
                          [key] (const auto &ecp) { return ecp.first == key; });

  if (it != equalCostPaths.end ())
    return it->second;

  else
    return std::vector<std::pair<std::vector<Ptr<Node>>, int>>{};
}

void
OspfController::StorePath (Ptr<Node> source, Ptr<Node> destination, std::vector<Ptr<Node>> path,
                           int distance)
{
  std::pair<Ptr<Node>, Ptr<Node>> key1 = OspfController::AddHostsKey (source, destination);
  for (auto &ecp : equalCostPaths)
    {
      if (ecp.first == key1)
        {
          std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
          if (!OspfController::CheckExistsPath (path, paths_))
            {
              std::vector<Ptr<Node>> path_ = path;
              paths_.emplace_back (std::move (path_), distance);
              ecp.second = paths_;
            }
          break;
        }
    }

  std::pair<Ptr<Node>, Ptr<Node>> key2 = OspfController::AddHostsKey (destination, source);
  for (auto &ecp : equalCostPaths)
    {
      if (ecp.first == key2)
        {
          std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
          if (!OspfController::CheckExistsPath (path, paths_))
            {
              std::reverse (path.begin (), path.end ());
              paths_.emplace_back (std::move (path), distance);
              ecp.second = paths_;
            }
          break;
        }
    }
}

void
OspfController::ResizeStoredPaths (int maxStorageNumber)
{
  OspfController::SortStoredPathsAscending ();
  for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
      if (int (paths_.size ()) > maxStorageNumber)
        {
          paths_.resize (maxStorageNumber);
        }

      ecp.second = paths_;
    }
}

// -----------------------------------------------------------------

int
OspfController::MyJumpCalculateCost (Ptr<Node> from, Ptr<Node> to)
{
  int index = int (Simulator::Now ().GetMinutes ()) % 60;
  float outFlex = EnergyAPI::GetFlexArrayAt(Names::FindName(to), index);

  Ptr<NodeEnergyModel> noem2 = to->GetObject<NodeEnergyModel> ();
  if (!noem2)
    return 0;
  double outConsumption = noem2->GetCurrentPowerConsumption ();

  Ptr<Channel> chnl = Topology::GetChannel (from, to);
  if (chnl == NULL)
    return 0;
  double linkUsage = chnl->GetChannelUsage ();
      
  int weight = ((1 + linkUsage) * outConsumption) + outFlex;

  if (weight <= 0)
    {
      if (weight < 0)
        std::cout << "WARNING NEGATIVE WEIGHT: " << weight << std::endl;
      weight = 1; // max?????
    }
    
  return weight;
}

int
OspfController::MyCalculateCost (std::vector<Ptr<Node>> path)
{
  int cost = 0;
  for (auto i = path.begin (); i != path.end () - 1; i++)
    {
      int c = MyJumpCalculateCost (*i, *(i + 1));
      if (cost < 0)
        std::cerr << "[Calculate Cost]Error: Edge not exists!" << std::endl;
      else
        cost += c;
    }
  return cost;
}

// -----------------------------------------------------------------

std::vector<std::vector<Ptr<Node>>>
OspfController::SearchWithDepth (Ptr<Node> source, Ptr<Node> destiny,
                                 std::vector<Ptr<Node>> &ignore, int maxDepth, int currentDepth)
{
  std::vector<std::vector<Ptr<Node>>> allPaths = {};
  if (source == destiny || currentDepth > maxDepth)
    return allPaths; // return empty vector to stop the search

  std::vector<Ptr<Node>> successors = Topology::GetSuccessors (source);
  if (std::find (successors.begin (), successors.end (), destiny) != successors.end ())
    {
      allPaths.push_back ({destiny});
      return allPaths;
    }

  ignore.push_back (source);
  for (auto i : successors)
    {
      if (std::find (ignore.begin (), ignore.end (), i) == ignore.end ())
        {
          std::vector<std::vector<Ptr<Node>>> res =
              SearchWithDepth (i, destiny, ignore, maxDepth, currentDepth + 1);
          for (auto &j : res)
            {
              j.reserve (j.size () + 1);
              j.insert (j.begin (), i);
              allPaths.push_back (std::move (j));
            }
        }
    }
  ignore.pop_back ();
  return allPaths;
}

std::vector<std::vector<Ptr<Node>>>
OspfController::SearchWithCost (Ptr<Node> source, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore,
                                int maxCost, int currentCost)
{
  std::vector<std::vector<Ptr<Node>>> allPaths = {};
  if (source == destiny || currentCost > maxCost)
    return allPaths; // return empty vector to stop the search

  std::vector<Ptr<Node>> successors = Topology::GetSuccessors (source);
  if (std::find (successors.begin (), successors.end (), destiny) != successors.end ())
    {
      allPaths.push_back ({destiny});
      return allPaths;
    }

  ignore.push_back (source);
  for (auto i : successors)
    {
      if (std::find (ignore.begin (), ignore.end (), i) == ignore.end ())
        {
          int edgWeight = Topology::GetEdgeWeight (source, i); //
          std::vector<std::vector<Ptr<Node>>> res =
              SearchWithCost (i, destiny, ignore, maxCost, currentCost + edgWeight);
          for (auto &j : res)
            {
              // rever como se estãoa inserir elementos
              j.reserve (j.size () + 1);
              j.insert (j.begin (), i);
              allPaths.push_back (std::move (j));
            }
        }
    }
  ignore.pop_back (); // retirar???
  return allPaths;
}

void
OspfController::FindAllPaths (Ptr<Node> source, Ptr<Node> destination)
{
  //  SEARCH WITH DEPTH n SHORTEST PATHS WITH LIMITATION ON NUMBER OF HOPS
  std::vector<Ptr<Node>> ignore = std::vector<Ptr<Node>> () = {};

  double ratio = 1.5; // change here to manage the max depth of the search
  int MAX_DEPTH = int (ratio * FindDepth (source, destination));
  //int MAX_COST = int (ratio * FindCost (source, destination));

  std::vector<std::vector<Ptr<Node>>> res =
      SearchWithDepth (source, destination, ignore, MAX_DEPTH, 0);

  for (auto &p : res)
    {
      if(Names::FindName(Topology::GetSuccessors(source)[0])=="S1" && Names::FindName(Topology::GetSuccessors(destination)[0])=="S9"){

        std::cout << "PATH:";
        for (auto &i : p)
          std::cout << Names::FindName(i) << " ";
        std::cout << std::endl;
      }

      p.insert (p.begin (), source);

      int distance = Topology::CalculateCost (p);

      StorePath (source, destination, p, distance);
    }

  int tam = (int) res.size ();
  std::cout << "------------> FindAllPaths from: " << Names::FindName(Topology::GetSuccessors(source)[0]) << " to: " << Names::FindName(Topology::GetSuccessors(destination)[0]) << " -> Number of paths foundd: " << tam << std::endl;
  if (tam > 0){
    std::cout << "FindAllPaths from: " << source->GetId () << " to: " << destination->GetId () << " -> Number of paths found: " << tam << std::endl;
  }
  else
    {
      NS_LOG_ERROR ("No paths found");
      std::cout << "No paths found from: " << source->GetId () << " to: " << destination->GetId ()
                << std::endl;
    }
}

void
OspfController::FindReferenceBandwidth ()
{
  base_graph = Topology::GetGraph ();

  bool findMax = !referenceBandwidthValue; // if not setted manually, find the max value

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
          uint64_t bitRate = chnl->GetDataRate ().GetBitRate ();
          boost::put (edge_weight_t (), base_graph, ed, bitRate);

          if (findMax)
            referenceBandwidthValue =
                bitRate > referenceBandwidthValue ? bitRate : referenceBandwidthValue;
        }
      else
        {
          edgesToRemove.push_back (ed);
          Topology::UpdateEdgeWeight (n1, n2, 0); // rever!!!!
        }
    }
  for (auto ed : edgesToRemove)
    {
      boost::remove_edge (ed, base_graph);
    }
}

int
OspfController::FindDepth (Ptr<Node> source, Ptr<Node> destiny)
{
  int nrJumps = int ((Topology::DijkstraShortestPath (source, destiny)).size ());
  return --nrJumps;
}

int
OspfController::FindCost (Ptr<Node> source, Ptr<Node> destiny)
{
  int cost = Topology::CalculateCost (Topology::DijkstraShortestPath (source, destiny));
  return cost;
}

void
OspfController::SetWeightsBandwidthBased ()
{
  cout << "Reference Bandwidth Value: " << referenceBandwidthValue << endl;

  boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
  for (boost::tie (edgeIt, edgeEnd) = boost::edges (base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      uint64_t bitRate =
          boost::get (edge_weight_t (), base_graph,
                      ed); // ver se preciso de fazer como esta ano topology a separar pelos Nodes

      uint64_t weight = (uint64_t) ((double) referenceBandwidthValue / (double) bitRate + 0.5);
      if (weight < 1)
        weight = 1;
      else if (weight > UINT64_MAX) // rever !!!!
        weight = UINT64_MAX;

      boost::put (edge_weight_t (), base_graph, ed, weight);
      // cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << weight << endl;

      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);
      Topology::UpdateEdgeWeight (n1, n2, weight);
    }
  cout << "-----------------------------" << endl;
}

void
OspfController::UpdateDistances ()
{
  for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;

      for (auto &path : paths_)
        {
          int distance = OspfController::MyCalculateCost (path.first); // mudar aqui !!!!! para calcular o custo com a fórmula personalizada !!!!!
          path.second = distance;
        }

      ecp.second = paths_;
    }
}

void
OspfController::UpdateWeights ()
{
  int index = int (Simulator::Now ().GetMinutes ()) % 60;

  boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
  Graph g = Topology::GetGraph ();
  for (boost::tie (edgeIt, edgeEnd) = boost::edges (g); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);

      // float flex1 = EnergyAPI::GetFlexArrayAt(Names::FindName(n1), index);
      float flex2 = EnergyAPI::GetFlexArrayAt(Names::FindName(n2), index);

      // link cost formula
      Ptr<NodeEnergyModel> noem2 = n2->GetObject<NodeEnergyModel> ();
      if (!noem2)
        continue;
      double n2Consumption = noem2->GetCurrentPowerConsumption ();
      // std::cout << "GetTotalPowerConsumption from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

      Ptr<Channel> chnl = Topology::GetChannel (n1, n2);
      if (chnl == NULL)
        continue;
      double linkUsage = chnl->GetChannelUsage ();
      // std::cout << "GetChannelUsage from node: " << n1->GetId() << " to: " << n2->GetId() << " : " << linkUsage << std::endl;
      
      int new_weight = ((1 + linkUsage) * n2Consumption) + flex2;

      if (new_weight <= 0)
        {
          if (new_weight < 0)
            std::cout << "WARNING NEGATIVE WEIGHT: " << new_weight << std::endl;
          new_weight = 1; // max?????
        }
      // std::cout << "New Weight: " << new_weight << std::endl;

      Topology::UpdateEdgeWeight (n1, n2, new_weight);
    }
  UpdateDistances ();
}

void
OspfController::ApplyRouting (uint64_t swDpId, Ptr<Node> host, Ptr<Node> nextJump)
{
  uint32_t swId = DpId2Id (swDpId);
  Ptr<Node> sw = NodeContainer::GetGlobal ().Get (swId);

  Ptr<OFSwitch13Device> ofDevice = sw->GetObject<OFSwitch13Device> ();

  Ipv4Address remoteAddr = host->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
  try
    {
      uint32_t port = ofDevice->GetPortNoConnectedTo (nextJump);

      std::ostringstream cmd;
      cmd << "flow-mod cmd=add,table=0 eth_type=0x800,ip_dst=" << remoteAddr
          << " apply:output=" << port;

      NS_LOG_DEBUG ("[" << swDpId << "]: " << cmd.str ());

      DpctlExecute (swDpId, cmd.str ());
    }
  catch (const std::exception &e)
    {
      std::cerr << e.what () << '\n';
    }
}

void
OspfController::ApplyRoutingFromPath (std::vector<Ptr<Node>> path)
{
  std::cout << "ApplyRoutingFromPath: ";
  for (auto &i : path)
    {
      std::cout << i->GetId () << " ";
    }
  std::cout 
  //<< "  (" << Topology::CalculateCost (path) << ")" 
  << std::endl;

  Ptr<Node> hostDst = path.back ();
  for (int i = 1; i < int (path.size ()) - 1; i++)
    {
      ApplyRouting (Id2DpId (path.at (i)->GetId ()), hostDst, path.at (i + 1));
    }
}

void
OspfController::PrintCosts ()
{
  int index = int (Simulator::Now ().GetMinutes ()) % 60;

  std::cout << "-----------------------------" << std::endl;
  std::cout << " Time(seconds): " << Simulator::Now ().GetSeconds () << std::endl;
  std::cout << "Link Costs: " << std::endl;
  boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
  Graph g = Topology::GetGraph ();
  for (boost::tie (edgeIt, edgeEnd) = boost::edges (g); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode (ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode (ed.m_target);

      float flex1 = EnergyAPI::GetFlexArrayAt(Names::FindName(n1), index);
      float flex2 = EnergyAPI::GetFlexArrayAt(Names::FindName(n2), index);

      Ptr<Channel> chnl = Topology::GetChannel (n1, n2);
      if (chnl == NULL)
        continue;

      double linkUsage = chnl->GetChannelUsage ();
      // std::cout << "GetChannelUsage from node: " << n1->GetId() << " to: " << n2->GetId() << " : " << linkUsage << std::endl;

      Ptr<NodeEnergyModel> noem1 = n1->GetObject<NodeEnergyModel> ();
      if (noem1)
        {
          double n1Consumption = noem1->GetCurrentPowerConsumption ();
          // std::cout << "GetTotalPowerConsumption from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

          int weight = ((1 + linkUsage) * n1Consumption) + flex1;

          std::cout << "Edge: " << n2->GetId () << " -> " << n1->GetId () << " | Weight: " << weight
                    << std::endl;
        }

      Ptr<NodeEnergyModel> noem2 = n2->GetObject<NodeEnergyModel> ();
      if (noem2)
        {
          double n2Consumption = noem2->GetCurrentPowerConsumption ();
          // std::cout << "GetTotalPowerConsumption from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

          int weight = ((1 + linkUsage) * n2Consumption) + flex2;

          std::cout << "Edge: " << n1->GetId () << " -> " << n2->GetId () << " | Weight: " << weight
                    << std::endl;
        }
    }
}

void
OspfController::UpdateRouting ()
{
  UpdateWeights ();
  SortStoredPathsAscending ();

  int i = 0;
  for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
      if (int (paths_.size ()) == 0)
        continue;

      // TO-DO: load balancer ???
      std::vector<Ptr<Node>> shortestPath = paths_.at (0).first;

      int index = int (Simulator::Now ().GetMinutes ()) % 60;
      std::cout << i << " " << index << " ";
      i++;
      ApplyRoutingFromPath (shortestPath);
    }
  PrintCosts ();
}

void
OspfController::StartRoutingLoop ()
{
  OspfController::UpdateRouting ();
  std::cout << "-----[New Routing Loop]-----" << std::endl;
  Simulator::Schedule (Minutes (1), &OspfController::StartRoutingLoop, this);
}

void
OspfController::HandshakeSuccessful (Ptr<const RemoteSwitch> sw)
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
      // associate id's with given names
      NodeContainer allNodes = NodeContainer::GetGlobal ();
      for (auto it = allNodes.Begin (); it != allNodes.End (); it++)
        {
          Ptr<Node> node = NodeContainer::GetGlobal ().Get ((*it)->GetId ());
          std::cout << "Node: " << node->GetId () << " | Name: " << Names::FindName (node)
                    << std::endl;
        }
      //associate links with given names
      ChannelContainer c = ChannelContainer::GetGlobal ();
      for (ChannelContainer::Iterator i = c.Begin (); i != c.End (); ++i)
        {
          Ptr<Node> src = (*i)->GetDevice (0)->GetNode ();
          Ptr<Node> dst = (*i)->GetDevice (1)->GetNode ();

          std::string linkName = Names::FindName ((*i));

          std::cout << "Link: " << linkName << " | Source: " << src->GetId ()
                    << " | Destiny: " << dst->GetId () << std::endl;
        }

      // Config::SetDefault ("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue (true));

      FindReferenceBandwidth ();
      SetWeightsBandwidthBased ();

      // pode ser melhorado este sistema de procura, aprender a usar o djikstra do boost
      NodeContainer hosts = NodeContainer::GetGlobalHosts ();
      for (auto hst1 = hosts.Begin (); hst1 != hosts.End (); hst1++)
        {
          Ptr<Node> hostNode1 = NodeContainer::GetGlobal ().Get ((*hst1)->GetId ());
          for (auto hst2 = hst1 + 1; hst2 != hosts.End (); hst2++)
            {
              Ptr<Node> hostNode2 = NodeContainer::GetGlobal ().Get ((*hst2)->GetId ());
              FindAllPaths (hostNode1, hostNode2);
            }
        }
      std::cout << "-----------------------------" << std::endl;

      ResizeStoredPaths (20); // Set here the max number of paths that can be stored VER MELHOR !!!!, se calhar tirar !!!

      StartRoutingLoop ();
      // StatsLoop ();
      m_isFirstUpdate = false;
      lastUpdate = Simulator::Now ();
    }
  else if (Simulator::Now () - lastUpdate >= Minutes (1))
    {
      UpdateRouting ();
      lastUpdate = Simulator::Now ();
    }
  // UpdateRouting(); todos os switchs estão a fazer esta chamada desta função que deixou de ser individual para cada switch
  //                  e passou a ser geral, fazer um indivual ????????
}
} // namespace ns3
