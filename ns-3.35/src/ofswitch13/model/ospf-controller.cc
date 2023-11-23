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
#include "ns3/path-store.h"

#include <chrono> //remove after

NS_LOG_COMPONENT_DEFINE("OspfController");

namespace ns3
{

  NS_OBJECT_ENSURE_REGISTERED(OspfController);

  std::list<std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore>> equalCostPaths;

  // if not initializade at 0 it will be the bandwidth reference value used to calculate the weights
  uint64_t OspfController::referenceBandwidthValue = 0;
  Time lastUpdate;

  OspfController::OspfController()
  {
    NS_LOG_FUNCTION(this);
    m_isFirstUpdate = true;
  }

  OspfController::~OspfController()
  {
    NS_LOG_FUNCTION(this);
  }

  TypeId OspfController::GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::OspfController")
                            .SetParent<OFSwitch13Controller>()
                            .SetGroupName("OFSwitch13")
                            .AddConstructor<OspfController>();
    return tid;
  }

  void OspfController::DoDispose(void)
  {
    NS_LOG_FUNCTION(this);
    OFSwitch13Controller::DoDispose();
  }

  void OspfController::AddSwitchHostKey(Ptr<Node> switchNode, Ptr<Node> hostNode)
  {
    std::pair<Ptr<Node>, Ptr<Node>> key = std::make_pair(switchNode, hostNode);
    std::list<std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore>>::iterator it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                                                                                                 [key](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore> &ecp)
                                                                                                 { return ecp.first.first == key.first && ecp.first.second == key.second; });

    if (it == equalCostPaths.end())
    {
      PathStore ecp = PathStore();
      equalCostPaths.emplace_back(std::make_pair(key, ecp));
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
      std::cout << "Illegal path [AddSwitchHostKey] from: " << switchNode->GetId() << " to: " << hostNode->GetId() << std::endl;
    }
  }

  void OspfController::StorePath(Ptr<Node> switchNode, Ptr<Node> hostNode, std::vector<Ptr<Node>> path)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [switchNode, hostNode](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore> &ecp)
                           { return ecp.first.first == switchNode && ecp.first.second == hostNode; });

    if (it != equalCostPaths.end())
    {
      it->second.AddPath(path);
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
      std::cout << "Illegal path [StorePath] from: " << switchNode->GetId() << " to: " << hostNode->GetId() << std::endl;
    }
  }

  void OspfController::CleanPaths(Ptr<Node> switchNode, Ptr<Node> hostNode)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [switchNode, hostNode](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore> &ecp)
                           { return ecp.first.first == switchNode && ecp.first.second == hostNode; });

    if (it != equalCostPaths.end())
    {
      it->second.CleanPaths();
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
      std::cout << "Illegal path [CleanPaths] from: " << switchNode->GetId() << " to: " << hostNode->GetId() << std::endl;
    }
  }

  std::vector<std::vector<Ptr<Node>>>
  OspfController::Search(Ptr<Node> init, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore)
  {
    std::vector<std::vector<Ptr<Node>>> allPaths;
    if (init == destiny)
    {
      return allPaths; // return empty vector
    }

    std::vector<Ptr<Node>> successors = Topology::GetSuccessors(init);
    if (std::find(successors.begin(), successors.end(), destiny) != successors.end())
    {
      allPaths.push_back({destiny});
      return allPaths;
    }

    ignore.push_back(init);
    for (auto i : successors)
    {
      if (std::find(ignore.begin(), ignore.end(), i) == ignore.end())
      {
        std::vector<std::vector<Ptr<Node>>> res = Search(i, destiny, ignore);
        if (!res.empty())
        {
          for (auto &j : res)
          {
            j.reserve(j.size() + 1);
            j.insert(j.begin(), i);
            allPaths.push_back(std::move(j));
          }
        }
      }
    }
    ignore.pop_back();
    return allPaths;
  }

  std::vector<std::vector<Ptr<Node>>>
  OspfController::SearchWithDepth(Ptr<Node> init, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore, int maxDepth, int currentDepth)
  {
    std::vector<std::vector<Ptr<Node>>> allPaths;
    if (init == destiny)
    {
      return allPaths; // return empty vector
    }

    if (currentDepth >= maxDepth)
    {
      return allPaths; // return empty vector to stop the search
    }

    std::vector<Ptr<Node>> successors = Topology::GetSuccessors(init);
    if (std::find(successors.begin(), successors.end(), destiny) != successors.end())
    {
      allPaths.push_back({destiny});
      return allPaths;
    }

    ignore.push_back(init);
    for (auto i : successors)
    {
      if (std::find(ignore.begin(), ignore.end(), i) == ignore.end())
      {
        std::vector<std::vector<Ptr<Node>>> res = SearchWithDepth(i, destiny, ignore, maxDepth, currentDepth + 1);
        if (!res.empty())
        {
          for (auto &j : res)
          {
            j.reserve(j.size() + 1);
            j.insert(j.begin(), i);
            allPaths.push_back(std::move(j));
          }
        }
      }
    }
    ignore.pop_back();
    return allPaths;
  }

  void
  OspfController::FindAllPaths(Ptr<Node> source, Ptr<Node> destination)
  {
    std::vector<Ptr<Node>> ignore = std::vector<Ptr<Node>>();

    // std::vector<std::vector<Ptr<Node>>> res = Search(source, destination, ignore);

    // double ratio = 1.5; // change here to manage the max depth of the search
    // int MAX_DEPTH = int(ratio * FindMaxDepth(source, destination));
    int MAX_DEPTH = FindMaxDepth(source, destination);
    MAX_DEPTH = int(std::sqrt(3.0 * MAX_DEPTH + 5)) + MAX_DEPTH;
    // std::cout << "MAX DEPTH from:" << source->GetId() << " to: " << destination->GetId() << " : " << MAX_DEPTH << std::endl;
    std::vector<std::vector<Ptr<Node>>> res = SearchWithDepth(source, destination, ignore, MAX_DEPTH, 0);

    if (!res.empty())
    {
      AddSwitchHostKey(source, destination);
      for (auto &i : res)
      {
        i.insert(i.begin(), source);
        StorePath(source, destination, i);
      }
    }
    else
    {
      NS_LOG_ERROR("No paths found");
      std::cout << "No paths found from: " << source->GetId() << " to: " << destination->GetId() << std::endl;
    }
  }

  std::vector<Ptr<Node>>
  OspfController::GetShortesPath(Ptr<Node> source, Ptr<Node> destination)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [source, destination](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, PathStore> &ecp)
                           { return ecp.first.first == source && ecp.first.second == destination; });

    if (it != equalCostPaths.end())
    {
      return it->second.GetShortestPath();
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
      std::cout << "Illegal path [GetShortesPath] from: " << source->GetId() << " to: " << destination->GetId() << std::endl;
      return std::vector<Ptr<Node>>{};
    }
  }

  void
  OspfController::ResizeStoredPaths(int maxStorageNumber)
  {
    for (auto &ecp : equalCostPaths)
    {
      ecp.second.CutNumberStoredPaths(maxStorageNumber);
    }
  }

  void
  OspfController::FindReferenceBandwidth()
  {
    base_graph = Topology::GetGraph();

    bool findMax = !referenceBandwidthValue;

    std::list<Edge> edgesToRemove;

    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);

      if (n1->IsSwitch() && n2->IsSwitch())
      {
        Ptr<Channel> chnl = Topology::GetChannel(n1, n2);
        uint64_t bitRate = chnl->GetDataRate().GetBitRate();
        boost::put(edge_weight_t(), base_graph, ed, bitRate);

        if (findMax)
          referenceBandwidthValue = bitRate > referenceBandwidthValue ? bitRate : referenceBandwidthValue;
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

  int OspfController::FindMaxDepth(Ptr<Node> source, Ptr<Node> destiny)
  {
    int nrJumps = int((Topology::DijkstraShortestPath(source, destiny)).size());
    return --nrJumps;
  }

  void OspfController::SetWeightsBandwidthBased()
  {
    cout << "Reference Bandwidth Value: " << referenceBandwidthValue << endl;
    if (referenceBandwidthValue) // preventing division by 0, if there is no switchs
    {
      boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
      for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
      {
        Edge ed = *edgeIt;
        uint64_t bitRate = boost::get(edge_weight_t(), base_graph, ed); // ver se preciso de fazer como esta ano topology a separar pelos Nodes

        uint64_t weight = (uint64_t)((double)referenceBandwidthValue / (double)bitRate + 0.5);
        if (weight < 1)
          weight = 1;
        else if (weight > UINT64_MAX)
          weight = UINT64_MAX;

        boost::put(edge_weight_t(), base_graph, ed, weight);
        // cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << weight << endl;

        Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
        Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
        Topology::UpdateEdgeWeight(n1, n2, weight);
      }
      cout << "-----------------------------" << endl;
    }
    //else throw error???
  }

  void
  OspfController::UpdateWeights()
  {
    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
      uint64_t weight = boost::get(edge_weight_t(), base_graph, ed); // ver se preciso de fazer como est ano topology a separar pelos Nodes

      int index = int(Simulator::Now().GetMinutes()) % 60;

      float flex1 = EnergyAPI::GetFlexArrayAt(Names::FindName(n1), index);

      float flex2 = EnergyAPI::GetFlexArrayAt(Names::FindName(n2), index);

      // tambem podemos usar a taxa de utilizacao da do link para a flexibilidade impactar + ou - no peso
      // Ptr<Channel> chnl = Topology::GetChannel(n1, n2);
      // double usagePercentage =  chnl->GetChannelUsage(); //valor em percentagem !!!
      // int new_weight = int (weight - (flex1 + flex2)/2 * (1-usagePercentage)); // quando maior for a utilizacao menor é o impacto da flexibilidade

      // ponderar se vemos a flexibildade como um valor unico e nao é preciso guardar a informacao inicial
      // pode ser bom caso os switches sejam iguais e tenham o mesmo consumo, a formula de cima seria boa????
      // mas podia escaxar porque calcula o min e nao o max, o djikstra so lida com pesos positivos, volta à situação??
      //

      // alterar aqui a expressão que se pretende usar para calcular o impacto da flexibilidade
      int new_weight = int(weight - (flex1 + flex2) / 2);
      if (new_weight < 0)
      {
        std::cout << "WARNING NEGATIVE WEIGHT: " << new_weight << std::endl;
      }
      new_weight = new_weight < 0 ? 0 : new_weight;
      // expressao produz nrs negativos, mas depois insere 0 no maximo

      Topology::UpdateEdgeWeight(n1, n2, new_weight);
    }
  }

  void
  OspfController::ApplyRouting(uint64_t swDpId, Ptr<Node> host, Ptr<Node> nextJump)
  {
    uint32_t swId = DpId2Id(swDpId);
    Ptr<Node> sw = NodeContainer::GetGlobal().Get(swId);

    Ptr<OFSwitch13Device> ofDevice = sw->GetObject<OFSwitch13Device>();

    Ipv4Address remoteAddr = host->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
    try
    {
      uint32_t port = ofDevice->GetPortNoConnectedTo(nextJump);

      std::ostringstream cmd;
      cmd << "flow-mod cmd=add,table=0 eth_type=0x800,ip_dst=" << remoteAddr
          << " apply:output=" << port;

      NS_LOG_DEBUG("[" << swDpId << "]: " << cmd.str());

      DpctlExecute(swDpId, cmd.str());
    }
    catch (const std::exception &e)
    {
      std::cerr << e.what() << '\n';
    }
  }

  void
  OspfController::ApplyRoutingFromPath(std::vector<Ptr<Node>> path)
  {
    Ptr<Node> hostBegin = path.front();
    for (int i = 1; i < int(path.size()) - 2; i++)
    {
      ApplyRouting(Id2DpId(path.at(i)->GetId()), hostBegin, path.at(i + 1));
    }
  }

  void 
  OspfController::UpdateRouting()
  {
    UpdateWeights();
    // calculate distances
    for (auto &ecp : equalCostPaths)
    {
      ecp.second.CalculateDistances();
    }

    std::cout << "Equal Cost Paths: " << std::endl;
    for (auto &ecp : equalCostPaths)
    {
      // get shortest path
      std::vector<Ptr<Node>> shortestPath = ecp.second.GetShortestPath();
      ApplyRoutingFromPath(shortestPath);
      // reverse shortest path
      std::reverse(shortestPath.begin(), shortestPath.end());
      ApplyRoutingFromPath(shortestPath);

      // std::cout << "HostBegin: " << ecp.first.first->GetId() << " - HostEnd: " << ecp.first.second->GetId() << "(and reversed)" << std::endl;
      // int i = 0;
      // for (auto &path : ecp.second.GetPaths())
      // {
      //   std::cout << "Best Path " << i++ << ": ";
      //   for (auto &node : path.first)
      //   {
      //     std::cout << node->GetId() << " ";
      //   }
      //   std::cout << "  (" << path.second << ")" << std::endl;
      //   if (i == 1)
      //   {
      //     break;
      //   }
      // }
      // std::cout << "Number of paths: " << ecp.second.GetPaths().size() << std::endl;
    }
    // std::cout << "------------ END STEP -----------------" << std::endl;
  }

  void
  OspfController::StartRoutingLoop()
  {
    OspfController::UpdateRouting();
    std::cout << "-----[New Routing Loop]-----" << std::endl;
    Simulator::Schedule(Minutes(1), &OspfController::StartRoutingLoop, this);
  }

  void
  OspfController::HandshakeSuccessful(Ptr<const RemoteSwitch> sw)
  {
    NS_LOG_FUNCTION(this << sw);

    uint64_t swDpId = sw->GetDpId();

    // Default rules
    DpctlExecute(swDpId, "flow-mod cmd=add,table=0,prio=0 "
                         "apply:output=ctrl:128");
    DpctlExecute(swDpId, "set-config miss=128");

    // restringir logo aqui os switchs para nao fazerem schedulings de nada ??????? REVER

    if (m_isFirstUpdate)
    {
      FindReferenceBandwidth();
      SetWeightsBandwidthBased();

      auto start = std::chrono::high_resolution_clock::now();

      // pode ser melhorado este sistema de procura, aprender a usar o djikstra do boost
      NodeContainer hosts = NodeContainer::GetGlobalHosts();
      for (auto hst1 = hosts.Begin(); hst1 != hosts.End(); hst1++)
      {
        Ptr<Node> hostNode1 = NodeContainer::GetGlobal().Get((*hst1)->GetId());
        for (auto hst2 = hst1 + 1; hst2 != hosts.End(); hst2++)
        {
          Ptr<Node> hostNode2 = NodeContainer::GetGlobal().Get((*hst2)->GetId());
          // std::cout << "FindAllPaths from: " << hostNode1->GetId() << " to: " << hostNode2->GetId() << std::endl;
          FindAllPaths(hostNode1, hostNode2);
        }
      }
      auto end = std::chrono::high_resolution_clock::now();
      auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
      uint64_t milliseconds = duration.count() / 1000;
      uint64_t seconds = milliseconds / 1000;
      milliseconds = milliseconds % 1000;
      uint64_t minutes = seconds / 60;
      seconds = seconds % 60;
      uint64_t hours = minutes / 60;
      minutes = minutes % 60;

      std::cout << "-----> FindAllPaths Time: " << hours << "h " << minutes << "m " << seconds << "s " << milliseconds << "ms" << std::endl;
      std::cout << "-----------------------------" << std::endl;

      ResizeStoredPaths(30); // Set here the max number of paths that can be stored

      StartRoutingLoop();
      lastUpdate = Simulator::Now();
      m_isFirstUpdate = false;
    }
    else if (Simulator::Now() - lastUpdate >= Minutes(1))
    {
      UpdateRouting();
      lastUpdate = Simulator::Now();
    }
    // UpdateRouting(); todos os switchs estão a fazer esta chamada desta função que deixou de ser individual para cada switch
    //                  e passou a ser geral, fazer um indivual ????????
  }
} // namespace ns3
