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

#include <chrono> //remove after

NS_LOG_COMPONENT_DEFINE("OspfController");

namespace ns3
{

  NS_OBJECT_ENSURE_REGISTERED(OspfController);

  std::vector<
      std::pair<
          std::pair< // source e destiny
              Ptr<Node>,
              Ptr<Node>>,
          std::vector< // todos caminhos existentes
              std::pair<
                  std::vector<Ptr<Node>>, // 1 caminho
                  int                     // distancia
                  >>>>
      equalCostPaths;

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

  // ---------------------- Stored Paths ----------------------
  void OspfController::SortStoredPathsAscending()
  {
    for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;

      std::sort(paths_.begin(), paths_.end(), [](const auto &lhs, const auto &rhs)
                { if (lhs.second == rhs.second)
                    return lhs.first.size() < rhs.first.size();
                  else
                    return lhs.second < rhs.second; });
    }
  }

  void OspfController::SortStoredPathsDescending()
  {
    for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;

      std::sort(paths_.begin(), paths_.end(), [](const auto &lhs, const auto &rhs)
                { if (lhs.second == rhs.second)
                    return lhs.first.size() > rhs.first.size();
                  else
                    return lhs.second > rhs.second; });
    }
  }

  void OspfController::AddHostsKey(std::pair<Ptr<Node>, Ptr<Node>> key)
  {
    if (std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                     [key](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, std::vector<std::pair<std::vector<Ptr<Node>>, int>>> &ecp)
                     { return ecp.first == key; }) == equalCostPaths.end())
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = std::vector<std::pair<std::vector<Ptr<Node>>, int>>();
      equalCostPaths.emplace_back(std::make_pair(key, paths_));
    }
  }

  bool OspfController::CheckExistsPath(std::vector<Ptr<Node>> path, std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_)
  {
    std::vector<std::pair<std::vector<Ptr<Node>>, int>>::iterator it = std::find_if(paths_.begin(), paths_.end(),
                                                                                    [path](const std::pair<std::vector<Ptr<Node>>, int> &p)
                                                                                    { return p.first == path; });

    return it != paths_.end();
  }

  std::vector<std::pair<std::vector<Ptr<Node>>, int>>
  OspfController::GetPaths(std::pair<Ptr<Node>, Ptr<Node>> key)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [key](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, std::vector<std::pair<std::vector<Ptr<Node>>, int>>> &ecp)
                           { return ecp.first == key; });

    if (it != equalCostPaths.end())
      return it->second;

    else
      return std::vector<std::pair<std::vector<Ptr<Node>>, int>>{};
  }

  void OspfController::StorePath(Ptr<Node> source, Ptr<Node> destination, std::vector<Ptr<Node>> path, int distance)
  {
    std::pair<Ptr<Node>, Ptr<Node>> key;
    if (source->GetId() < destination->GetId())
      key = std::make_pair(source, destination);
    else
      key = std::make_pair(destination, source);

    OspfController::AddHostsKey(key);

    for (auto &ecp : equalCostPaths)
    {
      if (ecp.first == key)
      {
        std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
        if (!OspfController::CheckExistsPath(path, paths_))
        {
          paths_.emplace_back(std::make_pair(path, distance));
          ecp.second = paths_;
        }
        break;
      }
    }
  }

  void OspfController::ResizeStoredPaths(int maxStorageNumber)
  {
    OspfController::SortStoredPathsAscending();
    for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
      if (int(paths_.size()) > maxStorageNumber)
      {
        paths_.resize(maxStorageNumber);
      }
    }
  }

  // -----------------------------------------------------------------

  std::vector<std::vector<Ptr<Node>>>
  OspfController::SearchWithDepth(Ptr<Node> source, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore, int maxDepth, int currentDepth)
  {
    std::vector<std::vector<Ptr<Node>>> allPaths;
    if (source == destiny)
    {
      return allPaths; // return empty vector
    }

    if (currentDepth >= maxDepth)
    {
      return allPaths; // return empty vector to stop the search
    }

    std::vector<Ptr<Node>> successors = Topology::GetSuccessors(source);
    if (std::find(successors.begin(), successors.end(), destiny) != successors.end())
    {
      allPaths.push_back({destiny});
      return allPaths;
    }

    ignore.push_back(source);
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

  void OspfController::FindAllPaths(Ptr<Node> source, Ptr<Node> destination)
  {
    std::cout << "FindAllPaths from: " << source->GetId() << " to: " << destination->GetId() << " -> ";
    //                      SEARCH WITH DEPTH n SHORTEST PATHS WITH LIMITATION ON NUMBER OF HOPS
    std::vector<Ptr<Node>> ignore = std::vector<Ptr<Node>>();

    double ratio = 1.5; // change here to manage the max depth of the search
    int MAX_DEPTH = int(ratio * FindMaxDepth(source, destination)) + 2;
    std::vector<std::vector<Ptr<Node>>> res = SearchWithDepth(source, destination, ignore, MAX_DEPTH, 0);

    if (!res.empty())
    {
      for (auto &i : res)
      {
        i.insert(i.begin(), source);

        int distance = Topology::CalculateCost(i);

        StorePath(source, destination, i, distance);
      }
    }

    int tam = (int)res.size();
    if (tam > 0)
    {
      std::cout << "Number of paths found: " << tam << std::endl;
    }
    else
    {
      NS_LOG_ERROR("No paths found");
      std::cout << "No paths found from: " << source->GetId() << " to: " << destination->GetId() << std::endl;
    }
  }

  // std::vector<Ptr<Node>>
  // OspfController::GetShortesPath(Ptr<Node> source, Ptr<Node> destination)
  // {
  //   auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
  //                          [source, destination](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, Ptr<PathStore>> &ecp)
  //                          { return ecp.first.first == source && ecp.first.second == destination; });

  //   if (it != equalCostPaths.end())
  //   {
  //     return it->second->GetShortestPath();
  //   }
  //   else
  //   {
  //     NS_LOG_ERROR("Illegal path");
  //     std::cout << "Illegal path [GetShortesPath] from: " << source->GetId() << " to: " << destination->GetId() << std::endl;
  //     return std::vector<Ptr<Node>>{};
  //   }
  // }

  void OspfController::FindReferenceBandwidth()
  {
    base_graph = Topology::GetGraph();

    bool findMax = !referenceBandwidthValue; // if not setted manually, find the max value

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

    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      uint64_t bitRate = boost::get(edge_weight_t(), base_graph, ed); // ver se preciso de fazer como esta ano topology a separar pelos Nodes

      uint64_t weight = (uint64_t)((double)referenceBandwidthValue / (double)bitRate + 0.5);
      if (weight < 1)
        weight = 1;
      else if (weight > UINT64_MAX) // rever !!!!
        weight = UINT64_MAX;

      boost::put(edge_weight_t(), base_graph, ed, weight);
      // cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << weight << endl;

      Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
      Topology::UpdateEdgeWeight(n1, n2, weight);
    }
    cout << "-----------------------------" << endl;
  }

  void OspfController::UpdateDistances()
  {
    for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;

      for (auto &path : paths_)
      {
        int distance = Topology::CalculateCost(path.first);
        path.second = distance;
      }
    }
  }

  void OspfController::UpdateWeights()
  {
    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
      uint64_t weight = boost::get(edge_weight_t(), base_graph, ed); // ver se preciso de fazer como est ano topology a separar pelos Nodes

      int index = int(Simulator::Now().GetMinutes()) % 60;

      // float flex1 = EnergyAPI::GetFlexArrayAt(Names::FindName(n1), index);

      // float flex2 = EnergyAPI::GetFlexArrayAt(Names::FindName(n2), index);

      // tambem podemos usar a taxa de utilizacao da do link para a flexibilidade impactar + ou - no peso
      // Ptr<Channel> chnl = Topology::GetChannel(n1, n2);
      // double usagePercentage =  chnl->GetChannelUsage(); //valor em percentagem !!!
      // int new_weight = int (weight - (flex1 + flex2)/2 * (1-usagePercentage)); // quando maior for a utilizacao menor é o impacto da flexibilidade

      // ponderar se vemos a flexibildade como um valor unico e nao é preciso guardar a informacao inicial
      // pode ser bom caso os switches sejam iguais e tenham o mesmo consumo, a formula de cima seria boa????
      // mas podia escaxar porque calcula o min e nao o max, o djikstra so lida com pesos positivos, volta à situação??
      //

      // alterar aqui a expressão que se pretende usar para calcular o impacto da flexibilidade
      // int new_weight = int(weight - (flex1 + flex2) / 2);
      // Ptr<NodeEnergyModel> noem1 = n1->GetObject<NodeEnergyModel> ();
      // double n1Consumption = 0, n2Consumption = 0;
      // if (noem1){
      //   Ptr<OFSwitch13Device> of = n1->GetObject<OFSwitch13Device> ();
      //   std::cout << "----[]------->CPU precentage from node: " << n1->GetId() << " : " << of->GetCpuUsage() << std::endl;

      //   n1Consumption = noem1->GetTotalPowerConsumption(n1);
      //   std::cout << "GetTotalPowerConsumption from node: " << n1->GetId() << " : " << n1Consumption << std::endl;

      //   // n1Consumption = noem1->GetPowerDrawn();
      //   // std::cout << "GetPowerDrawn from node: " << n1->GetId() << " : " << n1Consumption << std::endl;

      //   // n1Consumption = noem1->GetCurrentPowerConsumption();
      //   // std::cout << "GetCurrentPowerConsumption from node: " << n1->GetId() << " : " << n1Consumption << std::endl;

      //   // n1Consumption = noem1->GetPowerPerGB();
      //   // std::cout << "GetPowerPerGB from node: " << n1->GetId() << " : " << n1Consumption << std::endl;
      // }

      // Ptr<NodeEnergyModel> noem2 = n2->GetObject<NodeEnergyModel> ();
      // if (noem2){
      //   Ptr<OFSwitch13Device> of = n2->GetObject<OFSwitch13Device> ();
      //   std::cout << "----[]------->CPU precentage from node: " << n2->GetId() << " : " << of->GetCpuUsage() << std::endl;

      //   n2Consumption = noem2->GetTotalPowerConsumption(n2);
      //   std::cout << "GetTotalPowerConsumption from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

      //   // n2Consumption = noem2->GetPowerDrawn();
      //   // std::cout << "GetPowerDrawn from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

      //   // n2Consumption = noem2->GetCurrentPowerConsumption();
      //   // std::cout << "GetCurrentPowerConsumption from node: " << n2->GetId() << " : " << n2Consumption << std::endl;

      //   // n2Consumption = noem2->GetPowerPerGB();
      //   // std::cout << "GetPowerPerGB from node: " << n2->GetId() << " : " << n2Consumption << std::endl;
      // }
      // std::cout << "-----------------------------" << std::endl;

      double totCons = 0;
      // totCons = n1Consumption + n2Consumption;
      // std ::cout << "Total Consumption from node: " << n1->GetId() << " to: " << n2->GetId() << " : " << totCons << std::endl;

      int new_weight = weight + totCons;
      if (new_weight < 0)
      {
        std::cout << "WARNING NEGATIVE WEIGHT: " << new_weight << std::endl;
      }
      new_weight = new_weight < 0 ? 0 : new_weight;
      // expressao produz nrs negativos, mas depois insere 0 no maximo

      Topology::UpdateEdgeWeight(n1, n2, new_weight);
    }
    UpdateDistances();
  }

  void OspfController::ApplyRouting(uint64_t swDpId, Ptr<Node> host, Ptr<Node> nextJump)
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

  void OspfController::ApplyRoutingFromPath(std::vector<Ptr<Node>> path)
  {
    std::cout << "ApplyRoutingFromPath: ";
    for (auto &i : path)
    {
      std::cout << i->GetId() << " ";
    }
    std::cout << std::endl;
    Ptr<Node> hostDst = path.back();
    for (int i = 1; i < int(path.size()) - 2; i++)
    {
      ApplyRouting(Id2DpId(path.at(i)->GetId()), hostDst, path.at(i + 1));
    }
  }

  void OspfController::PrintCosts()
  {
    std::cout << "-----------------------------" << std::endl;
    std::cout << "Costs: " << std::endl;
    boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
    Graph g = Topology::GetGraph();
    for (boost::tie(edgeIt, edgeEnd) = boost::edges(g); edgeIt != edgeEnd; ++edgeIt)
    {
      Edge ed = *edgeIt;
      Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
      Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
      uint64_t weight = boost::get(edge_weight_t(), g, ed);
      std::cout << "Edge: " << n1->GetId() << " - " << n2->GetId() << " | Weight: " << weight << std::endl;
    }
  }

  void OspfController::UpdateRouting()
  {
    UpdateWeights();
    SortStoredPathsAscending();

    for (auto &ecp : equalCostPaths)
    {
      std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths_ = ecp.second;
      if (int(paths_.size()) == 0)
        continue;

      for (auto &path : paths_)
      {
        std::vector<Ptr<Node>> shortestPath = path.first;
        ApplyRoutingFromPath(shortestPath);

        // reverse shortest path
        std::reverse(shortestPath.begin(), shortestPath.end());
        ApplyRoutingFromPath(shortestPath);
      }

      // std::vector<Ptr<Node>> shortestPath = paths_.front().first;
      // ApplyRoutingFromPath(shortestPath);

      // // reverse shortest path
      // std::reverse(shortestPath.begin(), shortestPath.end());
      // ApplyRoutingFromPath(shortestPath);
    }
    PrintCosts();
  }

  void OspfController::StartRoutingLoop()
  {
    OspfController::UpdateRouting();
    std::cout << "-----[New Routing Loop]-----" << std::endl;
    Simulator::Schedule(Minutes(1), &OspfController::StartRoutingLoop, this);
  }

  void OspfController::StatsLoop()
  {
    std::cout << "-----[Stats Loop]-----" << std::endl;
    // iterate switches
    boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
    for (boost::tie(vertexIt, vertexEnd) = boost::vertices(base_graph); vertexIt != vertexEnd; ++vertexIt)
    {
      Ptr<Node> n1 = Topology::VertexToNode(*vertexIt);
      if (n1->IsSwitch())
      {
        Ptr<CpuLoadBasedEnergyModel> cpuLBE = n1->GetObject<CpuLoadBasedEnergyModel>();
        if (cpuLBE)
        {
          std::cout << "[MinMax] " << Names::FindName (n1) << 
                        " " << cpuLBE->GetMinPowerConsumption() << 
                        " " << cpuLBE->GetMaxPowerConsumption() << std::endl;
        }
      }
    }
  }

  void OspfController::HandshakeSuccessful(Ptr<const RemoteSwitch> sw)
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
      // Config::SetDefault ("ns3::Ipv4GlobalRouting::RandomEcmpRouting", BooleanValue (true));

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
          FindAllPaths(hostNode1, hostNode2); //.......
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

      ResizeStoredPaths(7); // Set here the max number of paths that can be stored

      StartRoutingLoop();
      StatsLoop();
      m_isFirstUpdate = false;
      lastUpdate = Simulator::Now();
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