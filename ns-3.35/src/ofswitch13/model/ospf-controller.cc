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

NS_LOG_COMPONENT_DEFINE("OspfController");

namespace ns3
{

  NS_OBJECT_ENSURE_REGISTERED(OspfController);

  class EqualCostPaths
  {
  public:
    EqualCostPaths() {}

    void AddPath(std::vector<Ptr<Node>> path)
    {
      paths_.emplace_back(std::make_pair(std::move(path), -1));
    }

    std::vector<Ptr<Node>> GetShortestPath() const
    {
      // Get the first path with the shortest distance
      return paths_.front().first;
    }

    std::pair<std::vector<Ptr<Node>>, int> GetShortestPathAndDistance() const
    {
      // Get the first path with the shortest distance
      return paths_.front();
    }

    std::list<std::pair<std::vector<Ptr<Node>>, int>> GetPaths() const
    {
      // Get all the paths and their distances
      return paths_;
    }

    std::list<std::pair<std::vector<Ptr<Node>>, int>> GetShortestsPathsInRange(int percentage) const
    {
      // Get all the paths with a distance in the range of the shortest path distance
      std::list<std::pair<std::vector<Ptr<Node>>, int>> shortestsPaths;
      int shortestDistance = paths_.front().second;
      int range = shortestDistance * percentage / 100;
      for (auto &path : paths_)
      {
        if (path.second <= shortestDistance + range)
        {
          shortestsPaths.push_back(path);
        }
        else
        {
          break;
        }
      }
      return shortestsPaths;
    }

    void CleanPaths()
    {
      paths_.clear();
    }

    void CalculateDistances()
    {
      for (auto &path : paths_)
      {
        int distance = 0;
        for (int i = 0; i < int(path.first.size()) - 1; i++)
        {
          Ptr<Node> n1 = path.first.at(i);
          Ptr<Node> n2 = path.first.at(i + 1);
          int acc = Topology::GetEdgeWeight(n1, n2);
          distance += acc;
        }

        path.second = distance;
      }
      // sort paths by distance
      paths_.sort([](const auto &lhs, const auto &rhs)
                  { return lhs.second < rhs.second; });
    }

  private:
    std::list<std::pair<std::vector<Ptr<Node>>, int>> paths_; // Vector of paths from switch to host
  };

  std::list<std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths>> equalCostPaths;

  // if not initializade at 0 it will be the bandwidth reference value used to calculate the weights
  uint64_t OspfController::referenceBandwidthValue = 2000000000;

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
    std::list<std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths>>::iterator it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                                                                                                      [key](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths> &ecp)
                                                                                                      { return ecp.first.first == key.first && ecp.first.second == key.second; });

    if (it == equalCostPaths.end())
    {
      EqualCostPaths ecp;
      equalCostPaths.emplace_back(std::make_pair(key, ecp));
    }
  }

  void OspfController::StorePath(Ptr<Node> switchNode, Ptr<Node> hostNode, std::vector<Ptr<Node>> path)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [switchNode, hostNode](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths> &ecp)
                           { return ecp.first.first == switchNode && ecp.first.second == hostNode; });

    if (it != equalCostPaths.end())
    {
      it->second.AddPath(path);
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
    }
  }

  void OspfController::CleanPaths(Ptr<Node> switchNode, Ptr<Node> hostNode)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [switchNode, hostNode](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths> &ecp)
                           { return ecp.first.first == switchNode && ecp.first.second == hostNode; });

    if (it != equalCostPaths.end())
    {
      it->second.CleanPaths();
    }
  }

  std::vector<std::vector<Ptr<Node>>>
  OspfController::Search(Ptr<Node> init, Ptr<Node> destiny, std::vector<Ptr<Node>> ignore)
  {
    std::vector<std::vector<Ptr<Node>>> allPaths = std::vector<std::vector<Ptr<Node>>>{};
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

    for (auto i : successors)
    {
      if (std::find(ignore.begin(), ignore.end(), i) == ignore.end())
      {
        ignore.push_back(init);
        std::vector<std::vector<Ptr<Node>>> res = Search(i, destiny, ignore);
        if (!res.empty())
        {
          for (auto j : res)
          {
            j.insert(j.begin(), i);
            allPaths.push_back(j);
          }
        }
      }
    }
    return allPaths;
  }

  void
  OspfController::FindAllPaths(Ptr<Node> source, Ptr<Node> destination)
  {
    std::cout << "FindAllPaths from: " << source->GetId() << " to: " << destination->GetId() << std::endl;

    std::vector<std::vector<Ptr<Node>>> res = Search(source, destination, std::vector<Ptr<Node>>{});
    for (auto i : res)
    {
      AddSwitchHostKey(source, destination);
      i.insert(i.begin(), source);
      StorePath(source, destination, i);
    }
  }

  std::vector<Ptr<Node>>
  OspfController::GetShortesPath(Ptr<Node> source, Ptr<Node> destination)
  {
    auto it = std::find_if(equalCostPaths.begin(), equalCostPaths.end(),
                           [source, destination](const std::pair<std::pair<Ptr<Node>, Ptr<Node>>, EqualCostPaths> &ecp)
                           { return ecp.first.first == source && ecp.first.second == destination; });

    if (it != equalCostPaths.end())
    {
      return it->second.GetShortestPath();
    }
    else
    {
      NS_LOG_ERROR("Illegal path");
      return std::vector<Ptr<Node>>{};
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
        {
          referenceBandwidthValue = bitRate > referenceBandwidthValue ? bitRate : referenceBandwidthValue;
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

  void OspfController::SetWeightsBandwidthBased()
  {
    cout << "Reference Bandwidth Value: " << referenceBandwidthValue << endl;
    if (referenceBandwidthValue) // preventing division by 0, if there is no switchs
    {
      boost::graph_traits<Graph>::edge_iterator edgeIt, edgeEnd;
      for (boost::tie(edgeIt, edgeEnd) = boost::edges(base_graph); edgeIt != edgeEnd; ++edgeIt)
      {
        Edge ed = *edgeIt;
        uint64_t bitRate = boost::get(edge_weight_t(), base_graph, ed); // ver se preciso de fazer como est ano topology a separar pelos Nodes

        uint64_t weight = bitRate >= referenceBandwidthValue ? 1 : referenceBandwidthValue / bitRate;

        boost::put(edge_weight_t(), base_graph, ed, weight);
        cout << "SAVED -> Edge: " << ed.m_source << " - " << ed.m_target << " | Weight: " << weight << endl;

        Ptr<Node> n1 = Topology::VertexToNode(ed.m_source);
        Ptr<Node> n2 = Topology::VertexToNode(ed.m_target);
        Topology::UpdateEdgeWeight(n1, n2, weight);
      }
      cout << "-----------------------------" << endl;
    }
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
      float flex1 = EnergyAPI::GetFlexArray(Names::FindName(n1)).at(index);
      float flex2 = EnergyAPI::GetFlexArray(Names::FindName(n2)).at(index);

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
      // mudar isto para ajudar o controlador a saber que aresta mudou e não ter de computar os caminhos todos de novo !!!
    }
  }

  void
  OspfController::ApplyRouting(uint64_t swDpId)
  {
    uint32_t swId = DpId2Id(swDpId);
    Ptr<Node> sw = NodeContainer::GetGlobal().Get(swId);

    Ptr<OFSwitch13Device> ofDevice = sw->GetObject<OFSwitch13Device>();
    NodeContainer hosts = NodeContainer::GetGlobalHosts();

    for (NodeContainer::Iterator i = hosts.Begin(); i != hosts.End(); i++)
    {
      Ipv4Address remoteAddr = (*i)->GetObject<Ipv4>()->GetAddress(1, 0).GetLocal();
      // ideia rui usar o djikstra apenas no inicio e depois usar outro algoritmo para ir computando os melhores caminhos
      // Topology::DijkstraShortestPaths(sw, *i);
      std::vector<Ptr<Node>> path = GetShortesPath(sw, *i);

      uint32_t port = ofDevice->GetPortNoConnectedTo(path.at(1));

      std::ostringstream cmd;
      cmd << "flow-mod cmd=add,table=0 eth_type=0x800,ip_dst=" << remoteAddr
          << " apply:output=" << port;

      NS_LOG_DEBUG("[" << swDpId << "]: " << cmd.str());

      DpctlExecute(swDpId, cmd.str());
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

    NodeContainer switches = NodeContainer::GetGlobalSwitches();
    for (auto sw = switches.Begin(); sw != switches.End(); sw++)
      ApplyRouting(Id2DpId((*sw)->GetId()));

    std::cout << "Equal Cost Paths: " << std::endl;
    for (auto &ecp : equalCostPaths)
    {
      std::cout << "Switch: " << ecp.first.first->GetId() << " - Host: " << ecp.first.second->GetId() << std::endl;
      for (auto &path : ecp.second.GetPaths())
      {
        std::cout << "  ";
        for (auto &node : path.first)
        {
          std::cout << node->GetId() << " ";
        }
        std::cout << "  (" << path.second << ")" << std::endl;
      }
    }
    std::cout << "-----------------------------" << std::endl;

    Simulator::Schedule(Minutes(1), &OspfController::UpdateRouting, this);
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

      // pode ser melhorado este sistema de procura, aprender a usar o djikstra do boost
      NodeContainer hosts = NodeContainer::GetGlobalHosts();
      NodeContainer switches = NodeContainer::GetGlobalSwitches();
      for (auto sw = switches.Begin(); sw != switches.End(); sw++)
      {
        Ptr<Node> switchNode = NodeContainer::GetGlobal().Get((*sw)->GetId());
        for (auto hst = hosts.Begin(); hst != hosts.End(); hst++)
        {
          Ptr<Node> hostNode = NodeContainer::GetGlobal().Get((*hst)->GetId());
          FindAllPaths(switchNode, hostNode);
        }
      }
      UpdateRouting();
      m_isFirstUpdate = false;
    }
    ApplyRouting(swDpId);
  }
} // namespace ns3
