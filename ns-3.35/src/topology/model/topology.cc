/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * The GPLv2 License (GPLv2)
 *
 * Copyright (c) 2023 Rui Pedro C. Monteiro
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
 * Author: Rui Pedro C. Monteiro <rui.p.monteiro@inesctec.pt>
 */

#include "topology.h"
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>

namespace ns3
{

  NS_OBJECT_ENSURE_REGISTERED(Topology);

  Graph Topology::m_graph = Graph();

  std::map<Ptr<Node>, Vertex> Topology::m_vertexes = std::map<Ptr<Node>, Vertex>();
  std::map<Vertex, Ptr<Node>> Topology::m_nodes = std::map<Vertex, Ptr<Node>>();
  std::map<Ipv4Address, Vertex> Topology::m_ip_to_vertex = std::map<Ipv4Address, Vertex>();
  std::map<Vertex, Ipv4Address> Topology::m_vertex_to_ip = std::map<Vertex, Ipv4Address>();
  std::map<Edge, Ptr<Channel>> Topology::m_channels = std::map<Edge, Ptr<Channel>>();

  TypeId
  Topology::GetTypeId(void)
  {
    static TypeId tid = TypeId("ns3::Topology").SetParent<Object>().AddConstructor<Topology>();
    return tid;
  }

  Topology::Topology()
  {
  }

  Topology::~Topology()
  {
  }

  Vertex
  Topology::AddNode(Ptr<Node> node)
  {
    Vertex vd = add_vertex(node, m_graph);
    m_vertexes[node] = vd;
    m_nodes[vd] = node;
    return vd;
  }

  void
  Topology::AddSwitch(Ptr<Node> sw)
  {
    AddNode(sw);
  }

  void
  Topology::AddHost(Ptr<Node> host, Ipv4Address ip)
  {
    Vertex vd = AddNode(host);
    m_ip_to_vertex[ip] = vd;
    m_vertex_to_ip[vd] = ip;
  }

  void
  Topology::AddLink(Ptr<Node> n1, Ptr<Node> n2, Ptr<Channel> channel)
  {
    Vertex vd1 = m_vertexes[n1];
    Vertex vd2 = m_vertexes[n2];
    Edge e = add_edge(vd1, vd2, 1, m_graph).first;
    m_channels[e] = channel;
  }

  std::vector<Vertex>
  Topology::DijkstraShortestPathsInternal(Vertex src)
  {
    std::vector<Vertex> predecessors(num_vertices(m_graph));
    dijkstra_shortest_paths(m_graph, src,
                            predecessor_map(boost::make_iterator_property_map(
                                predecessors.begin(), get(boost::vertex_index, m_graph)))

    );
    return predecessors;
  }

  std::vector<Vertex>
  Topology::DijkstraShortestPathInternal(Vertex src, Vertex dst)
  {
    std::vector<Vertex> predecessors = DijkstraShortestPathsInternal(src);
    std::vector<Vertex> path;
    Vertex currentVertex = dst;

    while (currentVertex != src)
    {
      path.push_back(currentVertex);
      currentVertex = predecessors[currentVertex];
    }
    path.push_back(src);
    std::reverse(path.begin(), path.end());

    return path;
  }

  std::vector<Ptr<Node>>
  Topology::VertexToNode(std::vector<Vertex> path)
  {
    std::vector<Ptr<Node>> path_nodes = std::vector<Ptr<Node>>();
    for (auto i = path.begin(); i != path.end(); i++)
      path_nodes.push_back(m_nodes[*i]);
    return path_nodes;
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPath(Ptr<Node> src, Ptr<Node> dst)
  {
    return VertexToNode(DijkstraShortestPathInternal(m_vertexes[src], m_vertexes[dst]));
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPath(Ptr<Node> src, Ipv4Address dst)
  {
    return DijkstraShortestPath(src, m_nodes[m_ip_to_vertex[dst]]);
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPath(Ipv4Address src, Ptr<Node> dst)
  {
    return DijkstraShortestPath(m_nodes[m_ip_to_vertex[src]], dst);
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPath(Ipv4Address src, Ipv4Address dst)
  {
    return DijkstraShortestPath(m_nodes[m_ip_to_vertex[src]], m_nodes[m_ip_to_vertex[dst]]);
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPath(std::string src, std::string dst)
  {
    Ptr<Node> srcNode = Names::Find<Node>(src);
    Ptr<Node> dstNode = Names::Find<Node>(dst);
    return DijkstraShortestPath(srcNode, dstNode);
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPaths(Ptr<Node> src)
  {
    return VertexToNode(DijkstraShortestPathsInternal(m_vertexes[src]));
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPaths(Ipv4Address src)
  {
    return DijkstraShortestPaths(m_nodes[m_ip_to_vertex[src]]);
  }

  std::vector<Ptr<Node>>
  Topology::DijkstraShortestPaths(std::string src)
  {
    Ptr<Node> srcNode = Names::Find<Node>(src);
    return DijkstraShortestPaths(srcNode);
  }

  // NOTE: Code partially generated by ChatGPT
  std::vector<std::pair<std::vector<Ptr<Node>>, int>>
  Topology::DijkstraShortestPaths(Ptr<Node> src, Ptr<Node> dst)
  {
    std::vector<Vertex> predecessors(num_vertices(m_graph));
    std::vector<int> distances(num_vertices(m_graph));
    Vertex source = NodeToVertex(src);
    Vertex target = NodeToVertex(dst);

    dijkstra_shortest_paths(m_graph, source,
                            predecessor_map(make_iterator_property_map(
                                                predecessors.begin(), get(vertex_index, m_graph)))
                                .distance_map(make_iterator_property_map(
                                    distances.begin(), get(vertex_index, m_graph))));
    // print predecessor map
    std::cout << "predecessor map:" << std::endl;
    boost::graph_traits<Graph>::vertex_iterator vertexIt, vertexEnd;
    for (boost::tie(vertexIt, vertexEnd) = boost::vertices(m_graph); vertexIt != vertexEnd; ++vertexIt)
    {
      std::cout << "predecessor[" << *vertexIt << "] = " << predecessors[*vertexIt] << std::endl;
    }
    std::cout << std::endl;
    // print distance map
    std::cout << "distance map:" << std::endl;
    for (boost::tie(vertexIt, vertexEnd) = boost::vertices(m_graph); vertexIt != vertexEnd; ++vertexIt)
    {
      std::cout << "distance[" << *vertexIt << "] = " << distances[*vertexIt] << std::endl;
    }
    std::cout << std::endl;

    // Create a vector to store the paths and their distances
    std::vector<std::pair<std::vector<Ptr<Node>>, int>> paths;

    // Loop through all vertices and store the path and distance from the source
    for (Vertex v = 0; v < num_vertices(m_graph); ++v)
    {
      if (v == source || v == target)
        continue;

      std::vector<Ptr<Node>> path;
      for (Vertex u = v; u != source; u = predecessors[u])
        path.push_back(VertexToNode(u));

      path.push_back(VertexToNode(source));
      std::reverse(path.begin(), path.end());
      path.push_back(VertexToNode(target));

      paths.emplace_back(std::move(path), distances[v]);
    }

    // Sort the paths by distance in ascending order
    std::sort(paths.begin(), paths.end(),
              [](const auto &lhs, const auto &rhs)
              { return lhs.second < rhs.second; });

    // Now, the 'paths' vector contains all paths between 'source' and 'target'
    // sorted by their distances in ascending order.
    // Each entry in the 'paths' vector is a pair (path, distance).
    std::cout << "Paths from " << src->GetId() << " to " << dst->GetId() << ":" << std::endl;
    for (const auto &path : paths)
    {
      std::cout << "  ";
      for (const auto &node : path.first)
        std::cout << node->GetId() << " ";
      std::cout << "  (" << path.second << ")" << std::endl;
    }
    return paths;
  }

  PathStore
  Topology::Dijkstra_k_ShortestPaths(Ptr<Node> src, Ptr<Node> dst, int k_paths)
  {
    
    PathStore shortestPaths = PathStore();
    PathStore paths = PathStore();
    if (k_paths <= 0)
    {
      std::cerr << "k must be greater than 0" << std::endl;
      return shortestPaths;
    }

    // Initialize with the source vertex
    std::vector<Ptr<Node>> path {src};
    paths.AddPath(path, 0);

    while (paths.GetNumberOfStoredPaths()>0 && shortestPaths.GetNumberOfStoredPaths()<k_paths)
    {
      std::pair<std::vector<Ptr<Node>>, int> pathNweight = paths.GetShortestPathAndDistance();
      std::vector<Ptr<Node>> currentPath = pathNweight.first;
      int currentWeight = pathNweight.second;

      Vertex currentVertex = NodeToVertex(currentPath.back());

      // Explore neighbors
      boost::graph_traits<Graph>::out_edge_iterator ei, ei_end;
      for (boost::tie(ei, ei_end) = out_edges(currentVertex, m_graph); ei != ei_end; ++ei)
      {
        int neighbor = target(*ei, m_graph);
        int weight = get(boost::edge_weight, m_graph, *ei);

        // Check if the neighbor is already in the path to avoid cycles
        if (std::find(currentPath.begin(), currentPath.end(), neighbor) == currentPath.end())
        {
          std::vector<Ptr<Node>> newPath = currentPath;
          newPath.push_back(VertexToNode(neighbor));
          paths.AddPath(newPath, currentWeight + weight);
        }
      }

      if (currentVertex != NodeToVertex(src))
      {
        int weight = paths.GetDistance(currentPath);
        if (weight < 0)
          std::cerr << "Error: path not found" << std::endl;
        else
          shortestPaths.AddPath(currentPath, weight);
      }
    }

    // Print the results
    for (const auto &path : shortestPaths.GetPaths())
    {
      std::vector<Ptr<Node>> p = path.first;
      std::cout << "Path: ";
      for (const auto &node : p)
        std::cout << node->GetId() << " ";
      std::cout << "Total Weight: " << path.second << std::endl;
    }

    return shortestPaths;
  }

  Graph
  Topology::GetGraph()
  {
    return m_graph;
  }

  Ptr<Node>
  Topology::VertexToNode(Vertex vd)
  {
    return m_nodes[vd];
  }

  Vertex
  Topology::NodeToVertex(Ptr<Node> node)
  {
    return m_vertexes[node];
  }

  void
  Topology::UpdateEdgeWeight(Ptr<Node> n1, Ptr<Node> n2, int newWeight)
  {
    std::pair<Edge, bool> pair = boost::edge(m_vertexes[n1], m_vertexes[n2], m_graph);

    if (pair.second)
      Topology::UpdateEdgeWeight(pair.first, newWeight);
  }

  void
  Topology::UpdateEdgeWeight(Edge ed, int newWeight)
  {
    // Prevent negative weights
    if (newWeight < 0)
      newWeight = 0;
    put(edge_weight_t(), m_graph, ed, newWeight);
  }

  int Topology::GetEdgeWeight(Ptr<Node> n1, Ptr<Node> n2)
  {
    std::pair<Edge, bool> pair = boost::edge(m_vertexes[n1], m_vertexes[n2], m_graph);

    if (pair.second)
      return Topology::GetEdgeWeight(pair.first);
    return -1;
  }

  int Topology::GetEdgeWeight(Edge e)
  {
    return get(edge_weight_t(), m_graph, e);
  }

  Ptr<Channel>
  Topology::GetChannel(Ptr<Node> n1, Ptr<Node> n2)
  {
    std::pair<Edge, bool> pair = boost::edge(m_vertexes[n1], m_vertexes[n2], m_graph);

    if (pair.second)
      return Topology::GetChannel(pair.first);
    return NULL;
  }

  Ptr<Channel>
  Topology::GetChannel(Edge e)
  {
    return m_channels[e];
  }

  std::vector<Ptr<Node>>
  Topology::GetSuccessors(Ptr<Node> node)
  {
    std::vector<Ptr<Node>> successors;
    Vertex vd = m_vertexes[node];
    boost::graph_traits<Graph>::adjacency_iterator ai, a_end;
    for (boost::tie(ai, a_end) = boost::adjacent_vertices(vd, m_graph); ai != a_end; ++ai)
      successors.push_back(m_nodes[*ai]);
    return successors;
  }

} // namespace ns3
