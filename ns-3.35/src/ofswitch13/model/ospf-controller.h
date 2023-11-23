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

#ifndef OSPF_CONTROLLER_H
#define OSPF_CONTROLLER_H

#include "ofswitch13-controller.h"
#include "ns3/topology.h"

namespace ns3
{
  class OspfController : public OFSwitch13Controller
  {
  public:
    OspfController();
    virtual ~OspfController();
    static TypeId GetTypeId(void);
    virtual void DoDispose();

  protected:
    void HandshakeSuccessful(Ptr<const RemoteSwitch> sw);
    void ApplyRouting(uint64_t swDpId);
    void ApplyRouting(uint64_t swDpId, Ptr<Node> host, Ptr<Node> nextJump);
    void ApplyRoutingFromPath(std::vector<Ptr<Node>> path);
    void FindReferenceBandwidth();

  private:
    void UpdateRouting();
    void UpdateWeights();
    void SetWeightsBandwidthBased();
    void AddSwitchHostKey(Ptr<Node> switchNode, Ptr<Node> hostNode);
    void StorePath(Ptr<Node> switchNode, Ptr<Node> hostNode, std::vector<Ptr<Node>> path);
    void CleanPaths(Ptr<Node> switchNode, Ptr<Node> hostNode);
    std::vector<std::vector<Ptr<Node>>> Search(Ptr<Node> init, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore);
    std::vector<std::vector<Ptr<Node>>> SearchWithDepth(Ptr<Node> init, Ptr<Node> destiny, std::vector<Ptr<Node>> &ignore, int maxDepth, int currentDepth);
    void FindAllPaths(Ptr<Node> source, Ptr<Node> destination);
    std::vector<Ptr<Node>> GetShortesPath(Ptr<Node> source, Ptr<Node> destination);
    int FindMaxDepth(Ptr<Node> source, Ptr<Node> destiny);
    void ResizeStoredPaths(int maxStorageNumber);
    void StartRoutingLoop();

    bool m_isFirstUpdate;
    Graph base_graph;
    static uint64_t referenceBandwidthValue;
  };

} // namespace ns3

#endif /* OSPF_CONTROLLER_H */
