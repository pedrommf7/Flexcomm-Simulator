/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2013 CNRS, France.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Anne-Cecile Orgerie <anne-cecile.orgerie@irisa.fr>
 */

#include "node-energy-model.h"
#include "ns3/uinteger.h"
#include "ns3/log.h"
#include "ns3/simulator.h"
#include "ns3/node-container.h"
#include "ns3/loopback-net-device.h"

namespace ns3 {

NS_OBJECT_ENSURE_REGISTERED (NodeEnergyModel);

TypeId
NodeEnergyModel::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::NodeEnergyModel")
          .SetParent<Object> ()
          .AddConstructor<NodeEnergyModel> ()
          .AddAttribute (
              "NodeState", "The default state of the node (0=off, 1=on)", UintegerValue (1),
              MakeUintegerAccessor (&NodeEnergyModel::SetNodeState, &NodeEnergyModel::GetNodeState),
              MakeUintegerChecker<uint32_t> ());
  return tid;
}

NodeEnergyModel::NodeEnergyModel () : m_nodeState (1)
{
}

NodeEnergyModel::~NodeEnergyModel ()
{
}

uint32_t
NodeEnergyModel::GetNodeState (void) const
{
  return m_nodeState;
}

void
NodeEnergyModel::SetNodeState (uint32_t nodeState)
{
  //NS_LOG_FUNCTION (this << nodeState);
  m_nodeState = nodeState;
}

uint32_t
NodeEnergyModel::GetNodeOnState (void) const
{
  return m_nodeOnState;
}

void
NodeEnergyModel::SetNodeOnState (uint32_t state)
{
  m_nodeOnState = state;
}

uint32_t
NodeEnergyModel::GetNodeOffState (void) const
{
  return m_nodeOffState;
}

void
NodeEnergyModel::SetNodeOffState (uint32_t state)
{
  m_nodeOffState = state;
}

void
NodeEnergyModel::SetNode (Ptr<Node> node)
{
  NS_ASSERT (node != NULL);
  m_node = node;
}

Ptr<Node>
NodeEnergyModel::GetNode (void) const
{
  return m_node;
}

double
NodeEnergyModel::GetPowerConsumption (void)
{
  return 0.0;
}

//TODO à modifier pour obtenir un fichier de traces avec les consos
void
NodeEnergyModel::GetTotalPowerConsumption (Ptr<Node> node)
{
  double conso = 0;
  conso = GetPowerConsumption (); //chassis consumption
  if (conso < 0)
    {
      NS_LOG_UNCOND ("AAA! Negative chassis conso of node " << node->GetId ());
    }
  uint32_t n = node->GetNDevices ();
  uint32_t i;
  for (i = 0; i < n; i++)
    {
      Ptr<NetDevice> dev = node->GetDevice (i);
      Ptr<NetdeviceEnergyModel> edev = dev->GetObject<NetdeviceEnergyModel> ();
      //NS_LOG_UNCOND ("NetDev " << i);
      if ((edev != NULL) && (dev->GetInstanceTypeId ().GetName () != "ns3::LoopbackNetDevice"))
        {
          double temp = edev->GetPowerConsumption (); // consumption of the net devices
          conso += temp;
          if (temp < 0)
            {
              NS_LOG_UNCOND ("AAA! Negative port conso on node " << node->GetId () << " netdev "
                                                                 << i);
            }
        }
    }
  NodeContainer nodes = NodeContainer::GetGlobal ();
  uint32_t nodenb = node->GetId ();
  NS_LOG_UNCOND ("Time " << Simulator::Now ().GetSeconds () << " Node " << nodenb << " Conso "
                         << conso);
  //return conso;
}

void
NodeEnergyModel::GetConso (Time interval, Time stop, Ptr<Node> node)
{
  Time i = Seconds (0.0);
  while (i <= stop)
    {
      Simulator::Schedule (i, &NodeEnergyModel::GetTotalPowerConsumption, this, node);
      i += interval;
    }
}

void
NodeEnergyModel::UpdateState (uint32_t state, double energy, Time duration)
{
}

} // namespace ns3
