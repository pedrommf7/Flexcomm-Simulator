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

#ifndef MY_CONTROLLER_H
#define MY_CONTROLLER_H

#include "ofswitch13-controller.h"
#include "ns3/topology.h"

namespace ns3 {

class MyController : public OFSwitch13Controller
{
public:
  MyController ();
  virtual ~MyController ();
  static TypeId GetTypeId (void);
  virtual void DoDispose ();
  void SaveDataRateInfos();

protected:
  void HandshakeSuccessful (Ptr<const RemoteSwitch> sw);
  void ApplyRouting (uint64_t swDpId);
  void SaveInfo(Edge ed, uint64_t weight);

private:
  void UpdateRouting ();
  void UpdateWeights ();
  void SetWeightsBandwidthBased();

  bool m_isFirstUpdate;
  std::map<Edge, uint64_t> edg_to_weight;
  static uint64_t referenceBandwidthValue;

};

} // namespace ns3

#endif /* MY_CONTROLLER_H */
