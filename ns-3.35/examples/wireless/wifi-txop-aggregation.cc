/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 Sébastien Deronne
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
 * Author: Sébastien Deronne <sebastien.deronne@gmail.com>
 */

#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/uinteger.h"
#include "ns3/boolean.h"
#include "ns3/string.h"
#include "ns3/pointer.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/ssid.h"
#include "ns3/mobility-helper.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/udp-client-server-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-mac.h"
#include "ns3/qos-txop.h"

// This is an example that illustrates how 802.11n aggregation is configured.
// It defines 4 independent Wi-Fi networks (working on different channels).
// Each network contains one access point and one station. Each station
// continuously transmits data packets to its respective AP.
//
// Network topology (numbers in parentheses are channel numbers):
//
//  Network A (36)   Network B (40)   Network C (44)   Network D (48)
//   *      *          *      *         *      *          *      *
//   |      |          |      |         |      |          |      |
//  AP A   STA A      AP B   STA B     AP C   STA C      AP D   STA D
//
// The aggregation parameters are configured differently on the 4 stations:
// - station A uses default aggregation parameter values (A-MSDU disabled, A-MPDU enabled with maximum size of 65 kB);
// - station B doesn't use aggregation (both A-MPDU and A-MSDU are disabled);
// - station C enables A-MSDU (with maximum size of 8 kB) but disables A-MPDU;
// - station D uses two-level aggregation (A-MPDU with maximum size of 32 kB and A-MSDU with maximum size of 4 kB).
//
// The user can select the distance between the stations and the APs, can enable/disable the RTS/CTS mechanism
// and can modify the duration of a TXOP.
// Example: ./waf --run "wifi-txop-aggregation --distance=10 --enableRts=0 --simulationTime=20"
//
// The output prints the throughput and the maximum TXOP duration measured for the 4 cases/networks
// described above. When default aggregation parameters are enabled, the
// maximum A-MPDU size is 65 kB and the throughput is maximal. When aggregation is disabled, the throughput is about the half of the physical
// bitrate. When only A-MSDU is enabled, the throughput is increased but is not maximal, since the maximum A-MSDU size is limited to 7935 bytes
// (whereas the maximum A-MPDU size is limited to 65535 bytes). When A-MSDU and A-MPDU are both enabled (= two-level aggregation),
// the throughput is slightly smaller than the first scenario since we set a smaller maximum A-MPDU size.
//
// When the distance is increased, the frame error rate gets higher, and the output shows how it affects the throughput for the 4 networks.
// Even through A-MSDU has less overheads than A-MPDU, A-MSDU is less robust against transmission errors than A-MPDU. When the distance is
// augmented, the throughput for the third scenario is more affected than the throughput obtained in other networks.

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("TxopMpduAggregation");

/**
 * Keeps the maximum duration among all TXOPs
 */
struct TxopDurationTracer
{
  void Trace (Time startTime, Time duration);
  Time m_max{Seconds (0)};
};

void
TxopDurationTracer::Trace (Time startTime, Time duration)
{
  if (duration > m_max)
    {
      m_max = duration;
    }
}

int
main (int argc, char *argv[])
{
  uint32_t payloadSize = 1472; //bytes
  double simulationTime = 10; //seconds
  double txopLimit = 3520; //microseconds
  double distance = 5; //meters
  bool enableRts = 0;
  bool enablePcap = 0;
  bool verifyResults = 0; //used for regression

  CommandLine cmd (__FILE__);
  cmd.AddValue ("payloadSize", "Payload size in bytes", payloadSize);
  cmd.AddValue ("enableRts", "Enable or disable RTS/CTS", enableRts);
  cmd.AddValue ("txopLimit", "TXOP duration in microseconds", txopLimit);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("distance", "Distance in meters between the station and the access point",
                distance);
  cmd.AddValue ("enablePcap", "Enable/disable pcap file generation", enablePcap);
  cmd.AddValue ("verifyResults", "Enable/disable results verification at the end of the simulation",
                verifyResults);
  cmd.Parse (argc, argv);

  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold",
                      enableRts ? StringValue ("0") : StringValue ("999999"));

  NodeContainer wifiStaNodes;
  wifiStaNodes.Create (4);
  NodeContainer wifiApNodes;
  wifiApNodes.Create (4);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy;
  phy.SetPcapDataLinkType (WifiPhyHelper::DLT_IEEE802_11_RADIO);
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211n_5GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", StringValue ("HtMcs7"),
                                "ControlMode", StringValue ("HtMcs0"));
  WifiMacHelper mac;

  NetDeviceContainer staDeviceA, staDeviceB, staDeviceC, staDeviceD, apDeviceA, apDeviceB,
      apDeviceC, apDeviceD;
  Ssid ssid;

  // Network A
  ssid = Ssid ("network-A");
  phy.Set ("ChannelNumber", UintegerValue (36));
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));
  staDeviceA = wifi.Install (phy, mac, wifiStaNodes.Get (0));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "EnableBeaconJitter",
               BooleanValue (false));
  apDeviceA = wifi.Install (phy, mac, wifiApNodes.Get (0));

  // Modify EDCA configuration (TXOP limit) for AC_BE
  Ptr<NetDevice> dev = wifiApNodes.Get (0)->GetDevice (0);
  Ptr<WifiNetDevice> wifi_dev = DynamicCast<WifiNetDevice> (dev);
  PointerValue ptr;
  Ptr<QosTxop> edca;
  wifi_dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  edca = ptr.Get<QosTxop> ();
  edca->SetTxopLimit (MicroSeconds (txopLimit));

  // Trace TXOP duration for BE on AP A
  TxopDurationTracer netA;
  edca->TraceConnectWithoutContext ("TxopTrace", MakeCallback (&TxopDurationTracer::Trace, &netA));

  // Network B
  ssid = Ssid ("network-B");
  phy.Set ("ChannelNumber", UintegerValue (40));
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));

  staDeviceB = wifi.Install (phy, mac, wifiStaNodes.Get (1));

  // Disable A-MPDU
  dev = wifiStaNodes.Get (1)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "EnableBeaconJitter",
               BooleanValue (false));
  apDeviceB = wifi.Install (phy, mac, wifiApNodes.Get (1));

  // Disable A-MPDU
  dev = wifiApNodes.Get (1)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));

  // Modify EDCA configuration (TXOP limit) for AC_BE
  wifi_dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  edca = ptr.Get<QosTxop> ();
  edca->SetTxopLimit (MicroSeconds (txopLimit));

  // Trace TXOP duration for BE on AP B
  TxopDurationTracer netB;
  edca->TraceConnectWithoutContext ("TxopTrace", MakeCallback (&TxopDurationTracer::Trace, &netB));

  // Network C
  ssid = Ssid ("network-C");
  phy.Set ("ChannelNumber", UintegerValue (44));
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));

  staDeviceC = wifi.Install (phy, mac, wifiStaNodes.Get (2));

  // Disable A-MPDU and enable A-MSDU with the highest maximum size allowed by the standard (7935 bytes)
  dev = wifiStaNodes.Get (2)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (7935));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "EnableBeaconJitter",
               BooleanValue (false));
  apDeviceC = wifi.Install (phy, mac, wifiApNodes.Get (2));

  // Disable A-MPDU and enable A-MSDU with the highest maximum size allowed by the standard (7935 bytes)
  dev = wifiApNodes.Get (2)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (0));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (7935));

  // Modify EDCA configuration (TXOP limit) for AC_BE
  wifi_dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  edca = ptr.Get<QosTxop> ();
  edca->SetTxopLimit (MicroSeconds (txopLimit));

  // Trace TXOP duration for BE on AP C
  TxopDurationTracer netC;
  edca->TraceConnectWithoutContext ("TxopTrace", MakeCallback (&TxopDurationTracer::Trace, &netC));

  // Network D
  ssid = Ssid ("network-D");
  phy.Set ("ChannelNumber", UintegerValue (48));
  mac.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid));

  staDeviceD = wifi.Install (phy, mac, wifiStaNodes.Get (3));

  // Enable A-MPDU with a smaller size than the default one and
  // enable A-MSDU with the smallest maximum size allowed by the standard (3839 bytes)
  dev = wifiStaNodes.Get (3)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (32768));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (3839));

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid), "EnableBeaconJitter",
               BooleanValue (false));
  apDeviceD = wifi.Install (phy, mac, wifiApNodes.Get (3));

  // Enable A-MPDU with a smaller size than the default one and
  // enable A-MSDU with the smallest maximum size allowed by the standard (3839 bytes)
  dev = wifiApNodes.Get (3)->GetDevice (0);
  wifi_dev = DynamicCast<WifiNetDevice> (dev);
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmpduSize", UintegerValue (32768));
  wifi_dev->GetMac ()->SetAttribute ("BE_MaxAmsduSize", UintegerValue (3839));

  // Modify EDCA configuration (TXOP limit) for AC_BE
  wifi_dev->GetMac ()->GetAttribute ("BE_Txop", ptr);
  edca = ptr.Get<QosTxop> ();
  edca->SetTxopLimit (MicroSeconds (txopLimit));

  // Trace TXOP duration for BE on AP D
  TxopDurationTracer netD;
  edca->TraceConnectWithoutContext ("TxopTrace", MakeCallback (&TxopDurationTracer::Trace, &netD));

  // Setting mobility model
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  // Set position for APs
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  positionAlloc->Add (Vector (20.0, 0.0, 0.0));
  positionAlloc->Add (Vector (30.0, 0.0, 0.0));
  // Set position for STAs
  positionAlloc->Add (Vector (distance, 0.0, 0.0));
  positionAlloc->Add (Vector (10 + distance, 0.0, 0.0));
  positionAlloc->Add (Vector (20 + distance, 0.0, 0.0));
  positionAlloc->Add (Vector (30 + distance, 0.0, 0.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.Install (wifiApNodes);
  mobility.Install (wifiStaNodes);

  // Internet stack
  InternetStackHelper stack;
  stack.Install (wifiApNodes);
  stack.Install (wifiStaNodes);

  Ipv4AddressHelper address;
  address.SetBase ("192.168.1.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterfaceA;
  StaInterfaceA = address.Assign (staDeviceA);
  Ipv4InterfaceContainer ApInterfaceA;
  ApInterfaceA = address.Assign (apDeviceA);

  address.SetBase ("192.168.2.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterfaceB;
  StaInterfaceB = address.Assign (staDeviceB);
  Ipv4InterfaceContainer ApInterfaceB;
  ApInterfaceB = address.Assign (apDeviceB);

  address.SetBase ("192.168.3.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterfaceC;
  StaInterfaceC = address.Assign (staDeviceC);
  Ipv4InterfaceContainer ApInterfaceC;
  ApInterfaceC = address.Assign (apDeviceC);

  address.SetBase ("192.168.4.0", "255.255.255.0");
  Ipv4InterfaceContainer StaInterfaceD;
  StaInterfaceD = address.Assign (staDeviceD);
  Ipv4InterfaceContainer ApInterfaceD;
  ApInterfaceD = address.Assign (apDeviceD);

  // Setting applications
  uint16_t port = 9;
  UdpServerHelper serverA (port);
  ApplicationContainer serverAppA = serverA.Install (wifiStaNodes.Get (0));
  serverAppA.Start (Seconds (0.0));
  serverAppA.Stop (Seconds (simulationTime + 1));

  UdpClientHelper clientA (StaInterfaceA.GetAddress (0), port);
  clientA.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
  clientA.SetAttribute ("Interval", TimeValue (Time ("0.0001"))); //packets/s
  clientA.SetAttribute ("PacketSize", UintegerValue (payloadSize));

  ApplicationContainer clientAppA = clientA.Install (wifiApNodes.Get (0));
  clientAppA.Start (Seconds (1.0));
  clientAppA.Stop (Seconds (simulationTime + 1));

  UdpServerHelper serverB (port);
  ApplicationContainer serverAppB = serverB.Install (wifiStaNodes.Get (1));
  serverAppB.Start (Seconds (0.0));
  serverAppB.Stop (Seconds (simulationTime + 1));

  UdpClientHelper clientB (StaInterfaceB.GetAddress (0), port);
  clientB.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
  clientB.SetAttribute ("Interval", TimeValue (Time ("0.0001"))); //packets/s
  clientB.SetAttribute ("PacketSize", UintegerValue (payloadSize));

  ApplicationContainer clientAppB = clientB.Install (wifiApNodes.Get (1));
  clientAppB.Start (Seconds (1.0));
  clientAppB.Stop (Seconds (simulationTime + 1));

  UdpServerHelper serverC (port);
  ApplicationContainer serverAppC = serverC.Install (wifiStaNodes.Get (2));
  serverAppC.Start (Seconds (0.0));
  serverAppC.Stop (Seconds (simulationTime + 1));

  UdpClientHelper clientC (StaInterfaceC.GetAddress (0), port);
  clientC.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
  clientC.SetAttribute ("Interval", TimeValue (Time ("0.0001"))); //packets/s
  clientC.SetAttribute ("PacketSize", UintegerValue (payloadSize));

  ApplicationContainer clientAppC = clientC.Install (wifiApNodes.Get (2));
  clientAppC.Start (Seconds (1.0));
  clientAppC.Stop (Seconds (simulationTime + 1));

  UdpServerHelper serverD (port);
  ApplicationContainer serverAppD = serverD.Install (wifiStaNodes.Get (3));
  serverAppD.Start (Seconds (0.0));
  serverAppD.Stop (Seconds (simulationTime + 1));

  UdpClientHelper clientD (StaInterfaceD.GetAddress (0), port);
  clientD.SetAttribute ("MaxPackets", UintegerValue (4294967295u));
  clientD.SetAttribute ("Interval", TimeValue (Time ("0.0001"))); //packets/s
  clientD.SetAttribute ("PacketSize", UintegerValue (payloadSize));

  ApplicationContainer clientAppD = clientD.Install (wifiApNodes.Get (3));
  clientAppD.Start (Seconds (1.0));
  clientAppD.Stop (Seconds (simulationTime + 1));

  if (enablePcap)
    {
      phy.EnablePcap ("AP_A", apDeviceA.Get (0));
      phy.EnablePcap ("STA_A", staDeviceA.Get (0));
      phy.EnablePcap ("AP_B", apDeviceB.Get (0));
      phy.EnablePcap ("STA_B", staDeviceB.Get (0));
      phy.EnablePcap ("AP_C", apDeviceC.Get (0));
      phy.EnablePcap ("STA_C", staDeviceC.Get (0));
      phy.EnablePcap ("AP_D", apDeviceD.Get (0));
      phy.EnablePcap ("STA_D", staDeviceD.Get (0));
    }

  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();

  // Show results
  uint64_t totalPacketsThroughA = DynamicCast<UdpServer> (serverAppA.Get (0))->GetReceived ();
  uint64_t totalPacketsThroughB = DynamicCast<UdpServer> (serverAppB.Get (0))->GetReceived ();
  uint64_t totalPacketsThroughC = DynamicCast<UdpServer> (serverAppC.Get (0))->GetReceived ();
  uint64_t totalPacketsThroughD = DynamicCast<UdpServer> (serverAppD.Get (0))->GetReceived ();

  Simulator::Destroy ();

  double throughput = totalPacketsThroughA * payloadSize * 8 / (simulationTime * 1000000.0);
  std::cout << "Default configuration (A-MPDU aggregation enabled, 65kB): " << '\n'
            << "  Throughput = " << throughput << " Mbit/s" << '\n';
  if (verifyResults && (throughput < 57.5 || throughput > 58.5))
    {
      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not in the expected boundaries!");
      exit (1);
    }
  if (txopLimit)
    {
      std::cout << "  Maximum TXOP duration (TXOP limit = " << txopLimit
                << "us): " << netA.m_max.GetMicroSeconds () << " us" << '\n';
      if (verifyResults && txopLimit &&
          (netA.m_max < MicroSeconds (3350) || netA.m_max > MicroSeconds (3520)))
        {
          NS_LOG_ERROR ("Maximum TXOP duration " << netA.m_max
                                                 << " is not in the expected boundaries!");
          exit (1);
        }
    }

  throughput = totalPacketsThroughB * payloadSize * 8 / (simulationTime * 1000000.0);
  std::cout << "Aggregation disabled: " << '\n'
            << "  Throughput = " << throughput << " Mbit/s" << '\n';
  if (verifyResults && (throughput < 38 || throughput > 39))
    {
      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not in the expected boundaries!");
      exit (1);
    }
  if (txopLimit)
    {
      std::cout << "  Maximum TXOP duration (TXOP limit = " << txopLimit
                << "us): " << netB.m_max.GetMicroSeconds () << " us" << '\n';
      if (verifyResults && (netB.m_max < MicroSeconds (3350) || netB.m_max > MicroSeconds (3520)))
        {
          NS_LOG_ERROR ("Maximum TXOP duration " << netB.m_max
                                                 << " is not in the expected boundaries!");
          exit (1);
        }
    }

  throughput = totalPacketsThroughC * payloadSize * 8 / (simulationTime * 1000000.0);
  std::cout << "A-MPDU disabled and A-MSDU enabled (8kB): " << '\n'
            << "  Throughput = " << throughput << " Mbit/s" << '\n';
  if (verifyResults && (throughput < 52 || throughput > 53))
    {
      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not in the expected boundaries!");
      exit (1);
    }
  if (txopLimit)
    {
      std::cout << "  Maximum TXOP duration (TXOP limit = " << txopLimit
                << "us): " << netC.m_max.GetMicroSeconds () << " us" << '\n';
      if (verifyResults && (netC.m_max < MicroSeconds (3350) || netC.m_max > MicroSeconds (3520)))
        {
          NS_LOG_ERROR ("Maximum TXOP duration " << netC.m_max
                                                 << " is not in the expected boundaries!");
          exit (1);
        }
    }

  throughput = totalPacketsThroughD * payloadSize * 8 / (simulationTime * 1000000.0);
  std::cout << "A-MPDU enabled (32kB) and A-MSDU enabled (4kB): " << '\n'
            << "  Throughput = " << throughput << " Mbit/s" << '\n';
  if (verifyResults && (throughput < 58 || throughput > 59))
    {
      NS_LOG_ERROR ("Obtained throughput " << throughput << " is not in the expected boundaries!");
      exit (1);
    }
  if (txopLimit)
    {
      std::cout << "  Maximum TXOP duration (TXOP limit = " << txopLimit
                << "us): " << netD.m_max.GetMicroSeconds () << " us" << '\n';
      if (verifyResults && txopLimit &&
          (netD.m_max < MicroSeconds (3350) || netD.m_max > MicroSeconds (3520)))
        {
          NS_LOG_ERROR ("Maximum TXOP duration " << netD.m_max
                                                 << " is not in the expected boundaries!");
          exit (1);
        }
    }

  return 0;
}
