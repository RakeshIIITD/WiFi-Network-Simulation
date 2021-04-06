/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2015, IMDEA Networks Institute
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
 * Author: Hany Assasa <hany.assasa@gmail.com>
.*
 * This is a simple example to test TCP over 802.11g (with MPDU aggregation enabled).
 *
 * Network topology:
 *
 *   n12      Ap    STA
 *   |        *      *
 *   |        |      |
 *   n0------n1     n2
 *
 * In this example, an HT station sends TCP packets to the access point. 
 * We report the total throughput received during a window of 100ms. 
 * The user can specify the application data rate and choose the variant
 * of TCP i.e. congestion control algorithm to use.
 */

#include <string>
#include "ns3/applications-module.h"
#include "ns3/core-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/network-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/wifi-module.h"

NS_LOG_COMPONENT_DEFINE ("wifi-tcp");

using namespace ns3;

Ptr<PacketSink> sink;                         /* Pointer to the packet sink application */
uint64_t lastTotalRx = 0;                     /* The value of the last total received bytes */
double simulationTime = 100;
int nwifi=12;
uint16_t port=8080;
uint32_t maxBytes = 10485760;
double endtime=0;
ApplicationContainer serverApp,sinkApp;
int flags[100]={0};

void CalculateThroughput ()
{
/*
  Time now = Simulator::Now ();                                         // Return the simulator's virtual time. 
  double cur = (sink->GetTotalRx() - lastTotalRx) * (double) 8/1e5;     // Convert Application RX Packets to MBits.
  std::cout << now.GetSeconds () << "s: \t" << cur << " Mbit/s" << std::endl;
  lastTotalRx = sink->GetTotalRx ();
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
*/
  
  Time now = Simulator::Now ();
  for(unsigned i = 0; i< sinkApp.GetN();i++){

  sink = StaticCast<PacketSink> (sinkApp.Get(i));                                        
  if( sink->GetTotalRx() >= maxBytes && flags[i]==0 ) {
        flags[i]=1 ;
        //endtime = now.GetSeconds ();
        std::cout << " Client "<<i+1<<"Completed in "<<(now.GetSeconds ()-1.1) << "s: "<<sink->GetTotalRx() <<std::endl;
        std::cout << " Throughput of client "<< i+1 <<" is : "<< ( (maxBytes/((now.GetSeconds ()-1.1))) * (double) 8/1e6) <<" Mbps"<<std::endl;
  }
 }
  Simulator::Schedule (MilliSeconds (100), &CalculateThroughput);
}


int
main(int argc, char *argv[])
{
  uint32_t payloadSize = 1472;                       /* Transport layer payload size in bytes. */
  std::string dataRate = "54Mbps";                  /* Application layer datarate. */
  std::string tcpVariant = "ns3::TcpNewReno";        /* TCP variant type. */
  //std::string phyRate = "HtMcs7";                    /* Physical layer bitrate. */
                                                        /* Simulation time in seconds. */
  bool pcapTracing = true;                          /* PCAP Tracing is enabled or not. */
/*
  CommandLine cmd;
  cmd.AddValue ("payloadSize", "Payload size in bytes", payloadSize);
  cmd.AddValue ("dataRate", "Application data rate", dataRate);
  cmd.AddValue ("tcpVariant", "Transport protocol to use: TcpTahoe, TcpReno, TcpNewReno, TcpWestwood, TcpWestwoodPlus ", tcpVariant);
  cmd.AddValue ("phyRate", "Physical layer bitrate", phyRate);
  cmd.AddValue ("simulationTime", "Simulation time in seconds", simulationTime);
  cmd.AddValue ("pcap", "Enable/disable PCAP Tracing", pcapTracing);
  cmd.Parse (argc, argv);
*/


  /* No fragmentation and no RTS/CTS */
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));

  /* Configure TCP Options */
  Config::SetDefault ("ns3::TcpSocket::SegmentSize", UintegerValue (payloadSize));


  /*==================================ADDED===============================*/
  NodeContainer p2pNodes;
  p2pNodes.Create (2);

  PointToPointHelper pointToPoint;
  pointToPoint.SetDeviceAttribute ("DataRate", StringValue ("100Mbps"));
  pointToPoint.SetChannelAttribute ("Delay", StringValue ("0.0000001ms"));

  NetDeviceContainer p2pDevices = pointToPoint.Install (p2pNodes);

  /*================================#ADDED#===============================*/
/*
  ansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  phy.SetChannel (channel.Create ());
*/

  WifiMacHelper wifiMac;
  WifiHelper wifiHelper;
  wifiHelper.SetStandard (WIFI_PHY_STANDARD_80211g);

  // Set up Legacy Channel 
  YansWifiChannelHelper wifiChannel ;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::FriisPropagationLossModel", "Frequency", DoubleValue (5e9));

  // Setup Physical Layer
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  wifiPhy.SetChannel (wifiChannel.Create ());
  wifiPhy.Set ("TxPowerStart", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerEnd", DoubleValue (10.0));
  wifiPhy.Set ("TxPowerLevels", UintegerValue (1));
  wifiPhy.Set ("TxGain", DoubleValue (0));
  wifiPhy.Set ("RxGain", DoubleValue (0));
  wifiPhy.Set ("RxNoiseFigure", DoubleValue (10));
  wifiPhy.Set ("CcaMode1Threshold", DoubleValue (-79));
  wifiPhy.Set ("EnergyDetectionThreshold", DoubleValue (-79 + 3));
  wifiPhy.SetErrorRateModel ("ns3::YansErrorRateModel");
  //wifiHelper.SetRemoteStationManager ("ns3::ConstantRateWifiManager","DataMode", StringValue (phyRate),"ControlMode", StringValue ("HtMcs0"));


  NodeContainer staWifiNode;
  staWifiNode.Create(nwifi);
  NodeContainer apWifiNode = p2pNodes.Get(1);              // EDITED
/*
  Ptr<Node> staWifiNode = networkNodes.Get (1);
  NodeContainer staWifiNode;
  staWifiNode.Create(2);

  
  NodeContainer networkNodes;
  networkNodes.Add(p2pNodes.Get(0));  //AP
  networkNodes.Create(1);

  Ptr<Node> apWifiNode = networkNodes.Get(0);
  Ptr<Node> staWifiNode = networkNodes.Get(1);
*/

  /* Configure AP */
  Ssid ssid = Ssid ("network");
  wifiMac.SetType ("ns3::ApWifiMac","Ssid", SsidValue (ssid));

  NetDeviceContainer apDevice = wifiHelper.Install (wifiPhy, wifiMac, apWifiNode);

  /* Configure STA */
  wifiMac.SetType ("ns3::StaWifiMac","Ssid", SsidValue (ssid));

  NetDeviceContainer staDevices = wifiHelper.Install (wifiPhy, wifiMac, staWifiNode);

  /* Mobility model */
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (1.0, 1.0, 0.0));

  mobility.SetPositionAllocator (positionAlloc);
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (apWifiNode);
  mobility.Install (staWifiNode);

  /* Internet stack */
  InternetStackHelper stack;
  stack.Install (staWifiNode);
  //stack.Install (apWifiNode);
  stack.Install (p2pNodes);                             //ADDED

  Ipv4AddressHelper address;


  address.SetBase ("10.1.1.0", "255.255.255.0");                                // ADDED        10.1.1.1 - Server
  Ipv4InterfaceContainer p2pInterfaces = address.Assign (p2pDevices);            // ADDED       10.1.1.2 - AP

  address.SetBase ("10.0.0.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface = address.Assign (apDevice);               // 10.0.0.1
  Ipv4InterfaceContainer staInterface = address.Assign (staDevices);

  /* Populate routing table */
  Ipv4GlobalRoutingHelper::PopulateRoutingTables ();

  

/*
  // Install TCP Receiver on the station node 
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));
  ApplicationContainer sinkApp = sinkHelper.Install (staWifiNode);          // EDITED
  sink = StaticCast<PacketSink> (sinkApp.Get (0));

  // Install TCP Transmitter on the server 
  OnOffHelper server ("ns3::TcpSocketFactory", (InetSocketAddress (staInterface.GetAddress (0), 9)));   //EDITED
  server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
  server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
  server.SetAttribute ("DataRate", DataRateValue (DataRate (dataRate)));
  ApplicationContainer serverApp = server.Install (p2pNodes.Get(0));   // EDITED , .Get(0)
*/


/*
  // Create a PacketSinkApplication and install it on node 1
  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (), port));
  ApplicationContainer sinkApp = sinkHelper.Install (staWifiNode.Get(1));
  sink = StaticCast<PacketSink> (sinkApp.Get (0));

  BulkSendHelper server ("ns3::TcpSocketFactory",InetSocketAddress (staInterface.GetAddress(1), port));
  server.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
  ApplicationContainer serverApp = server.Install (p2pNodes.Get(0));
  
  sinkApp.Start (Seconds (0.0));
  serverApp.Start (Seconds (1.0));
  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);
*/
 
  
  
  
  for(int i=0;i<nwifi;i++){

  BulkSendHelper server ("ns3::TcpSocketFactory",InetSocketAddress (staInterface.GetAddress(i),port));
  server.SetAttribute ("MaxBytes", UintegerValue (maxBytes));
  serverApp.Add(server.Install (p2pNodes.Get(0)));

  PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory",InetSocketAddress (Ipv4Address::GetAny (),port));
  sinkApp.Add(sinkHelper.Install (staWifiNode.Get(i)));
  
  }
  //sink = StaticCast<PacketSink> (sinkApp.Get(0));
  sinkApp.Start (Seconds (1.0));
  sinkApp.Stop (Seconds ( simulationTime + 1));
  serverApp.Start (Seconds (1.0));
  serverApp.Stop (Seconds (simulationTime + 1));
  
  Simulator::Schedule (Seconds (1.1), &CalculateThroughput);

  /* Enable Traces */
  if (pcapTracing)
    {
      wifiPhy.SetPcapDataLinkType (YansWifiPhyHelper::DLT_IEEE802_11_RADIO);
      wifiPhy.EnablePcap ("AccessPoint", apDevice);
      wifiPhy.EnablePcap ("Station", staDevices);
    }

  /* Start Simulation */
  Simulator::Stop (Seconds (simulationTime + 1));
  Simulator::Run ();
  Simulator::Destroy ();

/*
  double throughput = 0,s=0;
  for(unsigned i = 0; i< sinkApp.GetN();i++){
  uint64_t totalPacketsThrough = DynamicCast<PacketSink> (sinkApp.Get (i))->GetTotalRx ();
  s = ((totalPacketsThrough * 8) / (simulationTime * 1000000.0)); //Mbit/s
  std::cout << "Aggregated throughput of client : "<<i+1<<" : " << s << " Mbit/s" << std::endl;
  throughput += s;
  }
  if (throughput > 0)
  std::cout << "Aggregated throughput: " << throughput << " Mbit/s" << std::endl;
  else
  {
  NS_LOG_ERROR ("Obtained throughput is 0!");
  exit (1);
  }
*/

/*
  double averageThroughput = ((sink->GetTotalRx() * 8) / (1e6  * simulationTime));
  if (averageThroughput < 0)
    {
      NS_LOG_ERROR ("Obtained throughput is not in the expected boundaries!");
      exit (1);
    }
  std::cout << "\nAverage throughtput: " << averageThroughput*12 << " Mbit/s" << std::endl;
*/
  return 0;
}