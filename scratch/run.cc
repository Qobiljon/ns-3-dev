#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-module.h"
#include "ns3/network-module.h"
#include "ns3/csma-module.h"
#include "ns3/internet-module.h"
#include "ns3/bridge-helper.h"
#include "ns3/config-store-module.h"
#include "ns3/netanim-module.h"
#include "ns3/log.h"
#include "ns3/random-variable-stream.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "src/wifi/model/ap-wifi-mac.h"
#include "src/core/model/object.h"
#include "src/network/model/node.h"


#include <vector>
#include <stdint.h>
#include <sstream>
#include <fstream>
#include <cmath>
using namespace ns3;

// <editor-fold desc="Variables">
static std::ofstream logFile("log.csv");
static double t1 = 0, t2 = 0, t3 = 0, t4 = 0;
static Ipv4InterfaceContainer serverInterface;
// </editor-fold>

static void Handoff(uint32_t staNodeId, uint32_t destApNodeId){
    //logFile << "HANDOFF STARTED" << std::endl;
    std::cout << "-*-*- HANDOFF STARTED -*-*-" << std::endl;
    
    if(!ApWifiMac::lvap_mode){
        std::cout << "-*-*- HANDOFF ABORTED, LVAP TURNED OFF" << std::endl;
        logFile << "HANDOFF ABORTED" << std::endl;
        return;
    }
    
    Ptr<Node> staNode = NodeList::GetNode(staNodeId);
    Ptr<Node> apNode = NodeList::GetNode(destApNodeId);
    
    Ptr<WifiNetDevice> apNetDevice = DynamicCast<WifiNetDevice>(apNode->GetDevice(2));
    Ptr<WifiNetDevice> staNetDevice = DynamicCast<WifiNetDevice>(staNode->GetDevice(1));
    
    Mac48Address destApMacAddr = Mac48Address::ConvertFrom(apNetDevice->GetAddress());
    Mac48Address staMacAddr = Mac48Address::ConvertFrom(staNetDevice->GetAddress());
    
    Ptr<ApWifiMac> to_ap = ApWifiMac::ap_objects[destApMacAddr];
    Ptr<ApWifiMac> from_ap = ApWifiMac::sta_ap_glb_map[staMacAddr];
    
    if(from_ap == to_ap){
        std::cout << "-*-*- HANDOFF ABORTED, AP WASN'T CHANGED" << std::endl;
        //logFile << "HANDOFF ABORTED" << std::endl;
        return;
    }
    
    //printf("DETACHING LVAP FROM CURRENT AP\n");
    LvapState* lvap_state = from_ap->detach_sta_from_ap(staMacAddr);
    
    Ptr<WifiPhy> apPhy = apNetDevice->GetPhy();
    Ptr<WifiPhy> staPhy = staNetDevice->GetPhy();
    //int fromChannel = (int)(staPhy->GetChannelNumber());
    staPhy->SetChannelNumber(apPhy->GetChannelNumber());
    //std::cout << "CHANNEL STA CHANGED FROM: " << fromChannel << " TO: " << (int)(apPhy->GetChannelNumber()) << std::endl;
    
    to_ap->attach_lvap_to_ap(staMacAddr, lvap_state);
    
    Ptr<ArpL3Protocol> arpL3Prot = staNode->GetObject<ArpL3Protocol>();
    Ptr<ArpCache> arpCache = arpL3Prot->FindCache(staNetDevice);
    
    //std::cout << "SENDING ARP REQUEST NOW " << arpL3Prot << ' ' << arpCache << ' ' << serverInterface.GetAddress(0) << std::endl;
    Simulator::Schedule (Time (MilliSeconds (arpL3Prot->m_requestJitter->GetValue ())), &ArpL3Protocol::SendArpRequest, arpL3Prot, arpCache, serverInterface.GetAddress(0));
    
    ApWifiMac::handedOver = true;
    
    //std::cout << "STA: " << staMacAddr << " AP:" << destApMacAddr << std::endl;
    std::cout << "-*-*- HANDOFF COMPLETED -*-*-" << std::endl;
    //logFile << "HANDOFF FINISHED" << std::endl;
}

static void SetPosition (Ptr<Node> node, Vector position) {
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    mobility->SetPosition (position);
}

static void ThroughputMonitor (FlowMonitorHelper* fmhelper, Ptr<FlowMonitor> flowMon) {
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    //Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
    double lt1 = 0;
    double lt2 = 0;
    double lt3 = 0;
    double lt4 = 0;
    
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator stats = flowStats.begin (); stats != flowStats.end (); ++stats) {
        switch(stats->first) {
            case 1:
                lt1 = (stats->second.rxBytes * 8.0 - t1 * 8.0) / 0.1 / 1024 / 1024;
                t1 = stats->second.rxBytes;//-temp[0];
                break;
            case 2:
                lt2 = (stats->second.rxBytes * 8.0 - t2 * 8.0) / 0.1 / 1024 / 1024;
                t2 = stats->second.rxBytes;//-temp[1];
                break;
            case 3:
                lt3 = (stats->second.rxBytes * 8.0 - t3 * 8.0) / 0.1 / 1024 / 1024;
                t3 = stats->second.rxBytes;//-temp[2];
                break;
            case 4:
                lt4 = (stats->second.rxBytes * 8.0 - t4 * 8.0) / 0.1 / 1024 / 1024;
                t4 = stats->second.rxBytes;//-temp[3];
                break;
            default:
                break;
        }
    }
    
    logFile << (double)Simulator::Now().GetSeconds() << '\t' << (double)lt1 << '\t' << (double)lt2 << '\t' << (double)lt3 << '\t' << (double)lt4 << std::endl;
    Simulator::Schedule(Seconds(0.1),&ThroughputMonitor, fmhelper, flowMon);   
}

int main () {
    ApWifiMac::lvap_mode = true;
    
    uint32_t payloadSize = 1472;
    double simulateTime = 4.0;
    AsciiTraceHelper trace;
    uint32_t nAps = 2;
    uint32_t nStas = 2;
    uint32_t nServ = 1;
    NodeContainer apNodes;
    NodeContainer staNodes;
    Ptr<Node> servNode = CreateObject<Node>();
    NodeContainer switchNodes;
    Ptr<Node> switchNode = CreateObject<Node>();
    apNodes.Create(nAps);
    staNodes.Create(nStas);
    switchNodes.Add(switchNode);
    
    NetDeviceContainer* csmaDevices = new NetDeviceContainer[nAps + nServ];
    std::vector<NetDeviceContainer> staDevices;
    std::vector<NetDeviceContainer> apDevices;
    
    InternetStackHelper stack;
    
    CsmaHelper csmaAS; //Ap-Switch CSMA
    CsmaHelper csmaSS; //Switch-Server CSMA
    
    Ipv4AddressHelper ip;
    ip.SetBase("192.168.0.0","255.255.255.0");
    
    stack.Install(apNodes);
    stack.Install(staNodes);
    stack.Install(servNode);
    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    csmaAS.SetChannelAttribute("DataRate", StringValue("200Mbps"));
    csmaAS.EnableAsciiAll(trace.CreateFileStream("Ap2Switch.tr"));
    for(uint32_t i = 0; i < nAps; i++)
        csmaDevices[i] = csmaAS.Install(NodeContainer(apNodes.Get(i), switchNode));
    
    csmaSS.SetChannelAttribute("DataRate", StringValue("500Mbps"));
    csmaSS.EnableAsciiAll(trace.CreateFileStream("Serv2Switch.tr"));
    csmaDevices[nAps] = csmaSS.Install(NodeContainer(servNode, switchNode));
    
    // Set the central controller tag (boolean value)
    static Ptr<CsmaNetDevice> centralController = Ptr<CsmaNetDevice>(DynamicCast<CsmaNetDevice>(csmaDevices[nAps].Get(0)));
    
    BridgeHelper switch0;
    NetDeviceContainer switchDev;
    for(uint32_t n = 0; n < nAps + nServ; n++)
        switchDev.Add(csmaDevices[n].Get(1));
    switch0.Install(switchNode, switchDev);
    
    serverInterface = ip.Assign(csmaDevices[nAps].Get(0));
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
    
    Ssid ssid = Ssid ("priorityfirst");
    
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    NodeContainer sta;
    NetDeviceContainer staDev;
    NetDeviceContainer apDev;
    Ipv4InterfaceContainer staInterface;
    Ipv4InterfaceContainer apInterface;
    BridgeHelper bridge;
    WifiHelper wifi;
    WifiMacHelper wifiMac;
    
    wifi.SetRemoteStationManager ("ns3::ArfWifiManager");
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default();
    wifiPhy.SetChannel (wifiChannel.Create ());
    
    std::cout << "Server: " << serverInterface.GetAddress(0) << std::endl;
    for(uint32_t n = 0; n < nAps; n++) {
        MobilityHelper mobility;
        NetDeviceContainer bridgeDev;
        
        wifi.SetStandard(WIFI_PHY_STANDARD_80211g);
        
        mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
                "MinX", DoubleValue (0.0),
                "MinY", DoubleValue (0.0),
                "DeltaX", DoubleValue (5.0),
                "DeltaY", DoubleValue (5.0),
                "GridWidth", UintegerValue (1),
                "LayoutType", StringValue ("RowFirst"));
        
        // setup the AP.
        mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
        mobility.Install (apNodes.Get (n));
        mobility.Install (switchNode);
        mobility.Install (servNode);
        wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
        
        wifiPhy.Set("ChannelNumber", UintegerValue(n == 0 ? 1 : 2));
        apDev = wifi.Install(wifiPhy, wifiMac,apNodes.Get(n));
        
        bridgeDev = bridge.Install(apNodes.Get(n), NetDeviceContainer(apDev, csmaDevices[n].Get(0)));
        
        apInterface = ip.Assign(apDev);
        std::cout << "Ap" << n << ' ' << apInterface.GetAddress(0) << /*" channel "<< 1 + (n % 3) * 5 <<*/ ' ' << apDev.Get(0)->GetAddress() << std::endl;
        
        apDevices.push_back(apDev);
        SetPosition(apNodes.Get(n), Vector(100 + 10 * n, 1.0, 0.0));
    }
    
    for(uint32_t n = 0; n < nStas; n++) {
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        mobility.Install(staNodes.Get(n));
        
        wifiMac.SetType("ns3::StaWifiMac",
                "ActiveProbing", BooleanValue(true),
                "Ssid", SsidValue(ssid)
        );
        wifiPhy.Set("ChannelNumber", UintegerValue(1));//UintegerValue(1 + (n % 2) * 5));
        staDev = wifi.Install(wifiPhy, wifiMac, staNodes.Get(n));
        staInterface = ip.Assign(staDev);
        std::cout << "Sta" << n << ' ' << staInterface.GetAddress(0) << /*" channel" << 1 + (n % 2) * 5 <<*/ ' ' << staDev.Get(0)->GetAddress() << std::endl;
        staDevices.push_back(staDev);
        SetPosition(staNodes.Get(n), Vector(105, n, 0.0));
    }
    
    SetPosition(switchNode, Vector(150, 100,0));
    SetPosition(servNode, Vector(150, 150,0));
    
    PacketSinkHelper sinkHelper ("ns3::TcpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), 9));
    sinkHelper.Install (servNode);
    
    /* Install TCP/UDP Transmitter on the station */
    {
        OnOffHelper server ("ns3::TcpSocketFactory", (InetSocketAddress (serverInterface.GetAddress(0), 9)));
        server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        server.SetAttribute ("DataRate", DataRateValue (DataRate ("10Mbps")));
        ApplicationContainer serverApp = server.Install (staNodes.Get(0));
        serverApp.Start(Seconds(1));
        serverApp.Stop(Seconds(simulateTime));
    }
    {
        OnOffHelper server ("ns3::TcpSocketFactory", (InetSocketAddress (serverInterface.GetAddress(0), 9)));
        server.SetAttribute ("PacketSize", UintegerValue (payloadSize));
        server.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
        server.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
        server.SetAttribute ("DataRate", DataRateValue (DataRate ("10Mbps")));
        ApplicationContainer serverApp = server.Install (staNodes.Get(1));
        serverApp.Start(Seconds(1));
        serverApp.Stop(Seconds(simulateTime));
    }
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    ThroughputMonitor(&flowmon, monitor);
    Simulator::Schedule (Seconds (simulateTime / 2), &Handoff, staNodes.Get(0)->GetId(), apNodes.Get(1)->GetId());
    Simulator::Stop (Seconds (simulateTime));
    
    AnimationInterface anim("anim-network.xml");
    anim.EnablePacketMetadata();
    
    Simulator::Run ();
    Simulator::Destroy ();
    return 0;
}