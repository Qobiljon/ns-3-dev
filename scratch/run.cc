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


#include <vector>
#include <stdint.h>
#include <sstream>
#include <fstream>
using namespace ns3;


// <editor-fold desc="Variables">
static long rxSize = 0;
std::ofstream outFile("out1.stat");
double t1 = 0, t2 = 0, t3 = 0, t4 = 0;

// </editor-fold>


class MyApp : public Application {
public:
    MyApp (): m_socket (0), m_peer (), m_packetSize (0), m_nPackets (0), m_dataRate (0), m_sendEvent (), m_running (false), m_packetsSent (0) {
        
    }
    virtual ~MyApp() {
        m_socket = 0;
    }
    void Setup (Ptr<Socket> socket, Address address, uint32_t packetSize, uint32_t nPackets, DataRate dataRate,Ptr<WifiNetDevice> wifinet, int index,Ptr<Node> node) {
        m_socket = socket;
        m_peer = address;
        m_packetSize = packetSize;
        m_nPackets = nPackets;
        m_dataRate = dataRate;
        m_wNet	=	wifinet;
        m_index = index;
        m_node = node;
    }
    
    int count = 0;
private:
    virtual void StartApplication () {
        m_running = true;
        m_packetsSent = 0;
        m_socket->Bind ();
        m_socket->Connect (m_peer);
        FlowControl ();
    }
    virtual void StopApplication () {
        m_running = false;   
        if (m_sendEvent.IsRunning ())
            Simulator::Cancel (m_sendEvent);
        if (m_socket)
            m_socket->Close ();
    }
    void ScheduleTx () {
        if (m_running)
            m_sendEvent = Simulator::Schedule (Seconds (m_packetSize * 8 / static_cast<double> (m_dataRate.GetBitRate ())), &MyApp::SendPacket, this);
    }
    void SendPacket () {
        if(m_running)
            m_socket->Send (Create<Packet> (m_packetSize));
        if (++m_packetsSent < m_nPackets)
            ScheduleTx ();
        else return;
    }
    void FlowControl() {
        if(m_index % 2 == 0) {
            Simulator::Schedule(Seconds(0.0), &MyApp::StartPacketSend,this);
            if(count == 1 && m_index == 0){
                Simulator::Schedule(Seconds(0.1),&MyApp::StopPacketSend,this);
                // TODO: Switch Channel
                Simulator::Schedule(Seconds(0.16),&MyApp::StartPacketSend,this);
            }
            Simulator::Schedule(Seconds(3.0),&MyApp::StopPacketSend,this);
            Simulator::Schedule(Seconds(4.0),&MyApp::FlowControl,this);   
        }
        count++;
        std::cout << "STA: " << m_index << "\t Channel: " << (int)(m_wNet->GetPhy()->GetChannelNumber()) << std::endl;
    }
    void StartPacketSend() {
        m_running=true;
        ScheduleTx();
    }
    void StopPacketSend() {
        m_running=false;
        m_packetsSent=0;
    }
    
    Ptr<Socket>     m_socket;
    Address         m_peer;
    uint32_t        m_packetSize;
    uint32_t        m_nPackets;
    DataRate        m_dataRate;
    EventId         m_sendEvent;
    bool            m_running;
    uint32_t        m_packetsSent;
    Ptr<WifiNetDevice>  m_wNet;
    Ptr<Node>		  m_node;
    int m_index;
};

static void SetPosition (Ptr<Node> node, Vector position) {
    Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
    mobility->SetPosition (position);
}
static void ThroughputMonitor (FlowMonitorHelper *fmhelper, Ptr<FlowMonitor> flowMon) {
    std::map<FlowId, FlowMonitor::FlowStats> flowStats = flowMon->GetFlowStats();
    Ptr<Ipv4FlowClassifier> classing = DynamicCast<Ipv4FlowClassifier> (fmhelper->GetClassifier());
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
    
    outFile << (double)Simulator::Now().GetSeconds() << ' ' << (double)lt1 << ' ' << (double)lt2 << ' ' << (double)lt3 << ' ' << (double)lt4 << std::endl;
    Simulator::Schedule(Seconds(0.1),&ThroughputMonitor, fmhelper, flowMon);   
}
void PhyRxOkTrace (std::string context, Ptr<const Packet> packet, double snr, WifiMode mode, enum WifiPreamble preamble) {
    rxSize += packet->GetSize();
}

int main () {
    ApWifiMac::lvap_mode = true;
    
    uint32_t payloadSize = 1472;
    double simulateTime = 10.0;
    AsciiTraceHelper trace;
    uint32_t nAps = 2;
    uint32_t nStas_udp = 0;
    uint32_t nStas_tcp = 4;
    uint32_t nServ = 1;
    NodeContainer apNodes;
    NodeContainer staUdpNodes;
    NodeContainer staTcpNodes;
    NodeContainer servNodes;
    NodeContainer switchNodes;
    Ptr<Node> switchNode = CreateObject<Node>();
    apNodes.Create(nAps);
    staUdpNodes.Create(nStas_udp);
    staTcpNodes.Create(nStas_tcp);
    servNodes.Create(nServ);
    switchNodes.Add(switchNode);
    
    NetDeviceContainer staUdpDevices;
    NetDeviceContainer staTcpDevices;
    NetDeviceContainer servDevices;
    
    NetDeviceContainer *csmaDevices = new NetDeviceContainer[nAps+nServ];
    std::vector<NetDeviceContainer> staDevices;
    std::vector<NetDeviceContainer> apDevices;
    std::vector<Ipv4InterfaceContainer> staInterfaces_tcp;
    std::vector<Ipv4InterfaceContainer> staInterfaces_udp;
    std::vector<Ipv4InterfaceContainer> apInterfaces;
    
    InternetStackHelper stack;
    
    CsmaHelper csmaAS; //Ap-Switch CSMA
    CsmaHelper csmaSS; //Switch-Server CSMA
    
    Ipv4AddressHelper ip;
    ip.SetBase("192.168.0.0","255.255.255.0");
    
    stack.Install(apNodes);
    stack.Install(staUdpNodes);
    stack.Install(staTcpNodes);
    stack.Install(servNodes);
    
    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    for(uint32_t i = 0; i < nAps; i++)
        csmaDevices[i] = csmaAS.Install(NodeContainer(apNodes.Get(i), switchNode));
    csmaAS.SetChannelAttribute("DataRate", StringValue("200Mbps"));
    csmaSS.SetChannelAttribute("DataRate", StringValue("500Mbps"));
    
    csmaDevices[nAps] = csmaSS.Install(NodeContainer(servNodes.Get(0), switchNode));
    csmaAS.EnableAsciiAll(trace.CreateFileStream("Ap2Switch.tr"));
    csmaSS.EnableAsciiAll(trace.CreateFileStream("Serv2Switch.tr"));
    
    
    BridgeHelper switch0;
    NetDeviceContainer switchDev;
    for(uint32_t n = 0; n < nAps + nServ; n++)
        switchDev.Add(csmaDevices[n].Get(1));
    switch0.Install(switchNode, switchDev);
    
    Ipv4InterfaceContainer serverInterface;
    serverInterface = ip.Assign(csmaDevices[nAps].Get(0));
    UintegerValue ctsThr = UintegerValue (2200);
    Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("999999"));
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("999999"));
    
    Ssid ssid = Ssid ("priorityfirst");
    
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default();
    NodeContainer sta;
    NetDeviceContainer staDev;
    NetDeviceContainer apDev;
    Ipv4InterfaceContainer staInterface;
    Ipv4InterfaceContainer apInterface;
    MobilityHelper mobility;
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
        mobility.Install (servNodes.Get(0));
        wifiMac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
        
        //wifiPhy.Set("ChannelNumber", UintegerValue(1 + (n % 3) * 5));
        apDev = wifi.Install(wifiPhy, wifiMac,apNodes.Get(n));
        
        bridgeDev = bridge.Install(apNodes.Get(n), NetDeviceContainer(apDev, csmaDevices[n].Get(0)));
        
        apInterface = ip.Assign(apDev);
        std::cout << "Ap" << n << ' ' << apInterface.GetAddress(0) << /*" channel "<< 1 + (n % 3) * 5 <<*/ ' ' << apDev.Get(0)->GetAddress() << std::endl;
        
        apDevices.push_back(apDev);
        apInterfaces.push_back(apInterface);
        SetPosition(apNodes.Get(n), Vector(100 + 10 * n, 1.0, 0.0));
    }
    
    for(uint32_t n = 0; n < nStas_tcp; n++) {
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
        mobility.Install(staTcpNodes.Get(n));
        
        wifiMac.SetType("ns3::StaWifiMac",
                "ActiveProbing", BooleanValue(true),
                "Ssid", SsidValue(ssid)
        );
        //wifiPhy.Set("ChannelNumber", UintegerValue(1 + (n % 2) * 5));
        staDev = wifi.Install(wifiPhy, wifiMac, staTcpNodes.Get(n));
        staInterface = ip.Assign(staDev);
        std::cout << "Sta" << n << ' ' << staInterface.GetAddress(0) << /*" channel" << 1 + (n % 2) * 5 <<*/ ' ' << staDev.Get(0)->GetAddress() << std::endl;
        staInterfaces_tcp.push_back(staInterface);
        staDevices.push_back(staDev);
        SetPosition(staTcpNodes.Get(n), Vector(105, n, 0.0));
    }
    
    for(uint32_t n = 0; n < nStas_udp ; n++) {
        MobilityHelper mobility;
        mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobility.Install(staUdpNodes.Get(n));
        
        //if(n < nStas_udp / 2)
        //    wifiPhy.Set("ChannelNumber", UintegerValue(1 + (0 % 2) * 5));
        //else
        //    wifiPhy.Set("ChannelNumber", UintegerValue(1 + (1 % 2) * 5));
        
        wifiMac.SetType("ns3::StaWifiMac",
                "ActiveProbing", BooleanValue(true),
                "Ssid", SsidValue(ssid)
        );
        staDev = wifi.Install(wifiPhy, wifiMac, staUdpNodes.Get(n));
        staInterface = ip.Assign(staDev);
        staInterfaces_udp.push_back(staInterface);
        staDevices.push_back(staDev);
        
        SetPosition(staUdpNodes.Get(n), Vector(105.0, n+1, 0.0));
    }
    
    SetPosition(switchNode, Vector(150, 100,0));
    SetPosition(servNodes.Get(0), Vector(150, 150,0));
    
    Ptr<Socket> tcpSocket[nStas_tcp];
    Ptr<MyApp> app[nStas_tcp];
    
    PacketSinkHelper packetSinkHelper("ns3::TcpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(),8080));
    ApplicationContainer sinkApp = packetSinkHelper.Install(staTcpNodes.Get(0));
    ns3::Ipv4Address recvAddress = staInterfaces_tcp.at(0).GetAddress(0);
    
    for(uint32_t n = 1; n < nStas_tcp; n++) {
        Address sinkAddress(InetSocketAddress(recvAddress, 8080));
        sinkApp.Start (Seconds (0.0));
        sinkApp.Stop (Seconds (simulateTime));
        tcpSocket[n] = Socket::CreateSocket(staTcpNodes.Get(n), TcpSocketFactory::GetTypeId());
        app[n] = CreateObject<MyApp>();
        app[n]->Setup(tcpSocket[n], sinkAddress, payloadSize, 100000, DataRate("8Mbps"), staDevices[n].Get(0)->GetObject<WifiNetDevice>(), n, staTcpNodes.Get(n));
        staTcpNodes.Get (n)->AddApplication(app[n]);
        app[n]->SetStartTime(Seconds (1.0));
        app[n]->SetStopTime(Seconds (simulateTime));
    }
    
    Config::Connect ("/NodeList/5/DeviceList/*/Phy/State/RxOk", MakeCallback (&PhyRxOkTrace));
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    ThroughputMonitor(&flowmon, monitor);
    Simulator::Stop (Seconds (simulateTime));
    Simulator::Run ();
    Simulator::Destroy ();
    
    std::cout << "Rx bytes: " << rxSize << std::endl;
    std::cout << "Flow: " << t1 << std::endl;
    std::cout << "Flow: " << t3 << std::endl;
    
    return 0;
}