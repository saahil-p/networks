#include "ns3/command-line.h"
#include "ns3/config.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/ipv4-global-routing-helper.h"
#include "ns3/log.h"
#include "ns3/mobility-helper.h"
#include "ns3/mobility-model.h"
#include "ns3/on-off-helper.h"
#include "ns3/packet-sink-helper.h"
#include "ns3/packet-sink.h"
#include "ns3/ssid.h"
#include "ns3/string.h"
#include "ns3/tcp-westwood-plus.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/flow-monitor.h"
#include "ns3/flow-monitor-helper.h"
#include "ns3/internet-module.h"
#include "ns3/ipv4-flow-classifier.h"
#include "ns3/netanim-module.h"
#include "ns3/constant-velocity-mobility-model.h"

#include <fstream>
#include <vector>

NS_LOG_COMPONENT_DEFINE("proj");

using namespace ns3;

std::vector<Ptr<PacketSink>> sinks;
uint64_t lastTotalRx = 0;
std::ofstream throughputFile;
uint64_t totalRx = 0;
void
CalculateThroughput()
{
    Time now = Simulator::Now(); /* Return the simulator's virtual time. */

    for (auto sink : sinks)
    {
        totalRx += sink->GetTotalRx();
    }
    double cur = (totalRx - lastTotalRx) * 8.0 / 1e6; /* Convert Application RX Packets to MBits. */
    throughputFile << now.GetSeconds() << "\t" << cur << std::endl;

    lastTotalRx = totalRx;
    Simulator::Schedule(MilliSeconds(1000), &CalculateThroughput);
}

int
main(int argc, char *argv[])
{
    uint32_t payloadSize{1270};           /* Transport layer payload size in bytes. */
    DataRate dataRate{"41.76Kb/s"};       /* Application layer datarate. */
    std::string tcpVariant{"TcpVegas"};   /* TCP variant type. */
    std::string phyRate{"HtMcs7"};        /* Physical layer bitrate. */
    Time simulationTime{"300s"};          /* Simulation time. */
    bool pcapTracing{false};              /* PCAP Tracing is enabled or not. */
    std::string topo{"Grid"};             /* Network topology in use */
    std::string mobility{"01"};           /* Set the mobility model of the server-MSB and client-LSB */

    /* Command line argument parser setup. */
    CommandLine cmd(__FILE__);
    cmd.AddValue("payloadSize", "Payload size in bytes", payloadSize);
    cmd.AddValue("dataRate", "Application data rate", dataRate);
    cmd.AddValue("tcpVariant",
                 "Transport protocol to use: TcpNewReno, "
                 "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                 "TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat ",
                 tcpVariant);
    cmd.AddValue("phyRate", "Physical layer bitrate", phyRate);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("pcap", "Enable/disable PCAP Tracing", pcapTracing);
    cmd.AddValue("topo", "Topology to be used : Grid, Circle, Ellipse", topo);
    cmd.AddValue("mobility", "Mobility to be used : 00 01 10 11", mobility);
    cmd.Parse(argc, argv);
    std::string tcpName = tcpVariant;

    tcpVariant = std::string("ns3::") + tcpVariant;
    // Select TCP variant
    TypeId tcpTid;
    NS_ABORT_MSG_UNLESS(TypeId::LookupByNameFailSafe(tcpVariant, &tcpTid),
                        "TypeId " << tcpVariant << " not found");
    Config::SetDefault("ns3::TcpL4Protocol::SocketType",
                       TypeIdValue(TypeId::LookupByName(tcpVariant)));

    /* Configure TCP Options */
    Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));

    WifiMacHelper wifiMac;
    WifiHelper wifiHelper;
    wifiHelper.SetStandard(WIFI_STANDARD_80211n);

    /* Set up Legacy Channel */
    YansWifiChannelHelper wifiChannel;
    wifiChannel.SetPropagationDelay("ns3::ConstantSpeedPropagationDelayModel");
    wifiChannel.AddPropagationLoss("ns3::FriisPropagationLossModel", "Frequency", DoubleValue(5e9));

    /* Setup Physical Layer */
    YansWifiPhyHelper wifiPhy;
    wifiPhy.SetChannel(wifiChannel.Create());
    wifiPhy.SetErrorRateModel("ns3::YansErrorRateModel");
    wifiHelper.SetRemoteStationManager("ns3::ConstantRateWifiManager",
                                       "DataMode",
                                       StringValue(phyRate),
                                       "ControlMode",
                                       StringValue("HtMcs0"));

    NodeContainer apWifiNode;
    apWifiNode.Create(1);

    NodeContainer smartVehicleNodes;
    smartVehicleNodes.Create(30);

    Ssid ssid = Ssid("network");
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);

    wifiMac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid));

    NetDeviceContainer smartVehicleDevices;
    smartVehicleDevices = wifiHelper.Install(wifiPhy, wifiMac, smartVehicleNodes);

    MobilityHelper smartVehicleMobility;
    smartVehicleMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    smartVehicleMobility.SetPositionAllocator("ns3::GridPositionAllocator", "MinX", DoubleValue(0.0),
                                              "MinY", DoubleValue(0.0), "DeltaX", DoubleValue(10.0), "DeltaY", DoubleValue(20.0), "GridWidth", UintegerValue(15),
                                              "LayoutType", StringValue("RowFirst"));
    smartVehicleMobility.Install(smartVehicleNodes);
    for (int i = 0; i < 15; i++)
    {
        Ptr<Node> node = smartVehicleNodes.Get(i);
        Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(1.0, 0.0, 0.0));
    }

    for (int i = 15; i < 30; i++)
    {
        Ptr<Node> node = smartVehicleNodes.Get(i);
        Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
        mob->SetVelocity(Vector(-1.0, 0.0, 0.0));
    }

    MobilityHelper apMobility;
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(50.0,10.0,0.0));
    apMobility.SetPositionAllocator(apPositionAlloc);
    apMobility.Install(apWifiNode);

    InternetStackHelper stack;
    stack.Install(apWifiNode);
    stack.Install(smartVehicleNodes);

    Ipv4AddressHelper address;
    address.SetBase("192.168.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer smartVehicleInterface;
    smartVehicleInterface = address.Assign(smartVehicleDevices);

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();

    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory", InetSocketAddress(Ipv4Address::GetAny(), 9));
    ApplicationContainer sinkApp = sinkHelper.Install(smartVehicleNodes);
    ApplicationContainer sinkApp2 = sinkHelper.Install(apWifiNode);
    for (uint32_t i = 0; i < sinkApp.GetN(); ++i)
    {
        sinks.push_back(StaticCast<PacketSink>(sinkApp.Get(i)));
    }
    sinks.push_back(StaticCast<PacketSink>(sinkApp2.Get(0)));

    OnOffHelper server("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    server.SetAttribute("PacketSize", UintegerValue(100));
    server.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    server.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    server.SetAttribute("DataRate", DataRateValue(DataRate("800b/s")));
    ApplicationContainer serverApp = server.Install(smartVehicleNodes);
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(simulationTime);

    OnOffHelper server2("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    server2.SetAttribute("PacketSize", UintegerValue(200));
    server2.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    server2.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=10]"));
    server2.SetAttribute("DataRate", DataRateValue(DataRate("1600b/s")));
    ApplicationContainer serverApp2 = server2.Install(smartVehicleNodes);
    serverApp2.Start(Seconds(1.0));
    serverApp2.Stop(simulationTime);

    OnOffHelper server3("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    server3.SetAttribute("PacketSize", UintegerValue(3000));
    server3.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    server3.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    server3.SetAttribute("DataRate", DataRateValue(DataRate("24Kb/s")));
    ApplicationContainer serverApp3 = server3.Install(smartVehicleNodes);
    serverApp3.Start(Seconds(1.0));
    serverApp3.Stop(simulationTime);
 
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    std :: string throughputFileName = "throughput/throughput_" + tcpName + std:: string {".txt"};
    
    throughputFile.open(throughputFileName);
    AnimationInterface anim("proj_netanim.xml");
    anim.SetMaxPktsPerTraceFile(3145728);
   
   for (int i = 0; i < 1; i++)
   {
    Ptr<MobilityModel> apmob = apWifiNode.Get(i)->GetObject<MobilityModel>();
    double apx= apmob->GetPosition().x;
    double apy = apmob->GetPosition().y;

    anim.SetConstantPosition(apWifiNode.Get(i),apx,apy);
    anim.UpdateNodeColor(apWifiNode.Get(i),0,0,255);
   }
    

    
    
    sinkApp.Start(Seconds(0.0));
   
    
    Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
    
    for(int i = 0; i < 30; i++){
    	Ptr<MobilityModel> mob = smartVehicleNodes.Get(i)->GetObject<MobilityModel>();
    	double x = mob->GetPosition().x;
    	double y = mob->GetPosition().y;
    	anim.SetConstantPosition(smartVehicleNodes.Get(i),x,y);
        anim.UpdateNodeColor(smartVehicleNodes.Get(i),0,255,0);
    }
    
    
    Simulator::Stop(Seconds(300));
    Simulator :: Run();
    

    
    throughputFile.close();
    
    auto averageThroughput =
        (static_cast<double>(totalRx * 8) / simulationTime.GetMicroSeconds());
    
    std::cout << "\nAverage throughput: " << averageThroughput << " Mbit/s" << std::endl;
    
    //Flow monitor code
    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    std::ofstream flowStatsFile;
    
    std::string flowFileName = "flowstats/flow_stats_" + tcpName + ".txt";
     
    flowStatsFile.open(flowFileName);
    
    int total_tx = 0;
    int total_rx = 0;
    
    // Print Flow Monitor statistics for flows terminating at the sink
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        if (t.destinationAddress == apInterface.GetAddress (0))
        {
            total_tx+= i->second.txPackets;
            total_rx+= i->second.rxPackets;
            flowStatsFile << i->first << "\t"
                          << t.sourceAddress << "\t"
                          << t.destinationAddress << "\t"
                          << i->second.txBytes << "\t"
                          << i->second.rxBytes << "\t"
                          << i->second.txPackets << "\t"
                          << i->second.rxPackets << "\t"
                          << i->second.lostPackets << "\t"
                          << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024 << std::endl;
       
        }
    }
    

    flowStatsFile.close();
    
    std ::ofstream pktStatsFile;
    
    pktStatsFile.open("packet_stats.txt",std::ios::app);
    
    pktStatsFile << tcpName << "\t" << total_tx << "\t" << total_rx << std::endl;
    
    pktStatsFile.close();
   
    Simulator :: Destroy();

    std:: string opName{std::string("throughput") + tcpName + std::string(".png")};
    
    // Generate gnuplot scripts
    std::ofstream gnuplotScript;
    std ::string("throughput") + tcpVariant + std::string(".plt");
    gnuplotScript.open("plot_throughput.plt");
    gnuplotScript << "set terminal png size 800,600\n";
    gnuplotScript << "set output '" << opName << "'\n";
    gnuplotScript << "set title 'Throughput Over Time'\n";
    gnuplotScript << "set xlabel 'Time (s)'\n";
    gnuplotScript << "set ylabel 'Throughput (Mbit/s)'\n";
    gnuplotScript << "plot '" <<throughputFileName <<"' using 1:2 with lines title 'Throughput '" << opName << " \n";
    gnuplotScript.close();
    
    gnuplotScript.open("plot_flow_stats.plt");
    gnuplotScript << "set terminal png size 800,600\n";
    gnuplotScript << "set output 'flow_stats.png'\n";
    gnuplotScript << "set title 'Flow Statistics'\n";
    gnuplotScript << "set xlabel 'Flow ID'\n";
    gnuplotScript << "set ylabel 'Bytes'\n";
    gnuplotScript << "set style data histograms\n";
    gnuplotScript << "set style fill solid\n";
    gnuplotScript << "plot 'flow_stats.txt' using 5 with histogram title 'Rx Bytes', '' using 4 with histogram title 'Tx Bytes'\n";
    gnuplotScript.close();
    

  
    

    return 0;
}
