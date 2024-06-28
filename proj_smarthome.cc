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

#include <fstream>

NS_LOG_COMPONENT_DEFINE("proj");

using namespace ns3;

Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0;
std::ofstream throughputFile;

void
CalculateThroughput()
{
    Time now = Simulator::Now(); /* Return the simulator's virtual time. */
    double cur = (sink->GetTotalRx() - lastTotalRx) * 8.0 /1e3; /* Convert Application RX Packets to MBits. */
                
    throughputFile << now.GetSeconds() << "\t" << cur << std::endl;
    

    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(1000), &CalculateThroughput);
}

int
main(int argc, char *argv[]){

    uint32_t payloadSize{1270};           /* Transport layer payload size in bytes. */
    DataRate dataRate{"41.76Kb/s"};         /* Application layer datarate. */
    std::string tcpVariant{"TcpIllinois"}; /* TCP variant type. */
    std::string phyRate{"HtMcs7"};        /* Physical layer bitrate. */
    Time simulationTime{"300s"};           /* Simulation time. */
    bool pcapTracing{false};              /* PCAP Tracing is enabled or not. */
    std::string topo{"Grid"};		  /* network topology in use */
    std::string mobility{"01"};			/*Set the mobility model of the server-MSB and client-LSB*/

    /* Command line argument parser setup. */
    CommandLine cmd(__FILE__);
    cmd.AddValue("payloadSize", "Payload size in bytes", payloadSize);
    cmd.AddValue("dataRate", "Application data ate", dataRate);
    cmd.AddValue("tcpVariant",
                 "Transport protocol to use: TcpNewReno, "
                 "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                 "TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat ",
                 tcpVariant);
    cmd.AddValue("phyRate", "Physical layer bitrate", phyRate);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
    cmd.AddValue("pcap", "Enable/disable PCAP Tracing", pcapTracing);
    cmd.AddValue("topo","Topology to be used : Grid, Circle,Ellipse",topo);
    cmd.AddValue("mobility","Mobility to be used : 00 01 10 11",mobility);
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
                                       
                                       
    //NodeContainer staWifiNodes;
    //staWifiNodes.Create(10);
    
    NodeContainer apWifiNode;
    apWifiNode.Create(1);
    
    NodeContainer smartTvNodes;
    smartTvNodes.Create(2); //payload size of between 2 - 5 MB/s
    
    NodeContainer smartLightNodes;
    smartLightNodes.Create(18); //payload size of between 20 & 500 bytes
   
    NodeContainer smartAcNodes;
    smartAcNodes.Create(4); // Payload size of between 100 & 1000 bytes
    
    NodeContainer smartPhoneNodes;
    smartPhoneNodes.Create(4); // Payload size of 64 bytes if control signal being simulated, 1500 bytes for video streaming etc
    
    Ssid ssid = Ssid("network");
    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);
    
    wifiMac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid));
    //NetDeviceContainer staDevices;
    //staDevices = wifiHelper.Install(wifiPhy,wifiMac,staWifiNodes);
    NetDeviceContainer smartTvDevices;
    smartTvDevices = wifiHelper.Install(wifiPhy,wifiMac,smartTvNodes);
    NetDeviceContainer smartLightDevices;
    smartLightDevices = wifiHelper.Install(wifiPhy,wifiMac,smartLightNodes);
    NetDeviceContainer smartAcDevices;
    smartAcDevices = wifiHelper.Install(wifiPhy,wifiMac,smartAcNodes);
    NetDeviceContainer smartPhoneDevices;
    smartPhoneDevices = wifiHelper.Install(wifiPhy,wifiMac,smartPhoneNodes);
    
    MobilityHelper staMobility;
    MobilityHelper apMobility;
    MobilityHelper tvMobility;
    MobilityHelper acMobility;
    MobilityHelper phoneMobility;
    MobilityHelper lightMobility;
    
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    tvMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    acMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    lightMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    phoneMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel");
   
   /* 
    std::string mm;
    
    if(mobility == "00"){
    	staMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    	apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    	
    	mm = "ss";
    }
    else if(mobility == "01"){
    	staMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    	apMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel");
    	
    	mm = "sd";
    }
    else if(mobility == "10"){
    	apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    	staMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel");
    	
    	mm = "ds";
    }
    else if(mobility == "11"){
    	staMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel");
    	apMobility.SetMobilityModel("ns3::RandomDirection2dMobilityModel");
    	
    	mm = "dd";
    }
    */
    
    
    //Creating a smart home topology
    
    Ptr<ListPositionAllocator> tvPositionAlloc = CreateObject<ListPositionAllocator>();
    tvPositionAlloc->Add(Vector(40.0,80.0,0.0));
    tvPositionAlloc->Add(Vector(-40.0,20.0,0.0));
    tvMobility.SetPositionAllocator(tvPositionAlloc);
    tvMobility.Install(smartTvNodes);
    
    Ptr<ListPositionAllocator> acPositionAlloc = CreateObject<ListPositionAllocator>();
    acPositionAlloc->Add(Vector(20.0,100.0,0.0));
    acPositionAlloc->Add(Vector(-40.0,30.0,0.0));
    acPositionAlloc->Add(Vector(40.0,20.0,0.0));
    acPositionAlloc->Add(Vector(40.0,60.0,0.0));
    acMobility.SetPositionAllocator(acPositionAlloc);
    acMobility.Install(smartAcNodes);
    
    
    lightMobility.SetPositionAllocator("ns3::GridPositionAllocator","MinX",DoubleValue(-20.0),"MinY",DoubleValue(0.0),"DeltaX", DoubleValue(20.0),"DeltaY",DoubleValue(20.0),"GridWidth",UintegerValue(4),"LayoutType",StringValue("RowFirst"));
    lightMobility.Install(smartLightNodes);
    
    phoneMobility.SetPositionAllocator("ns3::GridPositionAllocator","MinX",DoubleValue(10.0),"MinY",DoubleValue(20.0),"DeltaX", DoubleValue(20.0),"DeltaY",DoubleValue(40.0),"GridWidth",UintegerValue(3),"LayoutType",StringValue("RowFirst"));
    phoneMobility.Install(smartPhoneNodes);
    
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(0.0,50.0,0.0));
    apMobility.SetPositionAllocator(apPositionAlloc);
    apMobility.Install(apWifiNode);
    
    
    
    
    
    
    /*
    if(topo == "Grid"){
    	staMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                 	 "MinX", DoubleValue(0.0),
                                	  "MinY", DoubleValue(0.0),
                                	  "DeltaX", DoubleValue(5.0),
                                 	"DeltaY", DoubleValue(10.0),
                                 	 "GridWidth", UintegerValue(5),
                                 	 "LayoutType", StringValue("RowFirst"));
                                 	 
         staMobility.Install(apWifiNode);
   	 staMobility.Install(staWifiNodes);  
     }
    else if(topo == "Circle"){
        Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        
        double radius = 50;
        
        for(int i = 0; i < 10; i++){
        	double angle = 2 * M_PI * i/ 10;
        	double x = radius + 50 * cos(angle);
        	double y = radius + 50 * sin(angle);
        	positionAlloc->Add(Vector(x,y,0.0));
        }
        
        staMobility.SetPositionAllocator(positionAlloc);

        
        Ptr<ListPositionAllocator> apPosAlloc = CreateObject<ListPositionAllocator>();
        double x = 50.0;
        double y = 50.0;
        double z = 0.0;
        
        apPosAlloc->Add(Vector(x,y,z));
        apMobility.SetPositionAllocator(apPosAlloc);

    }
    else if(topo == "Ellipse"){
         Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
        
        double a = 70;
        double b = 30;
        
        for(int i = 0; i < 10; i++){
        	double angle = 2 * M_PI * i/ 10;
        	double x = 50 + a * cos(angle);
        	double y = 50 + b * sin(angle);
        	positionAlloc->Add(Vector(x,y,0.0));
        }
        
        staMobility.SetPositionAllocator(positionAlloc);

        
        Ptr<ListPositionAllocator> apPosAlloc = CreateObject<ListPositionAllocator>();
        double x = 50.0;
        double y = 50.0;
        double z = 0.0;
        
        apPosAlloc->Add(Vector(x,y,z));
        apMobility.SetPositionAllocator(apPosAlloc);
    }
    
    if(topo != "Grid"){
    	staMobility.Install(staWifiNodes);
    	apMobility.Install(apWifiNode);
    }
    
    */
    
    InternetStackHelper stack;
    //stack.Install(staWifiNodes);
    stack.Install(apWifiNode);
    stack.Install(smartTvNodes);
    stack.Install(smartAcNodes);
    stack.Install(smartLightNodes);
    stack.Install(smartPhoneNodes);
    
    
    Ipv4AddressHelper address;
    address.SetBase("192.168.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevice);
    //Ipv4InterfaceContainer staInterface;
    //staInterface = address.Assign(staDevices);
    
    Ipv4InterfaceContainer tvInterface;
    tvInterface = address.Assign(smartTvDevices);
    Ipv4InterfaceContainer acInterface;
    acInterface = address.Assign(smartAcDevices);
    Ipv4InterfaceContainer lightInterface;
    lightInterface = address.Assign(smartLightDevices);
    Ipv4InterfaceContainer phoneInterface;
    phoneInterface = address.Assign(smartPhoneDevices);
    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",InetSocketAddress(Ipv4Address::GetAny(), 9));
    ApplicationContainer sinkApp = sinkHelper.Install(apWifiNode);
    sink = StaticCast<PacketSink>(sinkApp.Get(0));
    
    OnOffHelper tvServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    tvServer.SetAttribute("PacketSize", UintegerValue(4194304));
    tvServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    tvServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    tvServer.SetAttribute("DataRate", DataRateValue(DataRate("2Mb/s")));
    ApplicationContainer tvServerApp = tvServer.Install(smartTvNodes);
    
    OnOffHelper acServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    acServer.SetAttribute("PacketSize", UintegerValue(700));
    acServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    acServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    acServer.SetAttribute("DataRate", DataRateValue(DataRate("50Kb/s")));
    ApplicationContainer acServerApp = acServer.Install(smartAcNodes);
    
    OnOffHelper lightServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    lightServer.SetAttribute("PacketSize", UintegerValue(300));
    lightServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    lightServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    lightServer.SetAttribute("DataRate", DataRateValue(DataRate("7Kb/s")));
    ApplicationContainer lightServerApp = lightServer.Install(smartLightNodes);
    
    OnOffHelper phoneServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    phoneServer.SetAttribute("PacketSize", UintegerValue(1500));
    phoneServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    phoneServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    phoneServer.SetAttribute("DataRate", DataRateValue(DataRate("4Mb/s")));
    ApplicationContainer phoneServerApp = phoneServer.Install(smartPhoneNodes);
    
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    std :: string throughputFileName = "throughput/throughput_" + tcpName + std:: string {".txt"};
    
    throughputFile.open(throughputFileName);
    AnimationInterface anim("proj_netanim.xml");
    anim.SetMaxPktsPerTraceFile(524288);
   
    
    Ptr<MobilityModel> apmob = apWifiNode.Get(0)->GetObject<MobilityModel>();
    double apx = apmob->GetPosition().x;
    double apy = apmob->GetPosition().y;
    
    anim.SetConstantPosition(apWifiNode.Get(0),apx,apy);
    
    
    sinkApp.Start(Seconds(0.0));
    tvServerApp.Start(Seconds(1.0));
    acServerApp.Start(Seconds(1.0));
    lightServerApp.Start(Seconds(1.0));
    phoneServerApp.Start(Seconds(1.0));
   
    
    Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
    

    
    for(int i = 0; i < 2; i++){
    	Ptr<MobilityModel> mob = smartTvNodes.Get(i)->GetObject<MobilityModel>();
    	double x = mob->GetPosition().x;
    	double y = mob->GetPosition().y;
    	
    	anim.SetConstantPosition(smartTvNodes.Get(i),x,y);
    }
    
    for(int i = 0; i < 18; i++){
    	Ptr<MobilityModel> mob = smartLightNodes.Get(i)->GetObject<MobilityModel>();
    	double x = mob->GetPosition().x;
    	double y = mob->GetPosition().y;
    	
    	anim.SetConstantPosition(smartLightNodes.Get(i),x,y);
    }
    
    for(int i = 0; i < 4 ; i++){
    	Ptr<MobilityModel> mob = smartAcNodes.Get(i)->GetObject<MobilityModel>();
    	double x = mob->GetPosition().x;
    	double y = mob->GetPosition().y;
    	
    	anim.SetConstantPosition(smartAcNodes.Get(i),x,y);
    }
    
    for(int i = 0; i < 4; i++){
    	Ptr<MobilityModel> mob = smartPhoneNodes.Get(i)->GetObject<MobilityModel>();
    	double x = mob->GetPosition().x;
    	double y = mob->GetPosition().y;
    	
    	anim.SetConstantPosition(smartPhoneNodes.Get(i),x,y);
    }


    
    Simulator::Stop(Seconds(300));
    Simulator :: Run();
    

    
    throughputFile.close();
    
    auto averageThroughput =
        (static_cast<double>(sink->GetTotalRx() * 8 * 1e3) / simulationTime.GetMicroSeconds());
    
    std::cout << "\nAverage throughput: " << averageThroughput << " Kbit/s" << std::endl;
    
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
