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
#include "ns3/energy-module.h"
#include "ns3/wifi-radio-energy-model-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/three-gpp-propagation-loss-model.h"


#include <fstream>

NS_LOG_COMPONENT_DEFINE("proj");

using namespace ns3;

Ptr<PacketSink> sink;
Ptr<PacketSink> sink2;
Ptr<PacketSink> sink3;
uint64_t lastTotalRx = 0;
std::ofstream throughputFile;
double totalEnergyConsumed = 0.0;
std::vector<double> nodeEnergyConsumed;
int number_of_vehicles = 30;


void
CalculateThroughput()
{
    Time now = Simulator::Now(); /* Return the simulator's virtual time. */
    double cur = (sink->GetTotalRx() - lastTotalRx) * 8.0 /1e6; /* Convert Application RX Packets to MBits. */
                
    throughputFile << now.GetSeconds() << "\t" << cur << std::endl;
    

    lastTotalRx = sink->GetTotalRx();
    Simulator::Schedule(MilliSeconds(1000), &CalculateThroughput);
}
void CalculateEnergyConsumption(NodeContainer smartVehicleNodes) {
    totalEnergyConsumed = 0.0;
    for (size_t i = 0; i < nodeEnergyConsumed.size(); ++i) {
        Ptr<ns3::energy::BasicEnergySource> energySource = DynamicCast<ns3::energy::BasicEnergySource>(smartVehicleNodes.Get(i)->GetObject<ns3::energy::EnergySourceContainer>()->Get(0));
        if (energySource) {
            nodeEnergyConsumed[i] = energySource->GetInitialEnergy() - energySource->GetRemainingEnergy();
            totalEnergyConsumed += nodeEnergyConsumed[i];
        } else {
            NS_LOG_WARN("Energy source not found for node " << i);
        }
    }
    Simulator::Schedule(Seconds(1.0), &CalculateEnergyConsumption, smartVehicleNodes);
}

void setVehicleMobility(NodeContainer smartVehicleNodes, double minx, double miny,double speedx,double speedy, std::string row_col){
    MobilityHelper smartVehicleMobility;
    
    smartVehicleMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
    
    smartVehicleMobility.SetPositionAllocator("ns3::GridPositionAllocator","MinX",DoubleValue(minx),
    "MinY",DoubleValue(miny),"DeltaX",DoubleValue(10.0),"DeltaY",DoubleValue(20.0),"GridWidth",UintegerValue(15),
    "LayoutType",StringValue(row_col));
    
    
    smartVehicleMobility.Install(smartVehicleNodes);
    for(int i = 0; i < number_of_vehicles/2; i++){
    	Ptr<Node> node = smartVehicleNodes.Get(i);
    	Ptr<ConstantVelocityMobilityModel> mob = node->GetObject <ConstantVelocityMobilityModel>();
    	mob->SetVelocity(Vector(speedx,speedy,0.0));
    }
    
    for(int i  = number_of_vehicles/2; i < number_of_vehicles; i++){
    	Ptr<Node> node = smartVehicleNodes.Get(i);
    	Ptr<ConstantVelocityMobilityModel> mob = node->GetObject <ConstantVelocityMobilityModel>();
    	mob->SetVelocity(Vector(-speedx,speedy,0.0));
    }
}


int
main(int argc, char *argv[]){
    std::string tcpVariant{"TcpLedbat"}; /* TCP variant type. */
    std::string phyRate{"HtMcs7"};        /* Physical layer bitrate. */
    Time simulationTime{"10s"};           /* Simulation time. */

    /* Command line argument parser setup. */
    CommandLine cmd(__FILE__);
    cmd.AddValue("tcpVariant",
                 "Transport protocol to use: TcpNewReno, "
                 "TcpHybla, TcpHighSpeed, TcpHtcp, TcpVegas, TcpScalable, TcpVeno, "
                 "TcpBic, TcpYeah, TcpIllinois, TcpWestwood, TcpWestwoodPlus, TcpLedbat ",
                 tcpVariant);
    cmd.AddValue("phyRate", "Physical layer bitrate", phyRate);
    cmd.AddValue("simulationTime", "Simulation time in seconds", simulationTime);
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
    // Config::SetDefault("ns3::TcpSocket::SegmentSize", UintegerValue(payloadSize));
    
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
    apWifiNode.Create(3);

    NodeContainer smartVehicleNodes;
    smartVehicleNodes.Create(number_of_vehicles);

    NodeContainer smartVehicleNodes2;
    smartVehicleNodes2.Create(number_of_vehicles);

    NodeContainer smartVehicleNodes3;
    smartVehicleNodes3.Create(number_of_vehicles);
 
    Ssid ssid = Ssid("network");


    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);
    
    wifiMac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid));
    
    NetDeviceContainer smartVehicleDevices;
    smartVehicleDevices = wifiHelper.Install(wifiPhy,wifiMac,smartVehicleNodes);

    NetDeviceContainer smartVehicleDevices2;
    smartVehicleDevices2 = wifiHelper.Install(wifiPhy,wifiMac,smartVehicleNodes2);

    NetDeviceContainer smartVehicleDevices3;
    smartVehicleDevices3 = wifiHelper.Install(wifiPhy,wifiMac,smartVehicleNodes3);
    

    
    MobilityHelper apMobility;
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(0.0,10.0,0.0));
    apPositionAlloc->Add(Vector(0.0,610.0,0.0));
    apPositionAlloc->Add(Vector(580.0,210.0,0.0));
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.SetPositionAllocator(apPositionAlloc);
    apMobility.Install(apWifiNode);
    
    setVehicleMobility(smartVehicleNodes,0.0,0.0,1.0,0.0, "RowFirst");
    setVehicleMobility(smartVehicleNodes2,0.0,600.0,1.0,0.0, "RowFirst");
    setVehicleMobility(smartVehicleNodes3,570,200.0,0.0,1.0, "ColumnFirst");
   
    
    InternetStackHelper stack;
    stack.Install(apWifiNode);
    stack.Install(smartVehicleNodes);
    stack.Install(smartVehicleNodes2);
    stack.Install(smartVehicleNodes3);
    
  
    
    Ipv4AddressHelper address;
    address.SetBase("192.168.0.0", "255.255.0.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer smartVehicleInterface;
    smartVehicleInterface = address.Assign(smartVehicleDevices);
    Ipv4InterfaceContainer smartVehicleInterface2;
    smartVehicleInterface2 = address.Assign(smartVehicleDevices2);
    Ipv4InterfaceContainer smartVehicleInterface3;
    smartVehicleInterface3 = address.Assign(smartVehicleDevices3);
    
    

    
    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",InetSocketAddress(InetSocketAddress(Ipv4Address::GetAny(), 9)));
    ApplicationContainer sinkApp = sinkHelper.Install(apWifiNode.Get(0));
    sink = StaticCast<PacketSink>(sinkApp.Get(0));
    ApplicationContainer sinkApp2 = sinkHelper.Install(apWifiNode.Get(1));
    sink2 = StaticCast<PacketSink>(sinkApp2.Get(0));
    ApplicationContainer sinkApp3 = sinkHelper.Install(apWifiNode.Get(2));
    sink3 = StaticCast<PacketSink>(sinkApp3.Get(0));

    //cluster1 
    OnOffHelper smallPktServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    smallPktServer.SetAttribute("PacketSize", UintegerValue(100));
    smallPktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer.SetAttribute("DataRate", DataRateValue(DataRate("100Kb/s")));
    ApplicationContainer smallPktServerApp = smallPktServer.Install(smartVehicleNodes);

    
    OnOffHelper midPktServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    midPktServer.SetAttribute("PacketSize", UintegerValue(200));
    midPktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    midPktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=10]"));
    midPktServer.SetAttribute("DataRate", DataRateValue(DataRate("2Mb/s")));
    ApplicationContainer midPktServerApp = midPktServer.Install(smartVehicleNodes);

    
    OnOffHelper largePktServer("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(0), 9)));
    largePktServer.SetAttribute("PacketSize", UintegerValue(3000));
    largePktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    largePktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    largePktServer.SetAttribute("DataRate", DataRateValue(DataRate("100Mb/s")));
    ApplicationContainer largePktServerApp = largePktServer.Install(smartVehicleNodes);

    //cluster 2 
    OnOffHelper smallPktServer2("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(1), 9)));
    smallPktServer2.SetAttribute("PacketSize", UintegerValue(100));
    smallPktServer2.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer2.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer2.SetAttribute("DataRate", DataRateValue(DataRate("100Kb/s")));
    ApplicationContainer smallPktServerApp2 = smallPktServer2.Install(smartVehicleNodes2);
    
    OnOffHelper midPktServer2("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(1), 9)));
    midPktServer2.SetAttribute("PacketSize", UintegerValue(200));
    midPktServer2.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    midPktServer2.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=10]"));
    midPktServer2.SetAttribute("DataRate", DataRateValue(DataRate("2Mb/s")));
    ApplicationContainer midPktServerApp2 = midPktServer2.Install(smartVehicleNodes2);
    
    OnOffHelper largePktServer2("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(1), 9)));
    largePktServer2.SetAttribute("PacketSize", UintegerValue(3000));
    largePktServer2.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    largePktServer2.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    largePktServer2.SetAttribute("DataRate", DataRateValue(DataRate("100Mb/s")));
    ApplicationContainer largePktServerApp2 = largePktServer2.Install(smartVehicleNodes2);

    //cluster 3 
    OnOffHelper smallPktServer3("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(2), 9)));
    smallPktServer3.SetAttribute("PacketSize", UintegerValue(100));
    smallPktServer3.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer3.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer3.SetAttribute("DataRate", DataRateValue(DataRate("100Kb/s")));
    ApplicationContainer smallPktServerApp3 = smallPktServer3.Install(smartVehicleNodes3);
    
    OnOffHelper midPktServer3("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(2), 9)));
    midPktServer3.SetAttribute("PacketSize", UintegerValue(200));
    midPktServer3.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    midPktServer3.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=10]"));
    midPktServer3.SetAttribute("DataRate", DataRateValue(DataRate("2Mb/s")));
    ApplicationContainer midPktServerApp3 = midPktServer3.Install(smartVehicleNodes3);
    
    OnOffHelper largePktServer3("ns3::TcpSocketFactory", (InetSocketAddress(apInterface.GetAddress(2), 9)));
    largePktServer3.SetAttribute("PacketSize", UintegerValue(3000));
    largePktServer3.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    largePktServer3.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    largePktServer3.SetAttribute("DataRate", DataRateValue(DataRate("100Mb/s")));
    ApplicationContainer largePktServerApp3 = largePktServer2.Install(smartVehicleNodes3);
    
    
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    std :: string throughputFileName = "throughput/throughput_" + tcpName + std:: string {".txt"};
    
    throughputFile.open(throughputFileName);
    AnimationInterface anim("proj_netanim.xml");
    anim.SetMaxPktsPerTraceFile(3145728);
   
    
    Ptr<MobilityModel> apmob = apWifiNode.Get(0)->GetObject<MobilityModel>();
    double apx = apmob->GetPosition().x;
    double apy = apmob->GetPosition().y;
    
    anim.SetConstantPosition(apWifiNode.Get(0),apx,apy);
    anim.UpdateNodeColor(apWifiNode.Get(0),0,0,255);
    
    
    sinkApp.Start(Seconds(0.0));
    sinkApp2.Start(Seconds(0.0));
    smallPktServerApp.Start(Seconds(1.1));
    midPktServerApp.Start(Seconds(1.2));
    largePktServerApp.Start(Seconds(1.3));
    smallPktServerApp2.Start(Seconds(1.1));
    midPktServerApp2.Start(Seconds(1.1));
    largePktServerApp2.Start(Seconds(1.1));
    smallPktServerApp3.Start(Seconds(1.1));
    midPktServerApp3.Start(Seconds(1.1));
    largePktServerApp3.Start(Seconds(1.1));
    
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(1000.0));
    basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(12.0));
    
    WifiRadioEnergyModelHelper radioEnergyHelper;
    
    radioEnergyHelper.Set("TxCurrentA",DoubleValue(0.017));
    radioEnergyHelper.Set("RxCurrentA",DoubleValue(0.0197));
    radioEnergyHelper.Set("IdleCurrentA",DoubleValue(0.273));
    radioEnergyHelper.Set("SleepCurrentA",DoubleValue(0.033));
    
    ns3::energy::EnergySourceContainer sources = basicSourceHelper.Install(smartVehicleNodes);
    ns3::energy::DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(smartVehicleDevices, sources);


    ns3::energy::EnergySourceContainer sources2 = basicSourceHelper.Install(smartVehicleNodes2);
    ns3::energy::DeviceEnergyModelContainer deviceModels2 = radioEnergyHelper.Install(smartVehicleDevices2, sources2);


    ns3::energy::EnergySourceContainer sources3 = basicSourceHelper.Install(smartVehicleNodes3);
    ns3::energy::DeviceEnergyModelContainer deviceModels3 = radioEnergyHelper.Install(smartVehicleDevices3, sources3);

    nodeEnergyConsumed.resize(smartVehicleNodes.GetN(), 0.0);

    Simulator::Schedule(Seconds(1.0), &CalculateEnergyConsumption,smartVehicleNodes);
   
    
    Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
    
    
    
    for(int i = 0; i < number_of_vehicles; i++){
    	Ptr<MobilityModel> mob = smartVehicleNodes.Get(i)->GetObject<MobilityModel>();
    	double x1 = mob->GetPosition().x;
    	double y1 = mob->GetPosition().y;
    	anim.SetConstantPosition(smartVehicleNodes.Get(i),x1,y1);
        anim.UpdateNodeColor(smartVehicleNodes.Get(i),0,255,0);

        double x2 = mob->GetPosition().x;
    	double y2 = mob->GetPosition().y;
    	anim.SetConstantPosition(smartVehicleNodes2.Get(i),x2,y2);
        anim.UpdateNodeColor(smartVehicleNodes2.Get(i),0,255,0);

        double x3 = mob->GetPosition().x;
    	double y3 = mob->GetPosition().y;
    	anim.SetConstantPosition(smartVehicleNodes3.Get(i),x3,y3);
        anim.UpdateNodeColor(smartVehicleNodes3.Get(i),0,255,0);

    }
    
    
    Simulator::Stop(simulationTime);
    Simulator :: Run();
    
	double averageEnergyConsumption = totalEnergyConsumed / smartVehicleNodes.GetN();
    std::cout << "Average energy consumption: " << averageEnergyConsumption << " J" << std::endl;
    
    throughputFile.close();
    
    auto averageThroughput =
        (static_cast<double>(sink->GetTotalRx() * 8  ) / simulationTime.GetMicroSeconds());
    
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
    
    std::string energyStats = "energystats/energy_"+tcpName+".txt";
    
    std::ofstream energyFile;
    energyFile.open(energyStats);
    
    for (size_t i = 0; i < nodeEnergyConsumed.size(); ++i) {
    	double power_consumed = nodeEnergyConsumed[i] / 10;
        energyFile << "Node " << i << ": " << power_consumed << " W" << std::endl;
    }
    energyFile.close();

  
    
    return 0;
}
