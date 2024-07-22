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
uint64_t lastTotalRx = 0;
std::ofstream throughputFile;
double totalEnergyConsumed = 0.0;
std::vector<double> nodeEnergyConsumed;
int number_of_vehicles = 32;
int sink_count= 1;

std::string fileName = "_32_1.txt";


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

void setVehicleMobility(NodeContainer smartVehicleNodes, double minx, double miny,double speedx,double speedy, double delx, double dely){
 MobilityHelper smartVehicleMobility1;
MobilityHelper smartVehicleMobility2;


    NodeContainer rowFirstNodes;
    NodeContainer columnFirstNodes;
    for (int i = 0; i < number_of_vehicles / 2; ++i) {
        rowFirstNodes.Add(smartVehicleNodes.Get(i));
    }
    for (int i = number_of_vehicles / 2; i < number_of_vehicles; ++i) {
        columnFirstNodes.Add(smartVehicleNodes.Get(i));
    }

// First half with RowFirst layout
smartVehicleMobility1.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
smartVehicleMobility1.SetPositionAllocator("ns3::GridPositionAllocator",
                                           "MinX", DoubleValue(minx),
                                           "MinY", DoubleValue(miny),
                                           "DeltaX", DoubleValue(20.0),
                                           "DeltaY", DoubleValue(20.0),
                                           "GridWidth", UintegerValue(8),
                                           "LayoutType", StringValue("RowFirst"));

// Second half with ColumnFirst layout
smartVehicleMobility2.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
smartVehicleMobility2.SetPositionAllocator("ns3::GridPositionAllocator",
                                           "MinX", DoubleValue(minx),
                                           "MinY", DoubleValue(miny),
                                           "DeltaX", DoubleValue(20.0),
                                           "DeltaY", DoubleValue(20.0),
                                           "GridWidth", UintegerValue(8),
                                           "LayoutType", StringValue("ColumnFirst"));

// Install the mobility models
smartVehicleMobility1.Install(rowFirstNodes);
smartVehicleMobility2.Install(columnFirstNodes);


// Assign velocities
for (int i = 0; i < number_of_vehicles / 4; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(speedx, speedy, 0.0));
}

for (int i = number_of_vehicles / 4; i < number_of_vehicles/2; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(-speedx, speedy, 0.0));
}
for (int i = number_of_vehicles/2; i < 3 *number_of_vehicles / 4; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(speedy, speedx, 0.0));
}

for (int i = 3*number_of_vehicles / 4; i < number_of_vehicles; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(speedy, -speedx, 0.0));
}

}


int
main(int argc, char *argv[]){
    std::string tcpVariant{"TcpLedbat"}; /* TCP variant type. */
    std::string phyRate{"HtMcs7"};        /* Physical layer bitrate. */
    Time simulationTime{"300s"};           /* Simulation time. */

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

    wifiChannel.AddPropagationLoss ("ns3::NakagamiPropagationLossModel", "m0", DoubleValue (1.5),
                                    "m1", DoubleValue (1.0),
                                    "m2", DoubleValue (0.75),
                                    "Distance1", DoubleValue (100.0),
                                    "Distance2", DoubleValue (300.0));

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
    
    NodeContainer sinkNodes;
    sinkNodes.Create(sink_count);

    NodeContainer smartVehicleNodes;
    smartVehicleNodes.Create(number_of_vehicles);
 
    Ssid ssid = Ssid("network");


    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);
    
    wifiMac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid));
    
    NetDeviceContainer smartVehicleDevices;
    smartVehicleDevices = wifiHelper.Install(wifiPhy,wifiMac,smartVehicleNodes);
    
    NetDeviceContainer sinkDevices;
    sinkDevices = wifiHelper.Install(wifiPhy,wifiMac,sinkNodes);
    
    MobilityHelper apMobility;
    Ptr<ListPositionAllocator> apPositionAlloc = CreateObject<ListPositionAllocator>();
    apPositionAlloc->Add(Vector(0.0,10.0,0.0));
    apMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMobility.SetPositionAllocator(apPositionAlloc);
    apMobility.Install(apWifiNode);
    
    MobilityHelper sinkMobility;
    Ptr<ListPositionAllocator> sinkPositionAlloc = CreateObject<ListPositionAllocator>();
    sinkPositionAlloc->Add(Vector(10.0,10.0,0.0));
    sinkMobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    sinkMobility.SetPositionAllocator(sinkPositionAlloc);
    sinkMobility.Install(sinkNodes);
    
    setVehicleMobility(smartVehicleNodes,0.0,0.0,1.0,0.0, 20.0, 10.0);
     
    AodvHelper aodv;
    InternetStackHelper stack;
    stack.SetRoutingHelper(aodv);
    stack.Install(sinkNodes);
    stack.Install(apWifiNode);
    stack.Install(smartVehicleNodes);
    
    Ipv4AddressHelper address;
    address.SetBase("192.168.0.0", "255.255.0.0");
    Ipv4InterfaceContainer apInterface;
    apInterface = address.Assign(apDevice);
    Ipv4InterfaceContainer sinkInterface;
    sinkInterface = address.Assign(sinkDevices);
    Ipv4InterfaceContainer smartVehicleInterface;
    smartVehicleInterface = address.Assign(smartVehicleDevices); 
    

    Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    
    PacketSinkHelper sinkHelper("ns3::TcpSocketFactory",InetSocketAddress(InetSocketAddress(Ipv4Address::GetAny(), 9)));
    ApplicationContainer sinkApp = sinkHelper.Install(sinkNodes.Get(0));
    sink = StaticCast<PacketSink>(sinkApp.Get(0));
    /*ApplicationContainer sinkApp2 = sinkHelper.Install(apWifiNode.Get(1));
    sink2 = StaticCast<PacketSink>(sinkApp2.Get(0));
    ApplicationContainer sinkApp3 = sinkHelper.Install(apWifiNode.Get(2));
    sink3 = StaticCast<PacketSink>(sinkApp3.Get(0));
*/
    //cluster1 
    OnOffHelper smallPktServer("ns3::TcpSocketFactory", (InetSocketAddress(sinkInterface.GetAddress(0), 9)));
    smallPktServer.SetAttribute("PacketSize", UintegerValue(100));
    smallPktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    smallPktServer.SetAttribute("DataRate", DataRateValue(DataRate("100Kb/s")));
    ApplicationContainer smallPktServerApp = smallPktServer.Install(smartVehicleNodes);

    
    OnOffHelper midPktServer("ns3::TcpSocketFactory", (InetSocketAddress(sinkInterface.GetAddress(0), 9)));
    midPktServer.SetAttribute("PacketSize", UintegerValue(200));
    midPktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    midPktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=10]"));
    midPktServer.SetAttribute("DataRate", DataRateValue(DataRate("2Mb/s")));
    ApplicationContainer midPktServerApp = midPktServer.Install(smartVehicleNodes);

    
    OnOffHelper largePktServer("ns3::TcpSocketFactory", (InetSocketAddress(sinkInterface.GetAddress(0), 9)));
    largePktServer.SetAttribute("PacketSize", UintegerValue(3000));
    largePktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    largePktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    largePktServer.SetAttribute("DataRate", DataRateValue(DataRate("100Mb/s")));
    ApplicationContainer largePktServerApp = largePktServer.Install(smartVehicleNodes);
    
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();
    
    std :: string throughputFileName = "throughput/throughput_" + tcpName + fileName;
    
    throughputFile.open(throughputFileName);
    AnimationInterface anim("proj_netanim.xml");
    anim.SetMaxPktsPerTraceFile(3145728);
   
    
    Ptr<MobilityModel> apmob = apWifiNode.Get(0)->GetObject<MobilityModel>();
    double apx = apmob->GetPosition().x;
    double apy = apmob->GetPosition().y;
    
    
    anim.SetConstantPosition(apWifiNode.Get(0),apx,apy);
    anim.UpdateNodeColor(apWifiNode.Get(0),0,0,255);

    Ptr<MobilityModel> sinkMob = sinkNodes.Get(0)->GetObject<MobilityModel>();
    double sink_x = sinkMob->GetPosition().x;
    double sink_y = sinkMob->GetPosition().y;
    anim.SetConstantPosition(sinkNodes.Get(0),sink_x,sink_y);
    anim.UpdateNodeColor(sinkNodes.Get(0),100,100,100);
    
    
    sinkApp.Start(Seconds(0.0));


    smallPktServerApp.Start(Seconds(1.1));
    midPktServerApp.Start(Seconds(1.2));
    largePktServerApp.Start(Seconds(1.3));
    
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

    nodeEnergyConsumed.resize(smartVehicleNodes.GetN(), 0.0);

    Simulator::Schedule(Seconds(1.0), &CalculateEnergyConsumption, smartVehicleNodes);
   
    
    Simulator::Schedule(Seconds(1.1), &CalculateThroughput);
    
    
    
    for(int i = 0; i < number_of_vehicles; i++){
    	Ptr<MobilityModel> mob = smartVehicleNodes.Get(i)->GetObject<MobilityModel>();
    	double x1 = mob->GetPosition().x;
    	double y1 = mob->GetPosition().y;
    	anim.SetConstantPosition(smartVehicleNodes.Get(i),x1,y1);
        anim.UpdateNodeColor(smartVehicleNodes.Get(i),0,255,0);
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
    
    std::string flowFileName = "flowstats/flow_stats_" + tcpName +fileName;
     
    flowStatsFile.open(flowFileName);
    
    int total_tx = 0;
    int total_rx = 0;
    double delaySum = 0;
    
    // Print Flow Monitor statistics for flows terminating at the sink
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        if (t.destinationAddress == apInterface.GetAddress (0))
        {
            total_tx+= i->second.txPackets;
            total_rx+= i->second.rxPackets;
            delaySum +=  i->second.timeLastRxPacket.GetSeconds()-i->second.timeFirstTxPacket.GetSeconds();
            flowStatsFile << i->first << "\t"
                          << t.sourceAddress << "\t"
                          << t.destinationAddress << "\t"
                          << i->second.txBytes << "\t"
                          << i->second.rxBytes << "\t"
                          << i->second.txPackets << "\t"
                          << i->second.rxPackets << "\t"
                          << i->second.lostPackets << "\t"
                          << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds()) / 1024 / 1024<< std::endl;
            
       
        }
    }
    
    
    

    flowStatsFile.close();
    
    std::ofstream delayFile;
    
    delayFile.open("flowstats/delay.txt",std::ios::app);
    
    delayFile << tcpName << "\t" << delaySum << std::endl;
    
    delayFile.close();
    
    std::ofstream tpFile;
    tpFile.open("throughput/avg.txt",std::ios::app);
    tpFile << averageThroughput << std::endl;
    
    tpFile.close();
    
    std ::ofstream pktStatsFile;
    
    pktStatsFile.open("packet_stats.txt",std::ios::app);
    
    pktStatsFile << tcpName << "\t" << total_tx << "\t" << total_rx << std::endl;
    
    pktStatsFile.close();
   
    Simulator :: Destroy();
    
    std::string energyStats = "energystats/energy_"+tcpName+".txt";
    
    std::ofstream energyFile;
    energyFile.open(energyStats);
    
    double avg_energy_sum = 0;
    for (size_t i = 0; i < nodeEnergyConsumed.size(); ++i) {
    	double power_consumed = nodeEnergyConsumed[i] / 300;
    	
    	avg_energy_sum += power_consumed;
        //energyFile << "Node " << i << ": " << power_consumed << " W" << std::endl;
    }
    
    energyFile << tcpName << "\t" << number_of_vehicles << "\t" << sink_count << "\t" << avg_energy_sum << std::endl; 
    energyFile.close();

  
    
    return 0;
}
