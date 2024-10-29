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
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/applications-module.h"
#include "ns3/wifi-phy-common.h"
#include "ns3/wifi-module.h"


#include <fstream>

NS_LOG_COMPONENT_DEFINE("proj");
enum e_WifiPhyRxFailureReason {
  UNKNOWN,
  UNSUPPORTED_SETTINGS,
  CHANNEL_SWITCHING,
  RXING,
  TXING,
  SLEEPING,
  BUSY_DECODING_PREAMBLE,
  PREAMBLE_DETECT_FAILURE,
  RECEPTION_ABORTED_BY_TX,
  L_SIG_FAILURE,
  HT_SIG_FAILURE,
  SIG_A_FAILURE,
  SIG_B_FAILURE,
  PREAMBLE_CAPTURE_PACKET_SWITCH,
  FRAME_CAPTURE_PACKET_SWITCH,
  OBSS_PD_CCA_RESET,
  HE_TB_PPDU_TOO_LATE,
  FILTERED
};
using namespace ns3;

Ptr<PacketSink> sink;
uint64_t lastTotalRx = 0;
std::ofstream throughputFile;
double totalEnergyConsumed = 0.0;
std::vector<double> nodeEnergyConsumed;
int number_of_vehicles = 75;
int sink_count= 1;


std::string fileName = "_75_1.txt";
std::ofstream outputFile;

void
CalculateThroughput()
{
    Time now = Simulator::Now(); /* Return the simulator's virtual time. */
    double cur = (sink->GetTotalRx() - lastTotalRx) * 8.0 /(5*1e6); /* Convert Application RX Packets to MBits. */
    outputFile << now.GetSeconds() << "\t" << cur << std::endl;
    lastTotalRx = sink->GetTotalRx();
    std::cout << now.GetSeconds();
    Simulator::Schedule(MilliSeconds(5000), &CalculateThroughput);
}

void setVehicleMobility(NodeContainer smartVehicleNodes, double minx, double miny,double speedx,double speedy, double delx, double dely){
 
MobilityHelper smartVehicleMobility;
// First half with RowFirst layout
smartVehicleMobility.SetMobilityModel("ns3::ConstantVelocityMobilityModel");
smartVehicleMobility.SetPositionAllocator("ns3::GridPositionAllocator",
                                           "MinX", DoubleValue(minx),
                                           "MinY", DoubleValue(miny),
                                           "DeltaX", DoubleValue(10.0),
                                           "DeltaY", DoubleValue(20.0),
                                           "GridWidth", UintegerValue(16),
                                           "LayoutType", StringValue("RowFirst"));
// Install the mobility models
smartVehicleMobility.Install(smartVehicleNodes);


// Assign velocities
for (int i = 0; i < number_of_vehicles / 2; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(speedx, speedy, 0.0));
}

for (int i = number_of_vehicles / 2; i < number_of_vehicles; i++) {
    Ptr<Node> node = smartVehicleNodes.Get(i);
    Ptr<ConstantVelocityMobilityModel> mob = node->GetObject<ConstantVelocityMobilityModel>();
    mob->SetVelocity(Vector(-speedx, speedy, 0.0));
}
}


void RxDropCallback(Ptr<const Packet> p, WifiPhyRxfailureReason reason) {
    NS_LOG_UNCOND("Packet dropped at physical layer: Reason = " << reason << "payload size: " << p->GetSize());
    // Here, you can inspect the packet's properties, such as size, payload, etc.

}

int
main(int argc, char *argv[]){
    std::string tcpVariant{"TcpWestwoodPlus"}; /* TCP variant type. */
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
    
    outputFile.open("newdata.txt");

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
a
    NodeContainer smartVehicleNodes;
    smartVehicleNodes.Create(number_of_vehicles);
 
    Ssid ssid = Ssid("network");
    

    wifiMac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice;
    apDevice = wifiHelper.Install(wifiPhy, wifiMac, apWifiNode);
    
    wifiMac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid));

    // Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/Phy/PhyRxDrop", MakeCallback (&RxDrop));
    Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/Phy/PhyRxDrop", MakeCallback(&RxDropCallback));
    
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
    largePktServer.SetAttribute("PacketSize", UintegerValue(1500));
    largePktServer.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    largePktServer.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=25]"));
    largePktServer.SetAttribute("DataRate", DataRateValue(DataRate("20Mb/s")));
    ApplicationContainer largePktServerApp = largePktServer.Install(smartVehicleNodes);
    
    sinkApp.Start(Seconds(0.0));


    smallPktServerApp.Start(Seconds(1.1));
    midPktServerApp.Start(Seconds(1.2));
    largePktServerApp.Start(Seconds(1.3));
    
    Simulator::Schedule(Seconds(0.1), &CalculateThroughput);
    
    Simulator::Stop(simulationTime);
    Simulator :: Run();

    outputFile.close();
    return 0;
}
