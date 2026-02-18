#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/wave-module.h"
#include "ns3/applications-module.h"

#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <cstring>

using namespace ns3;

struct MobilityRow {
  double t;
  uint32_t nodeId;
  double x, y, v, psi;
  int laneIdx;
};

struct IntentRow {
  double t;
  uint32_t sender;
  int targetLane;
};

static std::vector<MobilityRow> ReadMobilityCsv(const std::string &path) {
  std::vector<MobilityRow> rows;
  std::ifstream f(path.c_str());
  if (!f.is_open()) { NS_FATAL_ERROR("Cannot open mobility trace: " << path); }
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    MobilityRow r{};
    std::getline(ss, tok, ','); r.t = std::stod(tok);
    std::getline(ss, tok, ','); r.nodeId = (uint32_t)std::stoul(tok);
    std::getline(ss, tok, ','); r.x = std::stod(tok);
    std::getline(ss, tok, ','); r.y = std::stod(tok);
    std::getline(ss, tok, ','); r.v = std::stod(tok);
    std::getline(ss, tok, ','); r.psi = std::stod(tok);
    std::getline(ss, tok, ','); r.laneIdx = std::stoi(tok);
    rows.push_back(r);
  }
  return rows;
}

static std::vector<IntentRow> ReadIntentCsv(const std::string &path) {
  std::vector<IntentRow> rows;
  std::ifstream f(path.c_str());
  if (!f.is_open()) { return rows; }
  std::string line;
  std::getline(f, line);
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::stringstream ss(line);
    std::string tok;
    IntentRow r{};
    std::getline(ss, tok, ','); r.t = std::stod(tok);
    std::getline(ss, tok, ','); r.sender = (uint32_t)std::stoul(tok);
    std::getline(ss, tok, ','); r.targetLane = std::stoi(tok);
    rows.push_back(r);
  }
  return rows;
}

static std::unordered_map<uint32_t, double> gV;
static std::unordered_map<uint32_t, double> gPsi;
static std::unordered_map<uint32_t, int> gLane;

class TraceMobilityPlayer {
public:
  TraceMobilityPlayer(NodeContainer nodes, std::vector<MobilityRow> rows)
      : m_nodes(nodes), m_rows(std::move(rows)) {}
  void Start() {
    for (const auto &r : m_rows) {
      Simulator::Schedule(Seconds(r.t), &TraceMobilityPlayer::Apply, this,
                          r.nodeId, r.x, r.y, r.v, r.psi, r.laneIdx);
    }
  }
private:
  void Apply(uint32_t nodeId, double x, double y, double v, double psi, int laneIdx) {
    Ptr<Node> n = m_nodes.Get(nodeId);
    Ptr<MobilityModel> mm = n->GetObject<MobilityModel>();
    Ptr<ConstantPositionMobilityModel> c = DynamicCast<ConstantPositionMobilityModel>(mm);
    if (c) c->SetPosition(Vector(x, y, 0.0));
    gV[nodeId] = v;
    gPsi[nodeId] = psi;
    gLane[nodeId] = laneIdx;
  }
  NodeContainer m_nodes;
  std::vector<MobilityRow> m_rows;
};

class BeaconIntentApp : public Application {
public:
  void Configure(uint16_t port, double hz, const std::string &rxLogPath, const std::string &txLogPath) {
    m_port = port; m_hz = hz; m_rxLogPath = rxLogPath; m_txLogPath = txLogPath;
  }
  void ScheduleIntents(const std::vector<IntentRow> &intents) {
    for (const auto &it : intents) {
      Simulator::Schedule(Seconds(it.t), &BeaconIntentApp::SendIntent, this, it.targetLane);
    }
  }
private:
  static constexpr uint32_t PAYLOAD_SIZE = 1 + 4 + 8 + 8 + 8 + 8 + 8 + 4 + 4;

  void StartApplication() override {
    m_socket = Socket::CreateSocket(GetNode(), UdpSocketFactory::GetTypeId());
    m_socket->SetAllowBroadcast(true);
    m_socket->Bind(InetSocketAddress(Ipv4Address::GetAny(), m_port));
    m_socket->SetRecvCallback(MakeCallback(&BeaconIntentApp::HandleRead, this));
    m_bcast = InetSocketAddress(Ipv4Address("255.255.255.255"), m_port);

    m_rxLog.open(m_rxLogPath.c_str(), std::ios::out | std::ios::app);
    if (m_rxLog.tellp() == 0) {
      m_rxLog << "t_tx,t_rx,sender_id,receiver_id,msg_type,size_bytes,dropped,x,y,v,psi,lane_idx,target_lane_idx\n";
    }
    m_txLog.open(m_txLogPath.c_str(), std::ios::out | std::ios::app);
    if (m_txLog.tellp() == 0) {
      m_txLog << "t_tx,sender_id,msg_type,lane_idx,target_lane_idx,size_bytes\n";
    }
    Simulator::Schedule(Seconds(0.1), &BeaconIntentApp::SendCam, this);
  }

  void StopApplication() override {
    if (m_camEvent.IsRunning()) Simulator::Cancel(m_camEvent);
    if (m_socket) m_socket->Close();
    if (m_rxLog.is_open()) m_rxLog.close();
    if (m_txLog.is_open()) m_txLog.close();
  }

  void BuildAndSend(uint8_t msgType, int targetLaneIdx) {
    uint32_t sender = GetNode()->GetId();
    double tTx = Simulator::Now().GetSeconds();

    Ptr<MobilityModel> mm = GetNode()->GetObject<MobilityModel>();
    Vector p = mm->GetPosition();
    double x = p.x, y = p.y;

    double v = 0.0, psi = 0.0;
    int laneIdx = -1;
    if (gV.find(sender) != gV.end()) v = gV[sender];
    if (gPsi.find(sender) != gPsi.end()) psi = gPsi[sender];
    if (gLane.find(sender) != gLane.end()) laneIdx = gLane[sender];

    uint8_t buf[PAYLOAD_SIZE];
    std::memset(buf, 0, PAYLOAD_SIZE);
    buf[0] = msgType;
    std::memcpy(buf + 1, &sender, 4);
    std::memcpy(buf + 5, &tTx, 8);
    std::memcpy(buf + 13, &x, 8);
    std::memcpy(buf + 21, &y, 8);
    std::memcpy(buf + 29, &v, 8);
    std::memcpy(buf + 37, &psi, 8);
    int32_t lane32 = (int32_t)laneIdx;
    int32_t tgt32  = (int32_t)targetLaneIdx;
    std::memcpy(buf + 45, &lane32, 4);
    std::memcpy(buf + 49, &tgt32, 4);

    Ptr<Packet> pkt = Create<Packet>(buf, PAYLOAD_SIZE);
    m_socket->SendTo(pkt, 0, m_bcast);

    m_txLog << tTx << "," << sender << "," << int(msgType) << ","
            << laneIdx << "," << targetLaneIdx << "," << pkt->GetSize() << "\n";
  }

  void SendCam() {
    BuildAndSend(1, -1);
    m_camEvent = Simulator::Schedule(Seconds(1.0 / m_hz), &BeaconIntentApp::SendCam, this);
  }

  void SendIntent(int targetLaneIdx) {
    BuildAndSend(2, targetLaneIdx);
  }

  void HandleRead(Ptr<Socket> socket) {
    Address from;
    Ptr<Packet> p = socket->RecvFrom(from);
    double tRx = Simulator::Now().GetSeconds();

    uint8_t buf[PAYLOAD_SIZE];
    uint32_t n = p->CopyData(buf, PAYLOAD_SIZE);

    uint8_t msgType = 0;
    uint32_t sender = 0;
    double tTx = -1.0;
    double x=0,y=0,v=0,psi=0;
    int32_t laneIdx = -1;
    int32_t targetLane = -1;

    if (n >= PAYLOAD_SIZE) {
      msgType = buf[0];
      std::memcpy(&sender, buf + 1, 4);
      std::memcpy(&tTx,    buf + 5, 8);
      std::memcpy(&x,      buf + 13, 8);
      std::memcpy(&y,      buf + 21, 8);
      std::memcpy(&v,      buf + 29, 8);
      std::memcpy(&psi,    buf + 37, 8);
      std::memcpy(&laneIdx,   buf + 45, 4);
      std::memcpy(&targetLane,buf + 49, 4);
    }

    uint32_t receiver = GetNode()->GetId();
    m_rxLog << tTx << "," << tRx << "," << sender << "," << receiver << ","
            << int(msgType) << "," << p->GetSize() << ",0,"
            << x << "," << y << "," << v << "," << psi << ","
            << laneIdx << "," << targetLane << "\n";
  }

  Ptr<Socket> m_socket;
  Address m_bcast;
  EventId m_camEvent;
  std::ofstream m_rxLog;
  std::ofstream m_txLog;
  uint16_t m_port{4444};
  double m_hz{10.0};
  std::string m_rxLogPath{"out/ns3/packets.csv"};
  std::string m_txLogPath{"out/ns3/tx.csv"};
};

int main(int argc, char **argv) {
  std::string mobPath = "out/ns3/mobility_ns3.csv";
  std::string intentPath = "out/ns3/intent.csv";
  std::string rxLogPath = "out/ns3/packets.csv";
  std::string txLogPath = "out/ns3/tx.csv";

  uint32_t nNodes = 20;
  double simTime = 120.0;
  double hz = 10.0;

  CommandLine cmd;
  cmd.AddValue("mobPath", "Mobility trace csv", mobPath);
  cmd.AddValue("intentPath", "Intent schedule csv", intentPath);
  cmd.AddValue("rxLogPath", "Output RX log csv", rxLogPath);
  cmd.AddValue("txLogPath", "Output TX log csv", txLogPath);
  cmd.AddValue("nNodes", "Number of nodes", nNodes);
  cmd.AddValue("simTime", "Simulation time", simTime);
  cmd.AddValue("hz", "CAM rate (Hz)", hz);
  cmd.Parse(argc, argv);

  NodeContainer nodes;
  nodes.Create(nNodes);

  InternetStackHelper internet;
  internet.Install(nodes);

  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodes);

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default();
  YansWifiChannelHelper chan = YansWifiChannelHelper::Default();
  phy.SetChannel(chan.Create());

  QosWaveMacHelper mac = QosWaveMacHelper::Default();
  Wifi80211pHelper wifi = Wifi80211pHelper::Default();
  NetDeviceContainer devs = wifi.Install(phy, mac, nodes);

  Ipv4AddressHelper ipv4;
  ipv4.SetBase("10.1.0.0", "255.255.0.0");
  ipv4.Assign(devs);

  { std::ofstream clear(rxLogPath.c_str(), std::ios::out); }
  { std::ofstream clear(txLogPath.c_str(), std::ios::out); }

  auto mobRows = ReadMobilityCsv(mobPath);
  TraceMobilityPlayer player(nodes, mobRows);
  player.Start();

  auto intents = ReadIntentCsv(intentPath);

  for (uint32_t i = 0; i < nNodes; i++) {
    Ptr<BeaconIntentApp> app = CreateObject<BeaconIntentApp>();
    app->Configure(4444, hz, rxLogPath, txLogPath);

    std::vector<IntentRow> mine;
    for (const auto &it : intents) {
      if (it.sender == i) mine.push_back(it);
    }
    app->ScheduleIntents(mine);

    nodes.Get(i)->AddApplication(app);
    app->SetStartTime(Seconds(0.05));
    app->SetStopTime(Seconds(simTime));
  }

  Simulator::Stop(Seconds(simTime));
  Simulator::Run();
  Simulator::Destroy();
  return 0;
}
