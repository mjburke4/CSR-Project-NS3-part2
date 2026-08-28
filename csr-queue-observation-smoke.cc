#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

struct TraceRow
{
  uint64_t eventIndex {0};
  double timeSeconds {0.0};
  std::string event;
  std::string node;
  std::string peer;
  std::string packetType;
  std::string source;
  std::string destination;
  std::string sequence;
  std::string sizeBytes;
  std::string success;
  std::string reason;
  std::string routeCost;
  std::string nextHop;
  std::string detail;
  std::string statistic;
  std::string value;
};

class TraceFile
{
public:
  explicit TraceFile (const std::string &label)
  {
    static const auto nonce =
      std::chrono::steady_clock::now ().time_since_epoch ().count ();
    static uint32_t ordinal = 0;
    m_path =
      (std::filesystem::temp_directory_path () /
       ("csr-queue-observation-smoke-" +
        std::to_string (nonce) + "-" +
        std::to_string (ordinal++) + "-" + label + ".csv"))
        .string ();
  }

  ~TraceFile ()
  {
    std::remove (m_path.c_str ());
  }

  const std::string &Path () const
  {
    return m_path;
  }

private:
  std::string m_path;
};

void
Require (bool condition, const std::string &message)
{
  if (!condition)
    {
      throw std::runtime_error (message);
    }
}

bool
NearlyEqual (double lhs, double rhs, double tolerance = 1e-9)
{
  return std::fabs (lhs - rhs) <= tolerance;
}

std::vector<std::string>
ParseCsvRow (const std::string &line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;

  for (std::size_t index = 0; index < line.size (); ++index)
    {
      char character = line[index];
      if (quoted)
        {
          if (character == '"')
            {
              if (index + 1 < line.size () && line[index + 1] == '"')
                {
                  field += '"';
                  ++index;
                }
              else
                {
                  quoted = false;
                }
            }
          else
            {
              field += character;
            }
          continue;
        }

      if (character == '"')
        {
          quoted = true;
        }
      else if (character == ',')
        {
          fields.push_back (field);
          field.clear ();
        }
      else
        {
          field += character;
        }
    }

  Require (!quoted, "unterminated quoted CSV field");
  fields.push_back (field);
  return fields;
}

std::vector<TraceRow>
ReadTrace (const std::string &path)
{
  std::ifstream stream (path);
  Require (stream.is_open (), "could not open queue-observation trace");

  std::string line;
  Require (static_cast<bool> (std::getline (stream, line)),
           "queue-observation trace has no header");
  std::vector<std::string> header = ParseCsvRow (line);

  std::map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size (); ++index)
    {
      columns[header[index]] = index;
    }

  for (const char *required : {"event_index",
                               "time_s",
                               "event",
                               "node",
                               "peer",
                               "packet_type",
                               "src",
                               "dst",
                               "sequence",
                               "size_bytes",
                               "success",
                               "reason",
                               "route_cost",
                               "next_hop",
                               "detail",
                               "statistic",
                               "value"})
    {
      Require (columns.count (required) == 1,
               std::string ("trace is missing column ") + required);
    }

  std::vector<TraceRow> rows;
  while (std::getline (stream, line))
    {
      if (line.empty ())
        {
          continue;
        }
      std::vector<std::string> fields = ParseCsvRow (line);
      Require (fields.size () == header.size (),
               "trace row has the wrong number of CSV fields");

      TraceRow row;
      row.eventIndex = std::stoull (fields[columns["event_index"]]);
      row.timeSeconds = std::stod (fields[columns["time_s"]]);
      row.event = fields[columns["event"]];
      row.node = fields[columns["node"]];
      row.peer = fields[columns["peer"]];
      row.packetType = fields[columns["packet_type"]];
      row.source = fields[columns["src"]];
      row.destination = fields[columns["dst"]];
      row.sequence = fields[columns["sequence"]];
      row.sizeBytes = fields[columns["size_bytes"]];
      row.success = fields[columns["success"]];
      row.reason = fields[columns["reason"]];
      row.routeCost = fields[columns["route_cost"]];
      row.nextHop = fields[columns["next_hop"]];
      row.detail = fields[columns["detail"]];
      row.statistic = fields[columns["statistic"]];
      row.value = fields[columns["value"]];
      rows.push_back (row);
    }
  return rows;
}

std::vector<TraceRow>
StatisticSamples (const std::vector<TraceRow> &rows)
{
  std::vector<TraceRow> samples;
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (samples),
                [] (const TraceRow &row) {
                  return row.event == "statistic_sample";
                });
  return samples;
}

std::vector<TraceRow>
RowsForStatistic (const std::vector<TraceRow> &rows,
                  const std::string &statistic)
{
  std::vector<TraceRow> samples;
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (samples),
                [&statistic] (const TraceRow &row) {
                  return row.event == "statistic_sample" &&
                         row.statistic == statistic;
                });
  return samples;
}

std::vector<TraceRow>
RowsForStatisticAtNode (const std::vector<TraceRow> &rows,
                        const std::string &statistic,
                        CsrNodeId node)
{
  std::vector<TraceRow> samples;
  const std::string nodeText = CsrTraceInteger (node);
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (samples),
                [&statistic, &nodeText] (const TraceRow &row) {
                  return row.event == "statistic_sample" &&
                         row.statistic == statistic &&
                         row.node == nodeText;
                });
  return samples;
}

std::vector<TraceRow>
RowsForEvent (const std::vector<TraceRow> &rows,
              const std::string &event)
{
  std::vector<TraceRow> selected;
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (selected),
                [&event] (const TraceRow &row) {
                  return row.event == event;
                });
  return selected;
}

std::vector<TraceRow>
RowsForEventAndSequence (const std::vector<TraceRow> &rows,
                         const std::string &event,
                         uint64_t sequence)
{
  std::vector<TraceRow> selected;
  const std::string sequenceText = CsrTraceInteger (sequence);
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (selected),
                [&event, &sequenceText] (const TraceRow &row) {
                  return row.event == event &&
                         row.sequence == sequenceText;
                });
  return selected;
}

std::vector<TraceRow>
RouteChangesForDestination (const std::vector<TraceRow> &rows,
                            CsrNodeId node,
                            CsrNodeId destination)
{
  std::vector<TraceRow> selected;
  const std::string nodeText = CsrTraceInteger (node);
  const std::string destinationText = CsrTraceInteger (destination);
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (selected),
                [&nodeText, &destinationText] (const TraceRow &row) {
                  return row.event == "route_change" &&
                         row.node == nodeText &&
                         row.destination == destinationText;
                });
  return selected;
}

double
SampleValue (const TraceRow &row)
{
  Require (!row.value.empty (), "statistic sample has no value");
  return std::stod (row.value);
}

void
StartTrace (const TraceFile &trace)
{
  SetDifferentialTraceAggregateOnly (false);
  OpenDifferentialTraceCsv (trace.Path ());
}

void
TestAggregateOnlyPathWriter ()
{
  TraceFile trace ("aggregate-path-writer");
  SetDifferentialTraceAggregateOnly (true);
  OpenDifferentialTraceCsv (trace.Path ());

  CsrDifferentialTraceEvent filtered;
  filtered.event = "tx_start";
  WriteDifferentialTrace (filtered);

  CsrDifferentialTraceEvent enqueue;
  enqueue.event = "nwk_enqueue";
  enqueue.node = "10";
  enqueue.source = "10";
  enqueue.destination = "30";
  enqueue.sequence = "7";
  enqueue.sizeBytes = "224";
  enqueue.reason = "local";
  WriteDifferentialTrace (enqueue);

  CsrDifferentialTraceEvent forward = enqueue;
  forward.event = "nwk_forward";
  forward.peer = "9";
  forward.nextHop = "20";
  forward.routeCost = "17";
  forward.detail = "num_hop=2;path=20>30";
  WriteDifferentialTrace (forward);

  CsrDifferentialTraceEvent route;
  route.event = "route_change";
  route.node = "10";
  route.destination = "30";
  route.nextHop = "20";
  route.routeCost = "17";
  route.detail = "num_hop=2;path=20>30";
  WriteDifferentialTrace (route);

  CloseDifferentialTraceCsv ();
  SetDifferentialTraceAggregateOnly (false);

  std::ifstream stream (trace.Path ());
  Require (stream.is_open (), "could not open aggregate-only path trace");
  std::string line;
  Require (static_cast<bool> (std::getline (stream, line)),
           "aggregate-only path trace has no header");
  const std::vector<std::string> expectedHeader = {
    "schema", "event_index", "time_s", "event", "src", "dst",
    "sequence", "size_bytes", "reason", "node", "statistic", "value",
    "peer", "next_hop", "route_cost", "detail",
  };
  const std::vector<std::string> header = ParseCsvRow (line);
  Require (header == expectedHeader,
           "aggregate-only path trace header is not the canonical compact schema");

  std::map<std::string, std::size_t> columns;
  for (std::size_t index = 0; index < header.size (); ++index)
    {
      columns[header[index]] = index;
    }
  std::vector<std::vector<std::string>> rows;
  while (std::getline (stream, line))
    {
      if (!line.empty ())
        {
          rows.push_back (ParseCsvRow (line));
        }
    }

  Require (rows.size () == 3,
           "aggregate-only writer retained a filtered event or lost a path event");
  for (const auto &row : rows)
    {
      Require (row.size () == header.size (),
               "aggregate-only writer emitted a malformed CSV row");
    }
  Require (rows[0][columns["event_index"]] == "0" &&
             rows[0][columns["event"]] == "nwk_enqueue" &&
             rows[1][columns["event_index"]] == "1" &&
             rows[1][columns["event"]] == "nwk_forward" &&
             rows[2][columns["event_index"]] == "2" &&
             rows[2][columns["event"]] == "route_change",
           "aggregate-only path events or indexes are not source ordered");
  Require (rows[1][columns["src"]] == "10" &&
             rows[1][columns["dst"]] == "30" &&
             rows[1][columns["sequence"]] == "7" &&
             rows[1][columns["peer"]] == "9" &&
             rows[1][columns["next_hop"]] == "20" &&
             rows[1][columns["route_cost"]] == "17" &&
             rows[1][columns["detail"]] == "num_hop=2;path=20>30",
           "aggregate-only writer dropped packet-path correlation fields");
}

Ptr<Packet>
BuildDataFrame (CsrNodeId source,
                CsrNodeId destination,
                uint16_t sequence,
                uint8_t dscp)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    dscp,
                    true,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetLinkControl (8, 0.0, 0.0);

  Ptr<Packet> frame = Create<Packet> (20);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildExactAck (CsrNodeId source,
               CsrNodeId destination,
               uint16_t sequence)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    0,
                    false,
                    true);
  header.SetType (CSR_PKT_ACK);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetLinkControl (8, 0.0, 0.0);

  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildWindowAck (CsrNodeId source,
                CsrNodeId destination,
                uint16_t sequence)
{
  Ptr<Packet> frame = BuildExactAck (source, destination, sequence);
  CsrHeader header;
  frame->RemoveHeader (header);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (1);
  header.SetDackBitmap (0);
  frame->AddHeader (header);
  return frame;
}

Time
NwkTic ()
{
  return Seconds (1.0 / 36.0e6);
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

void
ConnectNwkStack (Ptr<CsrNetDevice> device,
                 Ptr<CsrHopLayer> hop,
                 Ptr<CsrNetLayer> nwk,
                 CsrNodeId node)
{
  hop->SetNodeId (node);
  hop->SetMac (&device->GetMac ());
  nwk->SetNodeId (node);
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetAutomaticRoutePropagationEnabled (false);
  nwk->SetHop (hop);
  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

Ptr<Packet>
BuildRoutableHello (CsrNodeId source)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (1);
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (2);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

Ptr<Packet>
BuildTaggedPayload (uint32_t bytes, uint64_t sequence)
{
  Ptr<Packet> payload = Create<Packet> (bytes);
  payload->AddPacketTag (CsrDifferentialAppTag (sequence));
  return payload;
}

uint32_t g_nwkDeliveryCount = 0;
CsrNodeId g_expectedNwkSource = 0;

void
RecordNwkDelivery (Ptr<Packet>, CsrNodeId source)
{
  Require (source == g_expectedNwkSource,
           "NWK observation scenario delivered the wrong source");
  g_nwkDeliveryCount++;
}

void
RequireNwkSizeAndDelaySamples (const std::vector<TraceRow> &rows,
                               CsrNodeId node,
                               double expectedDelay,
                               const std::string &label)
{
  std::vector<TraceRow> sizes =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_SIZE, node);
  std::vector<TraceRow> delays =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_DELAY, node);

  Require (sizes.size () == 2 &&
             NearlyEqual (SampleValue (sizes[0]), 1.0) &&
             NearlyEqual (SampleValue (sizes[1]), 0.0),
           label + " did not emit the source-exact NWK size vector [1,0]");
  Require (delays.size () == 1 &&
             NearlyEqual (SampleValue (delays[0]), expectedDelay, 1e-12),
           label + " did not emit its source-exact NWK residence sample");
  Require (sizes[1].eventIndex < delays[0].eventIndex &&
             NearlyEqual (sizes[1].timeSeconds,
                          delays[0].timeSeconds,
                          1e-12),
           label + " did not write post-removal size before queue delay");
}

void
TestDirectHeldThenReleasedPath ()
{
  constexpr CsrNodeId sourceNode = 101;
  constexpr CsrNodeId destinationNode = 102;
  constexpr uint64_t appSequence = 7001;
  constexpr uint32_t payloadBytes = 24;
  const Time routeLearnTime = MilliSeconds (1);
  const double expectedResidence =
    (routeLearnTime + NwkTic ()).GetSeconds ();

  TraceFile trace ("nwk-direct-held");

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (destinationNode);
  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();

  sourceDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (sourceDevice);
  ConfigureNoErrors (sourceDevice);
  ConfigureNoErrors (destinationDevice);
  ConnectNwkStack (sourceDevice, sourceHop, sourceNwk, sourceNode);
  ConnectNwkStack (destinationDevice,
                   destinationHop,
                   destinationNwk,
                   destinationNode);
  sourceNwk->SetNodeType (CsrNodeType::Gateway);
  destinationNwk->SetNodeType (CsrNodeType::Routable);

  g_nwkDeliveryCount = 0;
  g_expectedNwkSource = sourceNode;
  destinationNwk->SetRxFromNetCallback (MakeCallback (&RecordNwkDelivery));

  StartTrace (trace);
  Ptr<Packet> payload = BuildTaggedPayload (payloadBytes, appSequence);
  sourceNwk->Send (destinationNode, 5, payload, true);

  Simulator::Schedule (MicroSeconds (500), [sourceNwk, sourceHop] () {
    Require (sourceNwk->GetNwkQueueSize () == 1,
             "route-less direct packet did not remain in NWK");
    Require (sourceHop->GetPendingDataCount () == 0,
             "route-less direct packet leaked into HOP");
  });
  Simulator::Schedule (routeLearnTime,
                       [sourceNwk, destinationNode] () {
                         sourceNwk->ProcessHello (
                           BuildRoutableHello (destinationNode),
                           destinationNode,
                           70.0,
                           30.0);
                       });

  Simulator::Stop (Seconds (12));
  Simulator::Run ();
  Require (g_nwkDeliveryCount == 1,
           "held direct packet was not delivered exactly once after route learning");

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  std::vector<TraceRow> enqueues =
    RowsForEventAndSequence (rows, "nwk_enqueue", appSequence);
  std::vector<TraceRow> forwards =
    RowsForEventAndSequence (rows, "nwk_forward", appSequence);
  std::vector<TraceRow> deliveries =
    RowsForEventAndSequence (rows, "nwk_delivery", appSequence);
  std::vector<TraceRow> routeChanges =
    RouteChangesForDestination (rows, sourceNode, destinationNode);
  std::vector<TraceRow> sizes =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_SIZE, sourceNode);
  std::vector<TraceRow> delays =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_DELAY, sourceNode);

  Require (enqueues.size () == 1 &&
             forwards.size () == 1 &&
             deliveries.size () == 1 &&
             routeChanges.size () == 1,
           "direct lifecycle did not emit one enqueue/route/forward/delivery chain");
  RequireNwkSizeAndDelaySamples (
    rows, sourceNode, expectedResidence, "held direct packet");

  const uint32_t networkBytes =
    payloadBytes + CsrNetHeader ().GetSerializedSize ();
  const std::string networkBytesText = CsrTraceInteger (networkBytes);
  const std::string sourceText = CsrTraceInteger (sourceNode);
  const std::string destinationText = CsrTraceInteger (destinationNode);
  Require (enqueues[0].node == sourceText &&
             enqueues[0].peer.empty () &&
             enqueues[0].source == sourceText &&
             enqueues[0].destination == destinationText &&
             enqueues[0].sizeBytes == networkBytesText &&
             enqueues[0].reason == "local" &&
             enqueues[0].success == "1",
           "direct local-enqueue correlation fields are incorrect");
  Require (forwards[0].node == sourceText &&
             forwards[0].peer.empty () &&
             forwards[0].source == sourceText &&
             forwards[0].destination == destinationText &&
             forwards[0].sizeBytes == networkBytesText &&
             forwards[0].nextHop == destinationText &&
             forwards[0].reason == "local" &&
             forwards[0].detail ==
               "num_hop=1;path=" + destinationText,
           "direct forward did not report its actual selected route");
  Require (routeChanges[0].nextHop == destinationText &&
             routeChanges[0].detail == forwards[0].detail &&
             routeChanges[0].routeCost == forwards[0].routeCost &&
             !forwards[0].routeCost.empty (),
           "direct route-change and forward decisions disagree");
  Require (deliveries[0].node == destinationText &&
             deliveries[0].peer == sourceText &&
             deliveries[0].source == sourceText &&
             deliveries[0].destination == destinationText &&
             deliveries[0].sizeBytes == networkBytesText,
           "direct delivery correlation fields are incorrect");
  Require (sizes[0].eventIndex < enqueues[0].eventIndex &&
             enqueues[0].eventIndex < routeChanges[0].eventIndex &&
             routeChanges[0].eventIndex < sizes[1].eventIndex &&
             sizes[1].eventIndex < delays[0].eventIndex &&
             delays[0].eventIndex < forwards[0].eventIndex &&
             forwards[0].eventIndex < deliveries[0].eventIndex,
           "direct NWK lifecycle is not in source/event order");
  Require (NearlyEqual (forwards[0].timeSeconds,
                        expectedResidence,
                        1e-12),
           "direct packet was not released one TIC after route learning");

  Simulator::Destroy ();
}

void
TestForcedTwoHopPath ()
{
  constexpr CsrNodeId sourceNode = 201;
  constexpr CsrNodeId relayNode = 202;
  constexpr CsrNodeId destinationNode = 203;
  constexpr uint64_t appSequence = 7002;
  constexpr uint32_t payloadBytes = 32;
  constexpr uint32_t sourceRouteCost = 17;
  constexpr uint32_t relayRouteCost = 9;
  const uint8_t routableCapability =
    static_cast<uint8_t> (CsrNodeType::Routable);

  TraceFile trace ("nwk-two-hop");

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrNetDevice> relayDevice =
    CreateObject<CsrNetDevice> (relayNode);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (destinationNode);
  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> relayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> relayNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();

  sourceDevice->AddPeer (relayDevice);
  relayDevice->AddPeer (sourceDevice);
  relayDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (relayDevice);
  ConfigureNoErrors (sourceDevice);
  ConfigureNoErrors (relayDevice);
  ConfigureNoErrors (destinationDevice);
  ConnectNwkStack (sourceDevice, sourceHop, sourceNwk, sourceNode);
  ConnectNwkStack (relayDevice, relayHop, relayNwk, relayNode);
  ConnectNwkStack (destinationDevice,
                   destinationHop,
                   destinationNwk,
                   destinationNode);
  sourceNwk->SetNodeType (CsrNodeType::Gateway);
  relayNwk->SetNodeType (CsrNodeType::Routable);
  destinationNwk->SetNodeType (CsrNodeType::Routable);

  g_nwkDeliveryCount = 0;
  g_expectedNwkSource = sourceNode;
  destinationNwk->SetRxFromNetCallback (MakeCallback (&RecordNwkDelivery));

  StartTrace (trace);
  Require (sourceNwk->AddOrUpdateRoute (
             destinationNode,
             relayNode,
             false,
             2,
             70.0,
             sourceRouteCost,
             0,
             relayNode,
             routableCapability),
           "could not install forced two-hop source route");
  Require (relayNwk->AddOrUpdateRoute (
             destinationNode,
             destinationNode,
             true,
             1,
             70.0,
             relayRouteCost,
             0,
             destinationNode,
             routableCapability,
             {destinationNode}),
           "could not install forced two-hop relay route");

  sourceNwk->Send (
    destinationNode,
    5,
    BuildTaggedPayload (payloadBytes, appSequence),
    true);

  Simulator::Stop (Seconds (20));
  Simulator::Run ();
  Require (g_nwkDeliveryCount == 1,
           "forced two-hop packet was not delivered exactly once");

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  std::vector<TraceRow> enqueues =
    RowsForEventAndSequence (rows, "nwk_enqueue", appSequence);
  std::vector<TraceRow> forwards =
    RowsForEventAndSequence (rows, "nwk_forward", appSequence);
  std::vector<TraceRow> deliveries =
    RowsForEventAndSequence (rows, "nwk_delivery", appSequence);
  std::vector<TraceRow> sourceRouteChanges =
    RouteChangesForDestination (rows, sourceNode, destinationNode);
  std::vector<TraceRow> relayRouteChanges =
    RouteChangesForDestination (rows, relayNode, destinationNode);

  Require (enqueues.size () == 2 &&
             forwards.size () == 2 &&
             deliveries.size () == 1,
           "two-hop lifecycle did not emit two queue legs and one delivery");
  Require (sourceRouteChanges.size () == 1 &&
             relayRouteChanges.size () == 1,
           "forced path did not emit one route decision per forwarding node");
  RequireNwkSizeAndDelaySamples (
    rows,
    sourceNode,
    NwkTic ().GetSeconds (),
    "two-hop source packet");
  RequireNwkSizeAndDelaySamples (
    rows,
    relayNode,
    NwkTic ().GetSeconds (),
    "two-hop relay packet");

  const TraceRow *sourceEnqueue = nullptr;
  const TraceRow *relayEnqueue = nullptr;
  const TraceRow *sourceForward = nullptr;
  const TraceRow *relayForward = nullptr;
  const std::string sourceText = CsrTraceInteger (sourceNode);
  const std::string relayText = CsrTraceInteger (relayNode);
  const std::string destinationText = CsrTraceInteger (destinationNode);
  for (const TraceRow &row : enqueues)
    {
      if (row.node == sourceText)
        {
          sourceEnqueue = &row;
        }
      else if (row.node == relayText)
        {
          relayEnqueue = &row;
        }
    }
  for (const TraceRow &row : forwards)
    {
      if (row.node == sourceText)
        {
          sourceForward = &row;
        }
      else if (row.node == relayText)
        {
          relayForward = &row;
        }
    }
  Require (sourceEnqueue != nullptr &&
             relayEnqueue != nullptr &&
             sourceForward != nullptr &&
             relayForward != nullptr,
           "two-hop trace is missing a source or relay queue leg");

  const std::string networkBytesText = CsrTraceInteger (
    payloadBytes + CsrNetHeader ().GetSerializedSize ());
  Require (sourceEnqueue->peer.empty () &&
             sourceEnqueue->reason == "local" &&
             sourceEnqueue->source == sourceText &&
             sourceEnqueue->destination == destinationText &&
             sourceEnqueue->sizeBytes == networkBytesText,
           "two-hop source enqueue fields are incorrect");
  Require (sourceForward->peer.empty () &&
             sourceForward->reason == "local" &&
             sourceForward->nextHop == relayText &&
             sourceForward->routeCost ==
               CsrTraceInteger (sourceRouteCost) &&
             sourceForward->detail ==
               "num_hop=2;path=" + relayText + ">" + destinationText,
           "two-hop source forward did not report the forced route");
  Require (relayEnqueue->peer == sourceText &&
             relayEnqueue->reason == "relay" &&
             relayEnqueue->source == sourceText &&
             relayEnqueue->destination == destinationText &&
             relayEnqueue->sizeBytes == networkBytesText,
           "two-hop relay enqueue fields are incorrect");
  Require (relayForward->peer == sourceText &&
             relayForward->reason == "relay" &&
             relayForward->nextHop == destinationText &&
             relayForward->routeCost ==
               CsrTraceInteger (relayRouteCost) &&
             relayForward->detail ==
               "num_hop=1;path=" + destinationText,
           "two-hop relay forward did not report the forced route");
  Require (deliveries[0].node == destinationText &&
             deliveries[0].peer == relayText &&
             deliveries[0].source == sourceText &&
             deliveries[0].destination == destinationText &&
             deliveries[0].sizeBytes == networkBytesText,
           "two-hop delivery fields are incorrect");
  Require (sourceEnqueue->eventIndex < sourceForward->eventIndex &&
             sourceForward->eventIndex < relayEnqueue->eventIndex &&
             relayEnqueue->eventIndex < relayForward->eventIndex &&
             relayForward->eventIndex < deliveries[0].eventIndex,
           "two-hop NWK lifecycle is not in path order");

  Simulator::Destroy ();
}

void
TestHopAndMacEnqueueOrdering ()
{
  TraceFile trace ("hop-mac-order");
  StartTrace (trace);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());

  hop->SendData (2, 5, Create<Packet> (1), true);
  Require (hop->GetResendQueueSize () == 1,
           "HOP resend entry was not queued");
  Require (device->GetMac ().GetDataQueuedFrameCount () == 1,
           "HOP frame was not forwarded into the MAC Tx queue");

  // The source writes no MAC Tx queue-size statistic for ACK-driven
  // cancellation.  Only the HOP resend erase produces a new size sample.
  hop->ReceiveFromMac (BuildExactAck (2, 1, 1), 0.0, 100.0);
  Require (hop->GetResendQueueSize () == 0,
           "exact ACK did not erase the HOP resend entry");
  Require (device->GetMac ().GetDataQueuedFrameCount () == 0,
           "exact ACK did not cancel the pending MAC copy");

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> samples =
    StatisticSamples (ReadTrace (trace.Path ()));

  Require (samples.size () == 3,
           "HOP/MAC enqueue/cancel path emitted an unexpected statistic sample");
  Require (samples[0].node == "1" &&
             samples[0].statistic == CSR_STAT_HOP_RESEND_QUEUE_SIZE &&
             NearlyEqual (SampleValue (samples[0]), 1.0),
           "HOP enqueue was not the first source-ordered sample");
  Require (samples[1].node == "1" &&
             samples[1].statistic == CSR_STAT_MAC_TX_QUEUE_SIZE &&
             NearlyEqual (SampleValue (samples[1]), 1.0),
           "MAC enqueue did not follow the HOP resend enqueue");
  Require (samples[2].node == "1" &&
             samples[2].statistic == CSR_STAT_HOP_RESEND_QUEUE_SIZE &&
             NearlyEqual (SampleValue (samples[2]), 0.0),
           "HOP erase did not emit the final zero-size sample");

  Simulator::Destroy ();
}

Ptr<CsrNetDevice> g_dataDevice;

void
EnqueueHigherPriorityData ()
{
  g_dataDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (10, 20, 2, 7),
    20,
    7,
    true);
}

void
TestMacSelectionSizeDelayOrdering ()
{
  TraceFile trace ("mac-data-order");
  StartTrace (trace);

  g_dataDevice = CreateObject<CsrNetDevice> (10);
  g_dataDevice->GetMac ().SetReservationSlotOverrideForDifferentialRun (1);
  g_dataDevice->GetMac ().EnqueueTxFrame (
    BuildDataFrame (10, 20, 1, 1),
    20,
    1,
    true);
  Simulator::Schedule (MilliSeconds (100),
                       &EnqueueHigherPriorityData);

  Simulator::Stop (Seconds (0.7));
  Simulator::Run ();

  Require (g_dataDevice->GetMac ().GetTransmittedFrameCount () == 1,
           "deterministic MAC data test did not produce one aggregate");
  Require (g_dataDevice->GetMac ().GetDataQueuedFrameCount () == 0,
           "deterministic MAC data aggregate did not drain both entries");

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  std::vector<TraceRow> samples = StatisticSamples (rows);
  std::vector<TraceRow> txStarts = RowsForEvent (rows, "tx_start");

  Require (txStarts.size () == 1,
           "MAC data trace did not contain exactly one tx_start");
  Require (txStarts[0].sequence == "2",
           "higher-DSCP frame was not first in the transmitted aggregate");
  Require (samples.size () == 6,
           "MAC data path emitted the wrong statistic-sample count");

  const char *expectedStatistics[] = {
    CSR_STAT_MAC_TX_QUEUE_SIZE,
    CSR_STAT_MAC_TX_QUEUE_SIZE,
    CSR_STAT_MAC_TX_QUEUE_SIZE,
    CSR_STAT_MAC_TX_QUEUE_DELAY,
    CSR_STAT_MAC_TX_QUEUE_SIZE,
    CSR_STAT_MAC_TX_QUEUE_DELAY,
  };
  const double expectedSizes[] = {1.0, 2.0, 1.0, 0.0};
  for (std::size_t index = 0; index < samples.size (); ++index)
    {
      Require (samples[index].node == "10" &&
                 samples[index].statistic == expectedStatistics[index],
               "MAC size/delay samples are not in source order");
    }
  Require (NearlyEqual (SampleValue (samples[0]), expectedSizes[0]) &&
             NearlyEqual (SampleValue (samples[1]), expectedSizes[1]) &&
             NearlyEqual (SampleValue (samples[2]), expectedSizes[2]) &&
             NearlyEqual (SampleValue (samples[4]), expectedSizes[3]),
           "MAC Tx queue-size samples have incorrect post-mutation values");

  double txTime = txStarts[0].timeSeconds;
  Require (NearlyEqual (samples[2].timeSeconds, txTime) &&
             NearlyEqual (samples[3].timeSeconds, txTime) &&
             NearlyEqual (samples[4].timeSeconds, txTime) &&
             NearlyEqual (samples[5].timeSeconds, txTime),
           "MAC dequeue statistics were not sampled at selection time");
  Require (NearlyEqual (SampleValue (samples[3]), txTime - 0.1) &&
             NearlyEqual (SampleValue (samples[5]), txTime),
           "MAC queuing delays do not follow DSCP dequeue order");
  Require (samples[5].eventIndex < txStarts[0].eventIndex,
           "MAC queue observations were emitted after the OTA tx_start");

  Simulator::Destroy ();
  g_dataDevice = nullptr;
}

void
TestAckAdmissionAndOverflowSampling ()
{
  TraceFile trace ("ack-admission");
  StartTrace (trace);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (30);
  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (30, 40, 1), 40, 0, false);

  // Neither an exact duplicate nor a cumulative update changes the list, so
  // the recovered source writes no ACK queue-size sample for either path.
  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (30, 40, 1), 40, 0, false);
  device->GetMac ().EnqueueTxFrame (
    BuildWindowAck (30, 40, 1000), 40, 0, false);

  for (uint16_t sequence = 2;
       sequence <= CsrMacCore::ACK_QUEUE_SIZE;
       ++sequence)
    {
      device->GetMac ().EnqueueTxFrame (
        BuildExactAck (30, 40, sequence), 40, 0, false);
    }

  Require (device->GetMac ().GetAckQueuedFrameCount () ==
             CsrMacCore::ACK_QUEUE_SIZE,
           "ACK admission test did not fill the recovered queue limit");

  // Unlike a DATA overflow, br_mac writes the unchanged ACK queue size on
  // the rejected path.  The final sample therefore repeats 256.
  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (30, 40, 257), 40, 0, false);

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> ackSamples =
    RowsForStatistic (ReadTrace (trace.Path ()),
                      CSR_STAT_MAC_ACK_QUEUE_SIZE);

  Require (ackSamples.size () == CsrMacCore::ACK_QUEUE_SIZE + 1,
           "duplicate/update ACK path sampled, or overflow sample is missing");
  for (uint32_t index = 0; index < CsrMacCore::ACK_QUEUE_SIZE; ++index)
    {
      Require (ackSamples[index].node == "30" &&
                 NearlyEqual (SampleValue (ackSamples[index]),
                              static_cast<double> (index + 1)),
               "accepted ACK size samples do not rise from 1 through 256");
    }
  Require (NearlyEqual (SampleValue (ackSamples.back ()),
                        static_cast<double> (CsrMacCore::ACK_QUEUE_SIZE)),
           "rejected ACK did not write the unchanged 256-entry size");

  Simulator::Destroy ();
}

void
TestAckRemovalSampling ()
{
  TraceFile trace ("ack-removal");
  StartTrace (trace);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (50);
  device->GetMac ().NoteHeardFrom (60, 0.0);
  device->GetMac ().SetNeighborPathloss (60, 0.0);
  device->GetMac ().SetReservationSlotOverrideForDifferentialRun (1);
  device->GetMac ().EnqueueTxFrame (
    BuildExactAck (50, 60, 1), 60, 0, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (device->GetMac ().GetTransmittedFrameCount () == 5,
           "ACK did not use the recovered one-plus-four send budget");
  Require (device->GetMac ().GetAckQueuedFrameCount () == 0,
           "ACK entry remained after its fifth transmission");

  CloseDifferentialTraceCsv ();
  std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  std::vector<TraceRow> ackSamples =
    RowsForStatistic (rows, CSR_STAT_MAC_ACK_QUEUE_SIZE);
  std::vector<TraceRow> txStarts = RowsForEvent (rows, "tx_start");

  Require (ackSamples.size () == 2 &&
             NearlyEqual (SampleValue (ackSamples[0]), 1.0) &&
             NearlyEqual (SampleValue (ackSamples[1]), 0.0),
           "ACK removal did not produce exactly the source [1,0] vector");
  Require (txStarts.size () == 5,
           "ACK removal trace did not contain five tx_start events");
  Require (ackSamples[1].eventIndex < txStarts.back ().eventIndex,
           "ACK removal sample was emitted after the fifth OTA tx_start");
  Require (NearlyEqual (ackSamples[1].timeSeconds,
                        txStarts.back ().timeSeconds),
           "ACK removal sample and fifth transmission do not share a timestamp");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  try
    {
      Time::SetResolution (Time::NS);
      TestAggregateOnlyPathWriter ();
      TestHopAndMacEnqueueOrdering ();
      TestMacSelectionSizeDelayOrdering ();
      TestAckAdmissionAndOverflowSampling ();
      TestAckRemovalSampling ();
      TestDirectHeldThenReleasedPath ();
      TestForcedTwoHopPath ();
    }
  catch (const std::exception &error)
    {
      CloseDifferentialTraceCsv ();
      Simulator::Destroy ();
      std::cerr << "FAIL: " << error.what () << std::endl;
      return 1;
    }

  std::cout << "PASS: source-ordered queue observation trace test"
            << std::endl;
  return 0;
}
