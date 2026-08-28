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
RowsForEventAndFlow (const std::vector<TraceRow> &rows,
                     const std::string &event,
                     CsrNodeId source,
                     CsrNodeId destination)
{
  std::vector<TraceRow> selected;
  const std::string sourceText = CsrTraceInteger (source);
  const std::string destinationText = CsrTraceInteger (destination);
  std::copy_if (rows.begin (),
                rows.end (),
                std::back_inserter (selected),
                [&event, &sourceText, &destinationText] (const TraceRow &row) {
                  return row.event == event &&
                         row.source == sourceText &&
                         row.destination == destinationText;
                });
  return selected;
}

std::map<std::string, std::string>
ParseDetail (const TraceRow &row)
{
  std::map<std::string, std::string> fields;
  std::size_t start = 0;
  while (start < row.detail.size ())
    {
      const std::size_t end = row.detail.find (';', start);
      const std::string token = row.detail.substr (
        start,
        end == std::string::npos ? std::string::npos : end - start);
      const std::size_t equals = token.find ('=');
      Require (equals != std::string::npos && equals != 0,
               "admission-ledger detail is not key=value");
      const std::string key = token.substr (0, equals);
      Require (fields.emplace (key, token.substr (equals + 1)).second,
               "admission-ledger detail contains a duplicate key");
      if (end == std::string::npos)
        {
          break;
        }
      start = end + 1;
    }
  return fields;
}

void
RequireDetail (const TraceRow &row,
               const std::string &key,
               const std::string &expected,
               const std::string &label)
{
  const std::map<std::string, std::string> detail = ParseDetail (row);
  const auto found = detail.find (key);
  Require (found != detail.end (), label + " is missing detail key " + key);
  Require (found->second == expected,
           label + " has the wrong " + key + " value");
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
  SetDifferentialAdmissionTraceEnabled (true);
  OpenDifferentialTraceCsv (trace.Path ());
}

void
TestAggregateOnlyPathWriter ()
{
  TraceFile trace ("aggregate-path-writer");
  SetDifferentialTraceAggregateOnly (true);
  SetDifferentialAdmissionTraceEnabled (true);
  OpenDifferentialTraceCsv (trace.Path ());

  CsrDifferentialTraceEvent filtered;
  filtered.event = "tx_start";
  WriteDifferentialTrace (filtered);

  CsrDifferentialTraceEvent admission;
  admission.event = "app_admission";
  admission.node = "10";
  admission.source = "10";
  admission.destination = "30";
  admission.success = "0";
  admission.reason = "nsdp_full";
  admission.detail = "attempt=8;nsdp_count=16;nsdp_limit=16";
  WriteDifferentialTrace (admission);

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
  SetDifferentialAdmissionTraceEnabled (false);

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

  Require (rows.size () == 4,
           "aggregate-only writer retained a filtered event or lost a path event");
  for (const auto &row : rows)
    {
      Require (row.size () == header.size (),
               "aggregate-only writer emitted a malformed CSV row");
    }
  Require (rows[0][columns["event_index"]] == "0" &&
             rows[0][columns["event"]] == "app_admission" &&
             rows[1][columns["event_index"]] == "1" &&
             rows[1][columns["event"]] == "nwk_enqueue" &&
             rows[2][columns["event_index"]] == "2" &&
             rows[2][columns["event"]] == "nwk_forward" &&
             rows[3][columns["event_index"]] == "3" &&
             rows[3][columns["event"]] == "route_change",
           "aggregate-only path events or indexes are not source ordered");
  Require (rows[0][columns["src"]] == "10" &&
             rows[0][columns["dst"]] == "30" &&
             rows[0][columns["reason"]] == "nsdp_full" &&
             rows[0][columns["detail"]] ==
               "attempt=8;nsdp_count=16;nsdp_limit=16",
           "aggregate-only writer dropped application-admission fields");
  Require (rows[2][columns["src"]] == "10" &&
             rows[2][columns["dst"]] == "30" &&
             rows[2][columns["sequence"]] == "7" &&
             rows[2][columns["peer"]] == "9" &&
             rows[2][columns["next_hop"]] == "20" &&
             rows[2][columns["route_cost"]] == "17" &&
             rows[2][columns["detail"]] == "num_hop=2;path=20>30",
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

Ptr<Packet>
BuildExactDack (CsrNodeId source,
                CsrNodeId destination,
                uint16_t sequence)
{
  Ptr<Packet> frame = BuildExactAck (source, destination, sequence);
  CsrHeader header;
  frame->RemoveHeader (header);
  header.SetIsDack (true);
  header.SetType (CSR_PKT_DACK);
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

Ptr<Packet>
BuildNetworkPayload (CsrNodeId source,
                     CsrNodeId destination,
                     uint8_t dscp,
                     uint64_t appSequence)
{
  Ptr<Packet> payload = BuildTaggedPayload (16, appSequence);
  payload->AddHeader (CsrNetHeader (source, destination, dscp));
  return payload;
}

Ptr<Packet>
BuildRelayDataFrame (CsrNodeId hopSource,
                     CsrNodeId relay,
                     uint16_t hopSequence,
                     CsrNodeId nwkSource,
                     CsrNodeId nwkDestination,
                     uint8_t dscp,
                     uint64_t appSequence)
{
  Ptr<Packet> frame =
    BuildNetworkPayload (nwkSource, nwkDestination, dscp, appSequence);
  CsrHeader header (hopSource,
                    relay,
                    hopSequence,
                    dscp,
                    true,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (header);
  return frame;
}

void
RunRelayFeedbackBoundary (uint32_t nsdpBefore,
                          const std::string &expectedReason,
                          bool injectDuplicate,
                          CsrNodeId nodeBase)
{
  TraceFile trace ("relay-feedback-" + std::to_string (nsdpBefore));
  const CsrNodeId hopSource = nodeBase;
  const CsrNodeId relay = nodeBase + 1;
  const CsrNodeId nwkSource = nodeBase + 2;
  const CsrNodeId nwkDestination = nodeBase + 3;
  const CsrNodeId nextHop = nodeBase + 4;
  const uint8_t dscp = 4;
  const uint16_t hopSequence = 41;
  const uint64_t appSequence = 8100 + nsdpBefore;

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (relay);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectNwkStack (device, hop, nwk, relay);
  nwk->SetNodeType (CsrNodeType::Routable);

  // Seed relay custody without an onward route.  ReceiveFromHop performs the
  // same NSDP increment and NWK enqueue that follows an accepted HOP frame,
  // while leaving the scheduled queue checks unexecuted for this fixture.
  SetDifferentialAdmissionTraceEnabled (false);
  for (uint32_t index = 0; index < nsdpBefore; ++index)
    {
      nwk->ReceiveFromHop (
        BuildNetworkPayload (nwkSource,
                             nwkDestination,
                             dscp,
                             appSequence + index + 1),
        hopSource);
    }

  Require (nwk->GetNsdpCount (nwkSource, nwkDestination) == nsdpBefore,
           "relay feedback fixture did not seed the requested NSDP count");
  Require (nwk->GetNwkQueueSize () == nsdpBefore,
           "relay feedback fixture did not seed the requested NWK queue");

  nwk->AddStaticRouteWithPathloss (nwkDestination, nextHop, 70.0);
  StartTrace (trace);

  hop->ReceiveFromMac (
    BuildRelayDataFrame (hopSource,
                         relay,
                         hopSequence,
                         nwkSource,
                         nwkDestination,
                         dscp,
                         appSequence),
    70.0,
    30.0);
  if (injectDuplicate)
    {
      hop->ReceiveFromMac (
        BuildRelayDataFrame (hopSource,
                             relay,
                             hopSequence,
                             nwkSource,
                             nwkDestination,
                             dscp,
                             appSequence),
        70.0,
        30.0);
    }

  const uint32_t nsdpAfter = nsdpBefore + 1;
  Require (nwk->GetNsdpCount (nwkSource, nwkDestination) == nsdpAfter,
           "accepted relay DATA did not increment NSDP exactly once");
  Require (nwk->GetNwkQueueSize () == nsdpAfter,
           "accepted relay DATA did not enqueue exactly once");

  CloseDifferentialTraceCsv ();
  const std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  const std::vector<TraceRow> feedback =
    RowsForEventAndSequence (rows, "hop_feedback", appSequence);
  const std::vector<TraceRow> enqueues =
    RowsForEventAndSequence (rows, "nwk_enqueue", appSequence);

  Require (feedback.size () == (injectDuplicate ? 2 : 1),
           "relay feedback boundary emitted the wrong feedback count");
  Require (enqueues.size () == 1,
           "relay feedback boundary enqueued a duplicate or lost first DATA");
  Require (feedback[0].reason == expectedReason && feedback[0].success == "1",
           "relay feedback boundary selected the wrong ACK/DACK reason");
  RequireDetail (feedback[0],
                 "first_reception",
                 "1",
                 "first relay feedback");
  RequireDetail (feedback[0],
                 "nsdp_count_before",
                 CsrTraceInteger (nsdpBefore),
                 "first relay feedback");
  RequireDetail (feedback[0],
                 "nsdp_count_after",
                 CsrTraceInteger (nsdpAfter),
                 "first relay feedback");

  if (injectDuplicate)
    {
      Require (feedback[1].reason == "ack" && feedback[1].success == "1",
               "duplicate relay DATA did not receive a plain ACK decision");
      RequireDetail (feedback[1],
                     "first_reception",
                     "0",
                     "duplicate relay feedback");
      RequireDetail (feedback[1],
                     "nsdp_count_before",
                     CsrTraceInteger (nsdpAfter),
                     "duplicate relay feedback");
      RequireDetail (feedback[1],
                     "nsdp_count_after",
                     CsrTraceInteger (nsdpAfter),
                     "duplicate relay feedback");
    }

  Simulator::Destroy ();
}

void
TestRelayFeedbackUsesPreEnqueueNsdpCount ()
{
  RunRelayFeedbackBoundary (15, "ack", false, 360);
  RunRelayFeedbackBoundary (16, "dack", true, 370);
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
  std::vector<TraceRow> feedback =
    RowsForEventAndSequence (rows, "hop_feedback", appSequence);
  std::vector<TraceRow> routeChanges =
    RouteChangesForDestination (rows, sourceNode, destinationNode);
  std::vector<TraceRow> sizes =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_SIZE, sourceNode);
  std::vector<TraceRow> delays =
    RowsForStatisticAtNode (rows, CSR_STAT_NWK_QUEUE_DELAY, sourceNode);

  Require (enqueues.size () == 1 &&
             forwards.size () == 1 &&
             deliveries.size () == 1 &&
             feedback.size () == 1 &&
             routeChanges.size () == 1,
           "direct lifecycle did not emit one enqueue/route/forward/delivery/feedback chain");
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
  Require (feedback[0].node == destinationText &&
             feedback[0].peer == sourceText &&
             feedback[0].source == sourceText &&
             feedback[0].destination == destinationText &&
             feedback[0].success == "1" &&
             feedback[0].reason == "ack",
           "direct receiver did not record its ACK feedback decision");
  Require (sizes[0].eventIndex < enqueues[0].eventIndex &&
             enqueues[0].eventIndex < routeChanges[0].eventIndex &&
             routeChanges[0].eventIndex < sizes[1].eventIndex &&
             sizes[1].eventIndex < delays[0].eventIndex &&
             delays[0].eventIndex < forwards[0].eventIndex &&
             forwards[0].eventIndex < deliveries[0].eventIndex,
           "direct NWK lifecycle is not in source/event order");
  Require (deliveries[0].eventIndex < feedback[0].eventIndex,
           "receiver feedback was recorded before NWK delivery");
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
TestAdmissionLedgerNoRouteAndSuccessfulAdmission ()
{
  constexpr CsrNodeId sourceNode = 301;
  constexpr CsrNodeId noRouteDestination = 302;
  constexpr CsrNodeId routedDestination = 303;
  constexpr uint64_t noRouteSequence = 7101;
  constexpr uint64_t routedSequence = 7102;

  TraceFile trace ("admission-no-route");
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectNwkStack (device, hop, nwk, sourceNode);
  nwk->AddStaticRouteWithPathloss (
    routedDestination,
    routedDestination,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));

  StartTrace (trace);
  // The route-less positive-DSCP entry stays at the head.  The source-exact
  // full scan must still admit the routed best-effort entry behind it.
  nwk->Send (noRouteDestination,
             5,
             BuildTaggedPayload (16, noRouteSequence),
             true);
  nwk->Send (routedDestination,
             0,
             BuildTaggedPayload (24, routedSequence),
             true);

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (nwk->GetNwkQueueSize () == 1,
           "admission ledger did not retain only the no-route packet");
  Require (nwk->GetNsdpCount (sourceNode, noRouteDestination) == 1 &&
             nwk->GetNsdpCount (sourceNode, routedDestination) == 1,
           "admission ledger changed NWK NSDP custody");
  const CsrHopLayer::DataAdmissionSnapshot routedState =
    hop->GetDataAdmissionSnapshot (routedDestination);
  Require (routedState.pendingData == 1 &&
             routedState.pendingThreshold == 16 &&
             routedState.globalSpad == 16 &&
             routedState.neighborOutstanding == 1 &&
             routedState.neighborThreshold == 0 &&
             routedState.neighborSpad == 0 &&
             routedState.globalAllowed &&
             !routedState.neighborAllowed,
           "successful admission produced the wrong read-only HOP snapshot");

  CloseDifferentialTraceCsv ();
  const std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  const std::vector<TraceRow> noRouteAdmissions =
    RowsForEventAndSequence (rows, "nwk_admission", noRouteSequence);
  const std::vector<TraceRow> routedAdmissions =
    RowsForEventAndSequence (rows, "nwk_admission", routedSequence);
  const std::vector<TraceRow> routedForwards =
    RowsForEventAndSequence (rows, "nwk_forward", routedSequence);
  const std::vector<TraceRow> hopAdmissions =
    RowsForEventAndSequence (rows, "hop_admission", routedSequence);

  Require (noRouteAdmissions.size () == 1 &&
             noRouteAdmissions[0].success == "0" &&
             noRouteAdmissions[0].reason == "no_route" &&
             noRouteAdmissions[0].node == CsrTraceInteger (sourceNode) &&
             noRouteAdmissions[0].destination ==
               CsrTraceInteger (noRouteDestination) &&
             noRouteAdmissions[0].nextHop.empty (),
           "no-route scan did not emit its correlated NWK block");
  RequireDetail (noRouteAdmissions[0],
                 "nsdp_count",
                 "1",
                 "no-route NWK block");
  RequireDetail (noRouteAdmissions[0],
                 "nsdp_limit",
                 "16",
                 "no-route NWK block");
  RequireDetail (noRouteAdmissions[0],
                 "pending",
                 "0",
                 "no-route NWK block");
  RequireDetail (noRouteAdmissions[0],
                 "global_spad",
                 "17",
                 "no-route NWK block");

  Require (routedAdmissions.size () == 1 &&
             routedAdmissions[0].success == "1" &&
             routedAdmissions[0].reason == "admitted" &&
             routedAdmissions[0].nextHop ==
               CsrTraceInteger (routedDestination),
           "routed queue entry did not emit its successful NWK admission");
  RequireDetail (routedAdmissions[0],
                 "outstanding",
                 "0",
                 "successful NWK admission");
  RequireDetail (routedAdmissions[0],
                 "threshold",
                 "0",
                 "successful NWK admission");
  RequireDetail (routedAdmissions[0],
                 "neighbor_spad",
                 "1",
                 "successful NWK admission");

  Require (routedForwards.size () == 1 && hopAdmissions.size () == 1,
           "successful admission lost its NWK-forward or HOP-admission row");
  Require (hopAdmissions[0].success == "1" &&
             hopAdmissions[0].reason == "admitted" &&
             hopAdmissions[0].peer == CsrTraceInteger (routedDestination),
           "HOP admission correlation fields are incorrect");
  RequireDetail (hopAdmissions[0],
                 "pending_after",
                 "1",
                 "successful HOP admission");
  RequireDetail (hopAdmissions[0],
                 "outstanding_after",
                 "1",
                 "successful HOP admission");
  RequireDetail (hopAdmissions[0],
                 "global_spad_after",
                 "16",
                 "successful HOP admission");
  Require (noRouteAdmissions[0].eventIndex < routedAdmissions[0].eventIndex &&
             routedAdmissions[0].eventIndex < routedForwards[0].eventIndex &&
             routedForwards[0].eventIndex < hopAdmissions[0].eventIndex,
           "no-route and successful admission rows are not source ordered");

  Simulator::Destroy ();
}

void
TestApplicationNsdpAdmissionBoundary ()
{
  constexpr CsrNodeId sourceNode = 321;
  constexpr CsrNodeId destinationNode = 322;
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  nwk->SetNodeId (sourceNode);

  Require (nwk->CanAdmitApplicationPacket (destinationNode),
           "empty NSDP flow was not application-admissible");
  for (uint64_t index = 0; index < 15; ++index)
    {
      nwk->Send (destinationNode,
                 0,
                 BuildTaggedPayload (8, 7200 + index),
                 true);
    }
  Require (nwk->GetNsdpCount (sourceNode, destinationNode) == 15 &&
             nwk->CanAdmitApplicationPacket (destinationNode),
           "application NSDP gate blocked before the source-exact limit");

  nwk->Send (destinationNode,
             0,
             BuildTaggedPayload (8, 7215),
             true);
  Require (nwk->GetNsdpCount (sourceNode, destinationNode) == 16 &&
             !nwk->CanAdmitApplicationPacket (destinationNode) &&
             nwk->GetNwkQueueSize () == 16,
           "application NSDP gate did not block exactly at count 16");

  Simulator::Destroy ();
}

void
TestAdmissionLedgerNeighborAckRelease ()
{
  constexpr CsrNodeId sourceNode = 331;
  constexpr CsrNodeId destinationNode = 332;
  constexpr uint64_t firstSequence = 7301;
  constexpr uint64_t secondSequence = 7302;

  TraceFile trace ("admission-neighbor-ack");
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectNwkStack (device, hop, nwk, sourceNode);
  nwk->AddStaticRouteWithPathloss (
    destinationNode,
    destinationNode,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));

  StartTrace (trace);
  nwk->Send (destinationNode,
             0,
             BuildTaggedPayload (16, firstSequence),
             true);
  nwk->Send (destinationNode,
             0,
             BuildTaggedPayload (16, secondSequence),
             true);

  Simulator::Schedule (MicroSeconds (1), [device, hop, nwk] () {
    const CsrHopLayer::DataAdmissionSnapshot held =
      hop->GetDataAdmissionSnapshot (destinationNode);
    Require (nwk->GetNwkQueueSize () == 1 &&
               nwk->GetNsdpCount (sourceNode, destinationNode) == 2 &&
               held.pendingData == 1 && held.globalAllowed &&
               held.neighborOutstanding == 1 &&
               held.neighborThreshold == 0 &&
               held.neighborSpad == 0 && !held.neighborAllowed,
             "second packet was not held by the initial neighbor window");

    hop->ReceiveFromMac (
      BuildExactAck (destinationNode, sourceNode, 1), 70.0, 30.0);
    const CsrHopLayer::DataAdmissionSnapshot released =
      hop->GetDataAdmissionSnapshot (destinationNode);
    Require (nwk->GetNsdpCount (sourceNode, destinationNode) == 1 &&
               released.pendingData == 0 &&
               released.neighborOutstanding == 0 &&
               released.globalSpad == 17 &&
               released.neighborSpad == 1 &&
               hop->GetResendQueueSize () == 0 &&
               device->GetMac ().GetDataQueuedFrameCount () == 0,
             "exact ACK did not release NSDP and HOP capacity immediately");
  });

  Simulator::Schedule (MicroSeconds (2), [hop, nwk] () {
    const CsrHopLayer::DataAdmissionSnapshot admitted =
      hop->GetDataAdmissionSnapshot (destinationNode);
    Require (nwk->GetNwkQueueSize () == 0 &&
               nwk->GetNsdpCount (sourceNode, destinationNode) == 1 &&
               admitted.pendingData == 1 &&
               admitted.neighborOutstanding == 1,
             "ACK release did not admit the held packet one TIC later");
    hop->ReceiveFromMac (
      BuildExactAck (destinationNode, sourceNode, 2), 70.0, 30.0);
    Require (nwk->GetNsdpCount (sourceNode, destinationNode) == 0 &&
               hop->GetPendingDataCount () == 0 &&
               hop->GetOutstandingDataCount (destinationNode) == 0,
             "second exact ACK did not close the deterministic flow");
  });

  Simulator::Stop (MicroSeconds (3));
  Simulator::Run ();
  CloseDifferentialTraceCsv ();

  const std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  const std::vector<TraceRow> firstNwkAdmissions =
    RowsForEventAndSequence (rows, "nwk_admission", firstSequence);
  const std::vector<TraceRow> secondNwkAdmissions =
    RowsForEventAndSequence (rows, "nwk_admission", secondSequence);
  const std::vector<TraceRow> firstHopAdmissions =
    RowsForEventAndSequence (rows, "hop_admission", firstSequence);
  const std::vector<TraceRow> secondHopAdmissions =
    RowsForEventAndSequence (rows, "hop_admission", secondSequence);
  const std::vector<TraceRow> releases =
    RowsForEventAndFlow (rows,
                         "nwk_nsdp_release",
                         sourceNode,
                         destinationNode);
  const std::vector<TraceRow> firstCompletions =
    RowsForEventAndSequence (rows, "hop_completion", firstSequence);
  const std::vector<TraceRow> secondCompletions =
    RowsForEventAndSequence (rows, "hop_completion", secondSequence);

  Require (firstNwkAdmissions.size () == 1 &&
             firstNwkAdmissions[0].reason == "admitted" &&
             firstHopAdmissions.size () == 1,
           "first neighbor-window packet was not admitted exactly once");
  Require (secondNwkAdmissions.size () == 2 &&
             secondNwkAdmissions[0].success == "0" &&
             secondNwkAdmissions[0].reason == "neighbor_flow_full" &&
             secondNwkAdmissions[1].success == "1" &&
             secondNwkAdmissions[1].reason == "admitted" &&
             secondHopAdmissions.size () == 1,
           "neighbor hold did not become admission after ACK release");
  RequireDetail (secondNwkAdmissions[0],
                 "outstanding",
                 "1",
                 "neighbor-flow block");
  RequireDetail (secondNwkAdmissions[0],
                 "threshold",
                 "0",
                 "neighbor-flow block");
  RequireDetail (secondNwkAdmissions[0],
                 "neighbor_spad",
                 "0",
                 "neighbor-flow block");

  Require (releases.size () == 2 &&
             releases[0].reason == "hop_feedback" &&
             releases[1].reason == "hop_feedback" &&
             firstCompletions.size () == 1 &&
             firstCompletions[0].reason == "ack" &&
             secondCompletions.size () == 1 &&
             secondCompletions[0].reason == "ack",
           "exact ACK lifecycle did not emit feedback/release/completion rows");
  RequireDetail (firstCompletions[0],
                 "pending_after",
                 "0",
                 "ACK completion");
  RequireDetail (firstCompletions[0],
                 "outstanding_after",
                 "0",
                 "ACK completion");
  Require (releases[0].eventIndex < firstCompletions[0].eventIndex &&
             firstCompletions[0].eventIndex <
               secondNwkAdmissions[1].eventIndex &&
             secondNwkAdmissions[1].eventIndex <
               secondHopAdmissions[0].eventIndex &&
             releases[1].eventIndex < secondCompletions[0].eventIndex,
           "ACK release and held-packet admission are not source ordered");

  Simulator::Destroy ();
}

void
TestAdmissionLedgerGlobalEffectiveLimit ()
{
  constexpr CsrNodeId sourceNode = 341;
  constexpr CsrNodeId firstDestination = 400;
  constexpr uint64_t firstSequence = 7400;
  constexpr uint32_t offeredPackets = 18;

  TraceFile trace ("admission-global-limit");
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectNwkStack (device, hop, nwk, sourceNode);

  for (uint32_t index = 0; index < offeredPackets; ++index)
    {
      const CsrNodeId destination = firstDestination + index;
      nwk->AddStaticRouteWithPathloss (
        destination,
        destination,
        70.0,
        true,
        static_cast<uint8_t> (CsrNodeType::Routable));
    }

  StartTrace (trace);
  for (uint32_t index = 0; index < offeredPackets; ++index)
    {
      nwk->Send (firstDestination + index,
                 0,
                 BuildTaggedPayload (8, firstSequence + index),
                 true);
    }

  Simulator::Stop (MicroSeconds (1));
  Simulator::Run ();

  Require (nwk->GetNwkQueueSize () == 1 &&
             hop->GetPendingDataCount () == 17 &&
             hop->GetResendQueueSize () == 17,
           "global HOP gate did not reproduce the effective limit of 17");
  for (uint32_t index = 0; index < 17; ++index)
    {
      Require (hop->GetOutstandingDataCount (firstDestination + index) == 1,
               "global-limit admission lost a per-neighbor HOP slot");
    }
  const CsrNodeId heldDestination = firstDestination + 17;
  const CsrHopLayer::DataAdmissionSnapshot held =
    hop->GetDataAdmissionSnapshot (heldDestination);
  Require (held.pendingData == 17 &&
             held.pendingThreshold == 16 &&
             held.globalSpad == 0 &&
             !held.globalAllowed &&
             held.neighborOutstanding == 0 &&
             held.neighborThreshold == 0 &&
             held.neighborSpad == 1 &&
             held.neighborAllowed,
           "global-limit hold exposed the wrong read-only HOP snapshot");

  CloseDifferentialTraceCsv ();
  const std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  uint32_t successfulNwkAdmissions = 0;
  uint32_t hopAdmissions = 0;
  for (const TraceRow &row : rows)
    {
      if (row.event == "nwk_admission" && row.success == "1" &&
          row.reason == "admitted")
        {
          successfulNwkAdmissions++;
        }
      if (row.event == "hop_admission")
        {
          hopAdmissions++;
        }
    }
  Require (successfulNwkAdmissions == 17 && hopAdmissions == 17,
           "global ledger did not report exactly 17 HOP admissions");
  const std::vector<TraceRow> globalBlocks =
    RowsForEventAndSequence (rows,
                             "nwk_admission",
                             firstSequence + 17);
  const std::vector<TraceRow> heldForwards =
    RowsForEventAndSequence (rows, "nwk_forward", firstSequence + 17);
  Require (globalBlocks.size () == 1 &&
             globalBlocks[0].success == "0" &&
             globalBlocks[0].reason == "global_hop_full" &&
             globalBlocks[0].destination ==
               CsrTraceInteger (heldDestination) &&
             heldForwards.empty (),
           "18th packet was not attributed to the global HOP gate");
  RequireDetail (globalBlocks[0],
                 "pending",
                 "17",
                 "global HOP block");
  RequireDetail (globalBlocks[0],
                 "pending_limit",
                 "16",
                 "global HOP block");
  RequireDetail (globalBlocks[0],
                 "global_spad",
                 "0",
                 "global HOP block");

  Simulator::Destroy ();
}

void
TestAdmissionLedgerDackSplitRelease ()
{
  constexpr CsrNodeId sourceNode = 351;
  constexpr CsrNodeId destinationNode = 352;
  constexpr uint64_t firstSequence = 7501;
  constexpr uint64_t secondSequence = 7502;
  const Time dackAt = Seconds (2.0);
  const Time nominalExpiry = dackAt + Seconds (20.0);
  const Time capacityReleaseAt = nominalExpiry + CsrOpnetTic ();
  const Time nwkReadmissionAt = capacityReleaseAt + CsrOpnetTic ();

  TraceFile trace ("admission-dack-split");
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (sourceNode);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> nwk = CreateObject<CsrNetLayer> ();
  ConnectNwkStack (device, hop, nwk, sourceNode);
  nwk->AddStaticRouteWithPathloss (
    destinationNode,
    destinationNode,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));

  StartTrace (trace);
  nwk->Send (destinationNode,
             0,
             BuildTaggedPayload (16, firstSequence),
             true);
  nwk->Send (destinationNode,
             0,
             BuildTaggedPayload (16, secondSequence),
             true);

  Simulator::Schedule (dackAt, [hop, nwk] () {
    Require (nwk->GetNwkQueueSize () == 1 &&
               nwk->GetNsdpCount (sourceNode, destinationNode) == 2 &&
               hop->GetPendingDataCount () == 1 &&
               hop->GetOutstandingDataCount (destinationNode) == 1,
             "DACK setup did not retain the second neighbor-flow packet");
    hop->ReceiveFromMac (
      BuildExactDack (destinationNode, sourceNode, 1), 70.0, 30.0);
    const CsrHopLayer::DataAdmissionSnapshot held =
      hop->GetDataAdmissionSnapshot (destinationNode);
    Require (nwk->GetNsdpCount (sourceNode, destinationNode) == 1 &&
               hop->GetResendQueueSize () == 0 &&
               held.pendingData == 1 &&
               held.neighborOutstanding == 1 &&
               held.neighborSpad == 0,
             "DACK did not release NSDP while retaining HOP capacity");
  });

  Simulator::Schedule (nominalExpiry + NanoSeconds (1), [hop, nwk] () {
    Require (nwk->GetNwkQueueSize () == 1 &&
               hop->GetPendingDataCount () == 1 &&
               hop->GetOutstandingDataCount (destinationNode) == 1,
             "DACK HOP capacity was released at its nominal expiry instead of expiry plus TIC");
  });

  Simulator::Schedule (capacityReleaseAt + NanoSeconds (1), [hop, nwk] () {
    Require (nwk->GetNwkQueueSize () == 1 &&
               nwk->GetNsdpCount (sourceNode, destinationNode) == 1 &&
               hop->GetPendingDataCount () == 0 &&
               hop->GetOutstandingDataCount (destinationNode) == 0,
             "NWK ran synchronously with the DACK capacity release");
  });

  Simulator::Schedule (nwkReadmissionAt + NanoSeconds (1), [hop, nwk] () {
    Require (nwk->GetNwkQueueSize () == 0 &&
               nwk->GetNsdpCount (sourceNode, destinationNode) == 1 &&
               hop->GetPendingDataCount () == 1 &&
               hop->GetOutstandingDataCount (destinationNode) == 1,
             "NWK did not re-admit the held packet one TIC after DACK release");
  });

  Simulator::Stop (nwkReadmissionAt + NanoSeconds (2));
  Simulator::Run ();
  CloseDifferentialTraceCsv ();

  const std::vector<TraceRow> rows = ReadTrace (trace.Path ());
  const std::vector<TraceRow> releases =
    RowsForEventAndFlow (rows,
                         "nwk_nsdp_release",
                         sourceNode,
                         destinationNode);
  const std::vector<TraceRow> completions =
    RowsForEventAndSequence (rows, "hop_completion", firstSequence);
  const std::vector<TraceRow> capacityReleases =
    RowsForEventAndSequence (rows, "hop_capacity_release", firstSequence);
  const std::vector<TraceRow> secondNwkAdmissions =
    RowsForEventAndSequence (rows, "nwk_admission", secondSequence);
  const std::vector<TraceRow> secondHopAdmissions =
    RowsForEventAndSequence (rows, "hop_admission", secondSequence);

  Require (releases.size () == 1 &&
             releases[0].reason == "hop_feedback" &&
             completions.size () == 1 && completions[0].reason == "dack" &&
             capacityReleases.size () == 1 &&
             capacityReleases[0].reason == "dack_expiry",
           "DACK did not emit its split feedback/release/completion lifecycle");
  RequireDetail (completions[0],
                 "pending_after",
                 "1",
                 "DACK completion");
  RequireDetail (completions[0],
                 "outstanding_after",
                 "1",
                 "DACK completion");
  RequireDetail (capacityReleases[0],
                 "pending_after",
                 "0",
                 "DACK capacity release");
  RequireDetail (capacityReleases[0],
                 "outstanding_after",
                 "0",
                 "DACK capacity release");
  RequireDetail (capacityReleases[0],
                 "dack_hold_seconds",
                 "20",
                 "DACK capacity release");
  RequireDetail (capacityReleases[0],
                 "dack_scheduled_timer_offset_seconds",
                 CsrTraceDouble (CsrOpnetTic ().GetSeconds ()),
                 "DACK capacity release");
  RequireDetail (capacityReleases[0],
                 "dack_effective_timer_offset_seconds",
                 CsrTraceDouble (CsrOpnetTic ().GetSeconds ()),
                 "DACK capacity release");
  RequireDetail (capacityReleases[0],
                 "nwk_wake_nominal_delay_seconds",
                 CsrTraceDouble (CsrOpnetTic ().GetSeconds ()),
                 "DACK capacity release");
  Require (NearlyEqual (capacityReleases[0].timeSeconds -
                          completions[0].timeSeconds,
                        20.0 + CsrOpnetTic ().GetSeconds (),
                        1e-12),
           "DACK capacity release was not at the nominal hold plus one TIC");
  Require (releases[0].eventIndex < completions[0].eventIndex &&
             completions[0].eventIndex < capacityReleases[0].eventIndex,
           "DACK split-release rows are not source ordered");

  Require (secondNwkAdmissions.size () >= 2 &&
             secondNwkAdmissions.front ().reason == "neighbor_flow_full" &&
             secondNwkAdmissions.back ().reason == "admitted" &&
             secondHopAdmissions.size () == 1 &&
             capacityReleases[0].eventIndex <
               secondNwkAdmissions.back ().eventIndex &&
             secondNwkAdmissions.back ().eventIndex <
               secondHopAdmissions[0].eventIndex,
           "held packet was not admitted after DACK capacity release");
  Require (NearlyEqual (secondNwkAdmissions.back ().timeSeconds -
                          capacityReleases[0].timeSeconds,
                        CsrOpnetTic ().GetSeconds (),
                        1e-12),
           "NWK readmission was not one TIC after DACK capacity release");

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
      TestRelayFeedbackUsesPreEnqueueNsdpCount ();
      TestMacSelectionSizeDelayOrdering ();
      TestAckAdmissionAndOverflowSampling ();
      TestAckRemovalSampling ();
      TestDirectHeldThenReleasedPath ();
      TestForcedTwoHopPath ();
      TestAdmissionLedgerNoRouteAndSuccessfulAdmission ();
      TestApplicationNsdpAdmissionBoundary ();
      TestAdmissionLedgerNeighborAckRelease ();
      TestAdmissionLedgerGlobalEffectiveLimit ();
      TestAdmissionLedgerDackSplitRelease ();
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
