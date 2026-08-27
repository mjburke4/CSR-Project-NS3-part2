#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"

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
  std::string sequence;
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
                               "sequence",
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
      row.sequence = fields[columns["sequence"]];
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
      TestHopAndMacEnqueueOrdering ();
      TestMacSelectionSizeDelayOrdering ();
      TestAckAdmissionAndOverflowSampling ();
      TestAckRemovalSampling ();
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
