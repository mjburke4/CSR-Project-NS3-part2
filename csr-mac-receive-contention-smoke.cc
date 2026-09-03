#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <streambuf>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

std::vector<CsrNodeId> g_receivedSources;
std::vector<double> g_txTimes;

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
       ("csr-mac-receive-contention-smoke-" +
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

struct SyncDecisionSample
{
  CsrNodeId transmitter {0};
  uint16_t sequence {0};
  double snrDb {0.0};
  double thresholdDb {0.0};
  bool accepted {false};
};

std::vector<SyncDecisionSample> g_syncDecisions;

class NullStreamBuffer : public std::streambuf
{
protected:
  int overflow (int character) override
  {
    return traits_type::eq_int_type (character, traits_type::eof ())
      ? traits_type::not_eof (character)
      : character;
  }
};

class ScopedCoutSilencer
{
public:
  ScopedCoutSilencer ()
    : m_original (std::cout.rdbuf (&m_null))
  {
  }

  ~ScopedCoutSilencer ()
  {
    std::cout.rdbuf (m_original);
  }

private:
  NullStreamBuffer m_null;
  std::streambuf *m_original;
};

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

std::size_t
CountTraceReason (const TraceFile &trace, const std::string &reason)
{
  std::ifstream stream (trace.Path ());
  Require (stream.is_open (), "could not open receive-contention trace");
  std::size_t count = 0;
  std::string line;
  const std::string field = "," + reason + ",";
  while (std::getline (stream, line))
    {
      if (line.find (",rx_drop,") != std::string::npos &&
          line.find (field) != std::string::npos)
        {
          count++;
        }
    }
  return count;
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

Ptr<Packet>
BuildFrame (CsrNodeId source,
            CsrNodeId destination,
            uint16_t sequence,
            bool ackable = false)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    ackable,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (128);

  Ptr<Packet> frame = Create<Packet> (16);
  frame->AddHeader (header);
  return frame;
}

void
RecordReception (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received frame did not contain a CSR header");
  g_receivedSources.push_back (header.GetSrc ());
}

void
RecordTransmission (CsrNodeId, uint16_t, Time sentTime)
{
  g_txTimes.push_back (sentTime.GetSeconds ());
}

void
RecordSyncDecision (CsrNodeId transmitter,
                    uint16_t sequence,
                    double snrDb,
                    double thresholdDb,
                    bool accepted)
{
  g_syncDecisions.push_back (
    {transmitter, sequence, snrDb, thresholdDb, accepted});
}

void
SendDirect (Ptr<CsrNetDevice> sender,
            CsrNodeId destination,
            uint16_t sequence,
            double txPowerDbm,
            PreambleType preamble = PREAMBLE_LONG)
{
  sender->SendToPeer (BuildFrame (sender->GetId (),
                                  destination,
                                  sequence),
                      destination,
                      128,
                      txPowerDbm,
                      preamble,
                      3,
                      false);
}

void
RunStateTransitionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 1, 0.0);

  Simulator::Schedule (MilliSeconds (5), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "receiver left Search before SYNC2TRACK_MIN");
  });
  Simulator::Schedule (MilliSeconds (10), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::TRACK,
             "receiver did not enter Track after SYNC2TRACK_MIN");
  });

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {1},
           "single tracked frame was not delivered exactly once");
  Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
           "receiver did not return to Search after packet completion");
  Simulator::Destroy ();
}

void
RunSimultaneousCollisionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender1 = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> sender2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  sender1->AddPeer (receiver);
  sender2->AddPeer (receiver);
  ConfigureNoErrors (sender1);
  ConfigureNoErrors (sender2);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender1, 3, 10, 0.0);
  SendDirect (sender2, 3, 20, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "equal-power simultaneous signals did not collide");
  Require (receiver->GetRxCollisionCount () >= 1,
           "overlapping preambles did not increment collision state");
  Simulator::Destroy ();
}

void
RunThreeSignalContentionScenario ()
{
  TraceFile trace ("three-signal");
  OpenDifferentialTraceCsv (trace.Path ());
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender1 = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> sender2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> sender3 = CreateObject<CsrNetDevice> (3);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (4);
  sender1->AddPeer (receiver);
  sender2->AddPeer (receiver);
  sender3->AddPeer (receiver);
  ConfigureNoErrors (sender1);
  ConfigureNoErrors (sender2);
  ConfigureNoErrors (sender3);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender1, 4, 21, 0.0);
  SendDirect (sender2, 4, 22, 0.0);
  SendDirect (sender3, 4, 23, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();
  CloseDifferentialTraceCsv ();

  Require (g_receivedSources.empty (),
           "three equal-power signals did not collide");
  Require (receiver->GetRxCollisionCount () >= 2,
           "three overlapping preambles did not record both contenders");
  Require (CountTraceReason (trace, "prior_stage") == 2,
           "unselected three-signal contenders were not prior-stage drops");
  Require (CountTraceReason (trace, "ecc") == 0,
           "prior-stage contenders polluted the ECC drop statistic");
  Simulator::Destroy ();
}

void
RunAcquisitionCaptureScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> weak = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> strong = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  weak->AddPeer (receiver);
  strong->AddPeer (receiver);
  ConfigureNoErrors (weak);
  ConfigureNoErrors (strong);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (strong, 3, 31, 20.0);
  SendDirect (weak, 3, 30, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {2},
           "Search did not capture the mature strongest preamble");
  Require (receiver->GetRxCaptureCount () >= 1,
           "strongest-preamble capture was not recorded");
  Simulator::Destroy ();
}

void
RunPostLockCollisionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> first = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> late = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  first->AddPeer (receiver);
  late->AddPeer (receiver);
  ConfigureNoErrors (first);
  ConfigureNoErrors (late);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (first, 3, 40, 0.0);
  Simulator::Schedule (MilliSeconds (20),
                       &SendDirect,
                       late,
                       3,
                       41,
                       20.0,
                       PREAMBLE_LONG);

  Simulator::Stop (Seconds (1.4));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {2},
           "surviving post-lock preamble was not reacquired after Track");
  Require (receiver->GetRxCollisionCount () >= 1,
           "post-lock overlap did not record a collision");
  Simulator::Destroy ();
}

void
RunSlotFreezeScenario ()
{
  g_receivedSources.clear ();
  g_txTimes.clear ();
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> contender = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> sink = CreateObject<CsrNetDevice> (3);
  jammer->AddPeer (contender);
  contender->AddPeer (sink);
  ConfigureNoErrors (jammer);
  ConfigureNoErrors (contender);
  ConfigureNoErrors (sink);
  contender->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordTransmission));

  contender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 3, 50, true), 3, 5, true);
  SendDirect (jammer, 2, 51, 0.0);

  Simulator::Schedule (MilliSeconds (900), [contender] () {
    Require (contender->GetMac ().GetTransmittedFrameCount () == 0,
             "Tx countdown advanced while receiver was in Track");
  });

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (contender->GetMac ().GetTransmittedFrameCount () == 1,
           "frozen Tx countdown did not resume after Track");
  Require (g_txTimes.size () == 1 && g_txTimes.front () > 1.0,
           "tracked reception did not defer the reserved Tx opportunity");
  Simulator::Destroy ();
}

void
RunOpnetAlignedDutyInitializationScenario ()
{
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  receiver->EnableOpnetAlignedDutyCycling (true);

  Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
           "OPNET-aligned duty cycle did not start in Idle at t=0");

  Simulator::Schedule (MilliSeconds (987), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
             "OPNET-aligned duty cycle woke before 0.988 seconds");
  });
  Simulator::Schedule (MilliSeconds (988), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned duty cycle did not wake at 0.988 seconds");
  });
  Simulator::Schedule (MicroSeconds (996800), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned wake window closed too early");
  });
  Simulator::Schedule (MilliSeconds (997), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
             "OPNET-aligned wake window did not return to Idle");
  });
  Simulator::Schedule (MicroSeconds (1976001), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned duty cycle did not schedule its second wake");
  });

  Simulator::Stop (MilliSeconds (1977));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
RunDutyCycleScenarios ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->EnableDutyCycling (true);
  receiver->SetDutyCyclePhase (MilliSeconds (500));
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 60, 0.0, PREAMBLE_LONG);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {1},
           "long preamble did not bridge sleep to a periodic wake");
  Simulator::Destroy ();

  g_receivedSources.clear ();
  sender = CreateObject<CsrNetDevice> (1);
  receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->EnableDutyCycling (true);
  receiver->SetDutyCyclePhase (MilliSeconds (500));
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 61, 0.0, PREAMBLE_SHORT);

  Simulator::Stop (MilliSeconds (200));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "short preamble incorrectly bridged a sleeping interval");
  Require (receiver->GetRxMissCount () == 1,
           "sleeping short-preamble miss was not recorded");
  Simulator::Destroy ();
}

void
RunHalfDuplexScenario ()
{
  TraceFile trace ("half-duplex");
  OpenDifferentialTraceCsv (trace.Path ());
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> dummy = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> interferer = CreateObject<CsrNetDevice> (3);
  receiver->AddPeer (dummy);
  interferer->AddPeer (receiver);
  ConfigureNoErrors (receiver);
  ConfigureNoErrors (dummy);
  ConfigureNoErrors (interferer);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (receiver, 2, 70, 0.0);
  SendDirect (interferer, 1, 71, 0.0, PREAMBLE_SHORT);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();
  CloseDifferentialTraceCsv ();

  Require (g_receivedSources.empty (),
           "half-duplex receiver decoded while transmitting");
  Require (receiver->GetRxMissCount () == 1,
           "half-duplex receive miss was not recorded");
  Require (CountTraceReason (trace, "half_duplex") == 1,
           "locked receiver drop was not classified as half-duplex");
  Require (CountTraceReason (trace, "ecc") == 0,
           "half-duplex drop polluted the ECC drop statistic");
  Simulator::Destroy ();
}

std::vector<double>
CollectSyncThresholds (int64_t stream,
                       bool consumeDutyPhase,
                       uint32_t sampleCount)
{
  static constexpr double sampleSpacingSeconds = 0.03;
  RngSeedManager::SetSeed (24680);
  RngSeedManager::SetRun (7);
  g_syncDecisions.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 2, 0.0);
  Require (receiver->AssignStreams (stream) == 2,
           "CSR device did not assign both owned random streams");
  if (consumeDutyPhase)
    {
      receiver->EnableDutyCycling (true);
      receiver->EnableDutyCycling (false);
    }
  Require (receiver->TraceConnectWithoutContext (
             "SyncDecision",
             MakeCallback (&RecordSyncDecision)),
           "SYNC-decision trace source could not be connected");

  double txPowerDbm = receiver->GetPhy ().profile.noiseFloorDbm;
  for (uint32_t index = 0; index < sampleCount; ++index)
    {
      Simulator::Schedule (
        Seconds (sampleSpacingSeconds * index),
        &SendDirect,
        sender,
        2,
        static_cast<uint16_t> (index),
        txPowerDbm,
        PREAMBLE_SHORT);
    }

  {
    ScopedCoutSilencer silence;
    Simulator::Stop (
      Seconds (sampleSpacingSeconds * sampleCount + 0.1));
    Simulator::Run ();
  }

  Require (g_syncDecisions.size () == sampleCount,
           "SYNC trace did not report exactly one draw per attempt");
  std::vector<double> thresholds;
  thresholds.reserve (sampleCount);
  for (uint32_t index = 0; index < sampleCount; ++index)
    {
      const SyncDecisionSample &sample = g_syncDecisions[index];
      Require (sample.transmitter == 1 && sample.sequence == index,
               "SYNC trace identity did not follow attempt order");
      Require (std::abs (sample.snrDb) < 1.0e-9 && sample.accepted,
               "high-SNR distribution attempt was unexpectedly rejected");
      thresholds.push_back (sample.thresholdDb);
    }
  Simulator::Destroy ();
  return thresholds;
}

SyncDecisionSample
RunControlledSyncDecision (double snrDb,
                           double thresholdDb,
                           bool stochasticEnabled,
                           bool useOverride)
{
  RngSeedManager::SetSeed (24680);
  RngSeedManager::SetRun (7);
  g_receivedSources.clear ();
  g_syncDecisions.clear ();

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetPhy ().SetLinkDistanceMeters (1, 2, 0.0);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));
  CsrPhyProfile profile = receiver->GetPhy ().profile;
  profile.syncSnrThresholdDb = thresholdDb;
  profile.stochasticSyncThreshold = stochasticEnabled;
  receiver->GetPhy ().SetProfile (profile);
  if (useOverride)
    {
      receiver->SetSyncSnrThresholdOverrideDb (thresholdDb);
    }
  receiver->AssignStreams (100);
  Require (receiver->TraceConnectWithoutContext (
             "SyncDecision",
             MakeCallback (&RecordSyncDecision)),
           "controlled SYNC-decision trace could not be connected");

  SendDirect (sender,
              2,
              1,
              receiver->GetPhy ().profile.noiseFloorDbm + snrDb,
              PREAMBLE_SHORT);
  {
    ScopedCoutSilencer silence;
    Simulator::Stop (MilliSeconds (100));
    Simulator::Run ();
  }

  Require (g_syncDecisions.size () == 1,
           "controlled SYNC attempt did not produce one decision trace");
  SyncDecisionSample sample = g_syncDecisions.front ();
  bool delivered = !g_receivedSources.empty ();
  Require (delivered == sample.accepted,
           "SYNC trace decision did not match receiver delivery");
  Simulator::Destroy ();
  return sample;
}

void
RunSyncThresholdDistributionScenario ()
{
  static constexpr uint32_t sampleCount = 512;
  std::vector<double> first = CollectSyncThresholds (400, false, sampleCount);
  std::vector<double> repeated = CollectSyncThresholds (400, false, sampleCount);
  std::vector<double> afterDutyDraw =
    CollectSyncThresholds (400, true, sampleCount);
  std::vector<double> otherStream =
    CollectSyncThresholds (402, false, sampleCount);

  Require (first == repeated,
           "fixed seed/run/stream did not reproduce SYNC thresholds");
  Require (first == afterDutyDraw,
           "duty-phase RNG draw perturbed the dedicated SYNC stream");
  Require (first != otherStream,
           "different assigned stream reproduced the same SYNC sequence");

  double mean = std::accumulate (first.begin (), first.end (), 0.0) /
    first.size ();
  double variance = 0.0;
  for (double value : first)
    {
      double offset = value - mean;
      variance += offset * offset;
    }
  variance /= first.size ();
  std::cout << "[SYNC threshold] samples=" << first.size ()
            << " mean_db=" << mean
            << " variance_db2=" << variance
            << std::endl;
  Require (std::abs (mean + 11.0) < 0.08,
           "SYNC threshold sample mean does not match -11 dB");
  Require (std::abs (variance - 0.25) < 0.07,
           "SYNC threshold sample variance does not match 0.25 dB squared");
}

void
RunSyncThresholdDecisionScenario ()
{
  SyncDecisionSample rejected =
    RunControlledSyncDecision (-10.75, -10.5, true, true);
  Require (std::abs (rejected.thresholdDb + 10.5) < 1.0e-12 &&
             !rejected.accepted,
           "controlled threshold did not reject SNR below its boundary");

  SyncDecisionSample accepted =
    RunControlledSyncDecision (-10.75, -11.0, true, true);
  Require (std::abs (accepted.thresholdDb + 11.0) < 1.0e-12 &&
             accepted.accepted,
           "controlled threshold did not accept SNR above its boundary");

  SyncDecisionSample deterministicCompatibility =
    RunControlledSyncDecision (-10.75, -11.0, false, false);
  Require (std::abs (deterministicCompatibility.thresholdDb + 11.0) <
             1.0e-12 &&
             deterministicCompatibility.accepted,
           "explicit deterministic compatibility mode did not use the mean");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  RunStateTransitionScenario ();
  RunSimultaneousCollisionScenario ();
  RunThreeSignalContentionScenario ();
  RunAcquisitionCaptureScenario ();
  RunPostLockCollisionScenario ();
  RunSlotFreezeScenario ();
  RunOpnetAlignedDutyInitializationScenario ();
  RunDutyCycleScenarios ();
  RunHalfDuplexScenario ();
  RunSyncThresholdDistributionScenario ();
  RunSyncThresholdDecisionScenario ();

  std::cout << "PASS: OPNET MAC receive/contention state parity test"
            << std::endl;
  return 0;
}
