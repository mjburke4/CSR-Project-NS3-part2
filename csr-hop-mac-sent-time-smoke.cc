#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

constexpr uint32_t BLOCKER_COUNT = 3;

uint32_t g_nsdpReleases = 0;
uint32_t g_linkFailures = 0;
std::vector<Time> g_nwkWakeTimes;
Ptr<CsrNetDevice> g_orderingDevice;
uint32_t g_orderingReleases = 0;

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
ReleaseNsdp (CsrNodeId src, CsrNodeId dst)
{
  Require (src == 1 && dst == 3,
           "final timeout released the wrong NSDP flow");
  g_nsdpReleases++;
}

void
ReportLinkFailure (CsrNodeId neighbor)
{
  Require (neighbor == 2,
           "unexpected link-failure callback named the wrong neighbor");
  g_linkFailures++;
}

void
WakeNwkQueue ()
{
  g_nwkWakeTimes.push_back (Simulator::Now ());
}

void
ReleaseBeforeRetry (CsrNodeId src, CsrNodeId dst)
{
  Require (src == 1 && dst == 3,
           "same-scan timeout released the wrong NSDP flow");
  Require (g_orderingDevice != nullptr &&
             g_orderingDevice->GetMac ().GetQueuedFrameCount () == 0,
           "same-scan retry was enqueued before all final deletions completed");
  g_orderingReleases++;
}

Ptr<Packet>
BuildBlockerFrame (uint16_t sequence)
{
  CsrHeader header (1, 2, sequence, 7, false, false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (200);
  frame->AddHeader (header);
  return frame;
}

void
CheckInitialFrameBehindBlockers (Ptr<CsrNetDevice> device,
                                 Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 2,
           "MAC did not transmit the expected leading blockers");
  Require (device->GetMac ().GetQueuedFrameCount () == 2,
           "HOP enqueued a premature retransmission before the initial TX");
  Require (hop->GetPendingDataCount () == 1,
           "deferred initial frame did not retain its HOP pending slot");
}

void
CheckBeforeFirstRetry (Ptr<CsrNetDevice> device,
                       Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () ==
             BLOCKER_COUNT + 1,
           "first retry was scheduled from HOP enqueue time instead of MAC sent time");
  Require (device->GetMac ().GetQueuedFrameCount () <= 1,
           "more than one retry was queued behind MAC slot contention");
  Require (hop->GetPendingDataCount () == 1,
           "initial transmission released HOP flow control without an ACK");
}

void
CheckFirstRetry (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () ==
             BLOCKER_COUNT + 2,
           "first OPNET resend did not occur two seconds after actual TX");
}

void
CheckSecondRetry (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () ==
             BLOCKER_COUNT + 3,
           "second OPNET resend did not occur two seconds after the first resend");
}

void
CheckFinalGraceStillHeld (Ptr<CsrHopLayer> hop)
{
  Require (hop->GetPendingDataCount () == 1,
           "HOP released the frame before the final four-second ACK grace expired");
  Require (g_nsdpReleases == 0,
           "NSDP was released before final timeout");
  Require (g_linkFailures == 0,
           "link failure was reported before final timeout");
}

void
CheckFinalTimeout (Ptr<CsrNetDevice> device,
                   Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () ==
             BLOCKER_COUNT + 3,
           "OPNET parity requires one initial TX plus exactly two resends");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC retained a frame after final HOP timeout");
  Require (hop->GetPendingDataCount () == 0,
           "final timeout did not release the global HOP pending count");
  Require (hop->GetOutstandingDataCount (2) == 0,
           "final timeout did not release per-neighbor flow control");
  Require (g_nsdpReleases == 1,
           "final timeout did not release NSDP exactly once");
  Require (g_linkFailures == 0,
           "DATA timeout was incorrectly reported as a routing-link failure");
}

void
CheckNoRetryAtNominal (Ptr<CsrNetDevice> device,
                       Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "resend was queued at the nominal deadline without the source TIC");
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "nominal resend boundary changed HOP custody state");
}

void
CheckRetryAtDelayedBoundary (Ptr<CsrNetDevice> device,
                             Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetQueuedFrameCount () == 1,
           "resend was not queued at its nominal deadline plus TIC");
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "resend timer released HOP custody instead of retransmitting");
}

void
ConfirmAndClearRetry (Ptr<CsrNetDevice> device,
                      Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().CancelAcknowledgedFrames (2, 1, 1, 0) == 1,
           "could not isolate the retransmission from MAC");
  hop->NotifyMacFrameSent (2, 1, Simulator::Now ());
}

void
CheckFinalNominalBoundary (Ptr<CsrHopLayer> hop)
{
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "final timeout released custody at the nominal four-second boundary");
  Require (g_nsdpReleases == 0 && g_nwkWakeTimes.empty (),
           "final timeout side effects occurred before the source TIC");
}

void
CheckFinalDelayedBoundary (Ptr<CsrNetDevice> device,
                           Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC retained a retransmission after final timeout");
  Require (hop->GetResendQueueSize () == 0 &&
             hop->GetPendingDataCount () == 0 &&
             hop->GetOutstandingDataCount (2) == 0,
           "final timeout at nominal plus TIC did not release HOP custody");
  Require (g_nsdpReleases == 0 && g_linkFailures == 0,
           "metadata-free timeout produced an NSDP/link-failure side effect");
  Require (g_nwkWakeTimes.empty (),
           "final-timeout NWK wake ran synchronously instead of one TIC later");
}

void
CheckFinalWake (Time expectedWake)
{
  Require (g_nwkWakeTimes.size () == 1 &&
             g_nwkWakeTimes[0] == expectedWake,
           "final-timeout NWK wake did not run one TIC after deletion");
}

void
ArmMissingEntryFallback (Ptr<CsrNetDevice> device,
                         Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().CancelAcknowledgedFrames (2, 1, 1, 0) == 1,
           "could not clear the first retry before fallback scan test");
  hop->NotifyMacFrameSent (2, 999, Simulator::Now ());
}

void
CheckFallbackRetry (Ptr<CsrNetDevice> device,
                    Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetQueuedFrameCount () == 1,
           "missing-entry sent indication did not arm the fallback global scan");
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "fallback global scan changed custody instead of retransmitting");
}

void
RunMissingEntryFallbackCase ()
{
  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());

  hop->SendData (2, 5, Create<Packet> (1), true);
  Require (device->GetMac ().CancelAcknowledgedFrames (2, 1, 1, 0) == 1,
           "could not isolate fallback test's initial transmission");
  hop->NotifyMacFrameSent (2, 1, Seconds (0));

  const Time tic = CsrOpnetTic ();
  const Time firstRetryAt = Seconds (2.0) + tic;
  const Time fallbackArmedAt = firstRetryAt + NanoSeconds (1);
  const Time fallbackAt = fallbackArmedAt + Seconds (2.0) + tic;

  Simulator::Schedule (fallbackArmedAt,
                       &ArmMissingEntryFallback,
                       device,
                       hop);
  Simulator::Schedule (fallbackArmedAt + Seconds (2.0),
                       &CheckNoRetryAtNominal,
                       device,
                       hop);
  Simulator::Schedule (fallbackAt + NanoSeconds (1),
                       &CheckFallbackRetry,
                       device,
                       hop);
  Simulator::Stop (fallbackAt + NanoSeconds (2));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
ConfirmOrderingRetryEntry (Ptr<CsrHopLayer> hop)
{
  hop->NotifyMacFrameSent (4, 1, Simulator::Now ());
}

void
CheckDeleteBeforeRetryResult (Ptr<CsrNetDevice> device,
                              Ptr<CsrHopLayer> hop)
{
  Require (g_orderingReleases == 1,
           "same-scan final entry did not release NSDP exactly once");
  Require (device->GetMac ().GetQueuedFrameCount () == 1,
           "same-scan retry entry was not enqueued after deletion pass");
  Require (hop->GetResendQueueSize () == 1 &&
             hop->GetPendingDataCount () == 1,
           "same-scan final deletion retained or over-released HOP custody");
  Require (hop->GetOutstandingDataCount (2) == 0 &&
             hop->GetOutstandingDataCount (4) == 1,
           "same-scan ordering released the wrong neighbor capacity");
}

void
RunDeleteBeforeRetryCase ()
{
  g_orderingReleases = 0;
  g_linkFailures = 0;

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  g_orderingDevice = device;
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseBeforeRetry));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));

  // Insert the retry-due entry first.  The final-expiring entry follows it in
  // list order, so the former interleaved scan would enqueue the retry before
  // ReleaseBeforeRetry() observes the final deletion.  Only the source's
  // delete-all-then-retry passes keep the MAC queue empty in that callback.
  Ptr<Packet> retryPayload = Create<Packet> (16);
  retryPayload->AddHeader (CsrNetHeader (1, 5, 5));
  hop->SendData (4, 5, retryPayload, true);
  Require (device->GetMac ().CancelAcknowledgedFrames (4, 1, 1, 0) == 1,
           "could not isolate same-scan retry entry from MAC");

  Ptr<Packet> payload = Create<Packet> (16);
  payload->AddHeader (CsrNetHeader (1, 3, 5));
  hop->SendData (2, 5, payload, true);
  Require (device->GetMac ().CancelAcknowledgedFrames (2, 1, 1, 0) == 1,
           "could not isolate same-scan final entry from MAC");
  hop->NotifyMacFrameSent (2, 1, Seconds (0));

  const Time tic = CsrOpnetTic ();
  const Time firstRetryAt = Seconds (2.0) + tic;
  const Time firstRetrySentAt = firstRetryAt + NanoSeconds (1);
  const Time secondRetryAt = firstRetrySentAt + Seconds (2.0) + tic;
  const Time secondRetrySentAt = secondRetryAt + NanoSeconds (1);
  const Time finalTimeoutAt = secondRetrySentAt + Seconds (4.0) + tic;
  const Time secondEntrySentAt = finalTimeoutAt - Seconds (2.0) - tic;

  Simulator::Schedule (firstRetrySentAt,
                       &ConfirmAndClearRetry,
                       device,
                       hop);
  Simulator::Schedule (secondRetrySentAt,
                       &ConfirmAndClearRetry,
                       device,
                       hop);
  Simulator::Schedule (secondEntrySentAt,
                       &ConfirmOrderingRetryEntry,
                       hop);
  Simulator::Schedule (finalTimeoutAt + NanoSeconds (1),
                       &CheckDeleteBeforeRetryResult,
                       device,
                       hop);
  Simulator::Stop (finalTimeoutAt + NanoSeconds (2));
  Simulator::Run ();
  Simulator::Destroy ();
  g_orderingDevice = nullptr;
}

void
RunExactTimerBoundaryCase ()
{
  g_nsdpReleases = 0;
  g_linkFailures = 0;
  g_nwkWakeTimes.clear ();

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseNsdp));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));

  // No NWK header: final expiry must request the generic wake even when there
  // is no NSDP or routing metadata to release.
  Ptr<Packet> payload = Create<Packet> (1);
  hop->SendData (2, 5, payload, true);
  Require (device->GetMac ().CancelAcknowledgedFrames (2, 1, 1, 0) == 1,
           "could not isolate the initial transmission from MAC");
  hop->NotifyMacFrameSent (2, 1, Seconds (0));

  const Time tic = CsrOpnetTic ();
  const Time firstRetryAt = Seconds (2.0) + tic;
  const Time firstRetryCheckAt = firstRetryAt + NanoSeconds (1);
  const Time firstRetrySentAt = firstRetryAt + NanoSeconds (2);
  const Time secondRetryAt = firstRetrySentAt + Seconds (2.0) + tic;
  const Time secondRetryCheckAt = secondRetryAt + NanoSeconds (1);
  const Time secondRetrySentAt = secondRetryAt + NanoSeconds (2);
  const Time finalTimeoutAt = secondRetrySentAt + Seconds (4.0) + tic;
  const Time finalTimeoutCheckAt = finalTimeoutAt + NanoSeconds (1);
  const Time finalWakeAt = finalTimeoutAt + tic;

  Simulator::Schedule (Seconds (2.0),
                       &CheckNoRetryAtNominal,
                       device,
                       hop);
  // Observe one nanosecond after the exact timer instant.  Later resend
  // timers are created dynamically by NotifyMacFrameSent(), so scheduling a
  // test callback at the same instant from here would run before the timer by
  // EventId order even though both timestamps are equal.
  Simulator::Schedule (firstRetryCheckAt,
                       &CheckRetryAtDelayedBoundary,
                       device,
                       hop);
  Simulator::Schedule (firstRetrySentAt,
                       &ConfirmAndClearRetry,
                       device,
                       hop);
  Simulator::Schedule (firstRetrySentAt + Seconds (2.0),
                       &CheckNoRetryAtNominal,
                       device,
                       hop);
  Simulator::Schedule (secondRetryCheckAt,
                       &CheckRetryAtDelayedBoundary,
                       device,
                       hop);
  Simulator::Schedule (secondRetrySentAt,
                       &ConfirmAndClearRetry,
                       device,
                       hop);
  Simulator::Schedule (secondRetrySentAt + Seconds (4.0),
                       &CheckFinalNominalBoundary,
                       hop);
  Simulator::Schedule (finalTimeoutCheckAt,
                       &CheckFinalDelayedBoundary,
                       device,
                       hop);
  Simulator::Schedule (finalWakeAt + NanoSeconds (1),
                       &CheckFinalWake,
                       finalWakeAt);
  Simulator::Stop (finalWakeAt + NanoSeconds (2));
  Simulator::Run ();
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (1);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (1);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseNsdp));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));

  // OPNET still transmits when the predicted link PER is one.  Three large,
  // higher-priority frames provide ordinary MAC contention so the DATA frame
  // remains queued long enough to verify that HOP starts its timer at the
  // actual transmission instant rather than the enqueue instant.
  CsrPerModelFn alwaysDrops =
    [] (int, double, uint32_t) { return 1.0; };
  device->GetPhy ().SetPerModel (alwaysDrops);

  for (uint16_t index = 0; index < BLOCKER_COUNT; ++index)
    {
      device->GetMac ().EnqueueTxFrame (
        BuildBlockerFrame (100 + index),
        2,
        7,
        false);
    }

  Ptr<Packet> payload = Create<Packet> (32);
  CsrNetHeader networkHeader (1, 3, 5);
  payload->AddHeader (networkHeader);

  hop->SendData (2, 5, payload, true);

  Simulator::Schedule (Seconds (2.5),
                       &CheckInitialFrameBehindBlockers,
                       device,
                       hop);
  // The initial frame follows all three blockers.  Its short 1--18-slot
  // reservation expires before the two-second HOP retry, but MAC remains in
  // post-TX Search, so each retry selects a new slot without restarting the
  // 300-ms holdoff.  These checkpoints cover every OPNET slot in [1,18].
  Simulator::Schedule (Seconds (5.7),
                       &CheckBeforeFirstRetry,
                       device,
                       hop);
  Simulator::Schedule (Seconds (7.6),
                       &CheckFirstRetry,
                       device);
  Simulator::Schedule (Seconds (10.2),
                       &CheckSecondRetry,
                       device);
  Simulator::Schedule (Seconds (12.2),
                       &CheckFinalGraceStillHeld,
                       hop);
  Simulator::Schedule (Seconds (14.3),
                       &CheckFinalTimeout,
                       device,
                       hop);

  Simulator::Stop (Seconds (14.6));
  Simulator::Run ();
  Simulator::Destroy ();

  RunExactTimerBoundaryCase ();
  RunMissingEntryFallbackCase ();
  RunDeleteBeforeRetryCase ();

  std::cout << "PASS: exact MAC/HOP resend timer and scan-order parity test"
            << std::endl;
  return 0;
}
