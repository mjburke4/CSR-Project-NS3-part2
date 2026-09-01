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

constexpr CsrNodeId SOURCE_NODE = 1;
constexpr CsrNodeId FIRST_NEIGHBOR = 2;
constexpr CsrNodeId SECOND_NEIGHBOR = 3;
constexpr CsrNodeId FIRST_NWK_DESTINATION = 10;
constexpr CsrNodeId SECOND_NWK_DESTINATION = 11;

uint32_t g_nsdpReleases = 0;
uint32_t g_nwkWakeups = 0;
uint32_t g_linkFailures = 0;
bool g_firstFlowReleased = false;
bool g_secondFlowReleased = false;
std::vector<Time> g_nwkWakeTimes;

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
  Require (src == SOURCE_NODE,
           "DACK released an NSDP flow with the wrong source");

  if (dst == FIRST_NWK_DESTINATION)
    {
      Require (!g_firstFlowReleased,
               "first DACK released NSDP more than once");
      g_firstFlowReleased = true;
    }
  else if (dst == SECOND_NWK_DESTINATION)
    {
      Require (!g_secondFlowReleased,
               "second DACK released NSDP more than once");
      g_secondFlowReleased = true;
    }
  else
    {
      Require (false, "DACK released the wrong NSDP destination");
    }

  g_nsdpReleases++;
}

void
WakeNwkQueue ()
{
  g_nwkWakeups++;
  g_nwkWakeTimes.push_back (Simulator::Now ());
}

void
ReportLinkFailure (CsrNodeId)
{
  g_linkFailures++;
}

Ptr<Packet>
BuildDack (CsrNodeId neighbor)
{
  CsrHeader header (neighbor,
                    SOURCE_NODE,
                    1,
                    7,
                    false,
                    true);
  header.SetIsDack (true);
  header.SetType (CSR_PKT_DACK);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);
  header.SetHasAckWindow (true);
  header.SetAckBitmap (0);
  header.SetDackBitmap (1);

  Ptr<Packet> body = Create<Packet> ();
  body->AddHeader (header);
  std::vector<uint8_t> plaintext (body->GetSize ());
  body->CopyData (plaintext.data (), plaintext.size ());

  CsrHopSecurityState sender;
  sender.SetNodeId (neighbor);
  CsrProtectedPairwiseMessage secured =
    sender.ProtectPairwiseMessage (
      SOURCE_NODE,
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      plaintext);
  Ptr<Packet> frame = Create<Packet> (secured.record.data (),
                                      secured.record.size ());
  header.SetSecurityCount (sender.GetOwnSecurityCount ());
  header.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (header);
  return frame;
}

void
InjectDack (Ptr<CsrHopLayer> hop, CsrNodeId neighbor)
{
  hop->ReceiveFromMac (BuildDack (neighbor), 60.0, 40.0);
}

void
CheckInitialTransmissions (Ptr<CsrNetDevice> device,
                           Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "MAC did not concatenate both initial DATA frames");
  Require (hop->GetPendingDataCount () == 2,
           "initial DATA frames did not occupy two global HOP slots");
  Require (hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1,
           "first neighbor did not retain its HOP slot");
  Require (hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 1,
           "second neighbor did not retain its HOP slot");
}

void
CheckFirstDack (Ptr<CsrHopLayer> hop)
{
  Require (g_nsdpReleases == 1 && g_firstFlowReleased,
           "first DACK did not immediately release its NSDP flow");
  Require (hop->GetPendingDataCount () == 2,
           "first DACK prematurely released global HOP flow control");
  Require (hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1,
           "first DACK prematurely released its neighbor slot");
  Require (g_nwkWakeups == 1 && g_nwkWakeTimes.size () == 1 &&
             g_nwkWakeTimes[0] == Seconds (2.0) + CsrOpnetTic (),
           "first DACK did not produce its generic wake one TIC later");
}

void
CheckBothDacksHeld (Ptr<CsrNetDevice> device,
                    Ptr<CsrHopLayer> hop)
{
  Require (g_nsdpReleases == 2 &&
             g_firstFlowReleased &&
             g_secondFlowReleased,
           "both DACKs did not immediately release NSDP");
  Require (hop->GetPendingDataCount () == 2,
           "DACKs did not retain both global HOP slots");
  Require (hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1 &&
             hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 1,
           "DACKs did not retain both per-neighbor slots");
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "DACKed frames were retransmitted");
  Require (g_nwkWakeups == 2 && g_nwkWakeTimes.size () == 2 &&
             g_nwkWakeTimes[1] == Seconds (2.25) + CsrOpnetTic (),
           "staggered DACK feedback did not produce two delayed wakes");
}

void
CheckFirstExpiryOnly (Ptr<CsrNetDevice> device,
                      Ptr<CsrHopLayer> hop)
{
  Require (hop->GetPendingDataCount () == 1,
           "first exact DACK expiry did not release one global slot");
  Require (hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 0,
           "first exact DACK expiry did not release its neighbor slot");
  Require (hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 1,
           "second staggered DACK was released too early");
  Require (g_nwkWakeups == 3,
           "first DACK expiry did not wake NWK exactly once");
  Require (g_nwkWakeTimes.size () == 3 &&
             g_nwkWakeTimes[2] == Seconds (22.0) + 2 * CsrOpnetTic (),
           "first DACK expiry wake was not one TIC after capacity release");
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "DACKed frames were retransmitted during the hold");
}

void
CheckFinalState (Ptr<CsrNetDevice> device,
                 Ptr<CsrHopLayer> hop)
{
  Require (hop->GetPendingDataCount () == 0,
           "second exact DACK expiry did not release global flow control");
  Require (hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 0 &&
             hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 0,
           "DACK expiry left a per-neighbor slot occupied");
  Require (g_nwkWakeups == 4,
           "staggered DACK expiries did not wake NWK once each");
  Require (g_nwkWakeTimes.size () == 4 &&
             g_nwkWakeTimes[3] == Seconds (22.25) + 2 * CsrOpnetTic (),
           "second DACK expiry wake was not one TIC after capacity release");
  Require (g_nsdpReleases == 2,
           "DACK expiry incorrectly released NSDP a second time");
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "OPNET DACK behavior allowed an unexpected retransmission");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC retained a DACKed frame");
  Require (g_linkFailures == 0,
           "DACKed traffic was incorrectly reported as a link failure");
}

void
CheckFirstNominalExpiry (Ptr<CsrHopLayer> hop)
{
  Require (hop->GetPendingDataCount () == 2 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1,
           "first DACK capacity released at the nominal 20-second expiry");
  Require (g_nwkWakeups == 2,
           "capacity released or an extra wake ran at nominal DACK expiry");
}

void
ResetObservations ()
{
  g_nsdpReleases = 0;
  g_nwkWakeups = 0;
  g_linkFailures = 0;
  g_firstFlowReleased = false;
  g_secondFlowReleased = false;
  g_nwkWakeTimes.clear ();
}

void
BackdateSentConfirmation (Ptr<CsrHopLayer> hop)
{
  // Use the production MAC-sent callback to make the real resend entry due
  // immediately.  CheckResend() then advances its resend count once without a
  // test-only model hook.
  hop->NotifyMacFrameSent (FIRST_NEIGHBOR,
                           1,
                           Simulator::Now () - Seconds (2.0) - CsrOpnetTic ());
}

void
CheckMaxResendDackHeld (Ptr<CsrHopLayer> hop)
{
  Require (g_nsdpReleases == 1 && g_firstFlowReleased,
           "max-resend DACK did not release NSDP immediately");
  Require (hop->GetPendingDataCount () == 1 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1,
           "max-resend DACK did not retain HOP capacity");
  Require (g_nwkWakeups == 1,
           "max-resend DACK did not issue one coalesced generic wake");
}

void
CheckMaxResendNominalExpiry (Ptr<CsrHopLayer> hop)
{
  Require (hop->GetPendingDataCount () == 1 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1,
           "max-resend DACK released capacity at the nominal 40-second expiry");
  Require (g_nwkWakeups == 1,
           "max-resend DACK issued an extra wake at nominal expiry");
}

void
CheckMaxResendFinalState (Ptr<CsrNetDevice> device,
                          Ptr<CsrHopLayer> hop,
                          Time expectedRelease)
{
  Require (hop->GetPendingDataCount () == 0 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 0,
           "max-resend DACK did not release HOP capacity");
  Require (g_nwkWakeups == 2 && g_nwkWakeTimes.size () == 2 &&
             g_nwkWakeTimes[1] == expectedRelease,
           "max-resend DACK wake was not one TIC after capacity release");
  Require (g_nsdpReleases == 1,
           "max-resend DACK expiry released NSDP twice");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "max-resend DACK left a retransmission queued at MAC");
  Require (g_linkFailures == 0,
           "max-resend DACK was reported as a link failure");
}

void
CheckCoalescedDacksHeld (Ptr<CsrHopLayer> hop)
{
  Require (g_nsdpReleases == 2 &&
             g_firstFlowReleased &&
             g_secondFlowReleased,
           "coalesced DACK setup did not release both NSDP flows");
  Require (hop->GetPendingDataCount () == 2 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 1 &&
             hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 1,
           "nearby DACK capacity released before the first timer scan");
  Require (g_nwkWakeups == 1,
           "nearby DACK feedback did not coalesce on its first generic wake");
}

void
CheckCoalescedDackFinalState (Ptr<CsrNetDevice> device,
                              Ptr<CsrHopLayer> hop,
                              Time expectedRelease)
{
  Require (hop->GetPendingDataCount () == 0 &&
             hop->GetOutstandingDataCount (FIRST_NEIGHBOR) == 0 &&
             hop->GetOutstandingDataCount (SECOND_NEIGHBOR) == 0,
           "one DACK timer scan did not release both nominally expired holds");
  Require (g_nwkWakeups == 2 && g_nwkWakeTimes.size () == 2 &&
             g_nwkWakeTimes[1] == expectedRelease,
           "nearby DACK expiry wakes did not coalesce after the timer scan");
  Require (g_nsdpReleases == 2,
           "coalesced DACK timer released NSDP more than once");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "coalesced DACKs left DATA queued at MAC");
  Require (g_linkFailures == 0,
           "coalesced DACK traffic was reported as a link failure");
}

void
RunMaxResendTimingCase ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> device =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (SOURCE_NODE);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseNsdp));
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));

  Ptr<Packet> payload = Create<Packet> (32);
  payload->AddHeader (
    CsrNetHeader (SOURCE_NODE, FIRST_NWK_DESTINATION, 5));
  hop->SendData (FIRST_NEIGHBOR, 5, payload, true);

  const Time dackAt = NanoSeconds (2);
  const Time nominalExpiry = dackAt + Seconds (40.0);
  const Time capacityReleaseAt = nominalExpiry + CsrOpnetTic ();
  const Time nwkWakeAt = capacityReleaseAt + CsrOpnetTic ();

  Simulator::ScheduleNow (&BackdateSentConfirmation, hop);
  Simulator::Schedule (NanoSeconds (1), &BackdateSentConfirmation, hop);
  Simulator::Schedule (dackAt, &InjectDack, hop, FIRST_NEIGHBOR);
  Simulator::Schedule (dackAt + CsrOpnetTic () + NanoSeconds (1),
                       &CheckMaxResendDackHeld,
                       hop);
  // Probe just after the nominal deadline so the assertion remains robust
  // even when an implementation schedules its timer at that exact timestamp.
  Simulator::Schedule (nominalExpiry + NanoSeconds (1),
                       &CheckMaxResendNominalExpiry,
                       hop);
  Simulator::Schedule (nwkWakeAt + NanoSeconds (1),
                       &CheckMaxResendFinalState,
                       device,
                       hop,
                       nwkWakeAt);
  Simulator::Stop (nwkWakeAt + NanoSeconds (2));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
RunCoalescedTimingCase ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> device =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (SOURCE_NODE);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseNsdp));
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));

  Ptr<Packet> firstPayload = Create<Packet> (32);
  firstPayload->AddHeader (
    CsrNetHeader (SOURCE_NODE, FIRST_NWK_DESTINATION, 5));
  Ptr<Packet> secondPayload = Create<Packet> (32);
  secondPayload->AddHeader (
    CsrNetHeader (SOURCE_NODE, SECOND_NWK_DESTINATION, 5));
  hop->SendData (FIRST_NEIGHBOR, 5, firstPayload, true);
  hop->SendData (SECOND_NEIGHBOR, 5, secondPayload, true);

  const Time firstDackAt = NanoSeconds (2);
  const Time secondDackAt = NanoSeconds (12);
  const Time secondNominalExpiry = secondDackAt + Seconds (20.0);
  const Time firstReleaseAt =
    firstDackAt + Seconds (20.0) + CsrOpnetTic ();
  const Time nwkWakeAt = firstReleaseAt + CsrOpnetTic ();

  Simulator::Schedule (firstDackAt,
                       &InjectDack,
                       hop,
                       FIRST_NEIGHBOR);
  Simulator::Schedule (secondDackAt,
                       &InjectDack,
                       hop,
                       SECOND_NEIGHBOR);
  Simulator::Schedule (secondNominalExpiry + NanoSeconds (1),
                       &CheckCoalescedDacksHeld,
                       hop);
  // The first entry's timer runs 18 ns after the second entry's nominal
  // expiry, so the source's list-wide check releases both entries then.  The
  // second entry's own timer remains pending but has no capacity left to free.
  Simulator::Schedule (nwkWakeAt + NanoSeconds (1),
                       &CheckCoalescedDackFinalState,
                       device,
                       hop,
                       nwkWakeAt);
  Simulator::Stop (nwkWakeAt + NanoSeconds (2));
  Simulator::Run ();
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  ResetObservations ();

  Ptr<CsrNetDevice> device =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();
  hop->SetNodeId (SOURCE_NODE);
  hop->SetMac (&device->GetMac ());
  hop->SetNsdpDecrementCallback (MakeCallback (&ReleaseNsdp));
  hop->SetNwkQueueWakeCallback (MakeCallback (&WakeNwkQueue));
  hop->SetLinkFailureCallback (MakeCallback (&ReportLinkFailure));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);

  Ptr<Packet> firstPayload = Create<Packet> (32);
  firstPayload->AddHeader (
    CsrNetHeader (SOURCE_NODE, FIRST_NWK_DESTINATION, 5));

  Ptr<Packet> secondPayload = Create<Packet> (32);
  secondPayload->AddHeader (
    CsrNetHeader (SOURCE_NODE, SECOND_NWK_DESTINATION, 5));

  hop->SendData (FIRST_NEIGHBOR, 5, firstPayload, true);
  hop->SendData (SECOND_NEIGHBOR, 5, secondPayload, true);

  // The two DATA frames now consume real OPNET MAC slot countdowns and
  // serialized airtime.  Both are guaranteed sent by 1.8 seconds for the
  // active_nodes=1 slot range.
  Simulator::Schedule (Seconds (1.8),
                       &CheckInitialTransmissions,
                       device,
                       hop);
  Simulator::Schedule (Seconds (2.0),
                       &InjectDack,
                       hop,
                       FIRST_NEIGHBOR);
  Simulator::Schedule (Seconds (2.1),
                       &CheckFirstDack,
                       hop);
  Simulator::Schedule (Seconds (2.25),
                       &InjectDack,
                       hop,
                       SECOND_NEIGHBOR);
  Simulator::Schedule (Seconds (2.5),
                       &CheckBothDacksHeld,
                       device,
                       hop);

  // The source stores nominal expiries at 22.0 and 22.25 seconds, but executes
  // each DACK_TIMER one 36-MHz TIC later.
  Simulator::Schedule (Seconds (22.0) + NanoSeconds (1),
                       &CheckFirstNominalExpiry,
                       hop);
  Simulator::Schedule (Seconds (22.1),
                       &CheckFirstExpiryOnly,
                       device,
                       hop);
  Simulator::Schedule (Seconds (22.3),
                       &CheckFinalState,
                       device,
                       hop);

  Simulator::Stop (Seconds (22.5));
  Simulator::Run ();
  Simulator::Destroy ();

  RunMaxResendTimingCase ();
  RunCoalescedTimingCase ();

  std::cout << "PASS: exact 20/40-second DACK timer parity test" << std::endl;
  return 0;
}
