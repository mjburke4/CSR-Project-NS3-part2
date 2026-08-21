#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>

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

  Ptr<Packet> frame = Create<Packet> ();
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
  Require (g_nwkWakeups == 0,
           "first DACK woke NWK before its hold expired");
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
  Require (g_nwkWakeups == 0,
           "NWK woke while both DACK holds were active");
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
  Require (g_nwkWakeups == 1,
           "first DACK expiry did not wake NWK exactly once");
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
  Require (g_nwkWakeups == 2,
           "staggered DACK expiries did not wake NWK once each");
  Require (g_nsdpReleases == 2,
           "DACK expiry incorrectly released NSDP a second time");
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "OPNET DACK behavior allowed an unexpected retransmission");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC retained a DACKed frame");
  Require (g_linkFailures == 0,
           "DACKed traffic was incorrectly reported as a link failure");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

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

  // The first hold expires at 22.0 s; the second must remain held until its
  // independent OPNET deadline at 22.25 s.
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

  std::cout << "PASS: staggered DACK hold parity test" << std::endl;
  return 0;
}
