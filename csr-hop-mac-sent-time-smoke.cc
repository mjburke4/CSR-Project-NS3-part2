#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

uint32_t g_nsdpReleases = 0;
uint32_t g_linkFailures = 0;

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
ReleaseNsdp (uint16_t src, uint16_t dst)
{
  Require (src == 1 && dst == 3,
           "final timeout released the wrong NSDP flow");
  g_nsdpReleases++;
}

void
ReportLinkFailure (uint16_t neighbor)
{
  Require (neighbor == 2,
           "final timeout reported the wrong failed neighbor");
  g_linkFailures++;
}

void
CheckDeferredInitialFrame (Ptr<CsrNetDevice> device,
                           Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 0,
           "MAC transmitted while the test link was unavailable");
  Require (device->GetMac ().GetQueuedFrameCount () == 1,
           "HOP enqueued a premature retransmission before the initial TX");
  Require (hop->GetPendingDataCount () == 1,
           "deferred initial frame did not retain its HOP pending slot");

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

void
CheckBeforeFirstRetry (Ptr<CsrNetDevice> device,
                       Ptr<CsrHopLayer> hop)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 1,
           "first retry was scheduled from HOP enqueue time instead of MAC sent time");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "unexpected frame remained queued before the first retry");
  Require (hop->GetPendingDataCount () == 1,
           "initial transmission released HOP flow control without an ACK");
}

void
CheckFirstRetry (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 2,
           "first OPNET resend did not occur two seconds after actual TX");
}

void
CheckSecondRetry (Ptr<CsrNetDevice> device)
{
  Require (device->GetMac ().GetTransmittedFrameCount () == 3,
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
  Require (device->GetMac ().GetTransmittedFrameCount () == 3,
           "OPNET parity requires one initial TX plus exactly two resends");
  Require (device->GetMac ().GetQueuedFrameCount () == 0,
           "MAC retained a frame after final HOP timeout");
  Require (hop->GetPendingDataCount () == 0,
           "final timeout did not release the global HOP pending count");
  Require (hop->GetOutstandingDataCount (2) == 0,
           "final timeout did not release per-neighbor flow control");
  Require (g_nsdpReleases == 1,
           "final timeout did not release NSDP exactly once");
  Require (g_linkFailures == 1,
           "final timeout did not report exactly one link failure");
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

  // Keep the original frame in MAC for three seconds.  OPNET does not start
  // the HOP resend timer until MAC confirms the actual transmission instant.
  CsrPerModelFn unavailable =
    [] (int, double, uint32_t) { return 1.0; };
  device->GetPhy ().SetPerModel (unavailable);

  Ptr<Packet> payload = Create<Packet> (32);
  CsrNetHeader networkHeader (1, 3, 5);
  payload->AddHeader (networkHeader);

  hop->SendData (2, 5, payload, true);

  Simulator::Schedule (Seconds (2.5),
                       &CheckDeferredInitialFrame,
                       device,
                       hop);
  Simulator::Schedule (Seconds (4.9),
                       &CheckBeforeFirstRetry,
                       device,
                       hop);
  Simulator::Schedule (Seconds (5.1),
                       &CheckFirstRetry,
                       device);
  Simulator::Schedule (Seconds (7.1),
                       &CheckSecondRetry,
                       device);
  Simulator::Schedule (Seconds (10.9),
                       &CheckFinalGraceStillHeld,
                       hop);
  Simulator::Schedule (Seconds (11.1),
                       &CheckFinalTimeout,
                       device,
                       hop);

  Simulator::Stop (Seconds (12.0));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "PASS: MAC/HOP sent-time parity test" << std::endl;
  return 0;
}
