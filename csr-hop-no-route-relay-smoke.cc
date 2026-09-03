#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>

using namespace ns3;

namespace
{

constexpr CsrNodeId NWK_SOURCE = 0;
constexpr CsrNodeId UPSTREAM_HOP = 1;
constexpr CsrNodeId RELAY_NODE = 2;
constexpr CsrNodeId NWK_DESTINATION = 3;
constexpr uint16_t DATA_SEQUENCE = 41;

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

Ptr<Packet>
BuildRelayData ()
{
  Ptr<Packet> frame = Create<Packet> (64);
  frame->AddHeader (
    CsrNetHeader (NWK_SOURCE, NWK_DESTINATION, 5));

  CsrHeader hopHeader (UPSTREAM_HOP,
                       RELAY_NODE,
                       DATA_SEQUENCE,
                       5,
                       true,
                       false);
  hopHeader.SetType (CSR_PKT_DATA);
  hopHeader.SetDestType (CSR_DEST_UNICAST);
  hopHeader.SetSpeedKey (8);
  frame->AddHeader (hopHeader);
  return frame;
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> relayDevice =
    CreateObject<CsrNetDevice> (RELAY_NODE);
  Ptr<CsrHopLayer> relayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> relayNwk = CreateObject<CsrNetLayer> ();
  relayNwk->SetArlNeighborAdmissionEnabled (false);

  relayHop->SetNodeId (RELAY_NODE);
  relayHop->SetMac (&relayDevice->GetMac ());
  relayHop->SetHopWireProfile (CsrHopWireProfile::HIST_ADB97C54_BARE);
  relayNwk->SetNodeId (RELAY_NODE);
  relayNwk->SetHop (relayHop);
  relayNwk->SetNodeType (CsrNodeType::Routable);

  Ptr<Packet> relayData = BuildRelayData ();

  relayHop->ReceiveFromMac (relayData->Copy (), 70.0, 30.0);

  Require (relayDevice->GetMac ().GetAckQueuedFrameCount () == 0,
           "first no-route relay reception generated an ACK");
  Require (relayNwk->GetNwkQueueSize () == 0,
           "first no-route relay reception entered the NWK queue");
  Require (relayNwk->GetNsdpCount (NWK_SOURCE, NWK_DESTINATION) == 0,
           "first no-route relay reception changed NSDP state");

  relayHop->ReceiveFromMac (relayData->Copy (), 70.0, 30.0);

  Require (relayDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "duplicate no-route relay reception did not generate an ACK");
  Require (relayNwk->GetNwkQueueSize () == 0,
           "duplicate no-route relay reception entered the NWK queue");
  Require (relayNwk->GetNsdpCount (NWK_SOURCE, NWK_DESTINATION) == 0,
           "duplicate no-route relay reception changed NSDP state");

  Simulator::Destroy ();
  std::cout << "PASS: no-route relay custody/ACK parity test" << std::endl;
  return 0;
}
