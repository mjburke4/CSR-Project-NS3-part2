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

constexpr CsrNodeId SOURCE_NODE = 1;
constexpr CsrNodeId DESTINATION_NODE = 2;

uint32_t g_deliveries = 0;

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
ConnectStack (Ptr<CsrNetDevice> device,
              Ptr<CsrHopLayer> hop,
              Ptr<CsrNetLayer> nwk,
              CsrNodeId nodeId)
{
  hop->SetNodeId (nodeId);
  hop->SetMac (&device->GetMac ());
  nwk->SetNodeId (nodeId);
  nwk->SetHop (hop);

  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

void
ReceivePayload (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "admitted DATA carried the wrong NWK source");
  Require (payload->GetSize () == 64,
           "admitted DATA payload changed size");
  g_deliveries++;
}

void
InjectBeforeAdmission (Ptr<CsrNetLayer> destinationNwk)
{
  Ptr<Packet> packet = Create<Packet> (32);
  CsrNetHeader header (SOURCE_NODE, DESTINATION_NODE, 0);
  packet->AddHeader (header);

  destinationNwk->ReceiveFromHop (packet, SOURCE_NODE);

  Require (g_deliveries == 0,
           "DATA from an inactive neighbor bypassed ARL admission");
  Require (destinationNwk->GetInactiveNeighborDropCount () == 1,
           "inactive-neighbor DATA drop was not recorded");
  Require (!destinationNwk->IsArlNeighborActive (SOURCE_NODE),
           "Neighbor became active before key/check completion");
}

void
CheckAdmissionAndSend (Ptr<CsrNetLayer> sourceNwk,
                       Ptr<CsrNetLayer> destinationNwk)
{
  Require (sourceNwk->HasReceivedHopKey (DESTINATION_NODE) &&
           sourceNwk->HasSentHopKey (DESTINATION_NODE),
           "source did not complete the two-sided group-key exchange");
  Require (destinationNwk->HasReceivedHopKey (SOURCE_NODE) &&
           destinationNwk->HasSentHopKey (SOURCE_NODE),
           "destination did not complete the two-sided group-key exchange");

  Require (sourceNwk->IsArlNeighborActive (DESTINATION_NODE),
           "source did not admit the neighbor after NeighborCheck");
  Require (destinationNwk->IsArlNeighborActive (SOURCE_NODE),
           "destination did not admit the neighbor after NeighborCheck");

  Require (sourceNwk->GetKeyRequestSentCount () +
             destinationNwk->GetKeyRequestSentCount () >= 1,
           "admission did not transmit a KeyRequest");
  Require (sourceNwk->GetKeyUpdateSentCount () >= 1 &&
           destinationNwk->GetKeyUpdateSentCount () >= 1,
           "reciprocal reliable KeyUpdates were not transmitted");
  Require (sourceNwk->GetKeyUpdateReceivedCount () >= 1 &&
           destinationNwk->GetKeyUpdateReceivedCount () >= 1,
           "reciprocal KeyUpdates were not received");

  uint32_t routeCost = 0;
  Require (sourceNwk->GetSelectedRouteCost (
             DESTINATION_NODE, routeCost),
           "admitted direct route did not become selectable");

  sourceNwk->Send (DESTINATION_NODE,
                   5,
                   Create<Packet> (64),
                   true);
}

void
CheckFinalState (Ptr<CsrNetLayer> sourceNwk,
                 Ptr<CsrHopLayer> sourceHop)
{
  Require (g_deliveries == 1,
           "DATA was not delivered exactly once after admission");
  Require (sourceNwk->GetNwkQueueSize () == 0,
           "source NWK queue did not drain after admission");
  Require (sourceHop->GetPendingDataCount () == 0,
           "source HOP DATA custody did not clear");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (DESTINATION_NODE);

  sourceDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (sourceDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sourceDevice->GetPhy ().SetPerModel (noErrors);
  destinationDevice->GetPhy ().SetPerModel (noErrors);
  sourceDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, DESTINATION_NODE, 1.0);
  destinationDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE_NODE, DESTINATION_NODE, 1.0);

  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();

  ConnectStack (sourceDevice, sourceHop, sourceNwk, SOURCE_NODE);
  ConnectStack (destinationDevice,
                destinationHop,
                destinationNwk,
                DESTINATION_NODE);

  sourceNwk->SetNodeType (CsrNodeType::Routable);
  destinationNwk->SetNodeType (CsrNodeType::Routable);
  destinationNwk->SetRxFromNetCallback (MakeCallback (&ReceivePayload));

  Simulator::ScheduleNow (
    &InjectBeforeAdmission,
    destinationNwk);

  Simulator::Schedule (Seconds (15),
                       &CheckAdmissionAndSend,
                       sourceNwk,
                       destinationNwk);

  Simulator::Stop (Seconds (30));
  Simulator::Run ();

  CheckFinalState (sourceNwk, sourceHop);

  Simulator::Destroy ();
  std::cout << "PASS: ARL key exchange and neighbor admission test"
            << std::endl;
  return 0;
}
