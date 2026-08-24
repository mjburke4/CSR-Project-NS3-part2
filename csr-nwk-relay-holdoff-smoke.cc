#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE_NODE = 1;
constexpr CsrNodeId RELAY_NODE = 2;
constexpr CsrNodeId DESTINATION_NODE = 3;

Ptr<CsrNetLayer> g_sourceNwk;
Ptr<CsrNetLayer> g_relayNwk;
Ptr<CsrHopLayer> g_sourceHop;
Ptr<CsrHopLayer> g_relayHop;
uint32_t g_delivered = 0;

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
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
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
  nwk->SetArlNeighborAdmissionEnabled (false);
  nwk->SetAutomaticRoutePropagationEnabled (false);
  nwk->SetHop (hop);

  device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop));
}

Ptr<Packet>
BuildHelloFrame (CsrNodeId source)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (1);
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (2);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (hello);

  CsrHeader hopHeader (
    source,
    CSR_BROADCAST_ID,
    1,
    7,
    false,
    false);
  hopHeader.SetType (CSR_PKT_HELLO);
  hopHeader.SetDestType (CSR_DEST_BROADCAST);
  hopHeader.SetLinkControl (8, 30.0, -105.0);
  frame->AddHeader (hopHeader);
  return frame;
}

Ptr<Packet>
BuildRelayControlFrame (CsrNodeId source, CsrSnmpCommand command)
{
  CsrSnmpHeader snmp;
  snmp.SetSource (source);
  snmp.SetDestination (RELAY_NODE);
  snmp.SetDestinationType (CSR_DEST_UNICAST);
  snmp.SetCommand (command);
  snmp.SetValue (0);

  Ptr<Packet> frame = Create<Packet> ();
  frame->AddHeader (snmp);

  CsrHeader hopHeader (
    source,
    RELAY_NODE,
    0,
    0,
    false,
    false);
  hopHeader.SetType (CSR_PKT_SNMP);
  hopHeader.SetDestType (CSR_DEST_UNICAST);
  hopHeader.SetLinkControl (8, 30.0, -105.0);
  frame->AddHeader (hopHeader);
  return frame;
}

void
CheckWireValues ()
{
  static_assert (CSR_SNMP_START_REDISCOVER == 3);
  static_assert (CSR_SNMP_PRINT_ROUTES_INFO == 4);
  static_assert (CSR_SNMP_RELAY_HOLDOFF == 5);
  static_assert (CSR_SNMP_RELAY_CLEAR == 6);

  CsrSnmpHeader header;
  header.SetSource (0x010203);
  header.SetDestination (0x040506);
  header.SetDestinationType (CSR_DEST_UNICAST);
  header.SetCommand (CSR_SNMP_RELAY_HOLDOFF);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);

  std::vector<uint8_t> bytes (packet->GetSize ());
  packet->CopyData (bytes.data (), bytes.size ());

  Require (bytes.size () == 13,
           "relay control changed the empty-node SNMP envelope size");
  Require (bytes[7] == 5,
           "SNMP_RELAY_HOLDOFF did not serialize as command 5");

  header.SetCommand (CSR_SNMP_RELAY_CLEAR);
  packet = Create<Packet> ();
  packet->AddHeader (header);
  packet->CopyData (bytes.data (), bytes.size ());

  Require (bytes[7] == 6,
           "SNMP_RELAY_CLEAR did not serialize as command 6");
}

void
RecordDelivery (Ptr<Packet>, CsrNodeId source)
{
  Require (source == SOURCE_NODE,
           "relay scenario delivered a payload with the wrong NWK source");
  g_delivered++;
}

void
CheckHoldoffThenClearOrdering ()
{
  Require (!g_relayNwk->IsRelayHoldoffSet (SOURCE_NODE),
           "same-priority HOLD/CLEAR controls did not retain FIFO order");
  Require (g_relayNwk->GetRelayHoldoffReceivedCount () == 1 &&
             g_relayNwk->GetRelayClearReceivedCount () == 1,
           "relay did not apply the first HOLD/CLEAR pair exactly once");
  Require (g_relayHop->GetNeighborLastHeardSeconds (SOURCE_NODE) == 0.0,
           "legacy SNMP incorrectly refreshed the HOP neighbor timestamp");
  Require (g_sourceHop->GetPendingDataCount () == 0 &&
             g_sourceHop->GetResendQueueSize () == 0,
           "relay control entered DATA flow control or the resend queue");
}

void
SendSecondHoldoff ()
{
  Require (g_sourceNwk->SendRelayHoldoff (RELAY_NODE),
           "second relay-holdoff control was not queued");
}

void
CheckHoldoffAndSendRelayData ()
{
  Require (g_relayNwk->IsRelayHoldoffSet (SOURCE_NODE),
           "second relay-holdoff control did not set the stored flag");
  Require (g_relayNwk->GetRelayHoldoffReceivedCount () == 2,
           "relay-holdoff receive ordering lost or duplicated a control");

  g_sourceNwk->Send (
    DESTINATION_NODE,
    5,
    Create<Packet> (32),
    true);
}

void
CheckStoredFlagIsNotTransitGate ()
{
  Require (g_delivered == 1,
           "stored legacy relay_holdoff flag incorrectly blocked transit");
  Require (g_relayNwk->IsRelayHoldoffSet (SOURCE_NODE),
           "transit DATA unexpectedly changed relay-control state");

  Require (g_sourceNwk->SendRelayClear (RELAY_NODE),
           "final relay-clear control was not queued");
}

void
CheckFinalState ()
{
  Require (!g_relayNwk->IsRelayHoldoffSet (SOURCE_NODE),
           "final relay-clear control did not clear stored state");
  Require (g_sourceNwk->GetRelayHoldoffSentCount () == 2 &&
             g_sourceNwk->GetRelayClearSentCount () == 2,
           "sender relay-control counters do not match queued controls");
  Require (g_relayNwk->GetRelayHoldoffReceivedCount () == 2 &&
             g_relayNwk->GetRelayClearReceivedCount () == 2,
           "receiver relay-control counters do not match applied controls");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  CheckWireValues ();

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (SOURCE_NODE);
  Ptr<CsrNetDevice> relayDevice =
    CreateObject<CsrNetDevice> (RELAY_NODE);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (DESTINATION_NODE);

  sourceDevice->AddPeer (relayDevice);
  relayDevice->AddPeer (sourceDevice);
  relayDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (relayDevice);

  ConfigureNoErrors (sourceDevice);
  ConfigureNoErrors (relayDevice);
  ConfigureNoErrors (destinationDevice);

  g_sourceHop = CreateObject<CsrHopLayer> ();
  g_relayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  g_sourceNwk = CreateObject<CsrNetLayer> ();
  g_relayNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();

  ConnectStack (
    sourceDevice, g_sourceHop, g_sourceNwk, SOURCE_NODE);
  ConnectStack (
    relayDevice, g_relayHop, g_relayNwk, RELAY_NODE);
  ConnectStack (
    destinationDevice,
    destinationHop,
    destinationNwk,
    DESTINATION_NODE);

  g_sourceNwk->AddStaticRouteWithPathloss (
    RELAY_NODE,
    RELAY_NODE,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));
  g_sourceNwk->AddStaticRouteWithPathloss (
    DESTINATION_NODE,
    RELAY_NODE,
    70.0,
    false,
    static_cast<uint8_t> (CsrNodeType::Routable));
  g_relayNwk->AddStaticRouteWithPathloss (
    DESTINATION_NODE,
    DESTINATION_NODE,
    70.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));

  destinationNwk->SetRxFromNetCallback (
    MakeCallback (&RecordDelivery));

  g_relayHop->ReceiveFromMac (
    BuildRelayControlFrame (99, CSR_SNMP_RELAY_HOLDOFF),
    70.0,
    20.0);
  Require (!g_relayNwk->IsRelayHoldoffSet (99) &&
             g_relayNwk->GetRelayHoldoffReceivedCount () == 0,
           "unknown relay control created or changed neighbor state");
  Require (g_relayHop->GetNeighborLastHeardSeconds (99) < 0.0,
           "unknown relay control created a HOP neighbor");

  // The legacy HOP path dispatches SNMP before update_neighbor().  Prime the
  // source once, then prove that the controls themselves do not refresh it.
  g_relayHop->ReceiveFromMac (
    BuildHelloFrame (SOURCE_NODE), 70.0, 20.0);
  Require (g_relayHop->GetNeighborLastHeardSeconds (SOURCE_NODE) == 0.0,
           "initial HELLO did not prime the relay neighbor");

  // Equal-DSCP SNMP controls retain arrival order in the MAC queue.  Both can
  // be concatenated into one transmission, but HOLD must still precede CLEAR.
  Require (g_sourceNwk->SendRelayHoldoff (RELAY_NODE),
           "initial relay-holdoff control was not queued");
  Require (g_sourceNwk->SendRelayClear (RELAY_NODE),
           "initial relay-clear control was not queued");
  Require (sourceDevice->GetMac ().GetDataQueuedFrameCount () == 2,
           "relay controls did not enter the ordinary OPNET MAC TX queue");
  Require (sourceDevice->GetMac ().GetAckQueuedFrameCount () == 0,
           "relay controls incorrectly entered the MAC ACK queue");

  Simulator::Schedule (
    Seconds (2.0), &CheckHoldoffThenClearOrdering);
  Simulator::Schedule (
    Seconds (2.1), &SendSecondHoldoff);
  Simulator::Schedule (
    Seconds (4.0), &CheckHoldoffAndSendRelayData);
  Simulator::Schedule (
    Seconds (8.0), &CheckStoredFlagIsNotTransitGate);
  Simulator::Schedule (
    Seconds (10.5), &CheckFinalState);

  Simulator::Stop (Seconds (11.0));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout
    << "PASS: relay-holdoff signaling and control-order parity test"
    << std::endl;
  return 0;
}
