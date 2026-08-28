#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId GATEWAY_NODE = 0;
constexpr CsrNodeId OLDER_DESTINATION = 1;
constexpr CsrNodeId NEWER_DESTINATION = 2;
constexpr CsrNodeId REVERSE_ONLY_DESTINATION = 3;

struct DiscoverySnapshot
{
  bool active {false};
  uint32_t starts {0};
  uint32_t broadcasts {0};
};

std::vector<std::string> g_failures;
uint32_t g_reverseStartFrames = 0;
CsrNodeId g_reverseStartHop = CSR_BROADCAST_ID;
CsrNodeId g_reverseStartDestination = CSR_BROADCAST_ID;
CsrDestType g_reverseStartDestinationType = CSR_DEST_BROADCAST;

Ptr<Packet>
MakePlainHello (CsrNodeId source)
{
  CsrHelloHeader hello;
  hello.SetNodeId (source);
  hello.SetHelloSeq (1);
  hello.SetNodeType (CsrNodeType::Routable);
  hello.SetSpeedKey (8);
  hello.SetRxPowerDbmX10 (-1050);
  hello.SetActiveNodes (1);
  hello.SetArlRouteMsgType (CsrArlRouteMsgType::None);

  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (hello);
  return packet;
}

void
Expect (bool condition, const char *message)
{
  if (!condition)
    {
      g_failures.emplace_back (message);
    }
}

void
CaptureDiscovery (Ptr<CsrNetLayer> nwk, DiscoverySnapshot *snapshot)
{
  snapshot->active = nwk->IsDiscoveryActive ();
  snapshot->starts = nwk->GetDiscoveryStartCount ();
  snapshot->broadcasts = nwk->GetDiscoveryBroadcastCount ();
}

void
ObserveReverseSnmp (Ptr<Packet> frame, double, double)
{
  CsrHeader hopHeader;
  if (!frame->RemoveHeader (hopHeader) ||
      hopHeader.GetType () != CSR_PKT_SNMP)
    {
      return;
    }

  CsrSnmpHeader snmpHeader;
  if (!frame->RemoveHeader (snmpHeader) ||
      snmpHeader.GetCommand () != CSR_SNMP_START_DISCOVERY)
    {
      return;
    }

  g_reverseStartFrames++;
  g_reverseStartHop = hopHeader.GetDst ();
  g_reverseStartDestination = snmpHeader.GetDestination ();
  g_reverseStartDestinationType = snmpHeader.GetDestinationType ();
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
RunCadenceAndCompletionCase ()
{
  Ptr<CsrNetLayer> gateway = CreateObject<CsrNetLayer> ();
  gateway->SetNodeId (GATEWAY_NODE);
  gateway->SetNodeType (CsrNodeType::Gateway);

  DiscoverySnapshot beforeStart;
  DiscoverySnapshot afterFirst;
  DiscoverySnapshot beforeSecond;
  DiscoverySnapshot afterSecond;
  DiscoverySnapshot beforeThird;
  DiscoverySnapshot afterThird;
  DiscoverySnapshot beforeCompletion;
  DiscoverySnapshot afterCompletion;

  // br_nwk schedules the gateway at +10 seconds.  routesDiscoveryInitLocal()
  // then broadcasts at 10, 15, and 20 seconds.  The fourth five-second
  // broadcaster callback ends discovery at 25 seconds; the +30-second NWK
  // stop is only a fallback.
  gateway->ScheduleGatewayStartupDiscovery ();

  Simulator::Schedule (Seconds (10.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeStart);
  Simulator::Schedule (Seconds (10.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterFirst);
  Simulator::Schedule (Seconds (15.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeSecond);
  Simulator::Schedule (Seconds (15.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterSecond);
  Simulator::Schedule (Seconds (20.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeThird);
  Simulator::Schedule (Seconds (20.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterThird);
  Simulator::Schedule (Seconds (25.0) - NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &beforeCompletion);
  Simulator::Schedule (Seconds (25.0) + NanoSeconds (1),
                       &CaptureDiscovery,
                       gateway,
                       &afterCompletion);

  Simulator::Stop (Seconds (26.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Expect (!beforeStart.active &&
            beforeStart.starts == 0 &&
            beforeStart.broadcasts == 0,
          "gateway discovery started before the OPNET +10-second event");
  Expect (afterFirst.active &&
            afterFirst.starts == 1 &&
            afterFirst.broadcasts == 1,
          "gateway did not emit its first discovery at 10 seconds");
  Expect (beforeSecond.active && beforeSecond.broadcasts == 1,
          "gateway emitted a repeat before the five-second OPNET interval");
  Expect (afterSecond.active && afterSecond.broadcasts == 2,
          "gateway did not emit its second discovery at 15 seconds");
  Expect (beforeThird.active && beforeThird.broadcasts == 2,
          "gateway emitted its third discovery before 20 seconds");
  Expect (afterThird.active && afterThird.broadcasts == 3,
          "gateway did not emit its third discovery at 20 seconds");
  Expect (beforeCompletion.active && beforeCompletion.broadcasts == 3,
          "gateway discovery completed before the 25-second fourth tick");
  Expect (!afterCompletion.active &&
            afterCompletion.starts == 1 &&
            afterCompletion.broadcasts == 3,
          "gateway discovery did not complete on the 25-second fourth tick");
}

void
RunNewestDestinationFirstCase ()
{
  Ptr<CsrNetDevice> gatewayDevice =
    CreateObject<CsrNetDevice> (GATEWAY_NODE);
  Ptr<CsrNetDevice> olderDevice =
    CreateObject<CsrNetDevice> (OLDER_DESTINATION);
  Ptr<CsrNetDevice> newerDevice =
    CreateObject<CsrNetDevice> (NEWER_DESTINATION);

  gatewayDevice->AddPeer (olderDevice);
  gatewayDevice->AddPeer (newerDevice);
  olderDevice->AddPeer (gatewayDevice);
  newerDevice->AddPeer (gatewayDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  gatewayDevice->GetPhy ().SetPerModel (noErrors);
  olderDevice->GetPhy ().SetPerModel (noErrors);
  newerDevice->GetPhy ().SetPerModel (noErrors);

  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  olderDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, NEWER_DESTINATION, 1.0);
  newerDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, NEWER_DESTINATION, 1.0);

  Ptr<CsrHopLayer> gatewayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> olderHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> newerHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> gatewayNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> olderNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> newerNwk = CreateObject<CsrNetLayer> ();

  ConnectStack (
    gatewayDevice, gatewayHop, gatewayNwk, GATEWAY_NODE);
  ConnectStack (
    olderDevice, olderHop, olderNwk, OLDER_DESTINATION);
  ConnectStack (
    newerDevice, newerHop, newerNwk, NEWER_DESTINATION);

  gatewayNwk->SetNodeType (CsrNodeType::Gateway);
  olderNwk->SetNodeType (CsrNodeType::Routable);
  newerNwk->SetNodeType (CsrNodeType::Routable);

  // Keep this case focused on logical-destination ordering rather than ARL
  // admission.  routesFindDestination() inserts every newly created logical
  // destination at destHead, so destination 2 must precede destination 1
  // when clear_discovery() tail-appends that walk into the discovery table.
  gatewayNwk->SetArlNeighborAdmissionEnabled (false);
  olderNwk->SetArlNeighborAdmissionEnabled (false);
  newerNwk->SetArlNeighborAdmissionEnabled (false);

  // routesGetRelay() creates a logical destination even when lookup fails.
  // Create destination 1 first and destination 2 second, then add route
  // records in the oldest-first vector order used by the old model.
  Expect (!gatewayNwk->HasRelayRoute (OLDER_DESTINATION),
          "route lookup unexpectedly found the older destination");
  Expect (!gatewayNwk->HasRelayRoute (NEWER_DESTINATION),
          "route lookup unexpectedly found the newer destination");
  gatewayNwk->AddStaticRoute (OLDER_DESTINATION, OLDER_DESTINATION);
  gatewayNwk->AddStaticRoute (NEWER_DESTINATION, NEWER_DESTINATION);

  // Use the explicit stop solely to isolate handoff order from the cadence
  // case above.  The first discovery-table entry is marked complete before
  // the SNMP START is sent, and no second target is considered until DONE or
  // the 60-second watchdog.
  gatewayNwk->StartDiscovery (Seconds (0.0), MilliSeconds (100));

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();
  Simulator::Destroy ();

  Expect (newerNwk->GetSnmpStartReceivedCount () == 1,
          "newest logical destination did not receive the first SNMP START");
  Expect (olderNwk->GetSnmpStartReceivedCount () == 0,
          "oldest logical destination was selected before the destHead entry");
  Expect (gatewayNwk->GetSnmpStartSentCount () == 1,
          "discovery ordering case sent more than one serialized SNMP START");
}

void
RunReverseOnlyDestinationCase ()
{
  Ptr<CsrNetDevice> gatewayDevice =
    CreateObject<CsrNetDevice> (GATEWAY_NODE);
  Ptr<CsrNetDevice> relayDevice =
    CreateObject<CsrNetDevice> (OLDER_DESTINATION);

  gatewayDevice->AddPeer (relayDevice);
  relayDevice->AddPeer (gatewayDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  gatewayDevice->GetPhy ().SetPerModel (noErrors);
  relayDevice->GetPhy ().SetPerModel (noErrors);
  gatewayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);
  relayDevice->GetPhy ().SetLinkDistanceMeters (
    GATEWAY_NODE, OLDER_DESTINATION, 1.0);

  Ptr<CsrHopLayer> gatewayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> relayHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> gateway = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> relay = CreateObject<CsrNetLayer> ();

  ConnectStack (
    gatewayDevice, gatewayHop, gateway, GATEWAY_NODE);
  ConnectStack (
    relayDevice, relayHop, relay, OLDER_DESTINATION);
  relayDevice->GetMac ().SetRxCallback (
    MakeCallback (&ObserveReverseSnmp));

  gateway->SetNodeType (CsrNodeType::Gateway);
  relay->SetNodeType (CsrNodeType::Routable);
  gateway->SetArlNeighborAdmissionEnabled (false);
  relay->SetArlNeighborAdmissionEnabled (false);

  gateway->ProcessHello (
    MakePlainHello (OLDER_DESTINATION),
    OLDER_DESTINATION,
    70.0,
    20.0);

  Ptr<Packet> reversePacket = Create<Packet> (8);
  CsrNetHeader networkHeader (
    REVERSE_ONLY_DESTINATION,
    GATEWAY_NODE,
    0);
  reversePacket->AddHeader (networkHeader);
  gateway->ReceiveFromHop (reversePacket, OLDER_DESTINATION);

  std::vector<CsrNodeId> knownNodes = gateway->GetKnownDiscoveryNodes ();
  Expect (knownNodes.size () >= 2,
          "reverse-only destination was omitted from the discovery walk");
  if (knownNodes.size () >= 2)
    {
      Expect (knownNodes[0] == REVERSE_ONLY_DESTINATION,
              "reverse-only destination did not retain newest-first order");
      Expect (knownNodes[1] == OLDER_DESTINATION,
              "direct neighbor moved ahead of the newer reverse destination");
    }

  // clear_discovery() copies the selected reverse hop into the NWK route
  // table before send_snmp_pk() looks it up.  The one-hop HOP recipient is
  // not the final destination, but NWK must still emit and count the START.
  g_reverseStartFrames = 0;
  g_reverseStartHop = CSR_BROADCAST_ID;
  g_reverseStartDestination = CSR_BROADCAST_ID;
  g_reverseStartDestinationType = CSR_DEST_BROADCAST;
  gateway->StartDiscovery (Seconds (0.0), MilliSeconds (100));
  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Expect (gateway->GetSnmpStartSentCount () == 1,
          "reverse-only discovery target did not emit an SNMP START");
  Expect (g_reverseStartFrames == 1,
          "reverse-only SNMP START was not observed exactly once");
  Expect (g_reverseStartHop == OLDER_DESTINATION,
          "reverse-only SNMP START used the wrong HOP destination");
  Expect (g_reverseStartDestination == REVERSE_ONLY_DESTINATION,
          "reverse-only SNMP START changed its final destination");
  Expect (g_reverseStartDestinationType == CSR_DEST_UNICAST,
          "reverse-only SNMP START was not unicast");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  RunCadenceAndCompletionCase ();
  RunNewestDestinationFirstCase ();
  RunReverseOnlyDestinationCase ();

  if (!g_failures.empty ())
    {
      for (const std::string &failure : g_failures)
        {
          std::cerr << "FAIL: " << failure << std::endl;
        }
      return 1;
    }

  std::cout << "PASS: OPNET discovery cadence/completion and destination order"
            << std::endl;
  return 0;
}
