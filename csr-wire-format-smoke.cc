#include "ns3/core-module.h"
#include "ns3/csr-arl-routing-message.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-nwk-layer.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId HIGH_SOURCE = 0x123456;
constexpr CsrNodeId HIGH_DESTINATION = 0xabcdef;

uint32_t g_highIdDeliveries = 0;

void
Require (bool condition, const char *message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

std::vector<uint8_t>
CopyPacketBytes (Ptr<Packet> packet)
{
  std::vector<uint8_t> bytes (packet->GetSize ());
  packet->CopyData (bytes.data (), bytes.size ());
  return bytes;
}

template <typename HeaderType>
std::vector<uint8_t>
SerializeHeader (const HeaderType &header)
{
  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);
  return CopyPacketBytes (packet);
}

void
RequireBytes (const std::vector<uint8_t> &actual,
              const std::vector<uint8_t> &expected,
              const char *message)
{
  if (actual == expected)
    {
      return;
    }

  std::cerr << "FAIL: " << message
            << " actualBytes=" << actual.size ()
            << " expectedBytes=" << expected.size ()
            << std::endl;
  std::exit (1);
}

void
CheckNetworkHeader ()
{
  CsrNetHeader header (0x123456, 0xabcdef, 0xa5);
  const std::vector<uint8_t> expected {
    0x12, 0x34, 0x56,
    0xab, 0xcd, 0xef,
    0xa5
  };

  Require (header.GetSerializedSize () == 7,
           "NWK header is not seven bytes");
  RequireBytes (SerializeHeader (header), expected,
                "NWK header does not use exact 24/24/8-bit wire fields");

  Ptr<Packet> packet = Create<Packet> (expected.data (), expected.size ());
  CsrNetHeader decoded;
  Require (packet->RemoveHeader (decoded) == expected.size (),
           "NWK header did not consume its exact wire size");
  Require (decoded.GetSrc () == 0x123456 &&
           decoded.GetDst () == 0xabcdef &&
           decoded.GetDscp () == 0xa5,
           "NWK header truncated a 24-bit node identifier");
}

void
CheckHopHeader ()
{
  CsrHeader header (0x123456,
                    0xabcdef,
                    0x1357,
                    0xa5,
                    true,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (0x7e);

  const std::vector<uint8_t> expected {
    0x12, 0x34, 0x56,
    0xab, 0xcd, 0xef,
    0x13, 0x57,
    0xa5,
    0x01,
    CSR_PKT_DATA,
    CSR_DEST_UNICAST,
    0x7e
  };

  Require (header.GetSerializedSize () == 13,
           "base HOP header is not thirteen bytes");
  RequireBytes (SerializeHeader (header), expected,
                "HOP header does not preserve 24-bit addresses and 16-bit sequence");

  CsrHeader grouped (0x010203,
                     0x0a0b0c,
                     0x1122,
                     7,
                     true,
                     false);
  grouped.SetType (CSR_PKT_ROUTING_CONTROL);
  grouped.SetDestType (CSR_DEST_MULTICAST);
  grouped.SetSpeedKey (8);
  grouped.SetDestinationSequences (
    {{0x0a0b0c, 0x1122}, {0xfedcba, 0xbeef}});

  const std::vector<uint8_t> expectedGrouped {
    0x01, 0x02, 0x03,
    0x0a, 0x0b, 0x0c,
    0x11, 0x22,
    0x07,
    0x11,
    CSR_PKT_ROUTING_CONTROL,
    CSR_DEST_MULTICAST,
    0x08,
    0x02,
    0x0a, 0x0b, 0x0c, 0x11, 0x22,
    0xfe, 0xdc, 0xba, 0xbe, 0xef
  };

  Require (grouped.GetSerializedSize () == 24,
           "grouped HOP header has the wrong wire size");
  RequireBytes (SerializeHeader (grouped), expectedGrouped,
                "grouped HOP destinations are not 24-bit wire values");

  Ptr<Packet> packet = Create<Packet> (
    expectedGrouped.data (), expectedGrouped.size ());
  CsrHeader decoded;
  Require (packet->RemoveHeader (decoded) == expectedGrouped.size (),
           "grouped HOP header did not consume its exact wire size");
  Require (decoded.GetSrc () == 0x010203 &&
           decoded.GetDst () == 0x0a0b0c &&
           decoded.GetSeq () == 0x1122,
           "grouped HOP header narrowed an address or sequence");
  Require (decoded.GetDestinationSequences () ==
             std::vector<CsrHeader::DestinationSequence> (
               {{0x0a0b0c, 0x1122}, {0xfedcba, 0xbeef}}),
           "grouped HOP target list did not round-trip exactly");
}

void
CheckSnmpHeader ()
{
  CsrSnmpHeader header;
  header.SetSource (0x112233);
  header.SetDestination (CSR_BROADCAST_ID);
  header.SetDestinationType (CSR_DEST_BROADCAST);
  header.SetCommand (CSR_SNMP_DISCOVERY_DONE);
  header.SetValue (0x12345678);
  header.SetNodes ({0x010203, 0xa0b0c0});

  const std::vector<uint8_t> expected {
    0x11, 0x22, 0x33,
    0xff, 0xff, 0xff,
    CSR_DEST_BROADCAST,
    CSR_SNMP_DISCOVERY_DONE,
    0x12, 0x34, 0x56, 0x78,
    0x02,
    0x01, 0x02, 0x03,
    0xa0, 0xb0, 0xc0
  };

  Require (header.GetSerializedSize () == 19,
           "SNMP header has the wrong 24-bit node-list size");
  RequireBytes (SerializeHeader (header), expected,
                "SNMP header does not use exact 24-bit addresses");

  Ptr<Packet> packet = Create<Packet> (expected.data (), expected.size ());
  CsrSnmpHeader decoded;
  Require (packet->RemoveHeader (decoded) == expected.size (),
           "SNMP header did not consume its exact wire size");
  Require (decoded.GetSource () == 0x112233 &&
           decoded.GetDestination () == CSR_BROADCAST_ID &&
           decoded.GetNodes () ==
             std::vector<CsrNodeId> ({0x010203, 0xa0b0c0}),
           "SNMP header truncated a 24-bit node identifier");
}

void
CheckHelloHeader ()
{
  CsrHelloHeader header;
  header.SetNodeId (0x123456);
  header.SetHelloSeq (0x789a);
  header.SetSpeedKey (0xbc);
  header.SetRxPowerDbmX10 (-2);
  header.SetActiveNodes (0xde);
  header.SetNodeType (CsrNodeType::Gateway);
  header.SetArlRouteMsgType (CsrArlRouteMsgType::NeighborCheck);
  header.SetNeighborCheckType (CsrNeighborCheckType::Overheard);
  header.SetNeighborCheckTarget (0xabcdef);
  header.SetDiscoverType (CsrDiscoverType::Chirp);
  header.SetDiscoverySequence (0x10203040);
  header.SetRoutingSequence (0x50607080);
  header.SetRoutingSection (0x90);
  header.SetRoutingTotalSections (0xa0);
  header.SetRoutingOperation (CsrRoutingOperation::Request);
  header.SetRoutingTarget (0x0a0b0c);
  Require (header.AddChirpNeighbor (0xfedcba),
           "HELLO rejected a valid 24-bit chirp neighbor");
  Require (header.AddAdvertisedRoute (
             0x010203,
             2,
             0x11223344,
             -3,
             0x55,
             {0x040506, 0x070809}),
           "HELLO rejected a valid 24-bit advertised route");

  const std::vector<uint8_t> expected {
    0x12, 0x34, 0x56,
    0x78, 0x9a,
    0xbc,
    0xff, 0xfe,
    0xde,
    0x02,
    0x03,
    0x03,
    0xab, 0xcd, 0xef,
    0x01,
    0x10, 0x20, 0x30, 0x40,
    0x50, 0x60, 0x70, 0x80,
    0x90,
    0xa0,
    0x03,
    0x0a, 0x0b, 0x0c,
    0x01,
    0xfe, 0xdc, 0xba,
    0x01,
    0x01, 0x02, 0x03,
    0x02,
    0x11, 0x22, 0x33, 0x44,
    0xff, 0xfd,
    0x55,
    0x02,
    0x04, 0x05, 0x06,
    0x07, 0x08, 0x09
  };

  Require (header.GetSerializedSize () == 53,
           "HELLO header has the wrong 24-bit wire size");
  RequireBytes (SerializeHeader (header), expected,
                "HELLO node-bearing fields are not exact 24-bit values");

  Ptr<Packet> packet = Create<Packet> (expected.data (), expected.size ());
  CsrHelloHeader decoded;
  Require (packet->RemoveHeader (decoded) == expected.size (),
           "HELLO header did not consume its exact wire size");
  CsrHelloHeader::AdvertisedRoute route = decoded.GetAdvertisedRoute (0);
  Require (decoded.GetNodeId () == 0x123456 &&
           decoded.GetNeighborCheckTarget () == 0xabcdef &&
           decoded.GetRoutingTarget () == 0x0a0b0c &&
           decoded.GetChirpNeighbor (0) == 0xfedcba &&
           route.dst == 0x010203 &&
           route.path == std::vector<CsrNodeId> ({0x040506, 0x070809}),
           "HELLO header truncated a 24-bit node-bearing field");
}

void
CheckArlRecord ()
{
  CsrArlRoutingMessage::Builder builder;
  std::string error;
  Require (builder.AddUpdate (0xabcdef,
                              0x5a,
                              2,
                              0x89abcdef,
                              {0x010203, 0xfedcba},
                              &error),
           "ARL builder rejected valid legacy widths");

  const std::vector<uint8_t> expected {
    0x02,
    0xab, 0xcd, 0xef,
    0x5a,
    0x00, 0x02,
    0x89, 0xab, 0xcd, 0xef,
    0x01, 0x02, 0x03,
    0xfe, 0xdc, 0xba
  };

  RequireBytes (builder.GetRecordStream (), expected,
                "ARL UPDATE does not preserve 24/16/32-bit wire fields");
}

void
ReceiveHighIdPayload (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == HIGH_SOURCE,
           "NWK callback narrowed the 24-bit source identifier");
  Require (payload->GetSize () == 37,
           "high-ID integration payload changed size");
  ++g_highIdDeliveries;
}

void
ConnectHighIdStack (Ptr<CsrNetDevice> device,
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
CheckHighIdIntegration ()
{
  g_highIdDeliveries = 0;

  Ptr<CsrNetDevice> sourceDevice =
    CreateObject<CsrNetDevice> (HIGH_SOURCE);
  Ptr<CsrNetDevice> destinationDevice =
    CreateObject<CsrNetDevice> (HIGH_DESTINATION);
  sourceDevice->AddPeer (destinationDevice);
  destinationDevice->AddPeer (sourceDevice);

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sourceDevice->GetPhy ().SetPerModel (noErrors);
  destinationDevice->GetPhy ().SetPerModel (noErrors);
  sourceDevice->GetPhy ().SetLinkDistanceMeters (
    HIGH_SOURCE, HIGH_DESTINATION, 1.0);
  destinationDevice->GetPhy ().SetLinkDistanceMeters (
    HIGH_SOURCE, HIGH_DESTINATION, 1.0);

  Ptr<CsrHopLayer> sourceHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> destinationHop = CreateObject<CsrHopLayer> ();
  Ptr<CsrNetLayer> sourceNwk = CreateObject<CsrNetLayer> ();
  Ptr<CsrNetLayer> destinationNwk = CreateObject<CsrNetLayer> ();
  ConnectHighIdStack (
    sourceDevice, sourceHop, sourceNwk, HIGH_SOURCE);
  ConnectHighIdStack (
    destinationDevice,
    destinationHop,
    destinationNwk,
    HIGH_DESTINATION);

  sourceNwk->AddStaticRouteWithPathloss (
    HIGH_DESTINATION,
    HIGH_DESTINATION,
    60.0,
    true,
    static_cast<uint8_t> (CsrNodeType::Routable));
  destinationNwk->SetRxFromNetCallback (
    MakeCallback (&ReceiveHighIdPayload));

  Simulator::ScheduleNow ([sourceNwk] () {
    sourceNwk->Send (
      HIGH_DESTINATION, 5, Create<Packet> (37), true);
  });
  Simulator::Stop (Seconds (8));
  Simulator::Run ();

  Require (g_highIdDeliveries == 1,
           "24-bit node IDs did not survive NWK/HOP/MAC delivery");
  Require (sourceNwk->GetNsdpCount (
             HIGH_SOURCE, HIGH_DESTINATION) == 0,
           "high-ID reliable DATA did not release NWK custody");
  Require (sourceHop->GetPendingDataCount () == 0,
           "high-ID reliable DATA did not release HOP custody");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  CheckNetworkHeader ();
  CheckHopHeader ();
  CheckSnmpHeader ();
  CheckHelloHeader ();
  CheckArlRecord ();
  CheckHighIdIntegration ();

  std::cout << "PASS: exact CSR 24-bit wire serialization test" << std::endl;
  return 0;
}
