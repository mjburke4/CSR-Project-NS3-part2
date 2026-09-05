#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-opnet-envelope.h"
#include "ns3/csr-opnet-packet-model.h"
#include "ns3/network-module.h"

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <span>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

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
RequireBytes (const std::vector<uint8_t> &actual,
              const std::vector<uint8_t> &expected,
              const char *message)
{
  if (actual != expected)
    {
      std::cerr << "FAIL: " << message
                << " actualBytes=" << actual.size ()
                << " expectedBytes=" << expected.size ()
                << std::endl;
      std::exit (1);
    }
}

std::vector<uint8_t>
Serialize (CsrOpnetPacketFormat format,
           const CsrOpnetPacketFields &fields,
           const std::vector<uint8_t> &payload = {})
{
  std::vector<uint8_t> bytes;
  std::string error;
  Require (CsrOpnetPacketModel::Serialize (format,
                                           fields,
                                           payload,
                                           bytes,
                                           &error),
           error.c_str ());
  return bytes;
}

void
RequireRoundTrip (CsrOpnetPacketFormat format,
                  const CsrOpnetPacketFields &fields,
                  const std::vector<uint8_t> &payload)
{
  std::vector<uint8_t> bytes = Serialize (format, fields, payload);
  CsrOpnetPacketFields decoded;
  std::vector<uint8_t> decodedPayload;
  std::string error;
  Require (CsrOpnetPacketModel::Deserialize (format,
                                             bytes,
                                             decoded,
                                             decodedPayload,
                                             &error),
           error.c_str ());
  RequireBytes (decodedPayload, payload,
                "inherited payload did not round-trip");

  std::vector<uint8_t> reencoded =
    Serialize (format, decoded, decodedPayload);
  RequireBytes (reencoded, bytes,
                "decoded fixed fields did not round-trip");
}

void
CheckFixedSizes ()
{
  const std::vector<std::pair<CsrOpnetPacketFormat, uint32_t>> expected {
    {CsrOpnetPacketFormat::Ack, 8},
    {CsrOpnetPacketFormat::Hello, 12},
    {CsrOpnetPacketFormat::Hop, 8},
    {CsrOpnetPacketFormat::Mac, 17},
    {CsrOpnetPacketFormat::MacHopInstance, 8},
    {CsrOpnetPacketFormat::Network, 7},
    {CsrOpnetPacketFormat::OtaLongPreamble, 996},
    {CsrOpnetPacketFormat::OtaShortPreamble, 23},
    {CsrOpnetPacketFormat::Routes, 11},
    {CsrOpnetPacketFormat::Snmp, 6},
    {CsrOpnetPacketFormat::SentInfo, 3}
  };

  for (const auto &[format, bytes] : expected)
    {
      Require (CsrOpnetPacketModel::GetFixedSizeBytes (format) == bytes,
               "OPNET fixed packet size differs from its .pk.m model");
      Require (CsrOpnetPacketModel::GetFixedSizeBits (format) == bytes * 8,
               "OPNET fixed bit count differs from its .pk.m model");
    }

  Require (CsrOpnetPacketModel::GetModeledSizeBytes (
             CsrOpnetPacketFormat::Mac, 19) == 36,
           "inherited MAC payload size was not included");
  Require (CsrOpnetPacketModel::GetModeledSizeBytes (
             CsrOpnetPacketFormat::Snmp, 19) == 6,
           "zero-bit SNMP metadata changed the fixed size");
}

void
CheckControlFormats ()
{
  CsrOpnetPacketFields fields;
  fields.source = 0x123456;
  fields.destination = 0xabcdef;
  fields.sequence16 = 0x1357;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Ack, fields),
    {0x12, 0x34, 0x56, 0xab, 0xcd, 0xef, 0x13, 0x57},
    "br_Ack fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Ack, fields, {});

  fields = {};
  fields.sequence8 = 0x12;
  fields.nodeId = 0x345678;
  fields.capabilityNumber = 0x9a;
  fields.capability = 0xbc;
  fields.cost = 0xdef0;
  fields.timeOffset = 0x12345678;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Hello, fields),
    {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
     0xde, 0xf0, 0x12, 0x34, 0x56, 0x78},
    "br_Hello fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Hello, fields, {});

  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Routes, fields),
    {0x34, 0x56, 0x78, 0x9a, 0xbc,
     0xde, 0xf0, 0x12, 0x34, 0x56, 0x78},
    "br_Routes fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Routes, fields, {});

  fields = {};
  fields.message = 0x1234;
  fields.value = 0x89abcdef;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Snmp, fields),
    {0x12, 0x34, 0x89, 0xab, 0xcd, 0xef},
    "br_SNMP fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Snmp, fields, {});

  fields = {};
  fields.source = 0x123456;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::SentInfo, fields),
    {0x12, 0x34, 0x56},
    "br_Sent_Info fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::SentInfo, fields, {});
}

void
CheckInheritedFormats ()
{
  const std::vector<uint8_t> payload {0xde, 0xad, 0xbe, 0xef};

  CsrOpnetPacketFields fields;
  fields.source = 0x123456;
  fields.numberOfDestinations = 2;
  fields.destination = 0xabcdef;
  fields.sequence8 = 0x7a;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Hop, fields, payload),
    {0x12, 0x34, 0x56, 0x02, 0xab, 0xcd, 0xef, 0x7a,
     0xde, 0xad, 0xbe, 0xef},
    "br_Hop fixed fields or inherited payload are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Hop, fields, payload);

  fields = {};
  fields.globalTime = 0x01020304;
  fields.globalAddress = 0x112233;
  fields.globalCost = 0x44;
  fields.txPower = 0x55;
  fields.rxPower = 0x66;
  fields.active = 0x77;
  fields.source = 0x8899aa;
  fields.payloadLength = 0xbbcc;
  fields.type = 0xdd;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Mac, fields, payload),
    {0x01, 0x02, 0x03, 0x04, 0x11, 0x22, 0x33,
     0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa,
     0xbb, 0xcc, 0xdd, 0xde, 0xad, 0xbe, 0xef},
    "br_Mac fixed fields or inherited payload are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::Mac, fields, payload);

  fields = {};
  fields.source = 0x123456;
  fields.destination = 0xabcdef;
  fields.dscp = 0x7e;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::Network, fields, payload),
    {0x12, 0x34, 0x56, 0xab, 0xcd, 0xef,
     0xde, 0xad, 0xbe, 0xef, 0x7e},
    "br_Network DSCP is not after its inherited payload");
  RequireRoundTrip (CsrOpnetPacketFormat::Network, fields, payload);
}

void
CheckMacHopInstance ()
{
  CsrOpnetPacketFields fields;
  fields.source = 0x123456;
  fields.destination = 0xabcdef;
  fields.sequence16 = 0x1357;
  RequireBytes (
    Serialize (CsrOpnetPacketFormat::MacHopInstance, fields),
    {0x12, 0x34, 0x56, 0xab, 0xcd, 0xef, 0x13, 0x57},
    "br_Mac_Hop_Inst fixed fields are not exact");
  RequireRoundTrip (CsrOpnetPacketFormat::MacHopInstance, fields, {});
}

void
CheckOtaFormats ()
{
  CsrOpnetPacketFields fields;
  fields.startOfFrame = 0x1234;
  fields.speed = 0x5678;
  fields.length = 0x9abc;
  fields.fcs = 0xdef01234;
  const std::vector<uint8_t> payload {0xa5, 0x5a};

  std::vector<uint8_t> shortBytes = Serialize (
    CsrOpnetPacketFormat::OtaShortPreamble, fields, payload);
  Require (shortBytes.size () == 25,
           "short OTA envelope is not 184 fixed bits plus payload");
  Require (std::all_of (shortBytes.begin (), shortBytes.begin () + 13,
                        [] (uint8_t byte) { return byte == 0; }),
           "unset 104-bit short preamble is not deterministic");
  RequireBytes (
    std::vector<uint8_t> (shortBytes.begin () + 13, shortBytes.end ()),
    {0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc,
     0xa5, 0x5a, 0xde, 0xf0, 0x12, 0x34},
    "short OTA field order is not preamble/header/payload/FCS");
  RequireRoundTrip (CsrOpnetPacketFormat::OtaShortPreamble,
                    fields,
                    payload);

  std::vector<uint8_t> longBytes = Serialize (
    CsrOpnetPacketFormat::OtaLongPreamble, fields, payload);
  Require (longBytes.size () == 998,
           "long OTA envelope is not 7968 fixed bits plus payload");
  Require (std::all_of (longBytes.begin (), longBytes.begin () + 986,
                        [] (uint8_t byte) { return byte == 0; }),
           "unset 7888-bit long preamble is not deterministic");
  RequireRoundTrip (CsrOpnetPacketFormat::OtaLongPreamble,
                    fields,
                    payload);

  // OTA Speed and Length are both 16-bit fields.  In particular, 1000 is a
  // literal 16-bit OPNET value (not the compact ns-3 compatibility code), and
  // an oversized source integer retains only its low 16 bits on the wire.
  fields.speed = 1000;
  fields.length = static_cast<uint16_t> (0x12345u);
  shortBytes = Serialize (
    CsrOpnetPacketFormat::OtaShortPreamble, fields, payload);
  RequireBytes (
    std::vector<uint8_t> (shortBytes.begin () + 15,
                          shortBytes.begin () + 19),
    {0x03, 0xe8, 0x23, 0x45},
    "OTA Speed/Length did not preserve 16-bit field semantics");
}

void
CheckEnvelopeTags ()
{
  Ptr<Packet> compatibilityPacket = Create<Packet> (100);
  CsrSetOpnetEnvelope (compatibilityPacket,
                       CsrOpnetPacketFormat::Snmp,
                       31);
  Require (compatibilityPacket->GetSize () == 100,
           "OPNET size annotation changed compatibility bytes");
  Require (CsrGetOpnetWireSize (compatibilityPacket) == 31,
           "OPNET size annotation was not used");

  Ptr<Packet> copy = compatibilityPacket->Copy ();
  Require (CsrGetOpnetWireSize (copy) == 31,
           "OPNET size annotation did not survive Packet::Copy");

  Ptr<Packet> untagged = Create<Packet> (10);
  Require (CsrGetOpnetAggregateWireSize ({copy, untagged}) == 41,
           "OPNET aggregate did not sum segment modeled sizes");
}

void
CheckCompatibilityInference ()
{
  Ptr<Packet> data = Create<Packet> (10);
  CsrNetHeader network (0x010203, 0x040506, 7);
  data->AddHeader (network);
  CsrHeader dataHop (0x010203, 0x0a0b0c, 0x1234, 7, true, false);
  dataHop.SetType (CSR_PKT_DATA);
  dataHop.SetLinkControl (64, 10.0, -90.0);
  data->AddHeader (dataHop);
  Require (CsrAnnotateOpnetEnvelope (data),
           "DATA compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (data) == 42,
           "DATA did not map to br_Mac + br_Hop + br_Network + payload");

  Ptr<Packet> controlAck = Create<Packet> ();
  CsrHeader controlAckHeader (
    0x010203, 0x040506, 0x1234, 7, false, true);
  controlAckHeader.SetType (CSR_PKT_ACK);
  controlAck->AddHeader (controlAckHeader);
  Require (CsrAnnotateOpnetEnvelope (controlAck),
           "control ACK compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (controlAck) == 25,
           "bare control ACK did not model 17 + 8 bytes");

  Ptr<Packet> securedAck = Create<Packet> (18);
  CsrHeader securedAckHeader (
    0x010203, 0x040506, 0x1234, 7, false, true);
  securedAckHeader.SetType (CSR_PKT_ACK);
  securedAckHeader.SetSecurityCount (7);
  securedAck->AddHeader (securedAckHeader);
  Require (CsrAnnotateOpnetEnvelope (securedAck),
           "Pairwise16 ACK compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (securedAck) == 30,
           "Pairwise16 control ACK did not model 17 + 8 + 5 bytes");

  Ptr<Packet> cumulativeAck = Create<Packet> ();
  CsrHeader cumulativeAckHeader (
    0x010203, 0x040506, 0x1234, 7, false, true);
  cumulativeAckHeader.SetType (CSR_PKT_ACK);
  cumulativeAckHeader.SetHasAckWindow (true);
  cumulativeAckHeader.SetAckBitmap (0x0123456789abcdefULL);
  cumulativeAckHeader.SetDackBitmap (0xfedcba9876543210ULL);
  cumulativeAck->AddHeader (cumulativeAckHeader);
  Require (CsrAnnotateOpnetEnvelope (cumulativeAck),
           "cumulative ACK compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (cumulativeAck) == 41,
           "bare cumulative ACK did not model 17 + 8 + 16 bytes");

  // Pairwise16 wraps the 29-byte cumulative logical body as
  // key/sequence(3) + plaintext(29) + tag(2).
  Ptr<Packet> securedCumulativeAck = Create<Packet> (34);
  CsrHeader securedCumulativeAckHeader (
    0x010203, 0x040506, 0x1234, 7, false, true);
  securedCumulativeAckHeader.SetType (CSR_PKT_ACK);
  securedCumulativeAckHeader.SetHasAckWindow (true);
  securedCumulativeAckHeader.SetAckBitmap (0x0123456789abcdefULL);
  securedCumulativeAckHeader.SetDackBitmap (0xfedcba9876543210ULL);
  securedCumulativeAckHeader.SetSecurityCount (7);
  securedCumulativeAck->AddHeader (securedCumulativeAckHeader);
  Require (CsrAnnotateOpnetEnvelope (securedCumulativeAck),
           "Pairwise16 cumulative ACK envelope was not recognized");
  Require (CsrGetOpnetWireSize (securedCumulativeAck) == 46,
           "Pairwise16 cumulative ACK did not model 17 + 8 + 16 + 5 bytes");

  Ptr<Packet> snmpPayload = Create<Packet> ();
  CsrSnmpHeader snmpHeader;
  snmpHeader.SetSource (0x010203);
  snmpHeader.SetDestination (0x040506);
  snmpHeader.SetCommand (CSR_SNMP_RELAY_HOLDOFF);
  snmpHeader.SetValue (0x12345678);
  snmpHeader.SetNodes ({0x111111, 0x222222, 0x333333});
  snmpPayload->AddHeader (snmpHeader);
  CsrHeader snmpHop (0x010203, 0x040506, 0, 0, false, false);
  snmpHop.SetType (CSR_PKT_SNMP);
  snmpPayload->AddHeader (snmpHop);
  Require (CsrAnnotateOpnetEnvelope (snmpPayload),
           "SNMP compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (snmpPayload) == 31,
           "SNMP did not map to br_Mac + br_Hop + six fixed bytes");

  Ptr<Packet> hello = Create<Packet> (80);
  CsrHeader helloHop (0x010203, CSR_BROADCAST_ID, 1, 7, false, false);
  helloHop.SetType (CSR_PKT_HELLO);
  helloHop.SetDestType (CSR_DEST_BROADCAST);
  hello->AddHeader (helloHop);
  Require (CsrAnnotateOpnetEnvelope (hello),
           "HELLO compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (hello) == 12,
           "zero-bit HELLO metadata changed the fixed envelope size");

  Ptr<Packet> protectedHello = Create<Packet> (100);
  CsrHeader protectedHelloHop (
    0x010203, CSR_BROADCAST_ID, 2, 7, false, false);
  protectedHelloHop.SetType (CSR_PKT_HELLO);
  protectedHelloHop.SetDestType (CSR_DEST_BROADCAST);
  protectedHelloHop.SetHasGroupSecurity (true);
  protectedHello->AddHeader (protectedHelloHop);
  Require (CsrAnnotateOpnetEnvelope (protectedHello),
           "Group16 HELLO compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (protectedHello) == 17,
           "Group16 HELLO did not add exactly five security bytes");

  Ptr<Packet> discover = Create<Packet> (100);
  CsrHeader discoverHop (
    0x010203, CSR_BROADCAST_ID, 3, 7, false, false);
  discoverHop.SetType (CSR_PKT_DISCOVER);
  discoverHop.SetDestType (CSR_DEST_BROADCAST);
  discoverHop.SetHasGroupSecurity (true);
  discover->AddHeader (discoverHop);
  Require (CsrAnnotateOpnetEnvelope (discover),
           "GroupEstablish Discover envelope was not recognized");
  Require (CsrGetOpnetWireSize (discover) == 19,
           "GroupEstablish Discover did not add seven security bytes");

  Ptr<Packet> neighborCheck = Create<Packet> (80);
  CsrHeader neighborHop (0x010203, 0x040506, 4, 7, true, false);
  neighborHop.SetType (CSR_PKT_NEIGHBOR_CHECK);
  neighborCheck->AddHeader (neighborHop);
  Require (CsrAnnotateOpnetEnvelope (neighborCheck),
           "NeighborCheck compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (neighborCheck) == 11,
           "zero-bit NeighborCheck metadata changed br_Routes size");

  const std::vector<uint8_t> routingSection {
    0x01, 0x02, 0x03, 0x04, 0x00, 0x01, 0x05, 0x06
  };
  Ptr<Packet> bareRouting = Create<Packet> (
    routingSection.data (), routingSection.size ());
  CsrHeader bareRoutingHop (0x010203, 0x040506, 5, 7, true, false);
  bareRoutingHop.SetType (CSR_PKT_ROUTING_CONTROL);
  bareRouting->AddHeader (bareRoutingHop);
  Require (CsrAnnotateOpnetEnvelope (bareRouting),
           "bare ARL routing envelope was not recognized");
  Require (CsrGetOpnetWireSize (bareRouting) == 19,
           "bare ARL routing section did not follow br_Routes");

  CsrHelloHeader routingMetadata;
  routingMetadata.SetNodeId (0x010203);
  routingMetadata.SetRoutingSequence (0x01020304);
  routingMetadata.SetRoutingSection (0);
  routingMetadata.SetRoutingTotalSections (1);
  routingMetadata.SetRoutingOperation (CsrRoutingOperation::Update);
  Ptr<Packet> protectedRoutingPlaintext = Create<Packet> (
    routingSection.data (), routingSection.size ());
  protectedRoutingPlaintext->AddHeader (routingMetadata);
  std::vector<uint8_t> protectedRoutingRecord (
    3 + protectedRoutingPlaintext->GetSize () + 2, 0);
  protectedRoutingPlaintext->CopyData (
    protectedRoutingRecord.data () + 3,
    protectedRoutingPlaintext->GetSize ());
  Ptr<Packet> protectedRouting = Create<Packet> (
    protectedRoutingRecord.data (), protectedRoutingRecord.size ());
  CsrHeader protectedRoutingHop (
    0x010203, 0x040506, 6, 7, true, false);
  protectedRoutingHop.SetType (CSR_PKT_ROUTING_CONTROL);
  protectedRoutingHop.SetHasGroupSecurity (true);
  protectedRouting->AddHeader (protectedRoutingHop);
  Require (CsrAnnotateOpnetEnvelope (protectedRouting),
           "Group16 ARL routing envelope was not recognized");
  Require (CsrGetOpnetWireSize (protectedRouting) == 24,
           "Group16 routing did not exclude compatibility metadata");

  Ptr<Packet> keyRequest = Create<Packet> (7);
  CsrHeader keyRequestHop (0x010203, 0x040506, 7, 7, false, false);
  keyRequestHop.SetType (CSR_PKT_KEY_REQUEST);
  keyRequest->AddHeader (keyRequestHop);
  Require (CsrAnnotateOpnetEnvelope (keyRequest),
           "KeyRequest compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (keyRequest) == 18,
           "KeyRequest did not add its exact seven-byte record");

  Ptr<Packet> keyUpdate = Create<Packet> (51);
  CsrHeader keyUpdateHop (0x010203, 0x040506, 8, 7, true, false);
  keyUpdateHop.SetType (CSR_PKT_KEY_UPDATE);
  keyUpdate->AddHeader (keyUpdateHop);
  Require (CsrAnnotateOpnetEnvelope (keyUpdate),
           "KeyUpdate compatibility envelope was not recognized");
  Require (CsrGetOpnetWireSize (keyUpdate) == 62,
           "KeyUpdate did not add its exact 51-byte record");
}

void
CheckArchivedHiddenEnvelopeSizes ()
{
  // Both recovered bare-HOP executable lineages carry the 592-byte logical
  // DATA body behind br_Hop and br_Mac without Pairwise16.  The resulting
  // source envelope is 592 + 8 + 17 = 617 bytes.
  Ptr<Packet> hiddenData = Create<Packet> (592);
  CsrHeader dataHeader (1, 2, 0xff, 0, true, false);
  dataHeader.SetType (CSR_PKT_DATA);
  dataHeader.SetDestType (CSR_DEST_UNICAST);
  hiddenData->AddHeader (dataHeader);
  Require (CsrAnnotateOpnetEnvelope (hiddenData),
           "archived hidden DATA envelope was not recognized");
  Require (CsrGetOpnetWireSize (hiddenData) == 617,
           "592-byte hidden DATA body did not become a 617-byte bare envelope");

  Ptr<Packet> cumulativeAck = Create<Packet> ();
  CsrHeader ackHeader (2, 1, 0xff, 0, false, true);
  ackHeader.SetType (CSR_PKT_ACK);
  ackHeader.SetDestType (CSR_DEST_UNICAST);
  ackHeader.SetHasAckWindow (true);
  ackHeader.SetAckBitmap (1);
  ackHeader.SetDackBitmap (0);
  cumulativeAck->AddHeader (ackHeader);
  Require (CsrAnnotateOpnetEnvelope (cumulativeAck),
           "archived cumulative ACK envelope was not recognized");
  Require (CsrGetOpnetWireSize (cumulativeAck) == 41,
           "archived cumulative ACK/DACK envelope was not 41 bytes");
  Require (CsrGetOpnetAggregateWireSize ({hiddenData, cumulativeAck}) == 658,
           "archived hidden DATA plus ACK envelope sum changed");
}

} // namespace

int
main ()
{
  CheckFixedSizes ();
  CheckControlFormats ();
  CheckInheritedFormats ();
  CheckMacHopInstance ();
  CheckOtaFormats ();
  CheckEnvelopeTags ();
  CheckCompatibilityInference ();
  CheckArchivedHiddenEnvelopeSizes ();

  std::cout << "PASS: exact OPNET packet/control-envelope parity"
            << std::endl;
  return 0;
}
