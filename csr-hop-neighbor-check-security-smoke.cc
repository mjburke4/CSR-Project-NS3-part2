#include "ns3/core-module.h"
#include "ns3/csr-hello-header.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-opnet-packet-model.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE = 0x010203;
constexpr CsrNodeId DESTINATION = 0x040506;
constexpr uint16_t SECURITY_COUNT = 7;
constexpr uint8_t LEGACY_NEIGHBOR_CHECK_PACKET_TYPE = 9;
constexpr uint8_t LEGACY_ROUTING_UPDATE_PACKET_TYPE = 8;
constexpr uint32_t DISCOVERY_SEQUENCE = 0x01020304;

CsrLegacyCrypto::MissionKey g_missionKey {};
CsrLegacyCrypto::PairwiseKeyMaterial g_pairwiseMaterial {};

uint32_t g_deliveries = 0;
CsrNeighborCheckType g_deliveredType = CsrNeighborCheckType::None;
uint32_t g_deliveredDiscoverySequence = 0;
std::vector<CsrNeighborCheckType> g_deliveredTypes;

struct CompletionEvent
{
  CsrNodeId neighbor {0};
  CsrNeighborCheckType type {CsrNeighborCheckType::None};
  uint32_t discoverySequence {0};
};

std::vector<CompletionEvent> g_successEvents;
std::vector<CompletionEvent> g_failureEvents;
std::vector<Ptr<Packet>> g_capturedFrames;
uint32_t g_capturedAckFrames = 0;
std::vector<uint32_t> g_capturedAckSizes;

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
InitializeKeys ()
{
  for (std::size_t i = 0; i < g_missionKey.size (); ++i)
    {
      g_missionKey[i] = static_cast<uint8_t> (i);
    }
  for (std::size_t i = 0; i < g_pairwiseMaterial.size (); ++i)
    {
      g_pairwiseMaterial[i] = static_cast<uint8_t> (0x40 + i);
    }
}

void
ResetObservations ()
{
  g_deliveries = 0;
  g_deliveredType = CsrNeighborCheckType::None;
  g_deliveredDiscoverySequence = 0;
  g_deliveredTypes.clear ();
  g_successEvents.clear ();
  g_failureEvents.clear ();
  g_capturedFrames.clear ();
  g_capturedAckFrames = 0;
  g_capturedAckSizes.clear ();
}

std::vector<uint8_t>
CopyBytes (Ptr<Packet> packet)
{
  std::vector<uint8_t> bytes (packet->GetSize ());
  if (!bytes.empty ())
    {
      packet->CopyData (bytes.data (), bytes.size ());
    }
  return bytes;
}

void
ConfigureSecurity (CsrHopSecurityState &state,
                   CsrNodeId nodeId,
                   CsrNodeId peer)
{
  state.SetNodeId (nodeId);
  state.SetOwnSecurityCount (SECURITY_COUNT);
  state.SetMissionKey (g_missionKey);
  state.SetPairwiseKeyMaterial (peer, g_pairwiseMaterial);
}

void
ConfigureHop (Ptr<CsrHopLayer> hop,
              CsrNodeId nodeId,
              CsrNodeId peer)
{
  hop->SetNodeId (nodeId);
  hop->SetOwnSecurityCount (SECURITY_COUNT);
  hop->SetMissionKey (g_missionKey);
  hop->SetPairwiseKeyMaterial (peer, g_pairwiseMaterial);
}

void
ConfigureNoErrorLink (Ptr<CsrNetDevice> first,
                      Ptr<CsrNetDevice> second)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  first->GetPhy ().SetPerModel (noErrors);
  second->GetPhy ().SetPerModel (noErrors);
  first->GetPhy ().SetLinkDistanceMeters (SOURCE, DESTINATION, 1.0);
  second->GetPhy ().SetLinkDistanceMeters (SOURCE, DESTINATION, 1.0);
}

Ptr<Packet>
MakeNeighborCheckPayload (
  CsrNeighborCheckType type = CsrNeighborCheckType::Verify)
{
  CsrHelloHeader control;
  control.SetNodeId (SOURCE);
  control.SetArlRouteMsgType (CsrArlRouteMsgType::NeighborCheck);
  control.SetNeighborCheckType (type);
  control.SetNeighborCheckTarget (DESTINATION);
  control.SetDiscoverySequence (DISCOVERY_SEQUENCE);

  Ptr<Packet> payload = Create<Packet> ();
  payload->AddHeader (control);
  return payload;
}

Ptr<Packet>
BuildOuterNeighborCheck (const std::vector<uint8_t> &record,
                         uint16_t hopSequence,
                         bool hasSecurityCount,
                         bool hasGroupSecurity)
{
  Ptr<Packet> frame = Create<Packet> (record.data (), record.size ());
  CsrHeader outer (SOURCE,
                   DESTINATION,
                   hopSequence,
                   7,
                   true,
                   false);
  outer.SetType (CSR_PKT_NEIGHBOR_CHECK);
  outer.SetDestType (CSR_DEST_UNICAST);
  if (hasSecurityCount)
    {
      outer.SetSecurityCount (SECURITY_COUNT);
    }
  outer.SetHasGroupSecurity (hasGroupSecurity);
  outer.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (outer);
  return frame;
}

Ptr<Packet>
BuildProtectedNeighborCheck (CsrHopSecurityState &sender,
                             uint8_t legacyPacketType,
                             uint16_t hopSequence,
                             bool tamperRecord = false,
                             bool hasGroupSecurity = false,
                             CsrNeighborCheckType type =
                               CsrNeighborCheckType::Verify)
{
  Ptr<Packet> payload = MakeNeighborCheckPayload (type);
  std::vector<uint8_t> plaintext = CopyBytes (payload);
  CsrProtectedPairwiseMessage protectedMessage =
    sender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise16,
      legacyPacketType,
      plaintext);
  if (tamperRecord)
    {
      protectedMessage.record.back () ^= 0x80;
    }
  return BuildOuterNeighborCheck (protectedMessage.record,
                                  hopSequence,
                                  true,
                                  hasGroupSecurity);
}

void
RecordDelivery (Ptr<Packet> payload,
                CsrNodeId source,
                double,
                double)
{
  Require (source == SOURCE,
           "NeighborCheck delivery reported the wrong source");
  CsrHelloHeader control;
  Require (payload->RemoveHeader (control) > 0,
           "NeighborCheck delivery lost its control header");
  Require (payload->GetSize () == 0,
           "NeighborCheck delivery gained unexpected raw bytes");
  Require (control.GetArlRouteMsgType () ==
             CsrArlRouteMsgType::NeighborCheck &&
             control.GetNeighborCheckTarget () == DESTINATION,
           "NeighborCheck delivery changed its control fields");
  g_deliveredType = control.GetNeighborCheckType ();
  g_deliveredDiscoverySequence = control.GetDiscoverySequence ();
  g_deliveredTypes.push_back (control.GetNeighborCheckType ());
  g_deliveries++;
}

void
RecordSuccess (CsrNodeId neighbor,
               CsrNeighborCheckType type,
               uint32_t discoverySequence)
{
  g_successEvents.push_back ({neighbor, type, discoverySequence});
}

void
RecordFailure (CsrNodeId neighbor,
               CsrNeighborCheckType type,
               uint32_t discoverySequence)
{
  g_failureEvents.push_back ({neighbor, type, discoverySequence});
}

void
CaptureFrame (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  if (frame->PeekHeader (header) &&
      header.GetType () == CSR_PKT_NEIGHBOR_CHECK)
    {
      g_capturedFrames.push_back (frame->Copy ());
    }
  else if (header.IsAck ())
    {
      g_capturedAckFrames++;
      g_capturedAckSizes.push_back (CsrGetOpnetWireSize (frame));
    }
}

void
RequireNoDeliveryOrAck (Ptr<CsrNetDevice> receiver,
                        const char *message)
{
  Require (g_deliveries == 0 &&
             receiver->GetMac ().GetAckQueuedFrameCount () == 0,
           message);
}

void
CheckAuthenticationBeforeDeliveryAndAck ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrNetDevice> sourceCapture =
    CreateObject<CsrNetDevice> (SOURCE);
  receiverDevice->AddPeer (sourceCapture);
  ConfigureNoErrorLink (receiverDevice, sourceCapture);
  sourceCapture->GetMac ().SetRxCallback (MakeCallback (&CaptureFrame));

  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  receiver->SetMac (&receiverDevice->GetMac ());
  ConfigureHop (receiver, DESTINATION, SOURCE);
  receiver->SetRxHelloFromHopCallback (MakeCallback (&RecordDelivery));

  CsrHopSecurityState sender;
  ConfigureSecurity (sender, SOURCE, DESTINATION);

  Ptr<Packet> plaintext = MakeNeighborCheckPayload ();
  CsrHeader plaintextOuter (SOURCE,
                            DESTINATION,
                            70,
                            7,
                            true,
                            false);
  plaintextOuter.SetType (CSR_PKT_NEIGHBOR_CHECK);
  plaintextOuter.SetDestType (CSR_DEST_UNICAST);
  plaintextOuter.SetLinkControl (8, 0.0, 0.0);
  plaintext->AddHeader (plaintextOuter);
  receiver->ReceiveFromMac (plaintext, 60.0, 40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "plaintext NeighborCheck earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildProtectedNeighborCheck (
      sender,
      LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
      71,
      false,
      true),
    60.0,
    40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "group-security-marked NeighborCheck earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildProtectedNeighborCheck (
      sender,
      LEGACY_ROUTING_UPDATE_PACKET_TYPE,
      72),
    60.0,
    40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "wrong-packet-type NeighborCheck earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildProtectedNeighborCheck (
      sender,
      LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
      73,
      true),
    60.0,
    40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "tampered NeighborCheck earned an ACK or delivery");

  const std::vector<uint8_t> truncatedRecord {0x00, 0x10, 0x02, 0x01};
  receiver->ReceiveFromMac (
    BuildOuterNeighborCheck (truncatedRecord, 74, true, false),
    60.0,
    40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "malformed NeighborCheck earned an ACK or delivery");

  const std::array<uint8_t, 1> truncatedPlaintext {0x01};
  CsrProtectedPairwiseMessage authenticatedTruncated =
    sender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise16,
      LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
      truncatedPlaintext);
  receiver->ReceiveFromMac (
    BuildOuterNeighborCheck (authenticatedTruncated.record,
                             75,
                             true,
                             false),
    60.0,
    40.0);
  RequireNoDeliveryOrAck (
    receiverDevice,
    "authenticated truncated NeighborCheck earned an ACK or delivery");

  Ptr<Packet> valid = BuildProtectedNeighborCheck (
    sender,
    LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
    76);
  receiver->ReceiveFromMac (valid->Copy (), 60.0, 40.0);
  Require (g_deliveries == 1 &&
             g_deliveredType == CsrNeighborCheckType::Verify &&
             g_deliveredDiscoverySequence == DISCOVERY_SEQUENCE &&
             receiverDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "authenticated NeighborCheck was not delivered and ACKed");

  // MAC keeps an exact ACK for its five-transmission budget. Let that first
  // ACK retire before presenting the authenticated HOP retransmission; while
  // it remains queued, MAC correctly coalesces an identical ACK request.
  Simulator::Stop (Seconds (10.0));
  Simulator::Run ();
  Require (g_capturedAckFrames == 5 &&
             g_capturedAckSizes.size () == 5 &&
             std::all_of (g_capturedAckSizes.begin (),
                          g_capturedAckSizes.end (),
                          [] (uint32_t size) { return size == 30; }) &&
             receiverDevice->GetMac ().GetAckQueuedFrameCount () == 0,
           "first NeighborCheck ACK was not exact Pairwise16");

  receiver->ReceiveFromMac (valid->Copy (), 60.0, 40.0);
  Require (g_deliveries == 1 &&
             receiverDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "authenticated NeighborCheck duplicate was delivered twice or not re-ACKed");
  Simulator::Stop (Simulator::Now () + Seconds (3.0));
  Simulator::Run ();
  Require (g_capturedAckFrames > 5 &&
             g_capturedAckSizes.size () == g_capturedAckFrames &&
             g_capturedAckSizes.back () == 30,
           "NeighborCheck duplicate did not send a second exact ACK");

  Simulator::Destroy ();
}

void
CheckAllOperationalSubtypes ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  receiver->SetMac (&receiverDevice->GetMac ());
  ConfigureHop (receiver, DESTINATION, SOURCE);
  receiver->SetRxHelloFromHopCallback (MakeCallback (&RecordDelivery));

  CsrHopSecurityState sender;
  ConfigureSecurity (sender, SOURCE, DESTINATION);

  const std::array<CsrNeighborCheckType, 5> operationalTypes {{
    CsrNeighborCheckType::Discovery,
    CsrNeighborCheckType::Message,
    CsrNeighborCheckType::NoPath,
    CsrNeighborCheckType::Overheard,
    CsrNeighborCheckType::Verify
  }};

  for (std::size_t index = 0; index < operationalTypes.size (); ++index)
    {
      receiver->ReceiveFromMac (
        BuildProtectedNeighborCheck (
          sender,
          LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
          static_cast<uint16_t> (100 + index),
          false,
          false,
          operationalTypes[index]),
        60.0,
        40.0);
    }

  Require (g_deliveries == operationalTypes.size () &&
             g_deliveredTypes == std::vector<CsrNeighborCheckType> (
               operationalTypes.begin (), operationalTypes.end ()) &&
             receiverDevice->GetMac ().GetAckQueuedFrameCount () ==
               operationalTypes.size (),
           "operational NeighborCheck subtypes did not share type-9 policy");

  Simulator::Destroy ();
}

void
InjectProtectedAck (Ptr<CsrHopLayer> senderHop,
                    uint16_t acknowledgedSequence)
{
  CsrHopSecurityState ackSender;
  ConfigureSecurity (ackSender, DESTINATION, SOURCE);

  CsrHeader authenticated (DESTINATION,
                           SOURCE,
                           acknowledgedSequence,
                           7,
                           false,
                           true);
  authenticated.SetType (CSR_PKT_ACK);
  authenticated.SetDestType (CSR_DEST_UNICAST);
  authenticated.SetSpeedKey (8);

  Ptr<Packet> authenticatedPacket = Create<Packet> ();
  authenticatedPacket->AddHeader (authenticated);
  std::vector<uint8_t> body = CopyBytes (authenticatedPacket);
  CsrProtectedPairwiseMessage protectedAck =
    ackSender.ProtectPairwiseMessage (
      SOURCE,
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      body);

  Ptr<Packet> frame = Create<Packet> (protectedAck.record.data (),
                                      protectedAck.record.size ());
  CsrHeader outer = authenticated;
  outer.SetSecurityCount (ackSender.GetOwnSecurityCount ());
  outer.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (outer);
  senderHop->ReceiveFromMac (frame, 60.0, 40.0);
}

void
CheckCapturedSourceExactWrapper (Ptr<Packet> frame)
{
  Require (CsrGetOpnetWireSize (frame) == 16,
           "Pairwise16 NeighborCheck did not model 11 + 5 bytes");

  Ptr<Packet> recordPacket = frame->Copy ();
  CsrHeader outer;
  Require (recordPacket->RemoveHeader (outer) > 0,
           "captured NeighborCheck lost its HOP header");
  Require (outer.GetType () == CSR_PKT_NEIGHBOR_CHECK &&
             outer.GetSrc () == SOURCE &&
             outer.GetDst () == DESTINATION &&
             outer.IsAckable () &&
             outer.HasSecurityCount () &&
             !outer.HasGroupSecurity (),
           "captured NeighborCheck changed its Pairwise16 envelope");

  std::vector<uint8_t> record = CopyBytes (recordPacket);
  CsrHopSecurityState verifier;
  ConfigureSecurity (verifier, DESTINATION, SOURCE);
  CsrReceivedPairwiseMessage received =
    verifier.ReceivePairwiseMessage (
      SOURCE,
      outer.GetSecurityCount (),
      CsrPairwiseSecurityMode::Pairwise16,
      LEGACY_NEIGHBOR_CHECK_PACKET_TYPE,
      record);
  Require (received.status == CsrHopSecurityReceiveStatus::Accepted,
           "outbound NeighborCheck did not authenticate as packet type 9");

  CsrHopSecurityState wrongTypeVerifier;
  ConfigureSecurity (wrongTypeVerifier, DESTINATION, SOURCE);
  Require (wrongTypeVerifier.ReceivePairwiseMessage (
             SOURCE,
             outer.GetSecurityCount (),
             CsrPairwiseSecurityMode::Pairwise16,
             LEGACY_ROUTING_UPDATE_PACKET_TYPE,
             record).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "outbound NeighborCheck did not bind packet type 9 into HMAC");

  Ptr<Packet> plaintext = Create<Packet> (received.payload.data (),
                                          received.payload.size ());
  CsrHelloHeader control;
  Require (plaintext->RemoveHeader (control) > 0 &&
             plaintext->GetSize () == 0 &&
             control.GetArlRouteMsgType () ==
               CsrArlRouteMsgType::NeighborCheck &&
             control.GetNeighborCheckType () ==
               CsrNeighborCheckType::Verify &&
             control.GetDiscoverySequence () == DISCOVERY_SEQUENCE,
           "outbound Pairwise16 record changed NeighborCheck plaintext");
}

void
CheckLiveOutboundAndSuccessMetadata ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> senderDevice = CreateObject<CsrNetDevice> (SOURCE);
  Ptr<CsrNetDevice> captureDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  senderDevice->AddPeer (captureDevice);
  ConfigureNoErrorLink (senderDevice, captureDevice);
  captureDevice->GetMac ().SetRxCallback (MakeCallback (&CaptureFrame));

  Ptr<CsrHopLayer> sender = CreateObject<CsrHopLayer> ();
  sender->SetMac (&senderDevice->GetMac ());
  ConfigureHop (sender, SOURCE, DESTINATION);
  sender->SetNeighborCheckSuccessCallback (MakeCallback (&RecordSuccess));
  sender->SetNeighborCheckFailureCallback (MakeCallback (&RecordFailure));

  sender->SendNeighborCheck (DESTINATION, MakeNeighborCheckPayload ());
  Simulator::Schedule (Seconds (1.7), &InjectProtectedAck, sender, 1);
  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();

  Require (g_capturedFrames.size () == 1,
           "successful NeighborCheck did not transmit exactly once");
  CheckCapturedSourceExactWrapper (g_capturedFrames.front ());
  Require (g_successEvents.size () == 1 &&
             g_successEvents.front ().neighbor == DESTINATION &&
             g_successEvents.front ().type == CsrNeighborCheckType::Verify &&
             g_successEvents.front ().discoverySequence ==
               DISCOVERY_SEQUENCE &&
             g_failureEvents.empty () &&
             sender->GetResendQueueSize () == 0,
           "NeighborCheck success lost its plaintext completion metadata");

  Simulator::Destroy ();
}

void
CheckIdenticalRetransmissionAndFailureMetadata ()
{
  ResetObservations ();

  Ptr<CsrNetDevice> senderDevice = CreateObject<CsrNetDevice> (SOURCE);
  Ptr<CsrNetDevice> captureDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  senderDevice->AddPeer (captureDevice);
  ConfigureNoErrorLink (senderDevice, captureDevice);
  captureDevice->GetMac ().SetRxCallback (MakeCallback (&CaptureFrame));

  Ptr<CsrHopLayer> sender = CreateObject<CsrHopLayer> ();
  sender->SetMac (&senderDevice->GetMac ());
  ConfigureHop (sender, SOURCE, DESTINATION);
  sender->SetNeighborCheckSuccessCallback (MakeCallback (&RecordSuccess));
  sender->SetNeighborCheckFailureCallback (MakeCallback (&RecordFailure));

  sender->SendNeighborCheck (DESTINATION, MakeNeighborCheckPayload ());
  Simulator::Stop (Seconds (15.0));
  Simulator::Run ();

  Require (g_capturedFrames.size () == 3,
           "NeighborCheck did not make one send plus two retries");
  std::vector<uint8_t> firstRecord;
  for (const Ptr<Packet> &captured : g_capturedFrames)
    {
      Require (CsrGetOpnetWireSize (captured) == 16,
               "NeighborCheck retry changed its 16-byte modeled size");
      Ptr<Packet> recordPacket = captured->Copy ();
      CsrHeader outer;
      recordPacket->RemoveHeader (outer);
      std::vector<uint8_t> record = CopyBytes (recordPacket);
      if (firstRecord.empty ())
        {
          firstRecord = record;
        }
      else
        {
          Require (record == firstRecord,
                   "NeighborCheck retry regenerated its protected record");
        }
    }

  Require (g_successEvents.empty () &&
             g_failureEvents.size () == 1 &&
             g_failureEvents.front ().neighbor == DESTINATION &&
             g_failureEvents.front ().type == CsrNeighborCheckType::Verify &&
             g_failureEvents.front ().discoverySequence ==
               DISCOVERY_SEQUENCE &&
             sender->GetResendQueueSize () == 0,
           "NeighborCheck final failure lost its plaintext metadata");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  InitializeKeys ();

  CheckAuthenticationBeforeDeliveryAndAck ();
  CheckAllOperationalSubtypes ();
  CheckLiveOutboundAndSuccessMetadata ();
  CheckIdenticalRetransmissionAndFailureMetadata ();

  std::cout << "PASS: source-exact Pairwise16 NeighborCheck wrapper test"
            << std::endl;
  return 0;
}
