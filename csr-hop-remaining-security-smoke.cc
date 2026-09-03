#include "ns3/csr-hop-security.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr CsrNodeId SOURCE = 0x010203;
constexpr CsrNodeId DESTINATION = 0x040506;
constexpr CsrNodeId RELAY_DESTINATION = 0x070809;
constexpr uint16_t SECURITY_COUNT = 7;

CsrLegacyCrypto::MissionKey g_missionKey {};
CsrLegacyCrypto::PairwiseKeyMaterial g_pairwiseMaterial {};
CsrLegacyCrypto::GroupKeyMaterial g_groupKey {};

uint32_t g_manualPairwiseDeliveries = 0;
uint32_t g_manualNeighborcastDeliveries = 0;
std::vector<uint8_t> g_manualLastPayload;

uint32_t g_livePairwiseDeliveries = 0;
uint32_t g_liveNeighborcastDeliveries = 0;
std::vector<std::vector<uint8_t>> g_livePayloads;

std::vector<Time> g_ackWakeTimes;
Ptr<Packet> g_capturedDack;
uint32_t g_capturedDackPacketBits = 0;
uint32_t g_liveDackRelayDeliveries = 0;
Ptr<CsrHopLayer> g_profileDataReceiver;
Ptr<Packet> g_capturedProfileData;
uint32_t g_capturedProfileDataPacketBits = 0;
uint32_t g_profileDataDeliveries = 0;
std::vector<uint8_t> g_profileDataPayload;

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
CopyBytes (Ptr<Packet> packet)
{
  std::vector<uint8_t> bytes (packet->GetSize ());
  if (!bytes.empty ())
    {
      packet->CopyData (bytes.data (), bytes.size ());
    }
  return bytes;
}

std::vector<uint8_t>
SerializeHeader (const CsrHeader &header)
{
  Ptr<Packet> packet = Create<Packet> ();
  packet->AddHeader (header);
  return CopyBytes (packet);
}

Ptr<Packet>
BuildSecuredAckFrame (CsrHopSecurityState &sender,
                      const CsrHeader &authenticatedHeader,
                      uint8_t legacyPacketType = 0,
                      bool tamperRecord = false,
                      bool clearOuterAckFlag = false)
{
  std::vector<uint8_t> body = SerializeHeader (authenticatedHeader);
  CsrProtectedPairwiseMessage secured =
    sender.ProtectPairwiseMessage (
      authenticatedHeader.GetDst (),
      CsrPairwiseSecurityMode::Pairwise16,
      legacyPacketType,
      body);
  if (tamperRecord)
    {
      secured.record.back () ^= 0x80;
    }

  Ptr<Packet> frame = Create<Packet> (secured.record.data (),
                                      secured.record.size ());
  CsrHeader outer = authenticatedHeader;
  if (clearOuterAckFlag)
    {
      outer.SetIsAck (false);
    }
  outer.SetSecurityCount (sender.GetOwnSecurityCount ());
  outer.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (outer);
  return frame;
}

Ptr<Packet>
BuildSecuredMalformedAckFrame (CsrHopSecurityState &sender,
                               const CsrHeader &outerHeader)
{
  const std::array<uint8_t, 1> truncatedBody {0x01};
  CsrProtectedPairwiseMessage secured =
    sender.ProtectPairwiseMessage (
      outerHeader.GetDst (),
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      truncatedBody);
  Ptr<Packet> frame = Create<Packet> (secured.record.data (),
                                      secured.record.size ());
  CsrHeader outer = outerHeader;
  outer.SetSecurityCount (sender.GetOwnSecurityCount ());
  outer.SetLinkControl (8, 0.0, 0.0);
  frame->AddHeader (outer);
  return frame;
}

void
InitializeKeys ()
{
  for (std::size_t i = 0; i < g_missionKey.size (); ++i)
    {
      g_missionKey[i] = static_cast<uint8_t> (i);
      g_groupKey[i] = static_cast<uint8_t> (0xa0 + i);
    }
  for (std::size_t i = 0; i < g_pairwiseMaterial.size (); ++i)
    {
      g_pairwiseMaterial[i] = static_cast<uint8_t> (0x40 + i);
    }
}

void
ConfigureState (CsrHopSecurityState &state,
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
RequireAcceptedPairwise (const CsrReceivedPairwiseMessage &received,
                         const std::vector<uint8_t> &expected,
                         const char *message)
{
  Require (received.status == CsrHopSecurityReceiveStatus::Accepted &&
             received.payload == expected,
           message);
}

void
CheckGoldenPairwiseRecords ()
{
  const std::vector<uint8_t> payload {
    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
    0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0
  };

  CsrHopSecurityState pairwise16Sender;
  CsrHopSecurityState pairwise16Receiver;
  ConfigureState (pairwise16Sender, SOURCE, DESTINATION);
  ConfigureState (pairwise16Receiver, DESTINATION, SOURCE);

  CsrProtectedPairwiseMessage pairwise16 =
    pairwise16Sender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise16,
      3,
      payload);
  const std::vector<uint8_t> expectedPairwise16 {
    0x00, 0x10, 0x02, 0x10, 0x20, 0x30, 0x40, 0x50,
    0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0,
    0xb5, 0x24
  };
  Require (pairwise16.keyId == 1 && pairwise16.sequence == 2 &&
             pairwise16.record == expectedPairwise16,
           "Pairwise16 record changed from its independent golden vector");
  RequireAcceptedPairwise (
    pairwise16Receiver.ReceivePairwiseMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrPairwiseSecurityMode::Pairwise16,
      3,
      pairwise16.record),
    payload,
    "valid Pairwise16 record failed authentication");
  Require (pairwise16Receiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise16,
             3,
             pairwise16.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticatedDuplicate,
           "Pairwise16 replay was not recognized after authentication");

  CsrHopSecurityState pairwise32Sender;
  CsrHopSecurityState pairwise32Receiver;
  ConfigureState (pairwise32Sender, SOURCE, DESTINATION);
  ConfigureState (pairwise32Receiver, DESTINATION, SOURCE);
  CsrProtectedPairwiseMessage pairwise32 =
    pairwise32Sender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise32,
      3,
      payload);
  const std::vector<uint8_t> expectedPairwise32 {
    0x00, 0x10, 0x02, 0x10, 0x20, 0x30, 0x40, 0x50,
    0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0,
    0xb5, 0x24, 0xec, 0x76
  };
  Require (pairwise32.record == expectedPairwise32,
           "Pairwise32 record changed from its independent golden vector");
  RequireAcceptedPairwise (
    pairwise32Receiver.ReceivePairwiseMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrPairwiseSecurityMode::Pairwise32,
      3,
      pairwise32.record),
    payload,
    "valid Pairwise32 record failed authentication");

  CsrHopSecurityState encryptedSender;
  CsrHopSecurityState encryptedReceiver;
  ConfigureState (encryptedSender, SOURCE, DESTINATION);
  ConfigureState (encryptedReceiver, DESTINATION, SOURCE);
  CsrProtectedPairwiseMessage encrypted =
    encryptedSender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise32Encrypt,
      3,
      payload);
  const std::vector<uint8_t> expectedEncrypted {
    0x00, 0x10, 0x02, 0x3b, 0xf9, 0xe1, 0x93, 0xc9,
    0xaa, 0xd0, 0x5f, 0x8d, 0x91, 0xb7, 0xf3, 0xfa,
    0x0a, 0x63, 0xaa, 0x15
  };
  Require (encrypted.record == expectedEncrypted,
           "Pairwise32Encrypt record changed from its golden vector");

  std::vector<uint8_t> tampered = encrypted.record;
  tampered[5] ^= 0x80;
  Require (encryptedReceiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise32Encrypt,
             3,
             tampered).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "tampered Pairwise32Encrypt ciphertext authenticated");
  RequireAcceptedPairwise (
    encryptedReceiver.ReceivePairwiseMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrPairwiseSecurityMode::Pairwise32Encrypt,
      3,
      encrypted.record),
    payload,
    "tampered Pairwise32Encrypt record consumed replay state");

  std::vector<uint8_t> badTag = encrypted.record;
  badTag.back () ^= 0x01;
  Require (encryptedReceiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise32Encrypt,
             3,
             badTag).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "a tampered duplicate bypassed authentication");

  CsrHopSecurityState wrongTypeReceiver;
  ConfigureState (wrongTypeReceiver, DESTINATION, SOURCE);
  Require (wrongTypeReceiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise32Encrypt,
             7,
             encrypted.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "Pairwise32Encrypt did not authenticate the legacy packet type");

  CsrHopSecurityState wrongKeyReceiver;
  ConfigureState (wrongKeyReceiver, DESTINATION, SOURCE);
  CsrLegacyCrypto::PairwiseKeyMaterial wrongMaterial = g_pairwiseMaterial;
  wrongMaterial[0] ^= 0xff;
  wrongKeyReceiver.SetPairwiseKeyMaterial (SOURCE, wrongMaterial);
  Require (wrongKeyReceiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise32Encrypt,
             3,
             encrypted.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "Pairwise32Encrypt authenticated with the wrong pairwise key");

  Require (wrongKeyReceiver.ReceivePairwiseMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrPairwiseSecurityMode::Pairwise16,
             3,
             std::array<uint8_t, 4> {0, 1, 2, 3}).status ==
             CsrHopSecurityReceiveStatus::Malformed,
           "short Pairwise16 record was not rejected as malformed");
}

Ptr<Packet>
BuildProtectedFrame (const std::vector<uint8_t> &record,
                     uint8_t packetType,
                     uint16_t hopSequence,
                     bool ackable,
                     bool groupSecurity)
{
  Ptr<Packet> frame = Create<Packet> (record.data (), record.size ());
  CsrHeader header (SOURCE,
                    groupSecurity ? CSR_BROADCAST_ID : DESTINATION,
                    hopSequence,
                    7,
                    ackable,
                    false);
  header.SetType (packetType);
  header.SetDestType (groupSecurity ? CSR_DEST_BROADCAST
                                    : CSR_DEST_UNICAST);
  header.SetSecurityCount (SECURITY_COUNT);
  header.SetHasGroupSecurity (groupSecurity);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildBareDataFrame (const std::vector<uint8_t> &payload,
                    uint16_t hopSequence,
                    bool ackable)
{
  Ptr<Packet> frame = Create<Packet> (payload.data (), payload.size ());
  CsrHeader header (SOURCE,
                    DESTINATION,
                    hopSequence,
                    7,
                    ackable,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  frame->AddHeader (header);
  return frame;
}

void
NoteManualPairwise (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE,
           "manual pairwise callback reported the wrong source");
  g_manualLastPayload = CopyBytes (payload);
  g_manualPairwiseDeliveries++;
}

void
NoteManualNeighborcast (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE,
           "manual neighborcast callback reported the wrong source");
  g_manualLastPayload = CopyBytes (payload);
  g_manualNeighborcastDeliveries++;
}

void
CheckAuthenticationBeforeAckAndDelivery ()
{
  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  receiver->SetMac (&receiverDevice->GetMac ());
  ConfigureHop (receiver, DESTINATION, SOURCE);
  receiver->SetRxFromHopCallback (MakeCallback (&NoteManualPairwise));
  receiver->SetRxNeighborcastFromHopCallback (
    MakeCallback (&NoteManualNeighborcast));

  CsrHopSecurityState sender;
  ConfigureState (sender, SOURCE, DESTINATION);
  sender.SetGroupKeyMaterial (g_groupKey);

  g_manualPairwiseDeliveries = 0;
  g_manualNeighborcastDeliveries = 0;
  g_manualLastPayload.clear ();

  const std::vector<uint8_t> pairwisePayload {0x21, 0x22, 0x23, 0x24};
  CsrProtectedPairwiseMessage pairwise =
    sender.ProtectPairwiseMessage (DESTINATION,
                                   CsrPairwiseSecurityMode::Pairwise16,
                                   3,
                                   pairwisePayload);
  std::vector<uint8_t> tamperedPairwise = pairwise.record;
  tamperedPairwise.back () ^= 0x80;

  receiver->ReceiveFromMac (
    BuildProtectedFrame (tamperedPairwise,
                         CSR_PKT_DATA,
                         10,
                         true,
                         false),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == 0 &&
             g_manualPairwiseDeliveries == 0,
           "tampered Pairwise16 DATA earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildProtectedFrame (pairwise.record,
                         CSR_PKT_DATA,
                         10,
                         true,
                         false),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == 1 &&
             g_manualPairwiseDeliveries == 1 &&
             g_manualLastPayload == pairwisePayload,
           "valid Pairwise16 DATA was not ACKed and delivered");

  receiver->ReceiveFromMac (
    BuildProtectedFrame (pairwise.record,
                         CSR_PKT_DATA,
                         10,
                         true,
                         false),
    60.0,
    40.0);
  Require (g_manualPairwiseDeliveries == 1,
           "authenticated Pairwise16 retransmit was delivered twice");

  const std::vector<uint8_t> encryptedPayload {
    0x31, 0x32, 0x33, 0x34, 0x35
  };
  CsrProtectedPairwiseMessage encrypted =
    sender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise32Encrypt,
      3,
      encryptedPayload);
  std::vector<uint8_t> tamperedEncrypted = encrypted.record;
  tamperedEncrypted[4] ^= 0x01;
  uint32_t ackCount = receiverDevice->GetMac ().GetAckQueuedFrameCount ();

  receiver->ReceiveFromMac (
    BuildProtectedFrame (tamperedEncrypted,
                         CSR_PKT_PAIRWISE32_DATA,
                         11,
                         true,
                         false),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == ackCount &&
             g_manualPairwiseDeliveries == 1,
           "tampered Pairwise32Encrypt DATA earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildProtectedFrame (encrypted.record,
                         CSR_PKT_PAIRWISE32_DATA,
                         11,
                         true,
                         false),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () >= ackCount &&
             g_manualPairwiseDeliveries == 2 &&
             g_manualLastPayload == encryptedPayload,
           "valid Pairwise32Encrypt DATA was not ACKed and decrypted");

  CsrKeyUpdateHeader keyUpdate = sender.BuildKeyUpdate (DESTINATION);
  Ptr<Packet> keyUpdateFrame = Create<Packet> ();
  keyUpdateFrame->AddHeader (keyUpdate);
  CsrHeader keyUpdateHop (SOURCE, DESTINATION, 12, 7, true, false);
  keyUpdateHop.SetType (CSR_PKT_KEY_UPDATE);
  keyUpdateHop.SetDestType (CSR_DEST_UNICAST);
  keyUpdateHop.SetSecurityCount (SECURITY_COUNT);
  keyUpdateFrame->AddHeader (keyUpdateHop);
  receiver->ReceiveFromMac (keyUpdateFrame, 60.0, 40.0);

  const std::vector<uint8_t> neighborcastPayload {
    0x41, 0x42, 0x43, 0x44, 0x45, 0x46
  };
  CsrProtectedGroupMessage neighborcast = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group32Encrypt,
    6,
    neighborcastPayload);
  std::vector<uint8_t> tamperedNeighborcast = neighborcast.record;
  tamperedNeighborcast.back () ^= 0x40;
  ackCount = receiverDevice->GetMac ().GetAckQueuedFrameCount ();

  receiver->ReceiveFromMac (
    BuildProtectedFrame (tamperedNeighborcast,
                         CSR_PKT_NEIGHBORCAST,
                         13,
                         false,
                         true),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == ackCount &&
             g_manualNeighborcastDeliveries == 0,
           "tampered AppNeighborcast earned an ACK or delivery");

  Ptr<Packet> malformedNeighborcast = BuildProtectedFrame (
    neighborcast.record,
    CSR_PKT_NEIGHBORCAST,
    13,
    false,
    true);
  CsrHeader malformedHeader;
  malformedNeighborcast->RemoveHeader (malformedHeader);
  malformedHeader.SetDst (DESTINATION);
  malformedHeader.SetDestType (CSR_DEST_UNICAST);
  malformedNeighborcast->AddHeader (malformedHeader);
  receiver->ReceiveFromMac (malformedNeighborcast, 60.0, 40.0);
  Require (g_manualNeighborcastDeliveries == 0,
           "unicast envelope was accepted as AppNeighborcast");

  receiver->ReceiveFromMac (
    BuildProtectedFrame (neighborcast.record,
                         CSR_PKT_NEIGHBORCAST,
                         13,
                         false,
                         true),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == ackCount &&
             g_manualNeighborcastDeliveries == 1 &&
             g_manualLastPayload == neighborcastPayload,
           "valid AppNeighborcast was not decrypted exactly once");

  receiver->ReceiveFromMac (
    BuildProtectedFrame (neighborcast.record,
                         CSR_PKT_NEIGHBORCAST,
                         13,
                         false,
                         true),
    60.0,
    40.0);
  Require (g_manualNeighborcastDeliveries == 1,
           "AppNeighborcast replay was delivered twice");

  Simulator::Destroy ();
}

void
CheckCrossProfileDataRejection ()
{
  const std::vector<uint8_t> payload {0x31, 0x32, 0x33, 0x34};
  constexpr uint16_t hopSequence = 93;
  CsrHopSecurityState sender;
  ConfigureState (sender, SOURCE, DESTINATION);
  CsrProtectedPairwiseMessage protectedData =
    sender.ProtectPairwiseMessage (DESTINATION,
                                   CsrPairwiseSecurityMode::Pairwise16,
                                   3,
                                   payload);

  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (DESTINATION);
    Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
    receiver->SetMac (&device->GetMac ());
    ConfigureHop (receiver, DESTINATION, SOURCE);
    receiver->SetRxFromHopCallback (MakeCallback (&NoteManualPairwise));
    g_manualPairwiseDeliveries = 0;
    g_manualLastPayload.clear ();

    receiver->ReceiveFromMac (
      BuildBareDataFrame (payload, hopSequence, true),
      60.0,
      40.0);
    Require (g_manualPairwiseDeliveries == 0 &&
               device->GetMac ().GetAckQueuedFrameCount () == 0,
             "production profile accepted bare ordinary DATA");

    receiver->ReceiveFromMac (
      BuildProtectedFrame (protectedData.record,
                           CSR_PKT_DATA,
                           hopSequence,
                           true,
                           false),
      60.0,
      40.0);
    Require (g_manualPairwiseDeliveries == 1 &&
               g_manualLastPayload == payload &&
               device->GetMac ().GetAckQueuedFrameCount () == 1,
             "bare mismatch poisoned the matching production DATA sequence");
    Simulator::Destroy ();
  }

  {
    Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (DESTINATION);
    Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
    receiver->SetMac (&device->GetMac ());
    ConfigureHop (receiver, DESTINATION, SOURCE);
    receiver->SetHopWireProfile (CsrHopWireProfile::HIST_ADB97C54_BARE);
    receiver->SetRxFromHopCallback (MakeCallback (&NoteManualPairwise));
    g_manualPairwiseDeliveries = 0;
    g_manualLastPayload.clear ();

    receiver->ReceiveFromMac (
      BuildProtectedFrame (protectedData.record,
                           CSR_PKT_DATA,
                           hopSequence,
                           true,
                           false),
      60.0,
      40.0);
    Require (g_manualPairwiseDeliveries == 0 &&
               device->GetMac ().GetAckQueuedFrameCount () == 0,
             "historical profile accepted Pairwise16 ordinary DATA");

    receiver->ReceiveFromMac (
      BuildBareDataFrame (payload, hopSequence, true),
      60.0,
      40.0);
    Require (g_manualPairwiseDeliveries == 1 &&
               g_manualLastPayload == payload &&
               device->GetMac ().GetAckQueuedFrameCount () == 1,
             "protected mismatch poisoned the matching historical DATA sequence");
    Simulator::Destroy ();
  }
}

enum class InvalidOrdinaryDataMetadata : uint8_t
{
  IsAck,
  IsDack,
  AckWindow,
  DestinationSequences,
  GroupSecurity,
  BroadcastDestType,
  MulticastDestType,
  BroadcastSource,
  BroadcastDestination
};

void
ApplyInvalidOrdinaryDataMetadata (CsrHeader &header,
                                  InvalidOrdinaryDataMetadata invalid,
                                  uint16_t hopSequence)
{
  switch (invalid)
    {
    case InvalidOrdinaryDataMetadata::IsAck:
      header.SetIsAck (true);
      break;
    case InvalidOrdinaryDataMetadata::IsDack:
      header.SetIsDack (true);
      break;
    case InvalidOrdinaryDataMetadata::AckWindow:
      header.SetHasAckWindow (true);
      break;
    case InvalidOrdinaryDataMetadata::DestinationSequences:
      header.SetDestinationSequences ({{DESTINATION, hopSequence}});
      break;
    case InvalidOrdinaryDataMetadata::GroupSecurity:
      header.SetHasGroupSecurity (true);
      break;
    case InvalidOrdinaryDataMetadata::BroadcastDestType:
      header.SetDestType (CSR_DEST_BROADCAST);
      break;
    case InvalidOrdinaryDataMetadata::MulticastDestType:
      header.SetDestType (CSR_DEST_MULTICAST);
      break;
    case InvalidOrdinaryDataMetadata::BroadcastSource:
      header.SetSrc (CSR_BROADCAST_ID);
      break;
    case InvalidOrdinaryDataMetadata::BroadcastDestination:
      header.SetDst (CSR_BROADCAST_ID);
      break;
    }
}

void
CheckMalformedOrdinaryDataEnvelopes (CsrHopWireProfile profile)
{
  constexpr std::array<InvalidOrdinaryDataMetadata, 9> invalidCases {
    InvalidOrdinaryDataMetadata::IsAck,
    InvalidOrdinaryDataMetadata::IsDack,
    InvalidOrdinaryDataMetadata::AckWindow,
    InvalidOrdinaryDataMetadata::DestinationSequences,
    InvalidOrdinaryDataMetadata::GroupSecurity,
    InvalidOrdinaryDataMetadata::BroadcastDestType,
    InvalidOrdinaryDataMetadata::MulticastDestType,
    InvalidOrdinaryDataMetadata::BroadcastSource,
    InvalidOrdinaryDataMetadata::BroadcastDestination
  };
  const std::vector<uint8_t> payload {0x41, 0x42, 0x43, 0x44};
  uint16_t hopSequence = 120;

  for (InvalidOrdinaryDataMetadata invalid : invalidCases)
    {
      Ptr<CsrNetDevice> device = CreateObject<CsrNetDevice> (DESTINATION);
      Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
      receiver->SetMac (&device->GetMac ());
      ConfigureHop (receiver, DESTINATION, SOURCE);
      receiver->SetHopWireProfile (profile);
      receiver->SetRxFromHopCallback (MakeCallback (&NoteManualPairwise));

      Ptr<Packet> body;
      if (profile == CsrHopWireProfile::HIST_ADB97C54_BARE)
        {
          body = Create<Packet> (payload.data (), payload.size ());
        }
      else
        {
          CsrHopSecurityState sender;
          ConfigureState (sender, SOURCE, DESTINATION);
          CsrProtectedPairwiseMessage protectedData =
            sender.ProtectPairwiseMessage (
              DESTINATION,
              CsrPairwiseSecurityMode::Pairwise16,
              3,
              payload);
          body = Create<Packet> (protectedData.record.data (),
                                 protectedData.record.size ());
        }

      CsrHeader validHeader (SOURCE,
                             DESTINATION,
                             hopSequence,
                             5,
                             true,
                             false);
      validHeader.SetType (CSR_PKT_DATA);
      validHeader.SetDestType (CSR_DEST_UNICAST);
      if (profile == CsrHopWireProfile::PRODUCTION_PAIRWISE16)
        {
          validHeader.SetSecurityCount (SECURITY_COUNT);
        }

      CsrHeader malformedHeader = validHeader;
      ApplyInvalidOrdinaryDataMetadata (malformedHeader,
                                        invalid,
                                        hopSequence);
      Ptr<Packet> malformedFrame = body->Copy ();
      malformedFrame->AddHeader (malformedHeader);

      g_manualPairwiseDeliveries = 0;
      g_manualLastPayload.clear ();
      receiver->ReceiveFromMac (malformedFrame, 60.0, 40.0);
      Require (g_manualPairwiseDeliveries == 0 &&
                 device->GetMac ().GetAckQueuedFrameCount () == 0 &&
                 !receiver->HasNeighbor (SOURCE),
               "malformed ordinary DATA changed receive state");

      Ptr<Packet> validFrame = body->Copy ();
      validFrame->AddHeader (validHeader);
      receiver->ReceiveFromMac (validFrame, 60.0, 40.0);
      Require (g_manualPairwiseDeliveries == 1 &&
                 g_manualLastPayload == payload &&
                 device->GetMac ().GetAckQueuedFrameCount () == 1,
               "malformed DATA poisoned the matching valid HOP sequence");

      Simulator::Destroy ();
      hopSequence++;
    }
}

void
NoteAckWake ()
{
  g_ackWakeTimes.push_back (Simulator::Now ());
}

void
CheckAckDackSecurityPolicy ()
{
  Ptr<CsrNetDevice> localDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> localHop = CreateObject<CsrHopLayer> ();
  Require (
    localHop->GetHopWireProfile () ==
      CsrHopWireProfile::PRODUCTION_PAIRWISE16,
    "production Pairwise16 HOP wire profile is not the default");
  localHop->SetMac (&localDevice->GetMac ());
  ConfigureHop (localHop, DESTINATION, SOURCE);
  localHop->SetNwkQueueWakeCallback (MakeCallback (&NoteAckWake));

  CsrHopSecurityState ackSender;
  ConfigureState (ackSender, SOURCE, DESTINATION);
  CsrHeader ackHeader (SOURCE,
                       DESTINATION,
                       1,
                       7,
                       false,
                       true);
  ackHeader.SetType (CSR_PKT_ACK);
  ackHeader.SetDestType (CSR_DEST_UNICAST);
  ackHeader.SetSpeedKey (8);

  g_ackWakeTimes.clear ();
  localHop->SendData (SOURCE, 5, Create<Packet> (4), true);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1,
           "ACK security test did not create one pending DATA record");

  Ptr<Packet> plaintextAck = Create<Packet> ();
  plaintextAck->AddHeader (ackHeader);
  localHop->ReceiveFromMac (plaintextAck, 60.0, 40.0);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1 &&
             g_ackWakeTimes.empty (),
           "plaintext ACK changed custody or requested an NWK wake");

  localHop->ReceiveFromMac (
    BuildSecuredAckFrame (ackSender, ackHeader, 3),
    60.0,
    40.0);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1 &&
             g_ackWakeTimes.empty (),
           "wrong-packet-type ACK authenticated or requested a wake");

  localHop->ReceiveFromMac (
    BuildSecuredAckFrame (ackSender, ackHeader, 0, true),
    60.0,
    40.0);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1 &&
             g_ackWakeTimes.empty (),
           "tampered ACK changed custody or requested an NWK wake");

  localHop->ReceiveFromMac (
    BuildSecuredMalformedAckFrame (ackSender, ackHeader),
    60.0,
    40.0);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1 &&
             g_ackWakeTimes.empty (),
           "truncated authenticated ACK changed custody or requested a wake");

  // The outer IsAck bit is transport metadata. The authenticated inner body
  // remains authoritative, so a cleared outer bit cannot bypass verification
  // or redirect the frame into the DATA path.
  Ptr<Packet> validAck = BuildSecuredAckFrame (
    ackSender,
    ackHeader,
    0,
    false,
    true);
  localHop->ReceiveFromMac (validAck->Copy (), 60.0, 40.0);
  Require (localHop->GetPendingDataCount () == 0 &&
             localHop->GetResendQueueSize () == 0 &&
             localDevice->GetMac ().GetDataQueuedFrameCount () == 0 &&
             g_ackWakeTimes.empty (),
           "valid Pairwise16 ACK did not complete DATA before delayed wake");

  const Time tic = CsrOpnetTic ();
  Simulator::Stop (tic + NanoSeconds (1));
  Simulator::Run ();
  Require (g_ackWakeTimes.size () == 1 &&
             g_ackWakeTimes[0] == tic,
           "authenticated ACK generic wake did not run one TIC later");

  Time duplicateTime = Simulator::Now ();
  localHop->ReceiveFromMac (validAck->Copy (), 60.0, 40.0);
  Require (g_ackWakeTimes.size () == 1,
           "authenticated duplicate ACK woke NWK synchronously");
  Simulator::Stop (duplicateTime + tic + NanoSeconds (1));
  Simulator::Run ();
  Require (g_ackWakeTimes.size () == 2 &&
             g_ackWakeTimes[1] == duplicateTime + tic,
           "authenticated duplicate ACK lost its packet-level generic wake");

  Simulator::Destroy ();
}

void
CheckBareExactAckCompatibilityReception ()
{
  Ptr<CsrNetDevice> localDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> localHop = CreateObject<CsrHopLayer> ();
  localHop->SetMac (&localDevice->GetMac ());
  ConfigureHop (localHop, DESTINATION, SOURCE);
  localHop->SetHopWireProfile (
    CsrHopWireProfile::HIST_ADB97C54_BARE);
  Require (
    localHop->GetHopWireProfile () ==
      CsrHopWireProfile::HIST_ADB97C54_BARE,
    "historical bare HOP wire profile did not round-trip configuration");

  g_ackWakeTimes.clear ();
  localHop->SetNwkQueueWakeCallback (MakeCallback (&NoteAckWake));
  localHop->SendData (SOURCE, 5, Create<Packet> (4), true);
  Require (localHop->GetPendingDataCount () == 1 &&
             localHop->GetResendQueueSize () == 1,
           "bare exact-ACK compatibility test did not create one pending DATA record");

  CsrHeader ackHeader (SOURCE,
                       DESTINATION,
                       1,
                       7,
                       false,
                       true);
  ackHeader.SetType (CSR_PKT_ACK);
  ackHeader.SetDestType (CSR_DEST_UNICAST);
  ackHeader.SetLinkControl (8, 0.0, 0.0);
  Ptr<Packet> bareAck = Create<Packet> ();
  bareAck->AddHeader (ackHeader);
  // The br_Ack base fields plus br_Mac total 25 bytes. The archived adb97
  // executable's five send_ack() callers all pass null PacketRxInfo and thus
  // set both cumulative registers; this synthetic exact frame verifies profile
  // receive compatibility, not an archived 25-byte transmit path.
  Require (CsrAnnotateOpnetEnvelope (bareAck) &&
             CsrGetOpnetWireSize (bareAck) == 25,
           "bare exact-ACK compatibility envelope is not 25 bytes");

  localHop->ReceiveFromMac (bareAck, 60.0, 40.0);
  Require (localHop->GetPendingDataCount () == 0 &&
             localHop->GetResendQueueSize () == 0 &&
             localDevice->GetMac ().GetDataQueuedFrameCount () == 0 &&
             g_ackWakeTimes.empty (),
           "bare exact-ACK compatibility frame did not release custody before delayed wake");

  const Time tic = CsrOpnetTic ();
  Simulator::Stop (tic + NanoSeconds (1));
  Simulator::Run ();
  Require (g_ackWakeTimes.size () == 1 && g_ackWakeTimes[0] == tic,
           "bare exact-ACK compatibility frame lost the one-TIC generic wake ordering");
  Simulator::Destroy ();
}

bool
AlwaysDack (CsrNodeId, CsrNodeId)
{
  return true;
}

bool
DackRelayRouteAvailable (CsrNodeId destination)
{
  return destination == RELAY_DESTINATION;
}

void
NoteDackRelayDelivery (Ptr<Packet> payload, CsrNodeId source)
{
  CsrNetHeader networkHeader;
  Require (source == SOURCE && payload->PeekHeader (networkHeader) &&
             networkHeader.GetSrc () == SOURCE &&
             networkHeader.GetDst () == RELAY_DESTINATION,
           "DACK wrapper fixture did not exercise a true relay flow");
  g_liveDackRelayDeliveries++;
}

void
CaptureDack (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  if (g_capturedDack == nullptr &&
      frame->PeekHeader (header) &&
      header.IsDack ())
    {
      g_capturedDack = frame->Copy ();
    }
}

void
CheckLiveOutboundDackWrapper (CsrHopWireProfile profile,
                              uint32_t expectedWireBytes)
{
  Ptr<CsrNetDevice> dackDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrNetDevice> captureDevice =
    CreateObject<CsrNetDevice> (SOURCE);
  dackDevice->AddPeer (captureDevice);

  CsrPerModelFn noErrors = [] (int, double, uint32_t) { return 0.0; };
  dackDevice->GetPhy ().SetPerModel (noErrors);
  captureDevice->GetPhy ().SetPerModel (
    [] (int, double, uint32_t packetBits) {
      g_capturedDackPacketBits = packetBits;
      return 0.0;
    });
  dackDevice->GetPhy ().SetLinkDistanceMeters (
    DESTINATION,
    SOURCE,
    1.0);
  captureDevice->GetPhy ().SetLinkDistanceMeters (
    DESTINATION,
    SOURCE,
    1.0);
  captureDevice->GetMac ().SetRxCallback (MakeCallback (&CaptureDack));

  Ptr<CsrHopLayer> dackHop = CreateObject<CsrHopLayer> ();
  dackHop->SetMac (&dackDevice->GetMac ());
  ConfigureHop (dackHop, DESTINATION, SOURCE);
  dackHop->SetHopWireProfile (profile);
  dackHop->SetShouldDackCallback (MakeCallback (&AlwaysDack));
  dackHop->SetRelayRouteAvailableCallback (
    MakeCallback (&DackRelayRouteAvailable));
  dackHop->SetRxFromHopCallback (MakeCallback (&NoteDackRelayDelivery));

  CsrHopSecurityState dataSender;
  ConfigureState (dataSender, SOURCE, DESTINATION);
  Ptr<Packet> networkPayload = Create<Packet> (3);
  CsrNetHeader networkHeader (SOURCE, RELAY_DESTINATION, 5);
  networkPayload->AddHeader (networkHeader);
  Ptr<Packet> dataFrame;
  const bool historicalBare =
    profile == CsrHopWireProfile::HIST_ADB97C54_BARE;
  if (historicalBare)
    {
      dataFrame = networkPayload->Copy ();
    }
  else
    {
      std::vector<uint8_t> plaintext = CopyBytes (networkPayload);
      CsrProtectedPairwiseMessage securedData =
        dataSender.ProtectPairwiseMessage (
          DESTINATION,
          CsrPairwiseSecurityMode::Pairwise16,
          3,
          plaintext);
      dataFrame = Create<Packet> (securedData.record.data (),
                                  securedData.record.size ());
    }
  CsrHeader dataHeader (SOURCE,
                        DESTINATION,
                        77,
                        5,
                        true,
                        false);
  dataHeader.SetType (CSR_PKT_DATA);
  dataHeader.SetDestType (CSR_DEST_UNICAST);
  if (!historicalBare)
    {
      dataHeader.SetSecurityCount (SECURITY_COUNT);
    }
  dataHeader.SetLinkControl (8, 0.0, 0.0);
  dataFrame->AddHeader (dataHeader);

  g_capturedDack = nullptr;
  g_capturedDackPacketBits = 0;
  g_liveDackRelayDeliveries = 0;
  dackHop->ReceiveFromMac (dataFrame, 60.0, 40.0);
  Require (g_liveDackRelayDeliveries == 1 &&
             dackDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "true relay DATA did not enqueue one Pairwise16 DACK-class frame");

  Simulator::Stop (Seconds (4.0));
  Simulator::Run ();
  Require (g_capturedDack != nullptr,
           "live DACK frame was not transmitted to the capture peer");
  Require (CsrGetOpnetWireSize (g_capturedDack) == expectedWireBytes,
           "live cumulative DACK used the wrong source-version envelope");
  uint32_t shortPacketBits = 104 + 48 + expectedWireBytes * 8 + 32;
  uint32_t longPacketBits = 7888 + 48 + expectedWireBytes * 8 + 32;
  Require (g_capturedDackPacketBits == shortPacketBits ||
             g_capturedDackPacketBits == longPacketBits,
           "PHY did not charge the selected DACK envelope to packet bits");

  Ptr<Packet> securedRecord = g_capturedDack->Copy ();
  CsrHeader outer;
  securedRecord->RemoveHeader (outer);
  Require (outer.IsAck () && outer.IsDack () &&
             outer.GetType () == CSR_PKT_ACK,
           "MAC-visible DACK metadata did not retain the common AckMsg type");

  if (profile == CsrHopWireProfile::HIST_ADB97C54_BARE)
    {
      Require (!outer.HasSecurityCount () &&
                 securedRecord->GetSize () == 0 &&
                 outer.HasAckWindow () &&
                 outer.GetAckBitmap () == 0 &&
                 outer.GetDackBitmap () == 1,
               "historical executable DACK was not a bare logical br_Ack");
      Simulator::Destroy ();
      return;
    }

  Require (outer.HasSecurityCount (),
           "production DACK lost its Pairwise16 security count");

  std::vector<uint8_t> record = CopyBytes (securedRecord);
  CsrHopSecurityState verifier;
  ConfigureState (verifier, SOURCE, DESTINATION);
  CsrReceivedPairwiseMessage received =
    verifier.ReceivePairwiseMessage (
      DESTINATION,
      outer.GetSecurityCount (),
      CsrPairwiseSecurityMode::Pairwise16,
      0,
      record);
  Require (received.status == CsrHopSecurityReceiveStatus::Accepted,
           "live DACK did not authenticate as Pairwise16 packet type 0");

  Ptr<Packet> body = Create<Packet> (received.payload.data (),
                                     received.payload.size ());
  CsrHeader authenticated;
  body->RemoveHeader (authenticated);
  Require (authenticated.IsAck () && authenticated.IsDack () &&
             authenticated.GetType () == CSR_PKT_DACK &&
             authenticated.GetSrc () == DESTINATION &&
             authenticated.GetDst () == SOURCE &&
             authenticated.GetSeq () == 77 &&
             authenticated.HasAckWindow () &&
             authenticated.GetAckBitmap () == 0 &&
             authenticated.GetDackBitmap () == 1 &&
             !authenticated.HasSecurityCount () &&
             !authenticated.HasLinkControl (),
           "Pairwise16 did not authenticate the complete logical DACK body");

  Simulator::Destroy ();
}

void
NoteProfileDataDelivery (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE,
           "profile DATA callback reported the wrong source");
  g_profileDataPayload = CopyBytes (payload);
  g_profileDataDeliveries++;
}

void
CaptureAndDeliverProfileData (Ptr<Packet> frame,
                              double pathlossDb,
                              double snrDb)
{
  CsrHeader header;
  if (g_capturedProfileData == nullptr &&
      frame->PeekHeader (header) &&
      header.GetType () == CSR_PKT_DATA)
    {
      g_capturedProfileData = frame->Copy ();
    }
  Require (g_profileDataReceiver != nullptr,
           "profile DATA receiver was not configured");
  g_profileDataReceiver->ReceiveFromMac (frame, pathlossDb, snrDb);
}

void
CheckLiveOrdinaryDataWireProfile (CsrHopWireProfile profile,
                                  uint32_t expectedWireBytes,
                                  bool expectSecurity)
{
  Ptr<CsrNetDevice> senderDevice = CreateObject<CsrNetDevice> (SOURCE);
  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  senderDevice->AddPeer (receiverDevice);
  receiverDevice->AddPeer (senderDevice);

  CsrPerModelFn noErrors = [] (int, double, uint32_t) { return 0.0; };
  senderDevice->GetPhy ().SetPerModel (noErrors);
  receiverDevice->GetPhy ().SetPerModel (
    [] (int, double, uint32_t packetBits) {
      g_capturedProfileDataPacketBits = packetBits;
      return 0.0;
    });
  senderDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE,
    DESTINATION,
    1.0);
  receiverDevice->GetPhy ().SetLinkDistanceMeters (
    SOURCE,
    DESTINATION,
    1.0);

  Ptr<CsrHopLayer> sender = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  sender->SetMac (&senderDevice->GetMac ());
  receiver->SetMac (&receiverDevice->GetMac ());
  ConfigureHop (sender, SOURCE, DESTINATION);
  ConfigureHop (receiver, DESTINATION, SOURCE);
  sender->SetHopWireProfile (profile);
  receiver->SetHopWireProfile (profile);
  receiver->SetRxFromHopCallback (MakeCallback (&NoteProfileDataDelivery));

  g_profileDataReceiver = receiver;
  g_capturedProfileData = nullptr;
  g_capturedProfileDataPacketBits = 0;
  g_profileDataDeliveries = 0;
  g_profileDataPayload.clear ();
  receiverDevice->GetMac ().SetRxCallback (
    MakeCallback (&CaptureAndDeliverProfileData));

  // br_app Packet Size 200 produces one 192-byte br_Network packet.  The
  // archived direct br_Network -> br_Hop -> br_Mac path is therefore 217 B;
  // production Pairwise16 adds its five-byte record and is 222 B.
  Ptr<Packet> networkPayload = Create<Packet> (185);
  CsrNetHeader networkHeader (SOURCE, DESTINATION, 5);
  networkPayload->AddHeader (networkHeader);
  Require (networkPayload->GetSize () == 192,
           "profile DATA fixture is not a 192-byte network packet");
  const std::vector<uint8_t> expectedPayload = CopyBytes (networkPayload);

  sender->SendData (DESTINATION, 5, networkPayload, false);
  Simulator::Stop (Seconds (4.0));
  Simulator::Run ();

  Require (g_capturedProfileData != nullptr,
           "live ordinary DATA was not transmitted to the receiver");
  Require (CsrGetOpnetWireSize (g_capturedProfileData) == expectedWireBytes,
           "live ordinary DATA used the wrong source-version wire size");
  const uint32_t shortPacketBits =
    104 + 48 + expectedWireBytes * 8 + 32;
  const uint32_t longPacketBits =
    7888 + 48 + expectedWireBytes * 8 + 32;
  Require (g_capturedProfileDataPacketBits == shortPacketBits ||
             g_capturedProfileDataPacketBits == longPacketBits,
           "PHY did not charge the selected DATA envelope to packet bits");

  Ptr<Packet> record = g_capturedProfileData->Copy ();
  CsrHeader outer;
  record->RemoveHeader (outer);
  Require (outer.GetType () == CSR_PKT_DATA &&
             outer.GetSrc () == SOURCE &&
             outer.GetDst () == DESTINATION &&
             outer.GetDscp () == 5 &&
             !outer.IsAckable (),
           "ordinary DATA lost its HOP metadata");
  Require (outer.HasSecurityCount () == expectSecurity,
           "ordinary DATA security-count presence disagrees with profile");
  Require (record->GetSize () == 192 + (expectSecurity ? 5 : 0),
           "ordinary DATA record contains the wrong authentication bytes");
  Require (g_profileDataDeliveries == 1 &&
             g_profileDataPayload == expectedPayload,
           "matching ordinary DATA profile did not deliver plaintext once");

  g_profileDataReceiver = nullptr;
  Simulator::Destroy ();
}

void
NoteLivePairwise (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE,
           "live pairwise callback reported the wrong source");
  g_livePayloads.push_back (CopyBytes (payload));
  g_livePairwiseDeliveries++;
}

void
NoteLiveNeighborcast (Ptr<Packet> payload, CsrNodeId source)
{
  Require (source == SOURCE,
           "live neighborcast callback reported the wrong source");
  g_livePayloads.push_back (CopyBytes (payload));
  g_liveNeighborcastDeliveries++;
}

void
SendLiveNeighborcast (Ptr<CsrHopLayer> sender)
{
  const std::array<uint8_t, 3> bytes {0x51, 0x52, 0x53};
  sender->SendNeighborcast (5, Create<Packet> (bytes.data (), bytes.size ()));
}

void
SendLivePairwise16 (Ptr<CsrHopLayer> sender)
{
  const std::array<uint8_t, 4> bytes {0x61, 0x62, 0x63, 0x64};
  sender->SendData (DESTINATION,
                    5,
                    Create<Packet> (bytes.data (), bytes.size ()),
                    true);
}

void
SendLivePairwise32Encrypt (Ptr<CsrHopLayer> sender)
{
  const std::array<uint8_t, 5> bytes {0x71, 0x72, 0x73, 0x74, 0x75};
  sender->SendPairwise32EncryptedData (
    DESTINATION,
    5,
    Create<Packet> (bytes.data (), bytes.size ()),
    true);
}

void
CheckLiveSendPaths ()
{
  Ptr<CsrNetDevice> senderDevice = CreateObject<CsrNetDevice> (SOURCE);
  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  senderDevice->AddPeer (receiverDevice);
  receiverDevice->AddPeer (senderDevice);

  CsrPerModelFn noErrors = [] (int, double, uint32_t) { return 0.0; };
  senderDevice->GetPhy ().SetPerModel (noErrors);
  receiverDevice->GetPhy ().SetPerModel (noErrors);
  senderDevice->GetPhy ().SetLinkDistanceMeters (SOURCE, DESTINATION, 1.0);
  receiverDevice->GetPhy ().SetLinkDistanceMeters (SOURCE, DESTINATION, 1.0);

  Ptr<CsrHopLayer> sender = CreateObject<CsrHopLayer> ();
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  sender->SetMac (&senderDevice->GetMac ());
  receiver->SetMac (&receiverDevice->GetMac ());
  ConfigureHop (sender, SOURCE, DESTINATION);
  ConfigureHop (receiver, DESTINATION, SOURCE);
  sender->SetGroupKeyMaterial (g_groupKey);

  senderDevice->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, sender));
  receiverDevice->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, receiver));
  receiver->SetRxFromHopCallback (MakeCallback (&NoteLivePairwise));
  receiver->SetRxNeighborcastFromHopCallback (
    MakeCallback (&NoteLiveNeighborcast));

  g_livePairwiseDeliveries = 0;
  g_liveNeighborcastDeliveries = 0;
  g_livePayloads.clear ();

  Require (sender->SendKeyUpdate (DESTINATION),
           "live KeyUpdate transaction did not start");
  Simulator::Schedule (Seconds (0.5), &SendLiveNeighborcast, sender);
  Simulator::Schedule (Seconds (1.0), &SendLivePairwise16, sender);
  Simulator::Schedule (Seconds (1.5),
                       &SendLivePairwise32Encrypt,
                       sender);
  Simulator::Stop (Seconds (10.0));
  Simulator::Run ();

  Require (g_liveNeighborcastDeliveries == 1 &&
             g_livePairwiseDeliveries == 2,
           "live protected send paths did not deliver exactly once");
  const std::vector<std::vector<uint8_t>> expectedPayloads {
    {0x51, 0x52, 0x53},
    {0x61, 0x62, 0x63, 0x64},
    {0x71, 0x72, 0x73, 0x74, 0x75}
  };
  Require (g_livePayloads.size () == expectedPayloads.size () &&
             std::all_of (
               expectedPayloads.begin (),
               expectedPayloads.end (),
               [] (const std::vector<uint8_t> &expected) {
                 return std::find (g_livePayloads.begin (),
                                   g_livePayloads.end (),
                                   expected) != g_livePayloads.end ();
               }),
           "live protected send paths changed payload bytes");
  Require (sender->GetPendingDataCount () == 0 &&
             sender->GetResendQueueSize () == 0,
           "authenticated ACKs did not complete live pairwise transactions");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  InitializeKeys ();
  CheckGoldenPairwiseRecords ();
  CheckAuthenticationBeforeAckAndDelivery ();
  CheckCrossProfileDataRejection ();
  CheckMalformedOrdinaryDataEnvelopes (
    CsrHopWireProfile::PRODUCTION_PAIRWISE16);
  CheckMalformedOrdinaryDataEnvelopes (
    CsrHopWireProfile::HIST_ADB97C54_BARE);
  CheckAckDackSecurityPolicy ();
  CheckBareExactAckCompatibilityReception ();
  CheckLiveOutboundDackWrapper (
    CsrHopWireProfile::PRODUCTION_PAIRWISE16,
    46);
  CheckLiveOutboundDackWrapper (
    CsrHopWireProfile::HIST_ADB97C54_BARE,
    41);
  CheckLiveOrdinaryDataWireProfile (
    CsrHopWireProfile::PRODUCTION_PAIRWISE16,
    222,
    true);
  CheckLiveOrdinaryDataWireProfile (
    CsrHopWireProfile::HIST_ADB97C54_BARE,
    217,
    false);
  CheckLiveSendPaths ();
  std::cout << "PASS: remaining OPNET HOP-security mode parity test"
            << std::endl;
  return 0;
}
