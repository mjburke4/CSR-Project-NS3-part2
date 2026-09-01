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

bool
AlwaysDack (CsrNodeId, CsrNodeId)
{
  return true;
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
CheckLiveOutboundDackWrapper ()
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
  dackHop->SetShouldDackCallback (MakeCallback (&AlwaysDack));

  CsrHopSecurityState dataSender;
  ConfigureState (dataSender, SOURCE, DESTINATION);
  Ptr<Packet> networkPayload = Create<Packet> (3);
  CsrNetHeader networkHeader (SOURCE, DESTINATION, 5);
  networkPayload->AddHeader (networkHeader);
  std::vector<uint8_t> plaintext = CopyBytes (networkPayload);
  CsrProtectedPairwiseMessage securedData =
    dataSender.ProtectPairwiseMessage (
      DESTINATION,
      CsrPairwiseSecurityMode::Pairwise16,
      3,
      plaintext);

  Ptr<Packet> dataFrame = Create<Packet> (securedData.record.data (),
                                          securedData.record.size ());
  CsrHeader dataHeader (SOURCE,
                        DESTINATION,
                        77,
                        5,
                        true,
                        false);
  dataHeader.SetType (CSR_PKT_DATA);
  dataHeader.SetDestType (CSR_DEST_UNICAST);
  dataHeader.SetSecurityCount (SECURITY_COUNT);
  dataHeader.SetLinkControl (8, 0.0, 0.0);
  dataFrame->AddHeader (dataHeader);

  g_capturedDack = nullptr;
  g_capturedDackPacketBits = 0;
  dackHop->ReceiveFromMac (dataFrame, 60.0, 40.0);
  Require (dackDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "live DACK producer did not enqueue one ACK-class frame");

  Simulator::Stop (Seconds (4.0));
  Simulator::Run ();
  Require (g_capturedDack != nullptr,
           "live DACK frame was not transmitted to the capture peer");
  Require (CsrGetOpnetWireSize (g_capturedDack) == 46,
           "Pairwise16 cumulative DACK did not model 41 + 5 bytes");
  uint32_t shortPacketBits = 104 + 48 + 46 * 8 + 32;
  uint32_t longPacketBits = 7888 + 48 + 46 * 8 + 32;
  Require (g_capturedDackPacketBits == shortPacketBits ||
             g_capturedDackPacketBits == longPacketBits,
           "PHY did not charge the 46-byte DACK to packet bits");

  Ptr<Packet> securedRecord = g_capturedDack->Copy ();
  CsrHeader outer;
  securedRecord->RemoveHeader (outer);
  Require (outer.IsAck () && outer.IsDack () &&
             outer.GetType () == CSR_PKT_ACK &&
             outer.HasSecurityCount (),
           "MAC-visible DACK metadata did not retain the common AckMsg type");

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
  CheckAckDackSecurityPolicy ();
  CheckLiveOutboundDackWrapper ();
  CheckLiveSendPaths ();
  std::cout << "PASS: remaining OPNET HOP-security mode parity test"
            << std::endl;
  return 0;
}
