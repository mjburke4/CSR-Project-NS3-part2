#include "ns3/csr-hop-security.h"
#include "ns3/csr-legacy-crypto.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/packet.h"
#include "ns3/simulator.h"

#include <array>
#include <cmath>
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

uint32_t g_groupKeyNeededCallbacks = 0;
uint32_t g_helloDeliveries = 0;
std::vector<uint8_t> g_lastPayload;
std::vector<std::string> g_callbackOrder;

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

CsrKeyUpdateHeader
InstallCurrentGroupKey (CsrHopSecurityState &sender,
                        CsrHopSecurityState &receiver)
{
  sender.SetNextDataPrn ({0x10, 0x20, 0x30, 0x40, 0x50});
  CsrKeyUpdateHeader update = sender.BuildKeyUpdate (DESTINATION);
  CsrHopSecurityReceiveStatus status =
    receiver.ReceiveKeyUpdate (SOURCE, SECURITY_COUNT, update);
  Require (status == CsrHopSecurityReceiveStatus::Accepted,
           "KeyUpdate did not install the sender's current group key");
  return update;
}

void
RequireAcceptedPayload (const CsrReceivedGroupMessage &received,
                        const std::vector<uint8_t> &expected,
                        const char *message)
{
  Require (received.status == CsrHopSecurityReceiveStatus::Accepted &&
             received.payload == expected,
           message);
}

void
CheckGoldenGroupRecords ()
{
  const std::vector<uint8_t> payload {
    0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
    0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0
  };

  CsrHopSecurityState establishSender;
  CsrHopSecurityState establishReceiver;
  ConfigureState (establishSender, SOURCE, DESTINATION);
  ConfigureState (establishReceiver, DESTINATION, SOURCE);
  establishSender.SetGroupKeyMaterial (g_groupKey);

  CsrProtectedGroupMessage establish =
    establishSender.ProtectGroupMessage (
      CsrGroupSecurityMode::GroupEstablish, 4, payload);
  const std::vector<uint8_t> expectedEstablish {
    0x00, 0x10, 0x02, 0xbd, 0x1d, 0x4d, 0x47, 0x3d,
    0x36, 0x0d, 0x65, 0x0d, 0x24, 0x0c, 0x9e, 0x6e,
    0x7c, 0x83, 0xb4, 0xa1
  };
  Require (establish.groupKeyId == 1 && establish.groupSequence == 2 &&
             establish.record == expectedEstablish,
           "GroupEstablish record changed from its independent golden vector");

  CsrReceivedGroupMessage deferred =
    establishReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::GroupEstablish,
      4,
      establish.record);
  Require (deferred.status ==
             CsrHopSecurityReceiveStatus::AuthenticatedGroupKeyNeeded &&
             deferred.payload.empty (),
           "mission-authenticated GroupEstablish was not deferred without a key");

  InstallCurrentGroupKey (establishSender, establishReceiver);
  RequireAcceptedPayload (
    establishReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::GroupEstablish,
      4,
      establish.record),
    payload,
    "deferred GroupEstablish did not decrypt after KeyUpdate");
  Require (establishReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::GroupEstablish,
             4,
             establish.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticatedDuplicate,
           "GroupEstablish replay was not rejected after authentication");

  CsrHopSecurityState lowerTagReceiver;
  ConfigureState (lowerTagReceiver, DESTINATION, SOURCE);
  std::vector<uint8_t> badMissionTag = establish.record;
  badMissionTag[badMissionTag.size () - 4] ^= 0x80;
  Require (lowerTagReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::GroupEstablish,
             4,
             badMissionTag).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed &&
           lowerTagReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::GroupEstablish,
             4,
             establish.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticatedGroupKeyNeeded,
           "bad GroupEstablish mission tag consumed receive state");

  CsrHopSecurityState upperTagReceiver;
  ConfigureState (upperTagReceiver, DESTINATION, SOURCE);
  InstallCurrentGroupKey (establishSender, upperTagReceiver);
  std::vector<uint8_t> badGroupTag = establish.record;
  badGroupTag.back () ^= 0x01;
  Require (upperTagReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::GroupEstablish,
             4,
             badGroupTag).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "bad GroupEstablish group-key tag authenticated");
  RequireAcceptedPayload (
    upperTagReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::GroupEstablish,
      4,
      establish.record),
    payload,
    "bad GroupEstablish tag poisoned the following valid record");

  CsrHopSecurityState group16Sender;
  CsrHopSecurityState group16Receiver;
  ConfigureState (group16Sender, SOURCE, DESTINATION);
  ConfigureState (group16Receiver, DESTINATION, SOURCE);
  group16Sender.SetGroupKeyMaterial (g_groupKey);
  InstallCurrentGroupKey (group16Sender, group16Receiver);
  CsrProtectedGroupMessage group16 =
    group16Sender.ProtectGroupMessage (
      CsrGroupSecurityMode::Group16, 8, payload);
  const std::vector<uint8_t> expectedGroup16 {
    0x00, 0x10, 0x02, 0x10, 0x20, 0x30, 0x40, 0x50,
    0x60, 0x70, 0x80, 0x90, 0xa0, 0xb0, 0xc0, 0xd0,
    0x6f, 0xa6
  };
  Require (group16.record == expectedGroup16,
           "Group16 record changed from its independent golden vector");
  RequireAcceptedPayload (
    group16Receiver.ReceiveGroupMessage (SOURCE,
                                         SECURITY_COUNT,
                                         CsrGroupSecurityMode::Group16,
                                         8,
                                         group16.record),
    payload,
    "valid Group16 record failed authentication");

  CsrHopSecurityState group16TamperReceiver;
  ConfigureState (group16TamperReceiver, DESTINATION, SOURCE);
  InstallCurrentGroupKey (group16Sender, group16TamperReceiver);
  std::vector<uint8_t> badGroup16 = group16.record;
  badGroup16.back () ^= 0x40;
  Require (group16TamperReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::Group16,
             8,
             badGroup16).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "tampered Group16 record authenticated");
  RequireAcceptedPayload (
    group16TamperReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group16,
      8,
      group16.record),
    payload,
    "tampered Group16 record consumed replay state");

  CsrHopSecurityState group32Sender;
  CsrHopSecurityState group32Receiver;
  ConfigureState (group32Sender, SOURCE, DESTINATION);
  ConfigureState (group32Receiver, DESTINATION, SOURCE);
  group32Sender.SetGroupKeyMaterial (g_groupKey);
  InstallCurrentGroupKey (group32Sender, group32Receiver);
  CsrProtectedGroupMessage group32 =
    group32Sender.ProtectGroupMessage (
      CsrGroupSecurityMode::Group32Encrypt, 6, payload);
  const std::vector<uint8_t> expectedGroup32 {
    0x00, 0x10, 0x02, 0xbd, 0x1d, 0x4d, 0x47, 0x3d,
    0x36, 0x0d, 0x65, 0x0d, 0x24, 0x0c, 0x9e, 0x6e,
    0x2a, 0x77, 0x56, 0xdc
  };
  Require (group32.record == expectedGroup32,
           "Group32Encrypt record changed from its independent golden vector");
  RequireAcceptedPayload (
    group32Receiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group32Encrypt,
      6,
      group32.record),
    payload,
    "Group32Encrypt did not authenticate and decrypt");

  CsrLegacyCrypto::GroupKeyMaterial wrongGroupKey = g_groupKey;
  wrongGroupKey[0] ^= 0xff;
  CsrHopSecurityState wrongKeySender;
  CsrHopSecurityState wrongKeyReceiver;
  ConfigureState (wrongKeySender, SOURCE, DESTINATION);
  ConfigureState (wrongKeyReceiver, DESTINATION, SOURCE);
  wrongKeySender.SetGroupKeyMaterial (wrongGroupKey);
  InstallCurrentGroupKey (wrongKeySender, wrongKeyReceiver);
  Require (wrongKeyReceiver.ReceiveGroupMessage (
             SOURCE,
             SECURITY_COUNT,
             CsrGroupSecurityMode::Group32Encrypt,
             6,
             group32.record).status ==
             CsrHopSecurityReceiveStatus::AuthenticationFailed,
           "Group32Encrypt authenticated with the wrong group key");
}

void
CheckGroupKeyEvolution ()
{
  const std::vector<uint8_t> payload {0x31, 0x41, 0x59, 0x26, 0x53};
  const CsrLegacyCrypto::GroupKeyMaterial expectedDerived {
    0x38, 0x6f, 0x20, 0x6a, 0x81, 0x7c, 0x7f, 0x5f,
    0xc2, 0x35, 0x01, 0xa8, 0xda, 0xbf, 0xa8, 0xae,
    0x93, 0x6f, 0xb8, 0x2d, 0x13, 0xc9, 0x1e, 0x25,
    0x56, 0x86, 0x3c, 0x08, 0x96, 0x83, 0xc6, 0xe5
  };

  CsrHopSecurityState sender;
  CsrHopSecurityState receiver;
  ConfigureState (sender, SOURCE, DESTINATION);
  ConfigureState (receiver, DESTINATION, SOURCE);
  sender.SetGroupKeyMaterial (g_groupKey);
  InstallCurrentGroupKey (sender, receiver);
  sender.SetGroupSequence (0x0fff);

  CsrProtectedGroupMessage evolved = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group16, 8, payload);
  Require (evolved.groupKeyId == 2 && evolved.groupSequence == 0 &&
             !evolved.generatedNewGroupKey &&
             sender.GetGroupKeyMaterial () == expectedDerived,
           "group sequence rollover did not apply the one-way key evolution");
  RequireAcceptedPayload (
    receiver.ReceiveGroupMessage (SOURCE,
                                  SECURITY_COUNT,
                                  CsrGroupSecurityMode::Group16,
                                  8,
                                  evolved.record),
    payload,
    "receiver did not derive a future group key in the same epoch");
  CsrLegacyCrypto::GroupKeyMaterial receiverDerived {};
  Require (receiver.GetReceivedGroupKeyMaterial (SOURCE, receiverDerived) &&
             receiverDerived == expectedDerived,
           "receiver did not commit the authenticated derived group key");

  CsrHopSecurityState previousSender;
  CsrHopSecurityState previousReceiver;
  ConfigureState (previousSender, SOURCE, DESTINATION);
  ConfigureState (previousReceiver, DESTINATION, SOURCE);
  previousSender.SetGroupKeyMaterial (g_groupKey);
  InstallCurrentGroupKey (previousSender, previousReceiver);
  CsrProtectedGroupMessage oldFirst = previousSender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group16, 8, payload);
  CsrProtectedGroupMessage oldSecond = previousSender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group16, 8, payload);
  RequireAcceptedPayload (
    previousReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group16,
      8,
      oldFirst.record),
    payload,
    "initial current group key was not accepted");

  previousSender.SetGroupKeyId (2);
  previousSender.SetGroupKeyMaterial (expectedDerived);
  InstallCurrentGroupKey (previousSender, previousReceiver);
  RequireAcceptedPayload (
    previousReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group16,
      8,
      oldSecond.record),
    payload,
    "one previous group key was not retained across KeyUpdate");
  CsrProtectedGroupMessage current = previousSender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group32Encrypt, 6, payload);
  RequireAcceptedPayload (
    previousReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group32Encrypt,
      6,
      current.record),
    payload,
    "current group key failed after accepting the previous key");

  CsrLegacyCrypto::GroupKeyMaterial freshEpochKey {};
  for (std::size_t i = 0; i < freshEpochKey.size (); ++i)
    {
      freshEpochKey[i] = static_cast<uint8_t> (0xe0 + i);
    }

  CsrHopSecurityState epochSender;
  CsrHopSecurityState epochReceiver;
  ConfigureState (epochSender, SOURCE, DESTINATION);
  ConfigureState (epochReceiver, DESTINATION, SOURCE);
  epochSender.SetGroupKeyMaterial (g_groupKey);
  epochSender.MarkKeyUpdateAcked (DESTINATION, Seconds (1.0));
  epochSender.SetGroupKeyId (15);
  epochSender.SetGroupSequence (0x0fff);
  epochSender.SetNextGeneratedGroupKeyMaterial (freshEpochKey);

  CsrProtectedGroupMessage newEpoch = epochSender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group32Encrypt, 6, payload);
  Require (newEpoch.groupKeyId == 16 && newEpoch.groupSequence == 0 &&
             newEpoch.generatedNewGroupKey &&
             epochSender.GetGroupKeyMaterial () == freshEpochKey &&
             !epochSender.HasGroupKeySentTo (DESTINATION),
           "fresh 16-key epoch did not invalidate group-key distribution");
  InstallCurrentGroupKey (epochSender, epochReceiver);
  RequireAcceptedPayload (
    epochReceiver.ReceiveGroupMessage (
      SOURCE,
      SECURITY_COUNT,
      CsrGroupSecurityMode::Group32Encrypt,
      6,
      newEpoch.record),
    payload,
    "fresh epoch Group32Encrypt did not round-trip after KeyUpdate");
}

Ptr<Packet>
BuildGroupFrame (const std::vector<uint8_t> &record,
                 uint8_t type,
                 uint16_t sequence,
                 bool ackable)
{
  Ptr<Packet> frame = Create<Packet> (record.data (), record.size ());
  CsrHeader header (SOURCE,
                    type == CSR_PKT_DISCOVER
                      ? CSR_BROADCAST_ID
                      : DESTINATION,
                    sequence,
                    7,
                    ackable,
                    false);
  header.SetType (type);
  header.SetDestType (type == CSR_PKT_DISCOVER
                        ? CSR_DEST_BROADCAST
                        : CSR_DEST_MULTICAST);
  header.SetSecurityCount (SECURITY_COUNT);
  header.SetHasGroupSecurity (true);
  frame->AddHeader (header);
  return frame;
}

void
NoteGroupKeyNeeded (CsrNodeId source, double pathlossDb, double snrDb)
{
  Require (source == SOURCE,
           "group-key-needed callback reported the wrong source");
  Require (std::abs (pathlossDb - 60.0) < 0.001 &&
             std::abs (snrDb - 40.0) < 0.001,
           "group-key-needed callback lost link observations");
  g_groupKeyNeededCallbacks++;
}

void
NoteHelloDelivery (Ptr<Packet> payload,
                   CsrNodeId source,
                   double,
                   double)
{
  Require (source == SOURCE,
           "deferred group delivery reported the wrong source");
  g_lastPayload.resize (payload->GetSize ());
  if (!g_lastPayload.empty ())
    {
      payload->CopyData (g_lastPayload.data (), g_lastPayload.size ());
    }
  g_helloDeliveries++;
  g_callbackOrder.push_back ("payload");
}

void
NoteKeyUpdateDelivery (CsrNodeId source)
{
  Require (source == SOURCE,
           "KeyUpdate callback reported the wrong source");
  g_callbackOrder.push_back ("key-update");
}

void
CheckLiveDeferredReplayAndRoutingAck ()
{
  CsrHopSecurityState sender;
  ConfigureState (sender, SOURCE, DESTINATION);
  sender.SetGroupKeyMaterial (g_groupKey);

  Ptr<CsrNetDevice> receiverDevice =
    CreateObject<CsrNetDevice> (DESTINATION);
  Ptr<CsrHopLayer> receiver = CreateObject<CsrHopLayer> ();
  receiver->SetNodeId (DESTINATION);
  receiver->SetMac (&receiverDevice->GetMac ());
  receiver->SetMissionKey (g_missionKey);
  receiver->SetPairwiseKeyMaterial (SOURCE, g_pairwiseMaterial);
  receiver->SetAuthenticatedGroupKeyNeededCallback (
    MakeCallback (&NoteGroupKeyNeeded));
  receiver->SetRxHelloFromHopCallback (
    MakeCallback (&NoteHelloDelivery));
  receiver->SetKeyUpdateReceivedCallback (
    MakeCallback (&NoteKeyUpdateDelivery));

  g_groupKeyNeededCallbacks = 0;
  g_helloDeliveries = 0;
  g_lastPayload.clear ();
  g_callbackOrder.clear ();

  const std::vector<uint8_t> firstPayload {0x01, 0x02, 0x03};
  const std::vector<uint8_t> newestPayload {0x04, 0x05, 0x06, 0x07};
  CsrProtectedGroupMessage first = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::GroupEstablish, 4, firstPayload);
  CsrProtectedGroupMessage newest = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::GroupEstablish, 4, newestPayload);

  receiver->ReceiveFromMac (
    BuildGroupFrame (first.record, CSR_PKT_DISCOVER, 10, false),
    60.0,
    40.0);
  receiver->ReceiveFromMac (
    BuildGroupFrame (newest.record, CSR_PKT_DISCOVER, 11, false),
    60.0,
    40.0);
  Require (receiver->GetPendingGroupEstablishCount () == 1 &&
             g_groupKeyNeededCallbacks == 2 &&
             g_helloDeliveries == 0,
           "live HOP did not replace and defer GroupEstablish records");

  CsrProtectedGroupMessage tamperedSource = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::GroupEstablish, 4, firstPayload);
  tamperedSource.record[tamperedSource.record.size () - 4] ^= 0x01;
  receiver->ReceiveFromMac (
    BuildGroupFrame (tamperedSource.record, CSR_PKT_DISCOVER, 12, false),
    60.0,
    40.0);
  Require (receiver->GetPendingGroupEstablishCount () == 1 &&
             g_groupKeyNeededCallbacks == 2,
           "unauthenticated Discover replaced the deferred record");

  sender.SetNextDataPrn ({0x01, 0x23, 0x45, 0x67, 0x89});
  CsrKeyUpdateHeader update = sender.BuildKeyUpdate (DESTINATION);
  CsrHeader updateHopHeader (SOURCE, DESTINATION, 20, 7, true, false);
  updateHopHeader.SetType (CSR_PKT_KEY_UPDATE);
  updateHopHeader.SetDestType (CSR_DEST_UNICAST);
  updateHopHeader.SetSecurityCount (SECURITY_COUNT);
  Ptr<Packet> updateFrame = Create<Packet> ();
  updateFrame->AddHeader (update);
  updateFrame->AddHeader (updateHopHeader);
  receiver->ReceiveFromMac (updateFrame, 60.0, 40.0);

  Require (receiver->GetPendingGroupEstablishCount () == 0 &&
             g_helloDeliveries == 1 &&
             g_lastPayload == newestPayload,
           "KeyUpdate did not replay only the newest deferred Discover");
  Require (g_callbackOrder ==
             std::vector<std::string> {"key-update", "payload"},
           "deferred Discover replay ran before the KeyUpdate callback");
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == 1,
           "valid KeyUpdate did not earn exactly one ACK");

  g_helloDeliveries = 0;
  g_lastPayload.clear ();
  g_callbackOrder.clear ();
  const std::vector<uint8_t> routingPayload {
    0xde, 0xad, 0xbe, 0xef, 0x01, 0x02
  };
  CsrProtectedGroupMessage routing = sender.ProtectGroupMessage (
    CsrGroupSecurityMode::Group16, 8, routingPayload);
  std::vector<uint8_t> badRouting = routing.record;
  badRouting.back () ^= 0x80;
  uint32_t ackCount = receiverDevice->GetMac ().GetAckQueuedFrameCount ();

  receiver->ReceiveFromMac (
    BuildGroupFrame (badRouting, CSR_PKT_ROUTING_CONTROL, 30, true),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () == ackCount &&
             g_helloDeliveries == 0,
           "tampered Group16 RoutingControl earned an ACK or delivery");

  receiver->ReceiveFromMac (
    BuildGroupFrame (routing.record, CSR_PKT_ROUTING_CONTROL, 30, true),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () ==
             ackCount + 1 &&
             g_helloDeliveries == 1 &&
             g_lastPayload == routingPayload,
           "valid Group16 RoutingControl was not ACKed and delivered");

  receiver->ReceiveFromMac (
    BuildGroupFrame (routing.record, CSR_PKT_ROUTING_CONTROL, 30, true),
    60.0,
    40.0);
  Require (receiverDevice->GetMac ().GetAckQueuedFrameCount () ==
             ackCount + 1 &&
             g_helloDeliveries == 1,
           "authenticated Group16 retransmit changed delivery or ACK state");

  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  InitializeKeys ();
  CheckGoldenGroupRecords ();
  CheckGroupKeyEvolution ();
  CheckLiveDeferredReplayAndRoutingAck ();
  std::cout << "PASS: legacy CSR group-security parity test" << std::endl;
  return 0;
}
