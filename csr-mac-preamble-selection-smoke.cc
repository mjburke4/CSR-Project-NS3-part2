#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

using namespace ns3;

namespace
{

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
BuildFrame (uint16_t destination,
            uint16_t sequence,
            uint8_t dscp,
            bool ack = false)
{
  CsrHeader header (1,
                    destination,
                    sequence,
                    dscp,
                    false,
                    ack);
  header.SetType (ack ? CSR_PKT_ACK : CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (8);
  frame->AddHeader (header);
  return frame;
}

Ptr<Packet>
BuildGroupedFrame (
  const std::vector<CsrHeader::DestinationSequence> &targets)
{
  Require (!targets.empty (), "grouped frame requires destinations");

  CsrHeader header (1,
                    targets.front ().first,
                    targets.front ().second,
                    7,
                    false,
                    false);
  header.SetType (CSR_PKT_ROUTING_CONTROL);
  header.SetDestType (CSR_DEST_MULTICAST);
  header.SetSpeedKey (8);
  header.SetDestinationSequences (targets);

  Ptr<Packet> frame = Create<Packet> (8);
  frame->AddHeader (header);
  return frame;
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

Ptr<CsrNetDevice>
BuildSenderWithPeers ()
{
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver3 = CreateObject<CsrNetDevice> (3);
  sender->AddPeer (receiver2);
  sender->AddPeer (receiver3);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver2);
  ConfigureNoErrors (receiver3);
  return sender;
}

void
CheckDestinationEnumerator ()
{
  CsrHeader scalar (1, 2, 10, 0, false, false);
  std::vector<uint16_t> scalarDestinations =
    scalar.EnumerateDestinations ();
  Require (scalarDestinations == std::vector<uint16_t> {2},
           "scalar header did not enumerate its destination");

  CsrHeader grouped (1, 2, 10, 0, false, false);
  grouped.SetDestinationSequences ({{2, 10}, {3, 20}});
  std::vector<uint16_t> groupedDestinations =
    grouped.EnumerateDestinations ();
  Require (groupedDestinations == std::vector<uint16_t> ({2, 3}),
           "grouped header did not enumerate every destination in order");
}

void
RunSingleGroupedLegacyScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  sender->GetMac ().EnqueueTxFrame (
    BuildGroupedFrame ({{2, 5}, {3, 6}}), 2, 7, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "single grouped frame was not transmitted once");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 0 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "single frame did not preserve OPNET's primary-only preamble quirk");
  Simulator::Destroy ();
}

void
RunGroupedUnknownDestinationScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  sender->GetMac ().EnqueueTxFrame (
    BuildGroupedFrame ({{2, 10}, {3, 20}}), 2, 7, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 11, 6), 2, 6, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "grouped-destination scenario did not form one aggregate");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 1 &&
             sender->GetMac ().GetShortPreambleTxCount () == 0,
           "unknown non-primary grouped destination did not select LONG");
  Simulator::Destroy ();
}

void
RunStaleLaterSegmentScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (3, 0.0);

  Simulator::Schedule (Seconds (28.0), [sender] () {
    sender->GetMac ().NoteHeardFrom (2,
      Simulator::Now ().GetSeconds ());
    sender->GetMac ().EnqueueTxFrame (
      BuildFrame (2, 30, 7), 2, 7, false);
    sender->GetMac ().EnqueueTxFrame (
      BuildFrame (3, 31, 6), 3, 6, false);
  });

  Simulator::Stop (Seconds (30.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "stale-destination scenario did not form one aggregate");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 1 &&
             sender->GetMac ().GetShortPreambleTxCount () == 0,
           "stale destination in a later segment did not select LONG");
  Simulator::Destroy ();
}

void
RunAllFreshScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);
  sender->GetMac ().NoteHeardFrom (3, 0.0);

  sender->GetMac ().EnqueueTxFrame (
    BuildGroupedFrame ({{2, 40}, {3, 50}}), 2, 7, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 41, 6), 2, 6, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "all-fresh scenario did not form one aggregate");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 0 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "all-fresh aggregate did not preserve SHORT preamble");
  Simulator::Destroy ();
}

void
RunAckUnknownDestinationScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  // OPNET packs ACKs first and includes their destinations when finding the
  // earliest last_rcvd_time.  The ACK remains queued for five transmissions.
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (3, 60, 0, true), 3, 0, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 61, 5), 2, 5, false);

  Simulator::Stop (Seconds (7.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 5,
           "ACK scenario did not preserve OPNET's five transmissions");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 5 &&
             sender->GetMac ().GetShortPreambleTxCount () == 0,
           "unknown ACK destination was omitted from preamble selection");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);
  CheckDestinationEnumerator ();
  RunSingleGroupedLegacyScenario ();
  RunGroupedUnknownDestinationScenario ();
  RunStaleLaterSegmentScenario ();
  RunAllFreshScenario ();
  RunAckUnknownDestinationScenario ();
  std::cout << "PASS: OPNET aggregate preamble selection parity test"
            << std::endl;
  return 0;
}
