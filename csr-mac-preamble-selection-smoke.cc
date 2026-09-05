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
BuildFrame (CsrNodeId destination,
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
  std::vector<CsrNodeId> scalarDestinations =
    scalar.EnumerateDestinations ();
  Require (scalarDestinations == std::vector<CsrNodeId> {2},
           "scalar header did not enumerate its destination");

  CsrHeader grouped (1, 2, 10, 0, false, false);
  grouped.SetDestinationSequences ({{2, 10}, {3, 20}});
  std::vector<CsrNodeId> groupedDestinations =
    grouped.EnumerateDestinations ();
  Require (groupedDestinations == std::vector<CsrNodeId> ({2, 3}),
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
  Require (sender->GetMac ().GetLongPreambleTxCount () == 0 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "known destination after an unknown did not preserve the source "
           "time0 reset quirk");
  Simulator::Destroy ();
}

void
RunFinalUnknownDestinationScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  // Equal DSCP retains FIFO order.  The scalar known receiver is visited
  // first and the grouped frame ends on unknown node 3, leaving ngbr_ptr NIL
  // at the source's final preamble branch.
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 21, 7), 2, 7, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildGroupedFrame ({{2, 22}, {3, 23}}), 2, 7, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "final-unknown scenario did not form one aggregate");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 1 &&
             sender->GetMac ().GetShortPreambleTxCount () == 0,
           "final unknown destination did not select LONG");
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
RunDutyCycleFreshDestinationScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->EnableDutyCycling (true);
  sender->GetMac ().SetActiveNodesForPostTx (1);
  sender->GetMac ().NoteHeardFrom (2, 0.0);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 55, 6), 2, 6, false);

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "duty-cycle freshness scenario did not transmit once");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 0 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "duty cycling incorrectly forced LONG for a fresh destination");
  Simulator::Destroy ();
}

void
RunLocalActiveNodeStaleScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();

  // OPNET uses local active_nodes=1 for freshness (17.0 seconds), while the
  // neighbor-reported value only expands the random-slot range.  The prior
  // hardcoded-eight implementation incorrectly treated this age as fresh.
  sender->GetMac ().SetActiveNodesForPostTx (1);
  sender->GetMac ().NoteReportedActiveNodes (10);
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  Simulator::Schedule (Seconds (18.0), [sender] () {
    sender->GetMac ().EnqueueTxFrame (
      BuildFrame (2, 56, 6), 2, 6, false);
  });

  Simulator::Stop (Seconds (21.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "local-active-node stale scenario did not transmit once");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 1 &&
             sender->GetMac ().GetShortPreambleTxCount () == 0,
           "freshness used a hardcoded or reported active-node count");
  Simulator::Destroy ();
}

void
RunExpandedFreshnessWindowScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();

  // With local active_nodes=10, OPNET's freshness window is 30.5 seconds.
  // A destination last heard at t=0 must therefore still receive SHORT when
  // this frame reaches the MAC shortly after t=28.
  sender->GetMac ().SetActiveNodesForPostTx (10);
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  Simulator::Schedule (Seconds (28.0), [sender] () {
    sender->GetMac ().EnqueueTxFrame (
      BuildFrame (2, 57, 6), 2, 6, false);
  });

  Simulator::Stop (Seconds (32.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 1,
           "expanded-freshness scenario did not transmit once");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 0 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "local active-node count did not expand the freshness window");
  Simulator::Destroy ();
}

void
RunAckUnknownDestinationScenario ()
{
  Ptr<CsrNetDevice> sender = BuildSenderWithPeers ();
  sender->GetMac ().NoteHeardFrom (2, 0.0);

  // OPNET packs ACKs first and includes their destinations in its mutable
  // time0/ngbr_ptr walk.  The first mixed aggregate ends on known node 2 and
  // is SHORT; the four ACK-only retries end on unknown node 3 and are LONG.
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (3, 60, 0, true), 3, 0, false);
  sender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 61, 5), 2, 5, false);

  Simulator::Stop (Seconds (7.0));
  Simulator::Run ();

  Require (sender->GetMac ().GetTransmittedFrameCount () == 5,
           "ACK scenario did not preserve OPNET's five transmissions");
  Require (sender->GetMac ().GetLongPreambleTxCount () == 4 &&
             sender->GetMac ().GetShortPreambleTxCount () == 1,
           "ACK-first destination walk did not preserve the final-pointer "
           "preamble quirk");
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
  RunFinalUnknownDestinationScenario ();
  RunStaleLaterSegmentScenario ();
  RunAllFreshScenario ();
  RunDutyCycleFreshDestinationScenario ();
  RunLocalActiveNodeStaleScenario ();
  RunExpandedFreshnessWindowScenario ();
  RunAckUnknownDestinationScenario ();
  std::cout << "PASS: OPNET preamble freshness and aggregate selection parity test"
            << std::endl;
  return 0;
}
