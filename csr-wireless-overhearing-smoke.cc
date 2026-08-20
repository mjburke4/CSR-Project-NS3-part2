#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

std::vector<uint16_t> g_node2ObservedDestinations;
uint32_t g_node3NwkDeliveries = 0;
uint32_t g_node1MacDeliveries = 0;

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
BuildDataFrame (uint16_t source,
                uint16_t destination,
                uint16_t sequence,
                bool ackable)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    ackable,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (8);

  Ptr<Packet> frame = Create<Packet> (16);
  frame->AddHeader (header);
  return frame;
}

void
RecordAtNode2Mac (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "node 2 received a frame without a CSR header");
  g_node2ObservedDestinations.push_back (header.GetDst ());
}

void
RecordAtNode3Nwk (Ptr<Packet>, uint16_t)
{
  g_node3NwkDeliveries++;
}

void
RecordAtNode1Mac (Ptr<Packet>, double, double)
{
  g_node1MacDeliveries++;
}

void
CheckFirstOverhear (Ptr<CsrNetDevice> node3,
                    Ptr<CsrHopLayer> hop3)
{
  Require (g_node2ObservedDestinations == std::vector<uint16_t> {2},
           "the intended receiver did not decode the first DATA frame");
  Require (g_node3NwkDeliveries == 0,
           "overheard DATA leaked through HOP into NWK");
  Require (node3->GetMac ().GetAckQueuedFrameCount () == 0,
           "overhearer generated an ACK for another node's DATA");
  Require (hop3->HasNeighbor (1),
           "overheard transmission did not create the HOP neighbor record");
  Require (hop3->GetNeighborLastHeardSeconds (1) > 0.0,
           "overheard transmission did not update last-heard time");
  Require (std::isfinite (hop3->GetNeighborPathlossDb (1)) &&
             std::isfinite (hop3->GetNeighborSnrDb (1)),
           "overheard transmission did not retain link observations");
}

void
CheckAddressedRetry (Ptr<CsrNetDevice> node3)
{
  Require (g_node3NwkDeliveries == 1,
           "overhearing polluted duplicate state for later addressed DATA");
  Require (node3->GetMac ().GetAckQueuedFrameCount () == 0,
           "non-ACKable addressed DATA unexpectedly queued an ACK");
}

void
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> node1 = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> node2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> node3 = CreateObject<CsrNetDevice> (3);
  Ptr<CsrHopLayer> hop3 = CreateObject<CsrHopLayer> ();

  node1->AddPeer (node2);
  node1->AddPeer (node3);
  node3->AddPeer (node1);

  ConfigureNoErrors (node1);
  ConfigureNoErrors (node2);
  ConfigureNoErrors (node3);

  hop3->SetNodeId (3);
  hop3->SetMac (&node3->GetMac ());
  hop3->SetRxFromHopCallback (MakeCallback (&RecordAtNode3Nwk));

  node1->GetMac ().SetRxCallback (MakeCallback (&RecordAtNode1Mac));
  node2->GetMac ().SetRxCallback (MakeCallback (&RecordAtNode2Mac));
  node3->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, hop3));

  // Node 3 is in radio range but is not the HOP destination.  It must learn
  // from this ACKable transmission without delivering or acknowledging it.
  node1->GetMac ().EnqueueTxFrame (
    BuildDataFrame (1, 2, 42, true), 2, 5, true);

  Simulator::Schedule (Seconds (1.6),
                       &CheckFirstOverhear,
                       node3,
                       hop3);

  // Reuse the same source/sequence for a later frame that really is for node
  // 3.  It must be treated as new because the overheard copy was filtered
  // before DATA duplicate bookkeeping.
  Simulator::Schedule (Seconds (1.7), [node1] () {
    node1->GetMac ().EnqueueTxFrame (
      BuildDataFrame (1, 3, 42, false), 3, 5, false);
  });

  Simulator::Schedule (Seconds (3.3),
                       &CheckAddressedRetry,
                       node3);

  // The passive observation of node 1 must also make node 1 fresh at node 3,
  // allowing the OPNET SHORT preamble decision for this return transmission.
  Simulator::Schedule (Seconds (3.4), [node3] () {
    node3->GetMac ().EnqueueTxFrame (
      BuildDataFrame (3, 1, 77, false), 1, 5, false);
  });

  Simulator::Stop (Seconds (4.6));
  Simulator::Run ();

  Require (g_node2ObservedDestinations ==
             std::vector<uint16_t> ({2, 3}),
           "node 2 did not overhear the later node-3 DATA frame");
  Require (g_node3NwkDeliveries == 1,
           "node 3 NWK delivery count changed after the addressed frame");
  Require (node3->GetMac ().GetTransmittedFrameCount () == 1,
           "node 3 transmitted something other than its explicit return DATA");
  Require (node3->GetMac ().GetLongPreambleTxCount () == 0 &&
             node3->GetMac ().GetShortPreambleTxCount () == 1,
           "overheard freshness did not select SHORT for the return DATA");
  Require (g_node1MacDeliveries == 1,
           "node 1 did not receive node 3's return DATA");

  Simulator::Destroy ();
  std::cout << "PASS: OPNET wireless overhearing parity test"
            << std::endl;
  return 0;
}
