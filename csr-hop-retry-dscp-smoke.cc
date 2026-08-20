#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-hop-layer.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

struct ReceivedFrame
{
  uint16_t sequence;
  uint8_t dscp;
  double time;
};

std::vector<ReceivedFrame> g_receivedFrames;

void
Require (bool condition, const char* message)
{
  if (!condition)
    {
      std::cerr << "FAIL: " << message << std::endl;
      std::exit (1);
    }
}

void
RecordFrameWithoutAck (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received retry segment has no CSR header");

  g_receivedFrames.push_back ({header.GetSeq (),
                               header.GetDscp (),
                               Simulator::Now ().GetSeconds ()});
}

void
CheckRetryOrder (Ptr<CsrNetDevice> sender)
{
  Require (sender->GetMac ().GetTransmittedFrameCount () == 2,
           "expected one initial aggregate and one retry aggregate");
  Require (g_receivedFrames.size () == 4,
           "receiver did not get two segments in each aggregate");

  Require (g_receivedFrames[0].sequence == 2 &&
             g_receivedFrames[0].dscp == 7 &&
             g_receivedFrames[1].sequence == 1 &&
             g_receivedFrames[1].dscp == 1,
           "initial aggregate did not follow OPNET DSCP ordering");
  Require (g_receivedFrames[0].time == g_receivedFrames[1].time,
           "initial DATA frames were not concatenated");

  Require (g_receivedFrames[2].sequence == 2 &&
             g_receivedFrames[2].dscp == 7 &&
             g_receivedFrames[3].sequence == 1 &&
             g_receivedFrames[3].dscp == 1,
           "HOP retransmission did not preserve original DSCP ordering");
  Require (g_receivedFrames[2].time == g_receivedFrames[3].time,
           "retransmitted DATA frames were not concatenated");
  Require (g_receivedFrames[2].time > g_receivedFrames[1].time,
           "retry aggregate did not follow the initial aggregate");
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  Ptr<CsrHopLayer> hop = CreateObject<CsrHopLayer> ();

  hop->SetNodeId (1);
  hop->SetMac (&sender->GetMac ());
  sender->AddPeer (receiver);

  // Record successful reception but intentionally omit a receiver HOP layer.
  // The missing ACK drives the sender's normal two-second resend path.
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordFrameWithoutAck));

  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  sender->GetPhy ().SetPerModel (noErrors);
  receiver->GetPhy ().SetPerModel (noErrors);

  // Resend entries retain insertion order (sequence 1, then sequence 2).
  // MAC must independently restore DSCP order on both the initial send and
  // every retransmission, matching OPNET's copied-packet behavior.
  hop->SendData (2, 1, Create<Packet> (8), true);
  hop->SendData (2, 7, Create<Packet> (8), true);

  Simulator::Schedule (Seconds (4.2), &CheckRetryOrder, sender);
  Simulator::Stop (Seconds (4.3));
  Simulator::Run ();
  Simulator::Destroy ();

  std::cout << "PASS: OPNET HOP retransmission DSCP parity test"
            << std::endl;
  return 0;
}
