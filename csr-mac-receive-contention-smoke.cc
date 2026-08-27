#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-net-device.h"

#include <cstdlib>
#include <iostream>
#include <vector>

using namespace ns3;

namespace
{

std::vector<CsrNodeId> g_receivedSources;
std::vector<double> g_txTimes;

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
ConfigureNoErrors (Ptr<CsrNetDevice> device)
{
  CsrPerModelFn noErrors =
    [] (int, double, uint32_t) { return 0.0; };
  device->GetPhy ().SetPerModel (noErrors);
}

Ptr<Packet>
BuildFrame (CsrNodeId source,
            CsrNodeId destination,
            uint16_t sequence,
            bool ackable = false)
{
  CsrHeader header (source,
                    destination,
                    sequence,
                    5,
                    ackable,
                    false);
  header.SetType (CSR_PKT_DATA);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetSpeedKey (128);

  Ptr<Packet> frame = Create<Packet> (16);
  frame->AddHeader (header);
  return frame;
}

void
RecordReception (Ptr<Packet> frame, double, double)
{
  CsrHeader header;
  Require (frame->PeekHeader (header),
           "received frame did not contain a CSR header");
  g_receivedSources.push_back (header.GetSrc ());
}

void
RecordTransmission (CsrNodeId, uint16_t, Time sentTime)
{
  g_txTimes.push_back (sentTime.GetSeconds ());
}

void
SendDirect (Ptr<CsrNetDevice> sender,
            CsrNodeId destination,
            uint16_t sequence,
            double txPowerDbm,
            PreambleType preamble = PREAMBLE_LONG)
{
  sender->SendToPeer (BuildFrame (sender->GetId (),
                                  destination,
                                  sequence),
                      destination,
                      128,
                      txPowerDbm,
                      preamble,
                      3,
                      false);
}

void
RunStateTransitionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 1, 0.0);

  Simulator::Schedule (MilliSeconds (5), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "receiver left Search before SYNC2TRACK_MIN");
  });
  Simulator::Schedule (MilliSeconds (10), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::TRACK,
             "receiver did not enter Track after SYNC2TRACK_MIN");
  });

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {1},
           "single tracked frame was not delivered exactly once");
  Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
           "receiver did not return to Search after packet completion");
  Simulator::Destroy ();
}

void
RunSimultaneousCollisionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender1 = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> sender2 = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  sender1->AddPeer (receiver);
  sender2->AddPeer (receiver);
  ConfigureNoErrors (sender1);
  ConfigureNoErrors (sender2);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender1, 3, 10, 0.0);
  SendDirect (sender2, 3, 20, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "equal-power simultaneous signals did not collide");
  Require (receiver->GetRxCollisionCount () >= 1,
           "overlapping preambles did not increment collision state");
  Simulator::Destroy ();
}

void
RunAcquisitionCaptureScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> weak = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> strong = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  weak->AddPeer (receiver);
  strong->AddPeer (receiver);
  ConfigureNoErrors (weak);
  ConfigureNoErrors (strong);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (strong, 3, 31, 20.0);
  SendDirect (weak, 3, 30, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {2},
           "Search did not capture the mature strongest preamble");
  Require (receiver->GetRxCaptureCount () >= 1,
           "strongest-preamble capture was not recorded");
  Simulator::Destroy ();
}

void
RunPostLockCollisionScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> first = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> late = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (3);
  first->AddPeer (receiver);
  late->AddPeer (receiver);
  ConfigureNoErrors (first);
  ConfigureNoErrors (late);
  ConfigureNoErrors (receiver);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (first, 3, 40, 0.0);
  Simulator::Schedule (MilliSeconds (20),
                       &SendDirect,
                       late,
                       3,
                       41,
                       20.0,
                       PREAMBLE_LONG);

  Simulator::Stop (Seconds (1.4));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "later stronger signal captured after receiver entered Track");
  Require (receiver->GetRxCollisionCount () >= 1,
           "post-lock overlap did not record a collision");
  Simulator::Destroy ();
}

void
RunSlotFreezeScenario ()
{
  g_receivedSources.clear ();
  g_txTimes.clear ();
  Ptr<CsrNetDevice> jammer = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> contender = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> sink = CreateObject<CsrNetDevice> (3);
  jammer->AddPeer (contender);
  contender->AddPeer (sink);
  ConfigureNoErrors (jammer);
  ConfigureNoErrors (contender);
  ConfigureNoErrors (sink);
  contender->GetMac ().SetTxSentCallback (
    MakeCallback (&RecordTransmission));

  contender->GetMac ().EnqueueTxFrame (
    BuildFrame (2, 3, 50, true), 3, 5, true);
  SendDirect (jammer, 2, 51, 0.0);

  Simulator::Schedule (MilliSeconds (900), [contender] () {
    Require (contender->GetMac ().GetTransmittedFrameCount () == 0,
             "Tx countdown advanced while receiver was in Track");
  });

  Simulator::Stop (Seconds (2.0));
  Simulator::Run ();

  Require (contender->GetMac ().GetTransmittedFrameCount () == 1,
           "frozen Tx countdown did not resume after Track");
  Require (g_txTimes.size () == 1 && g_txTimes.front () > 1.0,
           "tracked reception did not defer the reserved Tx opportunity");
  Simulator::Destroy ();
}

void
RunOpnetAlignedDutyInitializationScenario ()
{
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  receiver->EnableOpnetAlignedDutyCycling (true);

  Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
           "OPNET-aligned duty cycle did not start in Idle at t=0");

  Simulator::Schedule (MilliSeconds (987), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
             "OPNET-aligned duty cycle woke before 0.988 seconds");
  });
  Simulator::Schedule (MilliSeconds (988), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned duty cycle did not wake at 0.988 seconds");
  });
  Simulator::Schedule (MicroSeconds (996800), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned wake window closed too early");
  });
  Simulator::Schedule (MilliSeconds (997), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::IDLE,
             "OPNET-aligned wake window did not return to Idle");
  });
  Simulator::Schedule (MicroSeconds (1976001), [receiver] () {
    Require (receiver->GetMacState () == CsrMacCore::State::SEARCH,
             "OPNET-aligned duty cycle did not schedule its second wake");
  });

  Simulator::Stop (MilliSeconds (1977));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
RunDutyCycleScenarios ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> sender = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->EnableDutyCycling (true);
  receiver->SetDutyCyclePhase (MilliSeconds (500));
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 60, 0.0, PREAMBLE_LONG);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources == std::vector<CsrNodeId> {1},
           "long preamble did not bridge sleep to a periodic wake");
  Simulator::Destroy ();

  g_receivedSources.clear ();
  sender = CreateObject<CsrNetDevice> (1);
  receiver = CreateObject<CsrNetDevice> (2);
  sender->AddPeer (receiver);
  ConfigureNoErrors (sender);
  ConfigureNoErrors (receiver);
  receiver->EnableDutyCycling (true);
  receiver->SetDutyCyclePhase (MilliSeconds (500));
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (sender, 2, 61, 0.0, PREAMBLE_SHORT);

  Simulator::Stop (MilliSeconds (200));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "short preamble incorrectly bridged a sleeping interval");
  Require (receiver->GetRxMissCount () == 1,
           "sleeping short-preamble miss was not recorded");
  Simulator::Destroy ();
}

void
RunHalfDuplexScenario ()
{
  g_receivedSources.clear ();
  Ptr<CsrNetDevice> receiver = CreateObject<CsrNetDevice> (1);
  Ptr<CsrNetDevice> dummy = CreateObject<CsrNetDevice> (2);
  Ptr<CsrNetDevice> interferer = CreateObject<CsrNetDevice> (3);
  receiver->AddPeer (dummy);
  interferer->AddPeer (receiver);
  ConfigureNoErrors (receiver);
  ConfigureNoErrors (dummy);
  ConfigureNoErrors (interferer);
  receiver->GetMac ().SetRxCallback (MakeCallback (&RecordReception));

  SendDirect (receiver, 2, 70, 0.0);
  SendDirect (interferer, 1, 71, 0.0);

  Simulator::Stop (Seconds (1.3));
  Simulator::Run ();

  Require (g_receivedSources.empty (),
           "half-duplex receiver decoded while transmitting");
  Require (receiver->GetRxMissCount () == 1,
           "half-duplex receive miss was not recorded");
  Simulator::Destroy ();
}

} // namespace

int
main ()
{
  Time::SetResolution (Time::NS);

  RunStateTransitionScenario ();
  RunSimultaneousCollisionScenario ();
  RunAcquisitionCaptureScenario ();
  RunPostLockCollisionScenario ();
  RunSlotFreezeScenario ();
  RunOpnetAlignedDutyInitializationScenario ();
  RunDutyCycleScenarios ();
  RunHalfDuplexScenario ();

  std::cout << "PASS: OPNET MAC receive/contention state parity test"
            << std::endl;
  return 0;
}
