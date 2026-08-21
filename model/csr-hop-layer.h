#pragma once
#include "csr-common.h"
#include "csr-hello-header.h"
#include <algorithm>

class CsrHopLayer : public Object
{
public:
  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrHopLayer")
      .SetParent<Object> ()
      .AddConstructor<CsrHopLayer> ();
    return tid;
  }

  CsrHopLayer ()
    : m_mac (nullptr),
      m_nodeId (0),
      m_resendTime (Seconds (2.0)),
      m_maxNumResend (2)
  {}

  void SetMac (CsrMacCore *mac)
  {
    m_mac = mac;

    if (m_mac != nullptr)
      {
        m_mac->SetTxSentCallback (
          MakeCallback (
            &CsrHopLayer::NotifyMacFrameSent,
            this));
      }
  }
  void SetNodeId (CsrNodeId id)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (id),
                     "CSR HOP node identifier exceeds 24 bits");
    m_nodeId = id;
  }
  void SetActiveNodesForPostTx (uint32_t n)
  {
    if (m_mac != nullptr)
      {
        m_mac->SetActiveNodesForPostTx (n);
      }
  }

  void NoteReportedActiveNodes (uint32_t n)
  {
    if (m_mac != nullptr)
      {
        m_mac->NoteReportedActiveNodes (n);
      }
  }

  // Upper layer callback: payload + src
  void SetRxFromHopCallback (Callback<void, Ptr<Packet>, CsrNodeId> cb)
  {
    m_rxFromHopCb = cb;
  }

  void SetRxHelloFromHopCallback (
    Callback<void, Ptr<Packet>, CsrNodeId, double, double> cb)
  {
    m_rxHelloFromHopCb = cb;
  }

  void SetRxSnmpFromHopCallback (
    Callback<void, Ptr<Packet>, CsrNodeId> cb)
  {
    m_rxSnmpFromHopCb = cb;
  }

  // After SetRxFromHopCallback
  void SetNsdpDecrementCallback (
    Callback<void, CsrNodeId, CsrNodeId> cb)
  {
    m_nsdpDecrCb = cb;
  }

  void SetShouldDackCallback (
    Callback<bool, CsrNodeId, CsrNodeId> cb)
  {
    m_shouldDackCb = cb;
  }

  void SetRelayRouteAvailableCallback (Callback<bool, CsrNodeId> cb)
  {
    m_relayRouteAvailableCb = cb;
  }

  void SetLinkFailureCallback (Callback<void, CsrNodeId> cb)
  {
    m_linkFailureCb = cb;
  }

  void
  SetNwkQueueWakeCallback (
    Callback<void> cb)
  {
    m_nwkQueueWakeCb = cb;
  }

  void SetNeighborCheckSuccessCallback (
    Callback<void,
            CsrNodeId,
            CsrNeighborCheckType,
            uint32_t> cb)
  {
    m_neighborCheckSuccessCb = cb;
  }

  // App/NWK send
  void SendData (CsrNodeId dst, uint8_t dscp,
                 Ptr<Packet> payload, bool ack);

  // Legacy br_SNMP bypasses ACK/resend and DATA flow control.
  void SendSnmp (CsrNodeId hopDestination, Ptr<Packet> payload);

  // Called by MAC when frame arrives off-air
  void ReceiveFromMac (Ptr<Packet> frame, double pathlossDb, double snrDb);

  // Send HELLO broadcast for discovery
  //void SendHello ();
  // instead of void SendHello();
  void SendHello (Ptr<Packet> helloPayload);

  void SendNeighborCheck (CsrNodeId dst, Ptr<Packet> payload);

  void PrintNeighbors () const;

  bool HasNeighbor (CsrNodeId neighbor) const
  {
    return m_neighbors.find (neighbor) != m_neighbors.end ();
  }

  double GetNeighborLastHeardSeconds (CsrNodeId neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return it == m_neighbors.end ()
      ? -1.0
      : it->second.lastHeardSec;
  }

  double GetNeighborPathlossDb (CsrNodeId neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return it == m_neighbors.end ()
      ? std::numeric_limits<double>::quiet_NaN ()
      : it->second.lastPathlossDb;
  }

  double GetNeighborSnrDb (CsrNodeId neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return it == m_neighbors.end ()
      ? std::numeric_limits<double>::quiet_NaN ()
      : it->second.lastSnrDb;
  }

  uint32_t GetNeighborLinkCost (CsrNodeId neighbor) const
  {
    auto it = m_neighbors.find (neighbor);
    return it == m_neighbors.end () ? 0 : it->second.linkCost;
  }

  void SetNeighborFailureCount (CsrNodeId neighbor, uint32_t failures)
  {
    m_neighbors[neighbor].numFailures = failures;
  }

  void SendRoutingControl (
  CsrNodeId dst,
  Ptr<Packet> payload);

  void SendRoutingControl (
    const std::vector<CsrNodeId> &destinations,
    Ptr<Packet> payload);

  void SetRoutingControlSuccessCallback (
    Callback<void,
            CsrNodeId,
              uint32_t,
              CsrRoutingOperation,
              uint8_t,
              uint8_t,
              bool> cb)
  {
    m_routingControlSuccessCb = cb;
  }

  void SetRoutingControlFailureCallback (
    Callback<void,
            std::vector<CsrNodeId>,
            uint32_t,
            CsrRoutingOperation,
            uint8_t,
            uint8_t,
            bool> cb)
  {
    m_routingControlFailureCb = cb;
  }

private:
  struct FlowCtrlEntry
  {
    uint32_t outstanding {0};

    // Legacy flow_ctrl_threshold starts at
    // FLOW_CTRL_MIN_THRESHOLD = 0.
    // The effective number of packets allowed
    // in flight is threshold + 1.
    uint32_t threshold {0};

    // Number of consecutive packets ACKed
    // without requiring retransmission.
    uint32_t consecutiveCleanAcks {0};
  };

  FlowCtrlEntry& GetFlowCtrlEntry (CsrNodeId dest)
  {
    auto it = m_flowCtrlByDest.find (dest);
    if (it == m_flowCtrlByDest.end ())
      {
        FlowCtrlEntry e;
        // default threshold; you can tune this
        e.outstanding = 0;
        e.threshold = 0;
        e.consecutiveCleanAcks = 0;
        it = m_flowCtrlByDest.insert (std::make_pair (dest, e)).first;
      }
    return it->second;
  }

  // Legacy pending_pk_count:
  // total DATA packets still pending in HOP and below.
  uint32_t m_pendingDataCount {0};

  static constexpr uint32_t
    HOP_PENDING_MAX_THRESHOLD = 16;

public:
  static constexpr uint32_t RESEND_QUEUE_SIZE = 512;

  uint32_t GetPendingDataCount () const
  {
    return m_pendingDataCount;
  }

  uint32_t GetOutstandingDataCount (CsrNodeId dest) const
  {
    auto it = m_flowCtrlByDest.find (dest);
    return it == m_flowCtrlByDest.end () ? 0 : it->second.outstanding;
  }

  uint32_t GetResendQueueSize () const
  {
    return static_cast<uint32_t> (m_resendQueue.size ());
  }

  uint64_t GetResendQueueOverflowCount () const
  {
    return m_resendQueueOverflowCount;
  }

  void NotifyMacFrameSent (
    CsrNodeId dest,
    uint16_t seq,
    Time sentAt)
  {
    ResendEntry *entry =
      FindResendEntry (dest, seq);

    if (entry == nullptr)
      {
        return;
      }

    entry->lastTxTime = sentAt;
    entry->initialTxConfirmed = true;

    std::cout << "[HOP " << m_nodeId
              << "] MAC sent confirmation"
              << " dest=" << dest
              << " seq=" << seq
              << " transmission="
              << (entry->resendCount + 1)
              << " sentAt="
              << sentAt.GetSeconds ()
              << std::endl;

    Time wait =
      entry->resendCount >= m_maxNumResend
        ? m_resendTime + m_resendTime
        : m_resendTime;

    Time due = sentAt + wait;
    Time delay =
      due > Simulator::Now ()
        ? due - Simulator::Now ()
        : Seconds (0.0);

    // OPNET schedules one RESEND_TIMER event for every br_Mac_Hop_Inst.
    // Keeping these events independent prevents a later frame from inheriting
    // the check cadence of an earlier transmission.
    Simulator::Schedule (
      delay,
      &CsrHopLayer::CheckResend,
      this);
  }

  bool CanAcceptDataGlobally () const
  {
    // Legacy permits one final transfer while pending_pk_count is 16,
    // then blocks after SendData() raises the count to 17.
    return m_pendingDataCount <=
      HOP_PENDING_MAX_THRESHOLD;
  }

  bool CanSendToHop (CsrNodeId dest)
  {
    FlowCtrlEntry &e = GetFlowCtrlEntry (dest);
    
    bool neighborCanSend =
      e.outstanding <=
        e.threshold;

    // Legacy NWK computes:
    // FLOW_CTRL_MAX_THRESHOLD - pending_pk_count + 1.
    // Therefore pending=16 may admit one final packet,
    // producing pending=17 and congestion.
    bool nodeCanSend =
      CanAcceptDataGlobally ();

    bool canSend =
      neighborCanSend &&
      nodeCanSend;

    std::cout << "[HOP " << m_nodeId
              << "] CanSendToHop dest="
              << dest
              << " outstanding="
              << e.outstanding
              << " threshold="
              << e.threshold
              << " effectiveWindow="
              << (e.threshold + 1)
              << " pendingData="
              << m_pendingDataCount
              << " globalLimit="
              << HOP_PENDING_MAX_THRESHOLD
              << " neighborOK="
              << (neighborCanSend ? 1 : 0)
              << " nodeOK="
              << (nodeCanSend ? 1 : 0)
              << " -> "
              << (canSend ? "YES" : "NO")
              << std::endl;

    return canSend;
  }

private:
  struct ResendTarget
  {
    CsrNodeId dest {0};
    uint16_t seq {0};
    bool acked {false};
  };

  struct ResendEntry
  {
    CsrNodeId dest;
    uint16_t seq;
    uint8_t dscp;
    Ptr<Packet> frame;
    uint32_t resendCount;
    Time lastTxTime;
    bool initialTxConfirmed {false};
    std::vector<ResendTarget> targets;

    // Legacy network flow control applies to ordinary
    // DATA traffic, not reliable routing/control packets.
    bool flowControlTracked {false};
  };

  void HandleDataFrame (const CsrHeader &hdr, Ptr<Packet> payload, bool firstReception);
  void HandleAckFrame  (const CsrHeader &hdr);
  void HandleAckWindow (const CsrHeader &hdr);

  void HandleDackFrame (const CsrHeader &hdr);
  void CheckDack ();

  void EnqueueResend (
    CsrNodeId dst,
    uint16_t seq,
    Ptr<Packet> frame,
    bool flowControlTracked);
  void EnqueueResend (
    const std::vector<CsrHeader::DestinationSequence> &targets,
    Ptr<Packet> frame,
    bool flowControlTracked);
  void CheckResend ();
  ResendEntry* FindResendEntry (CsrNodeId dst, uint16_t seq);
  bool CheckReceivedSeq (CsrNodeId src, uint16_t seq, bool dataTraffic);
  void MarkDataDack (CsrNodeId src, uint16_t seq);
  static int32_t SeqDiff (uint16_t seq1, uint16_t seq2);

  Callback<void, CsrNodeId, CsrNodeId> m_nsdpDecrCb;

  Callback<void, Ptr<Packet>, CsrNodeId, double, double>
    m_rxHelloFromHopCb;

  Callback<void, Ptr<Packet>, CsrNodeId> m_rxSnmpFromHopCb;

  Callback<bool, CsrNodeId, CsrNodeId> m_shouldDackCb;

  Callback<bool, CsrNodeId> m_relayRouteAvailableCb;

  Callback<void, CsrNodeId> m_linkFailureCb;

  Callback<void> m_nwkQueueWakeCb;

  //Callback<void, uint16_t> m_neighborCheckSuccessCb;

  Callback<void,
         CsrNodeId,
         CsrNeighborCheckType,
         uint32_t> m_neighborCheckSuccessCb;

  void NotifyNsdpFromFrame (Ptr<Packet> frame)
  {
    if (m_nsdpDecrCb.IsNull ())
      {
        return;
      }

    // Copy, then remove MAC header and peek Net header
    Ptr<Packet> copy = frame->Copy ();

    CsrHeader mh;
    if (!copy->PeekHeader (mh))
      {
        return;
      }

    if (mh.GetType () != CSR_PKT_DATA)
      {
        return;
      }
    copy->RemoveHeader (mh);

    CsrNetHeader nh;
    if (!copy->PeekHeader (nh))
      {
        return;
      }

    CsrNodeId nwkSrc = nh.GetSrc ();
    CsrNodeId nwkDst = nh.GetDst ();
    m_nsdpDecrCb (nwkSrc, nwkDst);
  }

   struct NeighborInfo
  {
    double lastHeardSec { -1.0 };
    double lastPathlossDb  { std::numeric_limits<double>::quiet_NaN() };
    double lastSnrDb       { std::numeric_limits<double>::quiet_NaN() };
    double s0PowerDbm      { std::numeric_limits<double>::quiet_NaN() };
    uint32_t numFailures   {0};
    uint32_t linkCost      {0};
  };

  struct LinkControlResult
  {
    int speedKbps {8};
    double txPowerDbm {30.0};
    double rxPowerDbm {-105.0};
    uint32_t linkCost {0};
  };

  std::map<CsrNodeId, NeighborInfo> m_neighbors;

  LinkControlResult ComputeLinkControl (CsrNodeId dest);
  void ApplyLinkControl (CsrHeader &header,
                         const std::vector<CsrNodeId> &destinations);

  void UpdateNeighborHeard (CsrNodeId src,
                            double pathlossDb,
                            double snrDb,
                            double advertisedS0PowerDbm)
  {
    double now = Simulator::Now ().GetSeconds ();
    auto &ni = m_neighbors[src];
    bool isNew = (ni.lastHeardSec < 0.0);
    ni.lastHeardSec = now;
    ni.lastPathlossDb = pathlossDb;
    ni.lastSnrDb = snrDb;
    if (std::isfinite (advertisedS0PowerDbm))
      {
        ni.s0PowerDbm = advertisedS0PowerDbm;
      }

    if (isNew)
      {
        std::cout << "[HOP " << m_nodeId << "] New neighbor " << src
                  << " heard at t=" << now << "s" << std::endl;
      }
  }
  struct DackEntry
  {
    CsrNodeId dest;
    uint16_t seq;
    Ptr<Packet> frame;   // original data frame
    Time expiry;
  };

  std::list<DackEntry> m_dackList;
  Time                 m_dackHoldTime { Seconds(20.0) };  // 10*RESEND_TIME (2s)


private:
  CsrMacCore*                                   m_mac;
  CsrNodeId                                      m_nodeId;
  Callback<void, Ptr<Packet>, CsrNodeId>        m_rxFromHopCb;

  // HELLO broadcast config (OPNET-style)
  double   m_maxPower     { 30.0 };    // max TX power dBm
  double   m_minPower     { 0.0 };     // min TX power dBm
  int      m_maxSpeed     { 128 };     // max speed key
  int      m_minSpeed     { 8 };       // min speed key
  double   m_rxS0Base     { -115.0 };  // RX_S0_BASE_LEVEL dBm
  double   m_rxNoise      { -106.975 };// RX_NOISE_FLOOR dBm
  double   m_rxNoiseFloor { -106.975 };// legacy nominal noise floor
  double   m_linkMargin   { 10.0 };    // link margin dB
  double   m_txAmpBreakpoint { 14.0 }; // low/high-power crossover dBm
  uint8_t  m_capability   { 0 };       // capability flags
  uint16_t m_activeNodes  { 0 };       // active node count
  CsrHopLayer* m_hop      { nullptr }; // self-reference for SendHelloBroadcast

  std::map<CsrNodeId, uint16_t>                  m_lastSentSeqByDest;
  std::list<ResendEntry>                        m_resendQueue;
  uint64_t                                      m_resendQueueOverflowCount {0};
  Time                                          m_resendTime;
  uint32_t                                      m_maxNumResend;


  struct RxSeqState
  {
    int32_t  highest { -1 };
    uint64_t ackBitmap  { 0 };
    uint64_t dackBitmap { 0 };
  };


  std::map<CsrNodeId, FlowCtrlEntry> m_flowCtrlByDest;

  // DATA ACK windows must not include reliable routing/control sequence
  // numbers.  The legacy model maintains independent receive state for these
  // paths, so keep duplicate suppression separate here as well.
  std::map<CsrNodeId, RxSeqState>                m_rxDataStateBySrc;
  std::map<CsrNodeId, RxSeqState>                m_rxControlStateBySrc;

  Callback<void,
          CsrNodeId,
          uint32_t,
          CsrRoutingOperation,
          uint8_t,
          uint8_t,
          bool>
    m_routingControlSuccessCb;

  Callback<void,
        std::vector<CsrNodeId>,
        uint32_t,
        CsrRoutingOperation,
        uint8_t,
        uint8_t,
        bool>
  m_routingControlFailureCb;

};

CsrHopLayer::LinkControlResult
CsrHopLayer::ComputeLinkControl (CsrNodeId dest)
{
  LinkControlResult result;

  double interferenceDb = std::max (0.0,
                                    m_rxNoise - m_rxNoiseFloor);
  result.rxPowerDbm = m_rxS0Base + m_linkMargin + interferenceDb;
  result.txPowerDbm = m_maxPower;
  result.speedKbps = m_minSpeed;

  auto neighborIt = m_neighbors.find (dest);
  if (dest == CSR_BROADCAST_ID ||
      neighborIt == m_neighbors.end () ||
      !std::isfinite (neighborIt->second.lastPathlossDb))
    {
      std::cout << "[HOP " << m_nodeId
                << "] link_control dest=" << dest
                << " neighborKnown=0"
                << " speed=" << result.speedKbps
                << " txPower=" << result.txPowerDbm
                << " rxPower=" << result.rxPowerDbm
                << std::endl;
      return result;
    }

  NeighborInfo &neighbor = neighborIt->second;
  double remoteS0PowerDbm = std::isfinite (neighbor.s0PowerDbm)
    ? neighbor.s0PowerDbm
    : result.rxPowerDbm;
  double tx0PowerDbm = remoteS0PowerDbm + neighbor.lastPathlossDb;
  double marginAt8Kbps = m_maxPower - tx0PowerDbm;

  if (marginAt8Kbps >= 23.0)      { result.speedKbps = 1000; }
  else if (marginAt8Kbps >= 20.0) { result.speedKbps = 500; }
  else if (marginAt8Kbps >= 12.0) { result.speedKbps = 128; }
  else if (marginAt8Kbps >= 9.0)  { result.speedKbps = 64; }
  else if (marginAt8Kbps >= 6.0)  { result.speedKbps = 32; }
  else if (marginAt8Kbps >= 3.0)  { result.speedKbps = 16; }
  else if (marginAt8Kbps > 0.0)   { result.speedKbps = 8; }
  else                            { result.speedKbps = m_minSpeed; }

  result.speedKbps = std::clamp (result.speedKbps,
                                 m_minSpeed,
                                 m_maxSpeed);

  double speedPowerOffsetDb = 0.0;
  switch (result.speedKbps)
    {
    case 1000: speedPowerOffsetDb = 23.0; break;
    case 500:  speedPowerOffsetDb = 20.0; break;
    case 128:  speedPowerOffsetDb = 12.0; break;
    case 64:   speedPowerOffsetDb = 9.0;  break;
    case 32:   speedPowerOffsetDb = 6.0;  break;
    case 16:   speedPowerOffsetDb = 3.0;  break;
    case 8:
    default:   speedPowerOffsetDb = 0.0;  break;
    }

  double txmPowerDbm = tx0PowerDbm;
  switch (m_minSpeed)
    {
    case 1000: txmPowerDbm += 23.0; break;
    case 500:  txmPowerDbm += 20.0; break;
    case 128:  txmPowerDbm += 12.0; break;
    case 64:   txmPowerDbm += 9.0;  break;
    case 32:   txmPowerDbm += 6.0;  break;
    case 16:   txmPowerDbm += 3.0;  break;
    case 8:
    default:                            break;
    }

  double unclampedTxPowerDbm = tx0PowerDbm + speedPowerOffsetDb;
  double totalMarginDb = m_maxPower - txmPowerDbm;
  double speedMarginDb = m_maxPower - unclampedTxPowerDbm;

  double selectedTxPowerDbm = std::clamp (unclampedTxPowerDbm,
                                          m_minPower,
                                          m_maxPower);

  int estimatedDistance = selectedTxPowerDbm <= m_txAmpBreakpoint
    ? 75
    : static_cast<int> (
        std::floor (
          std::pow (10.0,
                    (selectedTxPowerDbm - m_txAmpBreakpoint) / 40.0) *
          100.0));

  uint32_t cost = static_cast<uint32_t> (
    std::floor (static_cast<double> (estimatedDistance) * 100.0 /
                static_cast<double> (result.speedKbps)));

  if ((totalMarginDb - 3.0 * neighbor.numFailures) < 0.0)
    {
      cost *= 2;
    }
  if ((speedMarginDb - 3.0 * neighbor.numFailures) > 3.0)
    {
      cost = static_cast<uint32_t> (
        std::floor (static_cast<double> (cost) / 2.0));
    }

  result.txPowerDbm = std::ceil (selectedTxPowerDbm);
  result.linkCost = cost;
  neighbor.linkCost = cost;

  std::cout << "[HOP " << m_nodeId
            << "] link_control dest=" << dest
            << " neighborKnown=1"
            << " s0=" << remoteS0PowerDbm
            << " pathloss=" << neighbor.lastPathlossDb
            << " failures=" << neighbor.numFailures
            << " speed=" << result.speedKbps
            << " txPower=" << result.txPowerDbm
            << " rxPower=" << result.rxPowerDbm
            << " cost=" << result.linkCost
            << std::endl;

  return result;
}

void
CsrHopLayer::ApplyLinkControl (
  CsrHeader &header,
  const std::vector<CsrNodeId> &destinations)
{
  NS_ABORT_MSG_IF (destinations.empty (),
                   "link control requires at least one destination");

  LinkControlResult selected =
    ComputeLinkControl (destinations.front ());

  for (uint32_t i = 1; i < destinations.size (); ++i)
    {
      LinkControlResult candidate =
        ComputeLinkControl (destinations[i]);

      if (candidate.speedKbps < selected.speedKbps)
        {
          selected = candidate;
        }
      else if (candidate.speedKbps == selected.speedKbps &&
               candidate.txPowerDbm > selected.txPowerDbm)
        {
          selected = candidate;
        }
    }

  header.SetLinkControl (
    static_cast<uint8_t> (selected.speedKbps),
    selected.txPowerDbm,
    selected.rxPowerDbm);
}

void CsrHopLayer::SendHello (Ptr<Packet> helloPayload)
{
  NS_ASSERT (m_mac != nullptr);

  // Outer MAC/HOP header
  static uint16_t helloSeq = 0;
  uint16_t seq = ++helloSeq;

  CsrHeader h;
  h.SetSrc (m_nodeId);
  h.SetDst (CSR_BROADCAST_ID);
  h.SetSeq (seq);
  h.SetDscp (7);
  h.SetAckable (false);
  h.SetType (CSR_PKT_HELLO);
  h.SetDestType (CSR_DEST_BROADCAST);
  ApplyLinkControl (h, {CSR_BROADCAST_ID});

  helloPayload->AddHeader (h);

  m_mac->EnqueueTxFrame (helloPayload, CSR_BROADCAST_ID, 7, false);
}

void
CsrHopLayer::SendSnmp (CsrNodeId hopDestination, Ptr<Packet> payload)
{
  NS_ASSERT (m_mac != nullptr);

  // OPNET wraps br_SNMP in a non-ACKable br_Hop packet without allocating a
  // reliable-HOP sequence number.  It also uses the configured minimum rate
  // and maximum power instead of running adaptive link control.
  CsrHeader header;
  header.SetSrc (m_nodeId);
  header.SetDst (hopDestination);
  header.SetSeq (0);
  header.SetDscp (0);
  header.SetAckable (false);
  header.SetType (CSR_PKT_SNMP);
  header.SetDestType (CSR_DEST_UNICAST);
  header.SetLinkControl (
    static_cast<uint8_t> (m_minSpeed),
    m_maxPower,
    m_rxS0Base + m_linkMargin);

  Ptr<Packet> frame = payload->Copy ();
  frame->AddHeader (header);

  std::cout << "[HOP " << m_nodeId
            << "] TX legacy SNMP"
            << " hopDestination=" << hopDestination
            << " ackable=0 dscp=0"
            << std::endl;

  m_mac->EnqueueTxFrame (
    frame,
    hopDestination,
    0,
    false);
}

/*void
CsrHopLayer::PrintNeighbors () const
{
  // TODO: Implement neighbor tracking
  std::cout << "[HOP " << m_nodeId << "] Neighbors: (not yet implemented)" << std::endl;
}*/

void
CsrHopLayer::PrintNeighbors () const
  {
    std::cout << "[HOP " << m_nodeId << "] Neighbors:";
    if (m_neighbors.empty ())
      {
        std::cout << " <none>";
      }
    else
      {
        for (const auto &kv : m_neighbors)
          {
            std::cout << " " << kv.first
            << "(t=" << kv.second.lastHeardSec
            << " pl=" << kv.second.lastPathlossDb
            << " snr=" << kv.second.lastSnrDb << ")";
          }
      }
    std::cout << std::endl;
  }

void
CsrHopLayer::SendData (CsrNodeId dst, uint8_t dscp,
                       Ptr<Packet> payload, bool ack)
{
  NS_ASSERT (m_mac != nullptr);

  uint16_t &lastSeq = m_lastSentSeqByDest[dst];
  uint16_t seq = static_cast<uint16_t> ((lastSeq + 1) & 0xFFFF);
  lastSeq = seq;

  CsrHeader hdr (m_nodeId, dst, seq, dscp, ack, false);
  hdr.SetType(CSR_PKT_DATA);
  hdr.SetDestType(CSR_DEST_UNICAST);
  ApplyLinkControl (hdr, {dst});
  Ptr<Packet> frame = payload->Copy ();
  frame->AddHeader (hdr);


  if (ack)
    {
      // Track this as an outstanding hop-level transmission
      FlowCtrlEntry &fc = GetFlowCtrlEntry (dst);
      fc.outstanding++;

      m_pendingDataCount++;

      std::cout << "[HOP " << m_nodeId
                << "] Pending DATA increment"
                << " dest=" << dst
                << " pending="
                << m_pendingDataCount
                << std::endl;

      if (m_pendingDataCount >
          HOP_PENDING_MAX_THRESHOLD)
        {
          std::cout << "[HOP " << m_nodeId
                    << "] GLOBAL HOP CONGESTED"
                    << " pending="
                    << m_pendingDataCount
                    << " threshold="
                    << HOP_PENDING_MAX_THRESHOLD
                    << std::endl;
        }

      EnqueueResend (
        dst,
        seq,
        frame->Copy (),
        true);
    }

  m_mac->EnqueueTxFrame (frame, dst, dscp, ack);
}

void
CsrHopLayer::SendNeighborCheck (CsrNodeId dst, Ptr<Packet> payload)
{
  NS_ASSERT (m_mac != nullptr);

  uint16_t &lastSeq = m_lastSentSeqByDest[dst];
  uint16_t seq = static_cast<uint16_t> ((lastSeq + 1) & 0xFFFF);
  lastSeq = seq;

  CsrHeader hdr (m_nodeId,
                 dst,
                 seq,
                 7,     // robust control priority
                 true,  // ACKable
                 false);

  hdr.SetType (CSR_PKT_NEIGHBOR_CHECK);
  hdr.SetDestType (CSR_DEST_UNICAST);
  ApplyLinkControl (hdr, {dst});

  Ptr<Packet> frame = payload->Copy ();
  frame->AddHeader (hdr);

  EnqueueResend (
    dst,
    seq,
    frame->Copy (),
    false);

  std::cout << "[HOP " << m_nodeId
            << "] TX reliable NeighborCheck to " << dst
            << " seq=" << seq
            << std::endl;

  m_mac->EnqueueTxFrame (frame,
                         dst,
                         7,
                         true);
}

void
CsrHopLayer::SendRoutingControl (
  CsrNodeId dst,
  Ptr<Packet> payload)
{
  SendRoutingControl (
    std::vector<CsrNodeId> {dst},
    payload);
}

void
CsrHopLayer::SendRoutingControl (
  const std::vector<CsrNodeId> &destinations,
  Ptr<Packet> payload)
{
  NS_ASSERT (m_mac != nullptr);
  NS_ABORT_MSG_IF (
    destinations.empty () ||
      destinations.size () > CsrHeader::MAX_DESTINATIONS,
    "reliable routing control requires 1 to 10 destinations");

  std::vector<CsrHeader::DestinationSequence> targets;
  targets.reserve (destinations.size ());

  for (CsrNodeId destination : destinations)
    {
      uint16_t &lastSeq =
        m_lastSentSeqByDest[destination];
      uint16_t sequence =
        static_cast<uint16_t> (
          (lastSeq + 1) & 0xFFFF);
      lastSeq = sequence;
      targets.emplace_back (
        destination,
        sequence);
    }

  CsrNodeId primaryDestination =
    targets.front ().first;
  uint16_t primarySequence =
    targets.front ().second;

  CsrHeader hdr (
    m_nodeId,
    primaryDestination,
    primarySequence,
    7,
    true,
    false);

  hdr.SetType (CSR_PKT_ROUTING_CONTROL);
  hdr.SetDestType (CSR_DEST_MULTICAST);
  hdr.SetDestinationSequences (targets);
  ApplyLinkControl (hdr, destinations);

  Ptr<Packet> frame = payload->Copy ();
  frame->AddHeader (hdr);

  EnqueueResend (
    targets,
    frame->Copy (),
    false);

  std::cout << "[HOP " << m_nodeId
            << "] TX reliable RoutingControl targets=";
  for (uint32_t i = 0; i < targets.size (); ++i)
    {
      if (i > 0)
        {
          std::cout << ",";
        }
      std::cout << targets[i].first
                << ":"
                << targets[i].second;
    }
  std::cout << std::endl;

  m_mac->EnqueueTxFrame (
    frame,
    primaryDestination,
    7,
    true);
}

void
CsrHopLayer::ReceiveFromMac (Ptr<Packet> frame, double pathlossDb, double snrDb)
{
  CsrHeader hdr;
  if (!frame->PeekHeader (hdr))
    {
      NS_LOG_ERROR ("CsrHopLayer::ReceiveFromMac(): missing CsrHeader");
      return;
    }

  frame->RemoveHeader (hdr);

  // br_hop forwards br_SNMP before update_neighbor().  Preserve that unusual
  // path: even an addressed SNMP packet does not refresh the HOP neighbor
  // table, and an overheard one is discarded without any link side effects.
  if (hdr.GetType () == CSR_PKT_SNMP)
    {
      if (hdr.GetDst () != m_nodeId)
        {
          std::cout << "[HOP " << m_nodeId
                    << "] Overheard legacy SNMP from " << hdr.GetSrc ()
                    << " for hop " << hdr.GetDst ()
                    << " ignored without neighbor update"
                    << std::endl;
          return;
        }

      CsrSnmpHeader snmp;
      if (!frame->PeekHeader (snmp))
        {
          NS_LOG_ERROR ("CsrHopLayer::ReceiveFromMac(): missing CsrSnmpHeader");
          return;
        }

      bool forThisNode =
        snmp.GetDestinationType () == CSR_DEST_BROADCAST ||
        (snmp.GetDestinationType () == CSR_DEST_UNICAST &&
         (snmp.GetDestination () == m_nodeId ||
          snmp.GetDestination () == CSR_BROADCAST_ID));

      if (forThisNode && !m_rxSnmpFromHopCb.IsNull ())
        {
          std::cout << "[HOP " << m_nodeId
                    << "] RX legacy SNMP from " << snmp.GetSource ()
                    << " command="
                    << unsigned (static_cast<uint8_t> (snmp.GetCommand ()))
                    << std::endl;
          m_rxSnmpFromHopCb (frame, hdr.GetSrc ());
        }
      else
        {
          // This intentionally does not relay a control message whose final
          // destination lies beyond this hop.  That is the legacy behavior.
          std::cout << "[HOP " << m_nodeId
                    << "] Drop legacy SNMP finalDestination="
                    << snmp.GetDestination ()
                    << " after one-hop delivery"
                    << std::endl;
        }

      return;
    }

  // OPNET update_neighbor() runs for every packet successfully decoded by
  // MAC, before HOP checks whether the packet is addressed to this node.
  // This preserves passive link learning from wireless overhearing.
  UpdateNeighborHeard (
    hdr.GetSrc (),
    pathlossDb,
    snrDb,
    hdr.HasLinkControl ()
      ? hdr.GetRxPowerDbm ()
      : std::numeric_limits<double>::quiet_NaN ());

  if (hdr.HasDestinationSequences ())
    {
      uint16_t localSequence = 0;
      if (!hdr.GetSequenceForDestination (
            m_nodeId,
            localSequence))
        {
          return;
        }

      // The OPNET for_me() helper selects the sequence paired with this node
      // from the destination structure before normal receive processing.
      hdr.SetDst (m_nodeId);
      hdr.SetSeq (localSequence);
    }

  // HELLO is discovery/control; no further processing needed yet
  /*if (hdr.GetType () == CSR_PKT_HELLO)
    {
      return;
    }*/
   // HELLO is discovery/control. OPNET sends br_Hello up to NWK proc_hello().
  if (hdr.GetType () == CSR_PKT_HELLO)
    {
      std::cout << "[HOP " << m_nodeId << "] RX HELLO from "
                << hdr.GetSrc ()
                << " pathloss=" << pathlossDb
                << " snr=" << snrDb
                << std::endl;

      if (!m_rxHelloFromHopCb.IsNull ())
        {
          m_rxHelloFromHopCb (frame, hdr.GetSrc (), pathlossDb, snrDb);
        }

      return;
    }

  // Non-HELLO traffic addressed elsewhere has already contributed its link
  // observation above, but must not affect ACK/resend, duplicate suppression,
  // routing control, or NWK delivery at this node.
  if (hdr.GetDst () != m_nodeId)
    {
      std::cout << "[HOP " << m_nodeId
                << "] Overheard frame from " << hdr.GetSrc ()
                << " for " << hdr.GetDst ()
                << " type=" << unsigned (hdr.GetType ())
                << " ignored after neighbor update"
                << std::endl;
      return;
    }

  if (hdr.IsAck ())
    {
      std::cout << "[HOP " << m_nodeId << "] RX "
                << (hdr.IsDack () ? "DACK" : "ACK")
                << " from " << hdr.GetSrc ()
                << " seq=" << hdr.GetSeq () << std::endl;

      if (hdr.HasAckWindow ())
        {
          HandleAckWindow (hdr);
        }
      else if (hdr.IsDack ())
        {
          m_mac->CancelAcknowledgedFrames (
            hdr.GetSrc (), hdr.GetSeq (), 0, 1);
          HandleDackFrame (hdr);
        }
      else
        {
          m_mac->CancelAcknowledgedFrames (
            hdr.GetSrc (), hdr.GetSeq (), 1, 0);
          HandleAckFrame (hdr);
        }
      return;
    }

  if (hdr.GetType () == CSR_PKT_NEIGHBOR_CHECK)
    {
      if (hdr.GetDst () != m_nodeId)
        {
          return;
        }

      bool firstReception =
        CheckReceivedSeq (hdr.GetSrc (), hdr.GetSeq (), false);

      std::cout << "[HOP " << m_nodeId
                << "] RX reliable NeighborCheck from "
                << hdr.GetSrc ()
                << " seq=" << hdr.GetSeq ()
                << " firstReception="
                << (firstReception ? 1 : 0)
                << std::endl;

      if (firstReception && !m_rxHelloFromHopCb.IsNull ())
        {
          m_rxHelloFromHopCb (frame,
                              hdr.GetSrc (),
                              pathlossDb,
                              snrDb);
        }

      // Always ACK, including duplicates. The original ACK may have been lost.
      if (hdr.IsAckable ())
        {
          CsrHeader ackHdr (m_nodeId,
                            hdr.GetSrc (),
                            hdr.GetSeq (),
                            7,
                            false,
                            true);

          ackHdr.SetType (CSR_PKT_ACK);
          ackHdr.SetDestType (CSR_DEST_UNICAST);
          ApplyLinkControl (ackHdr, {hdr.GetSrc ()});

          Ptr<Packet> ackPkt = Create<Packet> ();
          ackPkt->AddHeader (ackHdr);

          std::cout << "[HOP " << m_nodeId
                    << "] ACK NeighborCheck to "
                    << hdr.GetSrc ()
                    << " seq=" << hdr.GetSeq ()
                    << std::endl;

          m_mac->EnqueueTxFrame (ackPkt,
                                hdr.GetSrc (),
                                7,
                                false);
        }

      return;
    }

  if (hdr.GetType () == CSR_PKT_ROUTING_CONTROL)
    {
      if (hdr.GetDst () != m_nodeId)
        {
          return;
        }

      bool firstReception =
        CheckReceivedSeq (
          hdr.GetSrc (),
          hdr.GetSeq (),
          false);

      std::cout << "[HOP " << m_nodeId
                << "] RX reliable RoutingControl from "
                << hdr.GetSrc ()
                << " seq=" << hdr.GetSeq ()
                << " firstReception="
                << (firstReception ? 1 : 0)
                << std::endl;

      // Deliver the routing payload only once.
      // Duplicates still receive another ACK.
      if (firstReception &&
          !m_rxHelloFromHopCb.IsNull ())
        {
          m_rxHelloFromHopCb (
            frame,
            hdr.GetSrc (),
            pathlossDb,
            snrDb);
        }

      if (hdr.IsAckable ())
        {
          CsrHeader ackHdr (
            m_nodeId,
            hdr.GetSrc (),
            hdr.GetSeq (),
            7,
            false,
            true);

          ackHdr.SetType (CSR_PKT_ACK);
          ackHdr.SetDestType (
            CSR_DEST_UNICAST);
          ApplyLinkControl (ackHdr, {hdr.GetSrc ()});

          Ptr<Packet> ackPkt =
            Create<Packet> ();

          ackPkt->AddHeader (ackHdr);

          std::cout << "[HOP " << m_nodeId
                    << "] ACK RoutingControl to "
                    << hdr.GetSrc ()
                    << " seq=" << hdr.GetSeq ()
                    << std::endl;

          m_mac->EnqueueTxFrame (
            ackPkt,
            hdr.GetSrc (),
            7,
            false);
        }

      return;
    }
  // Duplicate suppression: check if this (src, seq) has been seen before
  bool firstReception =
    CheckReceivedSeq (hdr.GetSrc (), hdr.GetSeq (), true);

  HandleDataFrame (hdr, frame, firstReception);
}


void
CsrHopLayer::HandleDataFrame (const CsrHeader &hdr,
                              Ptr<Packet> payload,
                              bool firstReception)
{
  CsrNodeId dst = hdr.GetDst ();
  CsrNodeId src = hdr.GetSrc ();
  bool ackable = hdr.IsAckable ();

  if (dst != m_nodeId)
  {
    return;
  }

  // Legacy proc_mac_pk() records the sequence before consulting the route
  // table.  A first reception that cannot be relayed is discarded without an
  // ACK; a retry is therefore recognized as a duplicate and receives an ACK.
  if (firstReception &&
      !m_relayRouteAvailableCb.IsNull ())
    {
      CsrNetHeader nwkHeader;
      if (payload->PeekHeader (nwkHeader) &&
          nwkHeader.GetDst () != m_nodeId &&
          !m_relayRouteAvailableCb (nwkHeader.GetDst ()))
        {
          std::cout << "[HOP " << m_nodeId
                    << "] Drop first relay DATA without onward route"
                    << " nwkSrc=" << nwkHeader.GetSrc ()
                    << " nwkDst=" << nwkHeader.GetDst ()
                    << " hopSrc=" << src
                    << " seq=" << hdr.GetSeq ()
                    << " ACK suppressed"
                    << std::endl;
          return;
        }
    }

  // 1) Deliver up on first reception (lets Net update NSDP for relays)
  if (firstReception && !m_rxFromHopCb.IsNull ())
  {
     m_rxFromHopCb (payload, src);
  }

  // 2) Decide ACK vs DACK (duplicates get plain ACK)
  bool sendDack = false;
  if (ackable && firstReception && !m_shouldDackCb.IsNull ())
  {
      // payload still has CsrNetHeader at this point
    Ptr<Packet> tmp = payload->Copy ();
    CsrNetHeader nh;
    if (tmp->PeekHeader (nh))
    {
      sendDack = m_shouldDackCb (nh.GetSrc (), nh.GetDst ());
    }
  }

  if (ackable)
  {
    if (sendDack)
      {
        MarkDataDack (src, hdr.GetSeq ());
      }

    const RxSeqState &rxState = m_rxDataStateBySrc[src];

    CsrHeader ackHdr (m_nodeId, src, hdr.GetSeq (),
                        /*dscp*/ 7,
                        /*ackable*/ false,
                        /*isAck*/ true);
    ackHdr.SetIsDack (sendDack);
    // OPNET-parity metadata
    ackHdr.SetType (sendDack ? CSR_PKT_DACK : CSR_PKT_ACK);
    ackHdr.SetDestType (CSR_DEST_UNICAST);
    ApplyLinkControl (ackHdr, {src});
    ackHdr.SetHasAckWindow (true);
    ackHdr.SetAckBitmap (rxState.ackBitmap & ~rxState.dackBitmap);
    ackHdr.SetDackBitmap (rxState.dackBitmap);


    Ptr<Packet> ackPkt = Create<Packet> ();
    ackPkt->AddHeader (ackHdr);
     m_mac->EnqueueTxFrame (ackPkt, src, /*dscp*/ 7, /*ackable*/ false);
  }
}

void
CsrHopLayer::HandleAckWindow (const CsrHeader &hdr)
{
  CsrNodeId src = hdr.GetSrc ();
  uint16_t baseSeq = hdr.GetSeq ();

  // DACK wins if a malformed or stale peer marks a sequence in both fields.
  uint64_t dackBitmap = hdr.GetDackBitmap ();
  uint64_t ackBitmap = hdr.GetAckBitmap () & ~dackBitmap;

  m_mac->CancelAcknowledgedFrames (
    src, baseSeq, ackBitmap, dackBitmap);

  for (uint32_t bit = 0; bit < 64; ++bit)
    {
      uint64_t mask = 1ULL << bit;
      bool isDack = (dackBitmap & mask) != 0;

      if (!isDack && (ackBitmap & mask) == 0)
        {
          continue;
        }

      CsrHeader completed = hdr;
      completed.SetSeq (static_cast<uint16_t> (baseSeq - bit));
      completed.SetHasAckWindow (false);
      completed.SetAckBitmap (0);
      completed.SetDackBitmap (0);
      completed.SetIsDack (isDack);
      completed.SetType (isDack ? CSR_PKT_DACK : CSR_PKT_ACK);

      if (isDack)
        {
          HandleDackFrame (completed);
        }
      else
        {
          HandleAckFrame (completed);
        }
    }
}

void
CsrHopLayer::HandleAckFrame (const CsrHeader &hdr)
{
  CsrNodeId src = hdr.GetSrc ();
  uint16_t seq = hdr.GetSeq ();

  ResendEntry *entry = FindResendEntry (src, seq);
  if (!entry)
    {
      NS_LOG_INFO ("Hop " << m_nodeId
                          << " got ACK for unknown seq=" << seq
                          << " from " << src);
      return;
    }

  ResendTarget *acknowledgedTarget = nullptr;
  for (auto &target : entry->targets)
    {
      if (target.dest == src && target.seq == seq)
        {
          acknowledgedTarget = &target;
          break;
        }
    }

  if (acknowledgedTarget == nullptr)
    {
      return;
    }

  // Deliberately notify again for a duplicate partial ACK while the group is
  // still present.  OPNET writes acked[j]=1 without first testing that flag.
  acknowledgedTarget->acked = true;
  bool allTargetsAcked =
    std::all_of (
      entry->targets.begin (),
      entry->targets.end (),
      [] (const ResendTarget &target) {
        return target.acked;
      });

  bool neighborCheckCompleted = false;
  bool routingControlCompleted = false;
  uint32_t completedRoutingSequence = 0;

  CsrRoutingOperation completedRoutingOperation =
    CsrRoutingOperation::None;

  uint8_t completedRoutingSection {0};
  uint8_t completedRoutingTotalSections {1};

  CsrNeighborCheckType completedType =
    CsrNeighborCheckType::None;

  uint32_t completedDiscoverySequence = 0;

  if (entry->frame != nullptr)
    {
      Ptr<Packet> originalFrame =
        entry->frame->Copy ();

      CsrHeader originalHdr;

      if (originalFrame->RemoveHeader (
            originalHdr))
        {
          if (originalHdr.GetType () ==
              CSR_PKT_NEIGHBOR_CHECK)
            {
              neighborCheckCompleted = true;

              CsrHelloHeader controlHeader;

              if (originalFrame->RemoveHeader (
                    controlHeader))
                {
                  completedType =
                    controlHeader
                      .GetNeighborCheckType ();

                  completedDiscoverySequence =
                    controlHeader
                      .GetDiscoverySequence ();
                }
            }
          else if (
            originalHdr.GetType () ==
            CSR_PKT_ROUTING_CONTROL)
            {
              routingControlCompleted = true;

              CsrHelloHeader controlHeader;

              if (originalFrame->RemoveHeader (
                    controlHeader))
                {
	                  completedRoutingSequence =
	                    controlHeader
	                      .GetRoutingSequence ();
	                  completedRoutingOperation =
	                    controlHeader.GetRoutingOperation ();
	                  completedRoutingSection =
	                    controlHeader.GetRoutingSection ();
	                  completedRoutingTotalSections =
	                    controlHeader.GetRoutingTotalSections ();
	                }
              else
                {
                  std::cout << "[HOP " << m_nodeId
                            << "] Reliable RoutingControl completed with "
                            << src
                            << " seq=" << seq
                            << " routingSequence="
                            << completedRoutingSequence
                            << " operation="
                            << unsigned (
                                static_cast<uint8_t> (
                                  completedRoutingOperation))
                            << std::endl;
                }
            }
        }
    }

	  // Inform Net layer only when the single resend transaction is complete.
  if (allTargetsAcked)
    {
      NotifyNsdpFromFrame (entry->frame);
    }

  if (entry->flowControlTracked && allTargetsAcked)
    {
      FlowCtrlEntry &fc =
        GetFlowCtrlEntry (
          entry->dest);

      if (fc.outstanding > 0)
        {
          fc.outstanding--;

          if (m_pendingDataCount > 0)
            {
              m_pendingDataCount--;
            }

          std::cout << "[HOP " << m_nodeId
                    << "] Pending DATA decrement"
                    << " reason=ACK"
                    << " neighbor="
                    << entry->dest
                    << " pending="
                    << m_pendingDataCount
                    << std::endl;
        }

      if (entry->resendCount == 0)
        {
          fc.consecutiveCleanAcks++;

          if (fc.consecutiveCleanAcks >= 3)
            {
              if (fc.threshold < 16)
                {
                  uint32_t oldThreshold =
                    fc.threshold;

                  fc.threshold++;

                  std::cout
                    << "[HOP " << m_nodeId
                    << "] Adaptive flow threshold increased"
                    << " neighbor="
                    << entry->dest
                    << " old="
                    << oldThreshold
                    << " new="
                    << fc.threshold
                    << " effectiveWindow="
                    << (fc.threshold + 1)
                    << std::endl;
                }

              fc.consecutiveCleanAcks = 0;
            }
        }
      else
        {
          fc.consecutiveCleanAcks = 0;
        }
    }

  if (neighborCheckCompleted && allTargetsAcked)
    {
      std::cout << "[HOP " << m_nodeId
                << "] Reliable NeighborCheck completed with "
                << src
                << " seq=" << seq
                << " subtype="
                << unsigned (
                    static_cast<uint8_t> (completedType))
                << " discoverySequence="
                << completedDiscoverySequence
                << std::endl;

      if (!m_neighborCheckSuccessCb.IsNull ())
        {
          m_neighborCheckSuccessCb (
            src,
            completedType,
            completedDiscoverySequence);
        }
    }

  if (routingControlCompleted)
    {
      std::cout << "[HOP " << m_nodeId
                << "] Reliable RoutingControl "
                << (allTargetsAcked ? "completed" : "ACK progress")
                << " with "
                << src
                << " seq=" << seq
	                << " routingSequence="
	                << completedRoutingSequence
	                << " section="
	                << unsigned (completedRoutingSection)
	                << "/"
	                << unsigned (
	                    completedRoutingTotalSections)
	                << std::endl;

      if (!m_routingControlSuccessCb.IsNull ())
        {
          m_routingControlSuccessCb (
            src,
            completedRoutingSequence,
            completedRoutingOperation,
            completedRoutingSection,
            completedRoutingTotalSections,
            allTargetsAcked);
        }
    }

  if (!allTargetsAcked)
    {
      std::cout << "[HOP " << m_nodeId
                << "] RoutingControl partial ACK"
                << " neighbor=" << src
                << " seq=" << seq
                << " resendEntryRetained=1"
                << std::endl;
      return;
    }

  for (auto it = m_resendQueue.begin (); it != m_resendQueue.end (); ++it)
    {
      if (&(*it) == entry)
        {
          m_resendQueue.erase (it);
          break;
        }
    }

	  NS_LOG_INFO ("Hop " << m_nodeId
	                      << " cleared resend to " << src
	                      << " seq=" << seq
	                      << " section="
	                      << unsigned (completedRoutingSection)
	                      << "/"
	                      << unsigned (
	                          completedRoutingTotalSections));
}

void
CsrHopLayer::CheckDack ()
{
  Time now = Simulator::Now ();

  for (auto it = m_dackList.begin (); it != m_dackList.end (); )
    {
      if (now >= it->expiry)
        {
          // NSDP was already released when the DACK arrived.
          // Legacy holds only the HOP per-neighbor flow-control
          // slot until the DACK timer expires.
          FlowCtrlEntry &fc =
            GetFlowCtrlEntry (
              it->dest);

          if (fc.outstanding > 0)
            {
              fc.outstanding--;

              std::cout << "[HOP " << m_nodeId
                        << "] DACK hold expired"
                        << " neighbor="
                        << it->dest
                        << " seq="
                        << it->seq
                        << " outstanding="
                        << fc.outstanding
                        << std::endl;

              if (m_pendingDataCount > 0)
                {
                  m_pendingDataCount--;
                }

              std::cout << "[HOP " << m_nodeId
                        << "] Pending DATA decrement"
                        << " reason=DACK-expiry"
                        << " neighbor="
                        << it->dest
                        << " pending="
                        << m_pendingDataCount
                        << std::endl;
            }

          // Legacy check_dack() wakes the Network layer
          // whenever a delayed HOP flow-control slot is released.
          if (!m_nwkQueueWakeCb.IsNull ())
            {
              std::cout << "[HOP " << m_nodeId
                        << "] DACK release waking NWK queue"
                        << " neighbor="
                        << it->dest
                        << " seq="
                        << it->seq
                        << std::endl;

              m_nwkQueueWakeCb ();
            }

          it =
            m_dackList.erase (it);
        }
      else
        {
          ++it;
        }
    }

}

void
CsrHopLayer::HandleDackFrame (const CsrHeader &hdr)
{
  CsrNodeId src = hdr.GetSrc ();
  uint16_t seq = hdr.GetSeq ();

  ResendEntry *entry = FindResendEntry (src, seq);
  if (!entry)
    {
      NS_LOG_INFO ("Hop " << m_nodeId
                          << " got DACK for unknown seq=" << seq
                          << " from " << src);
      return;
    }

  FlowCtrlEntry &fc =
    GetFlowCtrlEntry (
      entry->dest);

  fc.consecutiveCleanAcks = 0;

  // Legacy behavior:
  // - release the NWK source/destination flow count immediately
  // - keep the HOP per-neighbor flow-control slot held until
  //   the DACK timer expires.
  NotifyNsdpFromFrame (
    entry->frame);

  DackEntry de;
  de.dest =
    entry->dest;

  de.seq =
    entry->seq;

  de.frame =
    entry->frame;

  // Legacy doubles the DACK hold time when the packet had
  // already reached the maximum resend count.
  Time holdTime =
    m_dackHoldTime;

  if (entry->resendCount >=
      m_maxNumResend)
    {
      holdTime =
        m_dackHoldTime +
        m_dackHoldTime;
    }

  de.expiry =
    Simulator::Now () +
    holdTime;

  std::cout << "[HOP " << m_nodeId
            << "] DACK received"
            << " neighbor="
            << entry->dest
            << " seq="
            << entry->seq
            << " resendCount="
            << entry->resendCount
            << " holdSec="
            << holdTime.GetSeconds ()
            << " NSDP=release-now"
            << " hopFlow=hold"
            << std::endl;

  m_dackList.push_back (de);

  // OPNET schedules one DACK_TIMER interrupt for each DACK entry at its
  // absolute expiry.  Do not quantize staggered entries onto a shared polling
  // cadence: each delayed flow-control slot must be released at its own
  // 20-second (or doubled 40-second) deadline.
  Simulator::Schedule (
    holdTime,
    &CsrHopLayer::CheckDack,
    this);

  for (auto it = m_resendQueue.begin (); it != m_resendQueue.end (); ++it)
    {
      if (&(*it) == entry)
        {
          m_resendQueue.erase (it);
          break;
        }
    }
}

void
CsrHopLayer::EnqueueResend (
  CsrNodeId dst,
  uint16_t seq,
  Ptr<Packet> frame,
  bool flowControlTracked)
{
  EnqueueResend (
    std::vector<CsrHeader::DestinationSequence> {
      std::make_pair (dst, seq)},
    frame,
    flowControlTracked);
}

void
CsrHopLayer::EnqueueResend (
  const std::vector<CsrHeader::DestinationSequence> &targets,
  Ptr<Packet> frame,
  bool flowControlTracked)
{
  NS_ABORT_MSG_IF (
    targets.empty () ||
      targets.size () > CsrHeader::MAX_DESTINATIONS,
    "resend entry requires 1 to 10 destinations");

  // OPNET assigns the sequence and updates DATA flow-control state before
  // discovering that the resend queue is full.  It then forwards the frame to
  // MAC without a resend record, so callers intentionally do not roll back.
  if (m_resendQueue.size () >= RESEND_QUEUE_SIZE)
    {
      m_resendQueueOverflowCount++;

      std::cout << "[HOP " << m_nodeId
                << "] Resend entry rejected: OPNET queue limit reached"
                << " dest=" << targets.front ().first
                << " seq=" << targets.front ().second
                << " limit=" << RESEND_QUEUE_SIZE
                << " overflows=" << m_resendQueueOverflowCount
                << std::endl;
      return;
    }

  ResendEntry e;

  e.dest = targets.front ().first;
  e.seq = targets.front ().second;
  e.dscp = 0;
  e.frame = frame;
  e.resendCount = 0;
  e.lastTxTime = Seconds (0.0);
  e.initialTxConfirmed = false;
  e.flowControlTracked =
    flowControlTracked;

  e.targets.reserve (targets.size ());
  for (const auto &target : targets)
    {
      e.targets.push_back (
        {target.first, target.second, false});
    }

  CsrHeader header;
  if (frame != nullptr && frame->PeekHeader (header))
    {
      // OPNET retransmits a copy of the original packet, so MAC reads the
      // original DSCP again when it places that copy into its priority queue.
      e.dscp = header.GetDscp ();
    }

  m_resendQueue.push_back (e);
}

void
CsrHopLayer::CheckResend ()
{
  if (m_resendQueue.empty ())
    {
      return;
    }

  Time now = Simulator::Now ();

  for (auto it = m_resendQueue.begin (); it != m_resendQueue.end (); /* no increment */)
    {
      ResendEntry &e = *it;

      // An OPNET resend entry starts with A_VERY_LARGE_NUMBER and cannot
      // expire until MAC reports the first real transmission instant.
      if (!e.initialTxConfirmed)
        {
          ++it;
          continue;
        }

      if (e.resendCount >= m_maxNumResend)
        {
          // Legacy br_hop waits 2 * RESEND_TIME after the
          // final retransmission before declaring the packet
          // unacknowledged. This gives a delayed ACK one last
          // chance to arrive.
          Time finalAckWait =
            m_resendTime +
            m_resendTime;

          Time sinceLastTx =
            now -
            e.lastTxTime;

          if (sinceLastTx <
              finalAckWait)
            {
              std::cout << "[HOP " << m_nodeId
                        << "] Waiting final ACK grace"
                        << " dest=" << e.dest
                        << " seq=" << e.seq
                        << " resendCount="
                        << e.resendCount
                        << " elapsedSec="
                        << sinceLastTx.GetSeconds ()
                        << " requiredSec="
                        << finalAckWait.GetSeconds ()
                        << std::endl;

              ++it;
              continue;
            }

          NS_LOG_INFO ("Hop " << m_nodeId
                              << " giving up on dest="
                              << e.dest
                              << " seq=" << e.seq);

          // Legacy HOP reports a final failure to NWK only when the resend
          // packet carries Packet_Tx_Info.  In this model that metadata is
          // represented by a reliable ROUTING_CONTROL frame.  Ordinary DATA
          // has no Packet_Tx_Info and therefore must not be converted into a
          // routing-link failure merely because its resend limit expired.
          if (e.frame != nullptr)
            {
              Ptr<Packet> failedFrame =
                e.frame->Copy ();

              CsrHeader failedHopHeader;

              if (failedFrame->RemoveHeader (
                    failedHopHeader) &&
                  failedHopHeader.GetType () ==
                    CSR_PKT_ROUTING_CONTROL)
                {
                  CsrHelloHeader controlHeader;

                  if (failedFrame->RemoveHeader (
                        controlHeader))
                    {
                      uint32_t routingSequence =
                        controlHeader
                          .GetRoutingSequence ();

                      CsrRoutingOperation operation =
                        controlHeader
                          .GetRoutingOperation ();

                      uint8_t routingSection =
                        controlHeader
                          .GetRoutingSection ();

                      uint8_t routingTotalSections =
                        controlHeader
                          .GetRoutingTotalSections ();

                      std::cout << "[HOP " << m_nodeId
                                << "] Reliable RoutingControl FAILED"
                                << " neighbor="
                                << e.dest
                                << " routingSequence="
                                << routingSequence
                                << " operation="
                                << unsigned (
                                    static_cast<uint8_t> (
                                      operation))
                                << " section="
                                << unsigned (
                                    routingSection)
                                << "/"
                                << unsigned (
                                    routingTotalSections)
                                << std::endl;

                      if (!m_routingControlFailureCb
                            .IsNull ())
                        {
                          std::vector<CsrNodeId>
                            failedDestinations;
                          failedDestinations.reserve (
                            e.targets.size ());
                          for (const auto &target : e.targets)
                            {
                              failedDestinations.push_back (
                                target.dest);
                            }

                          m_routingControlFailureCb (
                            failedDestinations,
                            routingSequence,
                            operation,
                            routingSection,
                            routingTotalSections,
                            true);
                        }
                    }
                }
            }

          // Do not call the generic link-failure callback here.  OPNET sends
          // routing completion through br_Sent_Info/routesHopSecSentPacket(),
          // and the callback above is its equivalent.  Calling
          // m_linkFailureCb(e.dest) as well incorrectly penalizes DATA loss
          // and, for a grouped routing frame, can penalize the primary target
          // even after that target already ACKed.

          // Inform Net layer that this flow lost a packet
          NotifyNsdpFromFrame (e.frame);

          if (e.flowControlTracked)
            {
              FlowCtrlEntry &fc =
                GetFlowCtrlEntry (
                  e.dest);

              if (fc.outstanding > 0)
                {
                  fc.outstanding--;

                  if (m_pendingDataCount > 0)
                    {
                      m_pendingDataCount--;
                    }

                  std::cout << "[HOP " << m_nodeId
                            << "] Pending DATA decrement"
                            << " reason=no-ACK"
                            << " neighbor="
                            << e.dest
                            << " pending="
                            << m_pendingDataCount
                            << std::endl;
                }

              fc.consecutiveCleanAcks = 0;

              if (fc.threshold > 0)
                {
                  uint32_t oldThreshold =
                    fc.threshold;

                  fc.threshold--;

                  std::cout
                    << "[HOP " << m_nodeId
                    << "] Adaptive flow threshold decreased"
                    << " neighbor="
                    << e.dest
                    << " old="
                    << oldThreshold
                    << " new="
                    << fc.threshold
                    << " effectiveWindow="
                    << (fc.threshold + 1)
                    << " reason=no-ACK"
                    << std::endl;
                }
            }

          it = m_resendQueue.erase (it);
          continue;
        }

      if (now - e.lastTxTime >= m_resendTime)
        {
          NS_LOG_INFO ("Hop " << m_nodeId << " resending dest="
                              << e.dest << " seq=" << e.seq
                              << " dscp=" << unsigned (e.dscp)
                              << " attempt=" << (e.resendCount + 1));

          e.resendCount++;

          // Keep the legacy provisional timestamp.  br_hop records the time
          // it hands a retransmission to MAC, then br_Mac_Hop_Inst replaces
          // it with the actual sent time when transmission occurs.
          e.lastTxTime = now;

          m_mac->EnqueueTxFrame (e.frame->Copy (), e.dest,
                                 e.dscp, /*ackable*/ true);
        }

      ++it;
    }

}

CsrHopLayer::ResendEntry*
CsrHopLayer::FindResendEntry (CsrNodeId dst, uint16_t seq)
{
  for (auto &e : m_resendQueue)
    {
      for (const auto &target : e.targets)
        {
          if (target.dest == dst && target.seq == seq)
            {
              return &e;
            }
        }
    }
  return nullptr;
}

// Compute relative position between two 16-bit sequence numbers, similar to compare_seq() in br_hop :contentReference[oaicite:3]{index=3}
int32_t
CsrHopLayer::SeqDiff (uint16_t seq1, uint16_t seq2)
{
  int32_t maxs = 0xFFFF; // 16-bit space
  int32_t diff = static_cast<int32_t> (seq2) - static_cast<int32_t> (seq1);

  if (diff > (maxs + 1) / 2)
    {
      diff = diff - maxs - 1;
    }
  if (diff < -(maxs + 1) / 2)
    {
      diff = diff + maxs + 1;
    }
  return diff;
}

bool
CsrHopLayer::CheckReceivedSeq (CsrNodeId src,
                               uint16_t seq,
                               bool dataTraffic)
{
  std::map<CsrNodeId, RxSeqState> &states =
    dataTraffic ? m_rxDataStateBySrc : m_rxControlStateBySrc;

  RxSeqState &state = states[src];

  // First packet ever from this src
  if (state.highest < 0)
    {
      state.highest = static_cast<int32_t> (seq);
      state.ackBitmap = 0x1ULL;
      state.dackBitmap = 0;
      return true;
    }

  int32_t diff = SeqDiff (static_cast<uint16_t> (state.highest),
                          seq);

  if (diff > 0)
    {
      // New highest sequence number
      if (diff >= 64)
        {
          // Jumped outside our 64-packet window; reset both registers.
          state.ackBitmap = 0x1ULL;
          state.dackBitmap = 0;
        }
      else
        {
          state.ackBitmap = (state.ackBitmap << diff) | 0x1ULL;
          state.dackBitmap <<= diff;
        }
      state.highest = static_cast<int32_t> (seq);
      return true;
    }
  else
    {
      // seq <= highest; check if within window and already marked
      int32_t idx = -diff; // how far behind the highest
      if (idx >= 64)
        {
          // Too old, outside window; treat as duplicate and drop
          return false;
        }

      uint64_t mask = (0x1ULL << idx);
      if ((state.ackBitmap | state.dackBitmap) & mask)
        {
          // Already seen; duplicate
          return false;
        }
      else
        {
          // Not seen yet within window; mark it
          state.ackBitmap |= mask;
          return true;
        }
    }
}

void
CsrHopLayer::MarkDataDack (CsrNodeId src, uint16_t seq)
{
  auto stateIt = m_rxDataStateBySrc.find (src);
  if (stateIt == m_rxDataStateBySrc.end () || stateIt->second.highest < 0)
    {
      return;
    }

  RxSeqState &state = stateIt->second;
  int32_t diff = SeqDiff (static_cast<uint16_t> (state.highest), seq);

  if (diff > 0 || -diff >= 64)
    {
      return;
    }

  uint64_t mask = 1ULL << static_cast<uint32_t> (-diff);
  state.ackBitmap &= ~mask;
  state.dackBitmap |= mask;
}

// ------------------------------------------------------------
// CsrNetLayer: network-layer with NWK queue + NSDP
// ------------------------------------------------------------
