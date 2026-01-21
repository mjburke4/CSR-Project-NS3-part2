#pragma once
#include "csr-common.h"

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
      m_maxNumResend (3)
  {}

  void SetMac (CsrMacCore *mac)   { m_mac = mac; }
  void SetNodeId (uint16_t id)    { m_nodeId = id; }

  // Upper layer callback: payload + src
  void SetRxFromHopCallback (Callback<void, Ptr<Packet>, uint16_t> cb)
  {
    m_rxFromHopCb = cb;
  }

  // After SetRxFromHopCallback
  void SetNsdpDecrementCallback (Callback<void, uint16_t, uint16_t> cb)
  {
    m_nsdpDecrCb = cb;
  }

  void SetShouldDackCallback (Callback<bool, uint16_t, uint16_t> cb)
  {
    m_shouldDackCb = cb;
  }

  // App/NWK send
  void SendData (uint16_t dst, uint8_t dscp,
                 Ptr<Packet> payload, bool ack);

  // Called by MAC when frame arrives off-air
  void ReceiveFromMac (Ptr<Packet> frame, double pathlossDb, double snrDb);

  void PrintNeighbors () const;

private:
  struct FlowCtrlEntry
  {
    uint32_t outstanding { 0 };   // how many ACKable frames we've sent but not completed
    uint32_t threshold  { 4 };   // max in-flight before we say "stop" (tuneable)
  };

  FlowCtrlEntry& GetFlowCtrlEntry (uint16_t dest)
  {
    auto it = m_flowCtrlByDest.find (dest);
    if (it == m_flowCtrlByDest.end ())
      {
        FlowCtrlEntry e;
        // default threshold; you can tune this
        e.outstanding = 0;
        e.threshold   = 4;
        it = m_flowCtrlByDest.insert (std::make_pair (dest, e)).first;
      }
    return it->second;
  }

public:
  bool CanSendToHop (uint16_t dest)
  {
    FlowCtrlEntry &e = GetFlowCtrlEntry (dest);
    
    std::cout << "[HOP " << m_nodeId << "] CanSendToHop dest=" << dest
            << " outstanding=" << e.outstanding
            << " threshold="  << e.threshold
            << " -> " << (e.outstanding < e.threshold ? "YES" : "NO")
            << std::endl;
    return e.outstanding < e.threshold;
  }

private:
  struct ResendEntry
  {
    uint16_t   dest;
    uint16_t   seq;
    Ptr<Packet> frame;
    uint32_t   resendCount;
    Time       lastTxTime;
  };

  void HandleDataFrame (const CsrHeader &hdr, Ptr<Packet> payload, bool firstReception);
  void HandleAckFrame  (const CsrHeader &hdr);

  void HandleDackFrame (const CsrHeader &hdr);
  void ScheduleDackCheck ();
  void CheckDack ();

  void EnqueueResend (uint16_t dst, uint16_t seq, Ptr<Packet> frame);
  void ScheduleResendCheck ();
  void CheckResend ();
  ResendEntry* FindResendEntry (uint16_t dst, uint16_t seq);
  bool CheckReceivedSeq (uint16_t src, uint16_t seq);
  static int32_t SeqDiff (uint16_t seq1, uint16_t seq2);
  Callback<void, uint16_t, uint16_t> m_nsdpDecrCb;

  Callback<bool, uint16_t, uint16_t> m_shouldDackCb;

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
    copy->RemoveHeader (mh);

    CsrNetHeader nh;
    if (!copy->PeekHeader (nh))
      {
        return;
      }

    uint16_t nwkSrc = nh.GetSrc ();
    uint16_t nwkDst = nh.GetDst ();
    m_nsdpDecrCb (nwkSrc, nwkDst);
  }

   struct NeighborInfo
  {
    double lastHeardSec { -1.0 };
    double lastPathlossDb  { std::numeric_limits<double>::quiet_NaN() };
    double lastSnrDb       { std::numeric_limits<double>::quiet_NaN() };
  };

  std::map<uint16_t, NeighborInfo> m_neighbors;

  void UpdateNeighborHeard (uint16_t src, double pathlossDb, double snrDb)
  {
    double now = Simulator::Now ().GetSeconds ();
    auto &ni = m_neighbors[src];
    bool isNew = (ni.lastHeardSec < 0.0);
    ni.lastHeardSec = now;
    ni.lastPathlossDb = pathlossDb;
    ni.lastSnrDb = snrDb;

    if (isNew)
      {
        std::cout << "[HOP " << m_nodeId << "] New neighbor " << src
                  << " heard at t=" << now << "s" << std::endl;
      }
  }
  struct DackEntry
  {
    uint16_t dest;
    uint16_t seq;
    Ptr<Packet> frame;   // original data frame
    Time expiry;
  };

  std::list<DackEntry> m_dackList;
  EventId              m_dackEvent;
  Time                 m_dackHoldTime { Seconds(20.0) };  // 10*RESEND_TIME (2s)


private:
  CsrMacCore*                                   m_mac;
  uint16_t                                      m_nodeId;
  Callback<void, Ptr<Packet>, uint16_t>         m_rxFromHopCb;

  std::map<uint16_t, uint16_t>                  m_lastSentSeqByDest;
  std::list<ResendEntry>                        m_resendQueue;
  EventId                                       m_resendEvent;
  Time                                          m_resendTime;
  uint32_t                                      m_maxNumResend;


  struct RxSeqState
  {
    int32_t  highest { -1 };
    uint64_t bitmap  { 0 };
  };


  std::map<uint16_t, FlowCtrlEntry> m_flowCtrlByDest;

  std::map<uint16_t, RxSeqState>                m_rxStateBySrc;
};

void
CsrHopLayer::SendHello ()
{
  NS_ASSERT (m_mac != nullptr);

  // HELLO is a control broadcast, no payload needed
  Ptr<Packet> p = Create<Packet> ();

  // Build MAC header directly (don’t use SendData which forces CSR_PKT_DATA)
  static uint16_t helloSeq = 0;
  uint16_t seq = ++helloSeq;

  CsrHeader h;
  h.SetSrc (m_nodeId);
  h.SetDst (CSR_BROADCAST_ID);
  h.SetSeq (seq);
  h.SetDscp (7);
  h.SetAckable (false);
  h.SetIsAck (false);
  h.SetIsDack (false);
  h.SetType (CSR_PKT_HELLO);
  h.SetDestType (CSR_DEST_BROADCAST);

  // OPNET start_discovery uses min_speed for hello
  h.SetSpeedKey (8);

  p->AddHeader (h);

  // Enqueue like any other TX frame
  m_mac->EnqueueTxFrame (p, CSR_BROADCAST_ID, /*dscp*/7, /*ackable*/false);
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
CsrHopLayer::SendData (uint16_t dst, uint8_t dscp,
                       Ptr<Packet> payload, bool ack)
{
  NS_ASSERT (m_mac != nullptr);

  uint16_t &lastSeq = m_lastSentSeqByDest[dst];
  uint16_t seq = static_cast<uint16_t> ((lastSeq + 1) & 0xFFFF);
  lastSeq = seq;

  CsrHeader hdr (m_nodeId, dst, seq, dscp, ack, false);
  hdr.SetType(CSR_PKT_DATA);
  hdr.SetDestType(CSR_DEST_UNICAST);
  hdr.SetSpeedKey(8);   // default, will be updated by MAC layer
  Ptr<Packet> frame = payload->Copy ();
  frame->AddHeader (hdr);


  if (ack)
    {
      // Track this as an outstanding hop-level transmission
      FlowCtrlEntry &fc = GetFlowCtrlEntry (dst);
      fc.outstanding++;

      EnqueueResend (dst, seq, frame->Copy ());
    }

  m_mac->EnqueueTxFrame (frame, dst, dscp, ack);
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

  UpdateNeighborHeard (hdr.GetSrc (), pathlossDb, snrDb);
  //UpdateNeighborHeard (hdr.GetSrc ());
  
  // HELLO is discovery/control; no further processing needed yet
  if (hdr.GetType () == CSR_PKT_HELLO)
    {
      return;
    }

  if (hdr.IsAck ())
    {
      std::cout << "[HOP " << m_nodeId << "] RX "
                << (hdr.IsDack () ? "DACK" : "ACK")
                << " from " << hdr.GetSrc ()
                << " seq=" << hdr.GetSeq () << std::endl;

      if (hdr.IsDack ())
        {
          HandleDackFrame (hdr);
        }
      else
        {
          HandleAckFrame (hdr);
        }
      return;
    }

  // Duplicate suppression: check if this (src, seq) has been seen before
  bool firstReception = CheckReceivedSeq (hdr.GetSrc (), hdr.GetSeq ());

  HandleDataFrame (hdr, frame, firstReception);
}


void
CsrHopLayer::HandleDataFrame (const CsrHeader &hdr,
                              Ptr<Packet> payload,
                              bool firstReception)
{
  uint16_t dst = hdr.GetDst ();
  uint16_t src = hdr.GetSrc ();
  bool ackable = hdr.IsAckable ();

  if (dst != m_nodeId)
  {
    return;
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
    CsrHeader ackHdr (m_nodeId, src, hdr.GetSeq (),
                        /*dscp*/ 7,
                        /*ackable*/ false,
                        /*isAck*/ true);
    ackHdr.SetIsDack (sendDack);
    // OPNET-parity metadata
    ackHdr.SetType (sendDack ? CSR_PKT_DACK : CSR_PKT_ACK);
    ackHdr.SetDestType (CSR_DEST_UNICAST);
    ackHdr.SetSpeedKey (8);  // control always uses robust rate key


    Ptr<Packet> ackPkt = Create<Packet> ();
    ackPkt->AddHeader (ackHdr);
     m_mac->EnqueueTxFrame (ackPkt, src, /*dscp*/ 7, /*ackable*/ false);
  }
}

void
CsrHopLayer::HandleAckFrame (const CsrHeader &hdr)
{
  uint16_t src = hdr.GetSrc ();
  uint16_t seq = hdr.GetSeq ();

  ResendEntry *entry = FindResendEntry (src, seq);
  if (!entry)
    {
      NS_LOG_INFO ("Hop " << m_nodeId
                          << " got ACK for unknown seq=" << seq
                          << " from " << src);
      return;
    }

  // Inform Net layer that this (nwkSrc,nwkDst) flow completed one packet
  NotifyNsdpFromFrame (entry->frame);

  // Decrement flow-control outstanding counter
  FlowCtrlEntry &fc = GetFlowCtrlEntry (entry->dest);
  if (fc.outstanding > 0)
    {
      fc.outstanding--;
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
                      << " seq=" << seq);
}

void
CsrHopLayer::ScheduleDackCheck ()
{
  if (!m_dackEvent.IsPending () && !m_dackList.empty ())
    {
      m_dackEvent = Simulator::Schedule (Seconds (1.0),
                                         &CsrHopLayer::CheckDack,
                                         this);
    }
}

void
CsrHopLayer::CheckDack ()
{
  Time now = Simulator::Now ();

  for (auto it = m_dackList.begin (); it != m_dackList.end (); )
    {
      if (now >= it->expiry)
        {
          // Release NSDP + flow control *after* delay
          NotifyNsdpFromFrame (it->frame);

          FlowCtrlEntry &fc = GetFlowCtrlEntry (it->dest);
          if (fc.outstanding > 0)
            {
              fc.outstanding--;
            }

          it = m_dackList.erase (it);
        }
      else
        {
          ++it;
        }
    }

  if (!m_dackList.empty ())
    {
      ScheduleDackCheck ();
    }
}

void
CsrHopLayer::HandleDackFrame (const CsrHeader &hdr)
{
  uint16_t src = hdr.GetSrc ();
  uint16_t seq = hdr.GetSeq ();

  ResendEntry *entry = FindResendEntry (src, seq);
  if (!entry)
    {
      NS_LOG_INFO ("Hop " << m_nodeId
                          << " got DACK for unknown seq=" << seq
                          << " from " << src);
      return;
    }

  // Stop retransmitting immediately by removing from resend queue,
  // but delay NSDP/flow release.
  DackEntry de;
  de.dest   = entry->dest;
  de.seq    = entry->seq;
  de.frame  = entry->frame;
  de.expiry = Simulator::Now () + m_dackHoldTime;

  m_dackList.push_back (de);
  ScheduleDackCheck ();

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
CsrHopLayer::EnqueueResend (uint16_t dst, uint16_t seq, Ptr<Packet> frame)
{
  ResendEntry e;
  e.dest        = dst;
  e.seq         = seq;
  e.frame       = frame;
  e.resendCount = 0;
  e.lastTxTime  = Simulator::Now ();

  m_resendQueue.push_back (e);
  ScheduleResendCheck ();
}

void
CsrHopLayer::ScheduleResendCheck ()
{
  if (!m_resendEvent.IsPending () && !m_resendQueue.empty ())
    {
      m_resendEvent = Simulator::Schedule (m_resendTime,
                                           &CsrHopLayer::CheckResend,
                                           this);
    }
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
      if (e.resendCount >= m_maxNumResend)
        {
          NS_LOG_INFO ("Hop " << m_nodeId << " giving up on dest="
                              << e.dest << " seq=" << e.seq);

          // Inform Net layer that this flow lost a packet
          NotifyNsdpFromFrame (e.frame);
          
          // Decrement flow-control outstanding counter
          FlowCtrlEntry &fc = GetFlowCtrlEntry (e.dest);
          if (fc.outstanding > 0)
            {
              fc.outstanding--;
            }

          it = m_resendQueue.erase (it);
          continue;
        }

      if (now - e.lastTxTime >= m_resendTime)
        {
          NS_LOG_INFO ("Hop " << m_nodeId << " resending dest="
                              << e.dest << " seq=" << e.seq
                              << " attempt=" << (e.resendCount + 1));

          e.lastTxTime = now;
          e.resendCount++;

          m_mac->EnqueueTxFrame (e.frame->Copy (), e.dest,
                                 /*dscp*/ 5, /*ackable*/ true);
        }

      ++it;
    }

  if (!m_resendQueue.empty ())
    {
      ScheduleResendCheck ();
    }
}

CsrHopLayer::ResendEntry*
CsrHopLayer::FindResendEntry (uint16_t dst, uint16_t seq)
{
  for (auto &e : m_resendQueue)
    {
      if (e.dest == dst && e.seq == seq)
        {
          return &e;
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
CsrHopLayer::CheckReceivedSeq (uint16_t src, uint16_t seq)
{
  RxSeqState &state = m_rxStateBySrc[src];

  // First packet ever from this src
  if (state.highest < 0)
    {
      state.highest = static_cast<int32_t> (seq);
      state.bitmap  = 0x1ULL; // mark this seq as seen
      return true;
    }

  int32_t diff = SeqDiff (static_cast<uint16_t> (state.highest),
                          seq);

  if (diff > 0)
    {
      // New highest sequence number
      if (diff >= 64)
        {
          // Jumped outside our 64-packet window; reset bitmap
          state.bitmap = 0x1ULL;
        }
      else
        {
          state.bitmap = (state.bitmap << diff) | 0x1ULL;
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
      if (state.bitmap & mask)
        {
          // Already seen; duplicate
          return false;
        }
      else
        {
          // Not seen yet within window; mark it
          state.bitmap |= mask;
          return true;
        }
    }
}

// ------------------------------------------------------------
// CsrNetLayer: network-layer with NWK queue + NSDP
// ------------------------------------------------------------

