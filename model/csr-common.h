#pragma once
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/simulator.h"
#include "ns3/log.h"
#include "ns3/packet.h"
#include "ns3/callback.h"
#include "ns3/random-variable-stream.h"
#include <map>
#include <deque>
#include <vector>
#include <utility>
#include <functional>
#include <cmath>
#include <fstream>

using namespace ns3;

static std::ofstream g_rxCsv;

static const uint16_t CSR_BROADCAST_ID = 0xFFFF;

static void
OpenRxCsv (const std::string& path)
{
  g_rxCsv.open (path, std::ios::out | std::ios::trunc);
  g_rxCsv << "t,tx,rx,seq,rateKbps,bits,dist_m,pathloss_db,snr_db,per,success\n";
}

static void
CloseRxCsv ()
{
  if (g_rxCsv.is_open ())
    {
      g_rxCsv.close ();
    }
}

static std::ofstream g_nsdpCsv;

static void
OpenNsdpCsv (const std::string& path)
{
  g_nsdpCsv.open (path, std::ios::out | std::ios::trunc);
  g_nsdpCsv << "t,node,src,dst,count,limit,nwk_q\n";
}

enum CsrPktType : uint8_t
{
  CSR_PKT_DATA            = 0,
  CSR_PKT_ACK             = 1,
  CSR_PKT_DACK            = 2,
  CSR_PKT_HELLO           = 3,
  CSR_PKT_DISCOVER        = 4,
  CSR_PKT_NEIGHBOR_CHECK  = 5,
  CSR_PKT_ROUTING_CONTROL = 6
};

enum CsrDestType : uint8_t
{
  CSR_DEST_UNICAST   = 0,
  CSR_DEST_BROADCAST = 1,
  CSR_DEST_MULTICAST = 2
};

// ------------------------------------------------------------
// CsrHeader: simple over-the-air header used by Hop + MAC
// ------------------------------------------------------------

class CsrHeader : public Header
{
public:
  using DestinationSequence = std::pair<uint16_t, uint16_t>;
  static constexpr uint8_t MAX_DESTINATIONS = 10;

  CsrHeader ()
    : m_src (0),
      m_dst (0),
      m_seq (0),
      m_dscp (0),
      m_ackable (false),
      m_isAck (false),
      m_isDack (false),
      m_hasAckWindow (false),
      m_ackBitmap (0),
      m_dackBitmap (0),
      m_type (CSR_PKT_DATA),
      m_destType (CSR_DEST_UNICAST),
      m_speedKey (8)
  {}

  CsrHeader (uint16_t src, uint16_t dst, uint16_t seq,
             uint8_t dscp, bool ackable, bool isAck)
    : m_src (src),
      m_dst (dst),
      m_seq (seq),
      m_dscp (dscp),
      m_ackable (ackable),
      m_isAck (isAck),
      m_isDack (false),
      m_hasAckWindow (false),
      m_ackBitmap (0),
      m_dackBitmap (0),
      m_type (isAck ? CSR_PKT_ACK : CSR_PKT_DATA),
      m_destType (CSR_DEST_UNICAST),
      m_speedKey (8)
  {}

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrHeader")
      .SetParent<Header> ()
      .SetGroupName ("Csr")
      .AddConstructor<CsrHeader> ();
    return tid;
  }

  virtual TypeId GetInstanceTypeId () const override
  {
    return GetTypeId ();
  }

  virtual uint32_t GetSerializedSize () const override
  {
    // src(2) + dst(2) + seq(2) + dscp(1) + flags(1)
    //return 2 + 2 + 2 + 1 + 1;
    // src(2) + dst(2) + seq(2) + dscp(1) + flags(1) + type(1) + destType(1) + speedKey(1)
    static constexpr uint32_t baseSize =
      2 + 2 + 2 + 1 + 1 + 1 + 1 + 1;

    // OPNET ACKs carry two 64-bit cumulative receive registers.  Keep the
    // fields optional so existing exact ACKs for routing/control traffic retain
    // their original wire size and behavior.
    uint32_t targetListSize =
      m_destinationSequences.empty ()
        ? 0
        : 1 + 4 * static_cast<uint32_t> (
                    m_destinationSequences.size ());

    return baseSize +
           (m_hasAckWindow ? 16 : 0) +
           targetListSize;

  }

  virtual void Serialize (Buffer::Iterator start) const override
  {
    uint8_t flags = 0;
    if (m_ackable) { flags |= 0x01; }
    if (m_isAck)   { flags |= 0x02; }
    if (m_isDack)  { flags |= 0x04; }
    if (m_hasAckWindow) { flags |= 0x08; }
    if (!m_destinationSequences.empty ()) { flags |= 0x10; }

    start.WriteHtonU16 (m_src);
    start.WriteHtonU16 (m_dst);
    start.WriteHtonU16 (m_seq);
    start.WriteU8 (m_dscp);
    start.WriteU8 (flags);
    start.WriteU8 (m_type);
    start.WriteU8 (m_destType);
    start.WriteU8 (m_speedKey);

    if (m_hasAckWindow)
      {
        start.WriteHtonU64 (m_ackBitmap);
        start.WriteHtonU64 (m_dackBitmap);
      }

    if (!m_destinationSequences.empty ())
      {
        start.WriteU8 (
          static_cast<uint8_t> (m_destinationSequences.size ()));

        for (const auto &target : m_destinationSequences)
          {
            start.WriteHtonU16 (target.first);
            start.WriteHtonU16 (target.second);
          }
      }
  }

  virtual uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_src  = start.ReadNtohU16 ();
    m_dst  = start.ReadNtohU16 ();
    m_seq  = start.ReadNtohU16 ();
    m_dscp = start.ReadU8 ();
    uint8_t flags = start.ReadU8 ();
    m_ackable = (flags & 0x01) != 0;
    m_isAck   = (flags & 0x02) != 0;
    m_isDack  = (flags & 0x04) != 0;
    m_hasAckWindow = (flags & 0x08) != 0;
    bool hasDestinationSequences = (flags & 0x10) != 0;

    // Always read the extended fields (Serialize always writes them)
    m_type     = start.ReadU8 ();
    m_destType = start.ReadU8 ();
    m_speedKey = start.ReadU8 ();

    if (m_hasAckWindow)
      {
        m_ackBitmap = start.ReadNtohU64 ();
        m_dackBitmap = start.ReadNtohU64 ();
      }
    else
      {
        m_ackBitmap = 0;
        m_dackBitmap = 0;
      }

    m_destinationSequences.clear ();
    uint32_t targetListSize = 0;

    if (hasDestinationSequences)
      {
        uint8_t count = start.ReadU8 ();
        NS_ABORT_MSG_IF (
          count == 0 || count > MAX_DESTINATIONS,
          "invalid CSR destination/sequence list size");

        targetListSize = 1 + 4 * count;
        m_destinationSequences.reserve (count);

        for (uint8_t i = 0; i < count; ++i)
          {
            uint16_t destination = start.ReadNtohU16 ();
            uint16_t sequence = start.ReadNtohU16 ();
            m_destinationSequences.emplace_back (
              destination,
              sequence);
          }
      }

    static constexpr uint32_t baseSize =
      2 + 2 + 2 + 1 + 1 + 1 + 1 + 1;

    return baseSize +
           (m_hasAckWindow ? 16 : 0) +
           targetListSize;
  }

  virtual void Print (std::ostream &os) const override
  {
    os << "src=" << m_src
       << " dst=" << m_dst
       << " seq=" << m_seq
       << " dscp=" << unsigned(m_dscp)
       << " ackable=" << m_ackable
       << " isAck=" << m_isAck
       << " isDack=" << m_isDack
       << " hasAckWindow=" << m_hasAckWindow
       << " type=" << unsigned(m_type)
       << " destType=" << unsigned(m_destType)
       << " speedKey=" << unsigned(m_speedKey);

    if (m_hasAckWindow)
      {
        os << " ackBitmap=0x" << std::hex << m_ackBitmap
           << " dackBitmap=0x" << m_dackBitmap << std::dec;
      }

    if (!m_destinationSequences.empty ())
      {
        os << " targets=[";
        for (uint32_t i = 0;
             i < m_destinationSequences.size ();
             ++i)
          {
            if (i > 0)
              {
                os << ",";
              }
            os << m_destinationSequences[i].first
               << ":"
               << m_destinationSequences[i].second;
          }
        os << "]";
      }

  }

  void     SetSrc (uint16_t v)  { m_src = v; }
  uint16_t GetSrc () const      { return m_src; }

  void     SetDst (uint16_t v)  { m_dst = v; }
  uint16_t GetDst () const      { return m_dst; }

  void     SetSeq (uint16_t v)  { m_seq = v; }
  uint16_t GetSeq () const      { return m_seq; }

  void     SetDscp (uint8_t v)  { m_dscp = v; }
  uint8_t  GetDscp () const     { return m_dscp; }

  void     SetAckable (bool v)  { m_ackable = v; }
  bool     IsAckable () const   { return m_ackable; }

  void     SetIsAck (bool v)    { m_isAck = v; }
  bool     IsAck () const       { return m_isAck; }

  void     SetIsDack (bool v)   { m_isDack = v; }
  bool     IsDack () const      { return m_isDack; }

  void SetHasAckWindow (bool v) { m_hasAckWindow = v; }
  bool HasAckWindow () const    { return m_hasAckWindow; }

  void SetAckBitmap (uint64_t v) { m_ackBitmap = v; }
  uint64_t GetAckBitmap () const { return m_ackBitmap; }

  void SetDackBitmap (uint64_t v) { m_dackBitmap = v; }
  uint64_t GetDackBitmap () const { return m_dackBitmap; }

  void SetType (uint8_t v)      { m_type = v; }
  uint8_t GetType () const      { return m_type; }

  void SetDestType (uint8_t v)  { m_destType = v; }
  uint8_t GetDestType () const  { return m_destType; }

  void SetSpeedKey (uint8_t v)  { m_speedKey = v; }
  uint8_t GetSpeedKey () const  { return m_speedKey; }

  void SetDestinationSequences (
    const std::vector<DestinationSequence> &targets)
  {
    NS_ABORT_MSG_IF (
      targets.empty () || targets.size () > MAX_DESTINATIONS,
      "CSR destination/sequence list must contain 1 to 10 entries");
    m_destinationSequences = targets;
  }

  bool HasDestinationSequences () const
  {
    return !m_destinationSequences.empty ();
  }

  const std::vector<DestinationSequence>&
  GetDestinationSequences () const
  {
    return m_destinationSequences;
  }

  bool GetSequenceForDestination (
    uint16_t destination,
    uint16_t &sequence) const
  {
    bool found = false;
    for (const auto &target : m_destinationSequences)
      {
        if (target.first == destination ||
            target.first == CSR_BROADCAST_ID)
          {
            sequence = target.second;
            found = true;
          }
      }
    return found;
  }

  bool IsForDestination (uint16_t destination) const
  {
    if (m_destinationSequences.empty ())
      {
        return m_dst == CSR_BROADCAST_ID ||
               m_dst == destination;
      }

    uint16_t ignoredSequence = 0;
    return GetSequenceForDestination (
      destination,
      ignoredSequence);
  }

private:
  uint16_t m_src;
  uint16_t m_dst;
  uint16_t m_seq;
  uint8_t  m_dscp;
  bool     m_ackable;
  bool     m_isAck;
  bool     m_isDack;
  bool     m_hasAckWindow;
  uint64_t m_ackBitmap;
  uint64_t m_dackBitmap;
  uint8_t  m_type;
  uint8_t  m_destType;
  uint8_t  m_speedKey;
  std::vector<DestinationSequence> m_destinationSequences;

};


// ------------------------------------------------------------
// CsrNetHeader: network-layer header (nwk src/dst + DSCP)
// ------------------------------------------------------------

class CsrNetHeader : public Header
{
public:
  CsrNetHeader ()
    : m_src (0),
      m_dst (0),
      m_dscp (0)
  {}

  CsrNetHeader (uint16_t src, uint16_t dst, uint8_t dscp)
    : m_src (src),
      m_dst (dst),
      m_dscp (dscp)
  {}

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrNetHeader")
      .SetParent<Header> ()
      .SetGroupName ("Csr")
      .AddConstructor<CsrNetHeader> ();
    return tid;
  }

  virtual TypeId GetInstanceTypeId () const override
  {
    return GetTypeId ();
  }

  virtual uint32_t GetSerializedSize () const override
  {
    // src(2) + dst(2) + dscp(1)
    return 2 + 2 + 1;
  }

  virtual void Serialize (Buffer::Iterator start) const override
  {
    start.WriteHtonU16 (m_src);
    start.WriteHtonU16 (m_dst);
    start.WriteU8 (m_dscp);
  }

  virtual uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_src  = start.ReadNtohU16 ();
    m_dst  = start.ReadNtohU16 ();
    m_dscp = start.ReadU8 ();
    return GetSerializedSize ();
  }

  virtual void Print (std::ostream &os) const override
  {
    os << "nwkSrc=" << m_src
       << " nwkDst=" << m_dst
       << " dscp="   << unsigned(m_dscp);
  }

  void     SetSrc (uint16_t v)  { m_src = v; }
  uint16_t GetSrc () const      { return m_src; }

  void     SetDst (uint16_t v)  { m_dst = v; }
  uint16_t GetDst () const      { return m_dst; }

  void     SetDscp (uint8_t v)  { m_dscp = v; }
  uint8_t  GetDscp () const     { return m_dscp; }

private:
  uint16_t m_src;
  uint16_t m_dst;
  uint8_t  m_dscp;
};

NS_LOG_COMPONENT_DEFINE ("CsrDemo");

// Forward decls
class CsrNetDevice;
class CsrMacCore;
class CsrHopLayer;

// ------------------------------------------------------------
// Minimal MAC core (no inheritance from Object, kept simple)
// ------------------------------------------------------------

enum PreambleType
{
  PREAMBLE_SHORT,
  PREAMBLE_LONG
};
