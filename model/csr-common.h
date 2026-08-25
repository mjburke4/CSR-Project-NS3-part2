#pragma once
#include "ns3/abort.h"
#include "ns3/callback.h"
#include "ns3/event-id.h"
#include "ns3/log.h"
#include "ns3/nstime.h"
#include "ns3/object.h"
#include "ns3/random-variable-stream.h"
#include "ns3/simulator.h"
#include "ns3/buffer.h"
#include "ns3/header.h"
#include "ns3/packet.h"
#include "csr-wire-format.h"
#include <map>
#include <deque>
#include <vector>
#include <utility>
#include <functional>
#include <algorithm>
#include <cmath>
#include <fstream>

using namespace ns3;

static std::ofstream g_rxCsv;

static void
OpenRxCsv (const std::string& path)
{
  g_rxCsv.open (path, std::ios::out | std::ios::trunc);
  g_rxCsv << "t,tx,rx,seq,rateKbps,bits,dist_m,path_model,"
          << "band_overlap_hz,pathloss_db,rx_power_dbm,noise_dbm,snr_db,"
          << "same_rate_interference,jsr_db,time_offset_s,per,success\n";
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
  CSR_PKT_ROUTING_CONTROL = 6,
  CSR_PKT_SNMP            = 7,
  CSR_PKT_KEY_REQUEST     = 8,
  CSR_PKT_KEY_UPDATE      = 9,
  CSR_PKT_PAIRWISE32_DATA = 10,
  CSR_PKT_NEIGHBORCAST    = 11
};

enum CsrDestType : uint8_t
{
  CSR_DEST_UNICAST   = 0,
  CSR_DEST_BROADCAST = 1,
  CSR_DEST_MULTICAST = 2
};

enum CsrSnmpCommand : uint8_t
{
  CSR_SNMP_START_DISCOVERY   = 1,
  CSR_SNMP_DISCOVERY_DONE    = 2,
  CSR_SNMP_START_REDISCOVER  = 3,
  CSR_SNMP_PRINT_ROUTES_INFO = 4,
  CSR_SNMP_RELAY_HOLDOFF     = 5,
  CSR_SNMP_RELAY_CLEAR       = 6
};

// ------------------------------------------------------------
// CsrHeader: simple over-the-air header used by Hop + MAC
// ------------------------------------------------------------

class CsrHeader : public Header
{
public:
  using DestinationSequence = std::pair<CsrNodeId, uint16_t>;
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
      m_hasLinkControl (false),
      m_hasSecurityCount (false),
      m_hasGroupSecurity (false),
      m_ackBitmap (0),
      m_dackBitmap (0),
      m_type (CSR_PKT_DATA),
      m_destType (CSR_DEST_UNICAST),
      m_speedKey (8),
      m_securityCount (0),
      m_txPowerDbmX10 (0),
      m_rxPowerDbmX10 (0)
  {}

  CsrHeader (CsrNodeId src, CsrNodeId dst, uint16_t seq,
             uint8_t dscp, bool ackable, bool isAck)
    : m_src (src),
      m_dst (dst),
      m_seq (seq),
      m_dscp (dscp),
      m_ackable (ackable),
      m_isAck (isAck),
      m_isDack (false),
      m_hasAckWindow (false),
      m_hasLinkControl (false),
      m_hasSecurityCount (false),
      m_hasGroupSecurity (false),
      m_ackBitmap (0),
      m_dackBitmap (0),
      m_type (isAck ? CSR_PKT_ACK : CSR_PKT_DATA),
      m_destType (CSR_DEST_UNICAST),
      m_speedKey (8),
      m_securityCount (0),
      m_txPowerDbmX10 (0),
      m_rxPowerDbmX10 (0)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (src) ||
                     !CsrIsValidNodeId (dst),
                     "CSR header address exceeds the legacy 24-bit field");
  }

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
    // src(3) + dst(3) + seq(2) + dscp(1) + flags(1)
    // + type(1) + destType(1) + speedKey(1)
    static constexpr uint32_t baseSize =
      3 + 3 + 2 + 1 + 1 + 1 + 1 + 1;

    // OPNET ACKs carry two 64-bit cumulative receive registers.  Keep the
    // fields optional so existing exact ACKs for routing/control traffic retain
    // their original wire size and behavior.
    uint32_t targetListSize =
      m_destinationSequences.empty ()
        ? 0
        : 1 + 5 * static_cast<uint32_t> (
                    m_destinationSequences.size ());

    return baseSize +
           (m_hasAckWindow ? 16 : 0) +
           (m_hasLinkControl ? 4 : 0) +
           (m_hasSecurityCount ? 2 : 0) +
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
    if (m_hasLinkControl) { flags |= 0x20; }
    if (m_hasSecurityCount) { flags |= 0x40; }
    if (m_hasGroupSecurity) { flags |= 0x80; }

    CsrWriteNodeId (start, m_src);
    CsrWriteNodeId (start, m_dst);
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

    if (m_hasLinkControl)
      {
        start.WriteHtonU16 (
          static_cast<uint16_t> (m_txPowerDbmX10));
        start.WriteHtonU16 (
          static_cast<uint16_t> (m_rxPowerDbmX10));
      }

    if (m_hasSecurityCount)
      {
        start.WriteHtonU16 (m_securityCount);
      }

    if (!m_destinationSequences.empty ())
      {
        start.WriteU8 (
          static_cast<uint8_t> (m_destinationSequences.size ()));

        for (const auto &target : m_destinationSequences)
          {
            CsrWriteNodeId (start, target.first);
            start.WriteHtonU16 (target.second);
          }
      }
  }

  virtual uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_src  = CsrReadNodeId (start);
    m_dst  = CsrReadNodeId (start);
    m_seq  = start.ReadNtohU16 ();
    m_dscp = start.ReadU8 ();
    uint8_t flags = start.ReadU8 ();
    m_ackable = (flags & 0x01) != 0;
    m_isAck   = (flags & 0x02) != 0;
    m_isDack  = (flags & 0x04) != 0;
    m_hasAckWindow = (flags & 0x08) != 0;
    bool hasDestinationSequences = (flags & 0x10) != 0;
    m_hasLinkControl = (flags & 0x20) != 0;
    m_hasSecurityCount = (flags & 0x40) != 0;
    m_hasGroupSecurity = (flags & 0x80) != 0;

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
    if (m_hasLinkControl)
      {
        m_txPowerDbmX10 =
          static_cast<int16_t> (start.ReadNtohU16 ());
        m_rxPowerDbmX10 =
          static_cast<int16_t> (start.ReadNtohU16 ());
      }
    else
      {
        m_txPowerDbmX10 = 0;
        m_rxPowerDbmX10 = 0;
      }

    if (m_hasSecurityCount)
      {
        m_securityCount = start.ReadNtohU16 ();
      }
    else
      {
        m_securityCount = 0;
      }

    m_destinationSequences.clear ();
    uint32_t targetListSize = 0;

    if (hasDestinationSequences)
      {
        uint8_t count = start.ReadU8 ();
        NS_ABORT_MSG_IF (
          count == 0 || count > MAX_DESTINATIONS,
          "invalid CSR destination/sequence list size");

        targetListSize = 1 + 5 * count;
        m_destinationSequences.reserve (count);

        for (uint8_t i = 0; i < count; ++i)
          {
            CsrNodeId destination = CsrReadNodeId (start);
            uint16_t sequence = start.ReadNtohU16 ();
            m_destinationSequences.emplace_back (
              destination,
              sequence);
          }
      }

    static constexpr uint32_t baseSize =
      3 + 3 + 2 + 1 + 1 + 1 + 1 + 1;

    return baseSize +
           (m_hasAckWindow ? 16 : 0) +
           (m_hasLinkControl ? 4 : 0) +
           (m_hasSecurityCount ? 2 : 0) +
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
       << " hasLinkControl=" << m_hasLinkControl
       << " hasSecurityCount=" << m_hasSecurityCount
       << " hasGroupSecurity=" << m_hasGroupSecurity
       << " type=" << unsigned(m_type)
       << " destType=" << unsigned(m_destType)
       << " speedKey=" << unsigned(m_speedKey);

    if (m_hasLinkControl)
      {
        os << " txPowerDbm="
           << static_cast<double> (m_txPowerDbmX10) / 10.0
           << " rxPowerDbm="
           << static_cast<double> (m_rxPowerDbmX10) / 10.0;
      }

    if (m_hasSecurityCount)
      {
        os << " securityCount=" << m_securityCount;
      }

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

  void SetSrc (CsrNodeId v)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (v),
                     "CSR source exceeds the legacy 24-bit field");
    m_src = v;
  }
  CsrNodeId GetSrc () const { return m_src; }

  void SetDst (CsrNodeId v)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (v),
                     "CSR destination exceeds the legacy 24-bit field");
    m_dst = v;
  }
  CsrNodeId GetDst () const { return m_dst; }

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

  void SetSecurityCount (uint16_t value)
  {
    m_securityCount = value;
    m_hasSecurityCount = true;
  }

  bool HasSecurityCount () const { return m_hasSecurityCount; }
  uint16_t GetSecurityCount () const { return m_securityCount; }

  void SetHasGroupSecurity (bool value) { m_hasGroupSecurity = value; }
  bool HasGroupSecurity () const { return m_hasGroupSecurity; }

  void SetLinkControl (uint8_t speedKey,
                       double txPowerDbm,
                       double rxPowerDbm)
  {
    m_speedKey = speedKey;
    m_txPowerDbmX10 = static_cast<int16_t> (
      std::round (txPowerDbm * 10.0));
    m_rxPowerDbmX10 = static_cast<int16_t> (
      std::round (rxPowerDbm * 10.0));
    m_hasLinkControl = true;
  }

  bool HasLinkControl () const { return m_hasLinkControl; }

  double GetTxPowerDbm () const
  {
    return static_cast<double> (m_txPowerDbmX10) / 10.0;
  }

  double GetRxPowerDbm () const
  {
    return static_cast<double> (m_rxPowerDbmX10) / 10.0;
  }

  void SetDestinationSequences (
    const std::vector<DestinationSequence> &targets)
  {
    NS_ABORT_MSG_IF (
      targets.empty () || targets.size () > MAX_DESTINATIONS,
      "CSR destination/sequence list must contain 1 to 10 entries");
    for (const auto &target : targets)
      {
        NS_ABORT_MSG_IF (
          !CsrIsValidNodeId (target.first),
          "CSR target destination exceeds the legacy 24-bit field");
      }
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

  std::vector<CsrNodeId> EnumerateDestinations () const
  {
    if (m_destinationSequences.empty ())
      {
        return {m_dst};
      }

    std::vector<CsrNodeId> destinations;
    destinations.reserve (m_destinationSequences.size ());
    for (const auto &target : m_destinationSequences)
      {
        destinations.push_back (target.first);
      }
    return destinations;
  }

  bool GetSequenceForDestination (
    CsrNodeId destination,
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

  bool IsForDestination (CsrNodeId destination) const
  {
    for (CsrNodeId target : EnumerateDestinations ())
      {
        if (target == CSR_BROADCAST_ID ||
            target == destination)
          {
            return true;
          }
      }
    return false;
  }

private:
  CsrNodeId m_src;
  CsrNodeId m_dst;
  uint16_t m_seq;
  uint8_t  m_dscp;
  bool     m_ackable;
  bool     m_isAck;
  bool     m_isDack;
  bool     m_hasAckWindow;
  bool     m_hasLinkControl;
  bool     m_hasSecurityCount;
  bool     m_hasGroupSecurity;
  uint64_t m_ackBitmap;
  uint64_t m_dackBitmap;
  uint8_t  m_type;
  uint8_t  m_destType;
  uint8_t  m_speedKey;
  uint16_t m_securityCount;
  int16_t  m_txPowerDbmX10;
  int16_t  m_rxPowerDbmX10;
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

  CsrNetHeader (CsrNodeId src, CsrNodeId dst, uint8_t dscp)
    : m_src (src),
      m_dst (dst),
      m_dscp (dscp)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (src) ||
                     !CsrIsValidNodeId (dst),
                     "CSR NWK address exceeds the legacy 24-bit field");
  }

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
    // src(3) + dst(3) + dscp(1)
    return 3 + 3 + 1;
  }

  virtual void Serialize (Buffer::Iterator start) const override
  {
    CsrWriteNodeId (start, m_src);
    CsrWriteNodeId (start, m_dst);
    start.WriteU8 (m_dscp);
  }

  virtual uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_src  = CsrReadNodeId (start);
    m_dst  = CsrReadNodeId (start);
    m_dscp = start.ReadU8 ();
    return GetSerializedSize ();
  }

  virtual void Print (std::ostream &os) const override
  {
    os << "nwkSrc=" << m_src
       << " nwkDst=" << m_dst
       << " dscp="   << unsigned(m_dscp);
  }

  void SetSrc (CsrNodeId v)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (v),
                     "CSR NWK source exceeds the legacy 24-bit field");
    m_src = v;
  }
  CsrNodeId GetSrc () const { return m_src; }

  void SetDst (CsrNodeId v)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (v),
                     "CSR NWK destination exceeds the legacy 24-bit field");
    m_dst = v;
  }
  CsrNodeId GetDst () const { return m_dst; }

  void     SetDscp (uint8_t v)  { m_dscp = v; }
  uint8_t  GetDscp () const     { return m_dscp; }

private:
  CsrNodeId m_src;
  CsrNodeId m_dst;
  uint8_t  m_dscp;
};

// ------------------------------------------------------------
// CsrSnmpHeader: legacy NWK discovery-coordination message
// ------------------------------------------------------------

class CsrSnmpHeader : public Header
{
public:
  static constexpr uint8_t MAX_NODES = 10;

  CsrSnmpHeader ()
    : m_source (0),
      m_destination (0),
      m_destinationType (CSR_DEST_UNICAST),
      m_command (CSR_SNMP_START_DISCOVERY),
      m_value (0)
  {}

  static TypeId GetTypeId (void)
  {
    static TypeId tid = TypeId ("ns3::CsrSnmpHeader")
      .SetParent<Header> ()
      .SetGroupName ("Csr")
      .AddConstructor<CsrSnmpHeader> ();
    return tid;
  }

  TypeId GetInstanceTypeId () const override
  {
    return GetTypeId ();
  }

  uint32_t GetSerializedSize () const override
  {
    // source(3) + destination(3) + destination type(1) + command(1)
    // + value(4) + node count(1) + node IDs(3 each)
    return 13 + 3 * static_cast<uint32_t> (m_nodes.size ());
  }

  void Serialize (Buffer::Iterator start) const override
  {
    CsrWriteNodeId (start, m_source);
    CsrWriteNodeId (start, m_destination);
    start.WriteU8 (m_destinationType);
    start.WriteU8 (m_command);
    start.WriteHtonU32 (static_cast<uint32_t> (m_value));
    start.WriteU8 (static_cast<uint8_t> (m_nodes.size ()));

    for (CsrNodeId node : m_nodes)
      {
        CsrWriteNodeId (start, node);
      }
  }

  uint32_t Deserialize (Buffer::Iterator start) override
  {
    m_source = CsrReadNodeId (start);
    m_destination = CsrReadNodeId (start);
    m_destinationType = start.ReadU8 ();
    m_command = start.ReadU8 ();
    m_value = static_cast<int32_t> (start.ReadNtohU32 ());

    uint8_t nodeCount = start.ReadU8 ();
    m_nodes.clear ();

    for (uint8_t index = 0; index < nodeCount; ++index)
      {
        CsrNodeId node = CsrReadNodeId (start);
        if (m_nodes.size () < MAX_NODES)
          {
            m_nodes.push_back (node);
          }
      }

    return 13 + 3 * static_cast<uint32_t> (nodeCount);
  }

  void Print (std::ostream &os) const override
  {
    os << "snmpSrc=" << m_source
       << " snmpDst=" << m_destination
       << " destType=" << unsigned (m_destinationType)
       << " command=" << unsigned (m_command)
       << " value=" << m_value
       << " nodes=" << m_nodes.size ();
  }

  void SetSource (CsrNodeId source)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (source),
                     "CSR SNMP source exceeds the legacy 24-bit field");
    m_source = source;
  }
  CsrNodeId GetSource () const { return m_source; }

  void SetDestination (CsrNodeId destination)
  {
    NS_ABORT_MSG_IF (!CsrIsValidNodeId (destination),
                     "CSR SNMP destination exceeds the legacy 24-bit field");
    m_destination = destination;
  }
  CsrNodeId GetDestination () const { return m_destination; }

  void SetDestinationType (CsrDestType type)
  {
    m_destinationType = static_cast<uint8_t> (type);
  }
  CsrDestType GetDestinationType () const
  {
    return static_cast<CsrDestType> (m_destinationType);
  }

  void SetCommand (CsrSnmpCommand command)
  {
    m_command = static_cast<uint8_t> (command);
  }
  CsrSnmpCommand GetCommand () const
  {
    return static_cast<CsrSnmpCommand> (m_command);
  }

  void SetValue (int32_t value) { m_value = value; }
  int32_t GetValue () const { return m_value; }

  void SetNodes (const std::vector<CsrNodeId> &nodes)
  {
    for (CsrNodeId node : nodes)
      {
        NS_ABORT_MSG_IF (
          !CsrIsValidNodeId (node),
          "CSR SNMP node identifier exceeds the legacy 24-bit field");
      }
    m_nodes.assign (
      nodes.begin (),
      nodes.begin () + std::min<size_t> (nodes.size (), MAX_NODES));
  }
  const std::vector<CsrNodeId>& GetNodes () const { return m_nodes; }

private:
  CsrNodeId m_source;
  CsrNodeId m_destination;
  uint8_t m_destinationType;
  uint8_t m_command;
  int32_t m_value;
  std::vector<CsrNodeId> m_nodes;
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
