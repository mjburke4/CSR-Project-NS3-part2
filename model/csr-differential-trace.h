#pragma once

#include "csr-wire-format.h"
#include "ns3/simulator.h"
#include "ns3/tag.h"

#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace ns3 {

/**
 * Observation-only application identity carried through Packet::Copy().
 *
 * Packet tags do not add modeled wire bytes.  This tag lets aggregate tools
 * pair an application send with its final delivery even when MAC/HOP retries
 * or positive-DSCP queue ordering change delivery order.
 */
class CsrDifferentialAppTag : public Tag
{
public:
  CsrDifferentialAppTag () = default;
  explicit CsrDifferentialAppTag (uint64_t sequence)
    : m_sequence (sequence)
  {
  }

  static TypeId GetTypeId ()
  {
    static TypeId tid =
      TypeId ("ns3::CsrDifferentialAppTag")
        .SetParent<Tag> ()
        .SetGroupName ("Csr")
        .AddConstructor<CsrDifferentialAppTag> ();
    return tid;
  }

  TypeId GetInstanceTypeId () const override
  {
    return GetTypeId ();
  }

  uint32_t GetSerializedSize () const override
  {
    return 8;
  }

  void Serialize (TagBuffer buffer) const override
  {
    buffer.WriteU64 (m_sequence);
  }

  void Deserialize (TagBuffer buffer) override
  {
    m_sequence = buffer.ReadU64 ();
  }

  void Print (std::ostream &stream) const override
  {
    stream << "sequence=" << m_sequence;
  }

  uint64_t GetSequence () const
  {
    return m_sequence;
  }

private:
  uint64_t m_sequence {0};
};

/** Canonical ordered trace schema consumed by the OPNET/ns-3 comparator. */
struct CsrDifferentialTraceEvent
{
  std::string event;
  std::string node;
  std::string peer;
  std::string packetType;
  std::string source;
  std::string destination;
  std::string sequence;
  std::string rateKbps;
  std::string sizeBytes;
  std::string success;
  std::string reason;
  std::string pathlossDb;
  std::string rxPowerDbm;
  std::string noiseDbm;
  std::string snrDb;
  std::string jsrDb;
  std::string headerErrors;
  std::string payloadErrors;
  std::string totalErrors;
  std::string routeCost;
  std::string nextHop;
  std::string securityCount;
  std::string reservationSlot;
  std::string reservationCounter;
  std::string detail;
  std::string statistic;
  std::string value;
};

inline constexpr const char* CSR_STAT_HOP_RESEND_QUEUE_SIZE =
  "HOP.Resend Queue Size (packets)";
inline constexpr const char* CSR_STAT_MAC_ACK_QUEUE_SIZE =
  "MAC.ACK Queue Size (packets)";
inline constexpr const char* CSR_STAT_MAC_TX_QUEUE_SIZE =
  "MAC.Tx Queue Size (packets)";
inline constexpr const char* CSR_STAT_MAC_TX_QUEUE_DELAY =
  "MAC.Tx Queuing Delay (sec)";
inline constexpr const char* CSR_STAT_NWK_QUEUE_SIZE =
  "NWK.Network Queue Size (packets)";
inline constexpr const char* CSR_STAT_NWK_QUEUE_DELAY =
  "NWK.Network Queuing Delay (sec)";

inline std::ofstream g_csrDifferentialTrace;
inline uint64_t g_csrDifferentialTraceIndex = 0;
inline bool g_csrDifferentialAggregateOnly = false;
inline bool g_csrDifferentialAdmissionTraceEnabled = false;

/** Limit trace output to events consumed by the aggregate-series builder. */
inline void
SetDifferentialTraceAggregateOnly (bool enabled)
{
  g_csrDifferentialAggregateOnly = enabled;
}

/** Enable the opt-in application/NWK/HOP admission ledger. */
inline void
SetDifferentialAdmissionTraceEnabled (bool enabled)
{
  g_csrDifferentialAdmissionTraceEnabled = enabled;
}

/** @return Whether the opt-in admission ledger is enabled. */
inline bool
IsDifferentialAdmissionTraceEnabled ()
{
  return g_csrDifferentialAdmissionTraceEnabled;
}

/** Return whether an event belongs to the frozen admission-ledger surface. */
inline bool
CsrIsDifferentialAdmissionTraceEvent (const std::string &event)
{
  return event == "app_admission" ||
         event == "nwk_admission" ||
         event == "hop_admission" ||
         event == "hop_feedback" ||
         event == "nwk_nsdp_release" ||
         event == "hop_completion" ||
         event == "hop_capacity_release";
}

inline std::string
CsrTraceCsvEscape (const std::string &value)
{
  if (value.find_first_of (",\"\r\n") == std::string::npos)
    {
      return value;
    }
  std::string escaped = "\"";
  for (char character : value)
    {
      if (character == '\"')
        {
          escaped += '\"';
        }
      escaped += character;
    }
  escaped += "\"";
  return escaped;
}

inline std::string
CsrTraceInteger (uint64_t value)
{
  return std::to_string (value);
}

inline std::string
CsrTraceSignedInteger (int64_t value)
{
  return std::to_string (value);
}

inline std::string
CsrTraceDouble (double value)
{
  if (value == std::numeric_limits<double>::infinity ())
    {
      return "inf";
    }
  if (value == -std::numeric_limits<double>::infinity ())
    {
      return "-inf";
    }
  std::ostringstream stream;
  stream << std::setprecision (17) << value;
  return stream.str ();
}

inline void
OpenDifferentialTraceCsv (const std::string &path)
{
  if (g_csrDifferentialTrace.is_open ())
    {
      g_csrDifferentialTrace.close ();
    }
  g_csrDifferentialTrace.open (path, std::ios::out | std::ios::trunc);
  NS_ABORT_MSG_IF (!g_csrDifferentialTrace.is_open (),
                   "cannot open CSR differential trace: " << path);
  g_csrDifferentialTraceIndex = 0;
  if (g_csrDifferentialAggregateOnly)
    {
      g_csrDifferentialTrace
        << "schema,event_index,time_s,event,src,dst,sequence,size_bytes,reason,"
        << "node,statistic,value,peer,next_hop,route_cost,detail\n";
      return;
    }
  g_csrDifferentialTrace
    << "schema,event_index,time_s,event,node,peer,packet_type,src,dst,"
    << "sequence,rate_kbps,size_bytes,success,reason,pathloss_db,"
    << "rx_power_dbm,noise_dbm,snr_db,jsr_db,header_errors,"
    << "payload_errors,total_errors,route_cost,next_hop,security_count,"
    << "reservation_slot,reservation_counter,detail,statistic,value\n";
}

inline void
CloseDifferentialTraceCsv ()
{
  if (g_csrDifferentialTrace.is_open ())
    {
      g_csrDifferentialTrace.close ();
    }
}

inline bool
IsDifferentialTraceOpen ()
{
  return g_csrDifferentialTrace.is_open ();
}

inline void
WriteDifferentialTrace (const CsrDifferentialTraceEvent &event)
{
  if (!g_csrDifferentialTrace.is_open ())
    {
      return;
    }
  if (g_csrDifferentialAggregateOnly &&
      event.event != "app_send" &&
      event.event != "nwk_enqueue" &&
      event.event != "nwk_forward" &&
      event.event != "nwk_delivery" &&
      event.event != "route_change" &&
      event.event != "rx_drop" &&
      event.event != "statistic_sample" &&
      !(g_csrDifferentialAdmissionTraceEnabled &&
        CsrIsDifferentialAdmissionTraceEvent (event.event)))
    {
      return;
    }
  if (g_csrDifferentialAggregateOnly)
    {
      const std::string aggregateFields[] = {
        "csr-differential-trace-v1",
        CsrTraceInteger (g_csrDifferentialTraceIndex++),
        CsrTraceDouble (Simulator::Now ().GetSeconds ()),
        event.event,
        event.source,
        event.destination,
        event.sequence,
        event.sizeBytes,
        event.reason,
        event.node,
        event.statistic,
        event.value,
        event.peer,
        event.nextHop,
        event.routeCost,
        event.detail,
      };
      for (std::size_t index = 0;
           index < sizeof (aggregateFields) / sizeof (aggregateFields[0]);
           ++index)
        {
          if (index != 0)
            {
              g_csrDifferentialTrace << ',';
            }
          g_csrDifferentialTrace << CsrTraceCsvEscape (aggregateFields[index]);
        }
      g_csrDifferentialTrace << '\n';
      return;
    }
  const std::string fields[] = {
    "csr-differential-trace-v1",
    CsrTraceInteger (g_csrDifferentialTraceIndex++),
    CsrTraceDouble (Simulator::Now ().GetSeconds ()),
    event.event,
    event.node,
    event.peer,
    event.packetType,
    event.source,
    event.destination,
    event.sequence,
    event.rateKbps,
    event.sizeBytes,
    event.success,
    event.reason,
    event.pathlossDb,
    event.rxPowerDbm,
    event.noiseDbm,
    event.snrDb,
    event.jsrDb,
    event.headerErrors,
    event.payloadErrors,
    event.totalErrors,
    event.routeCost,
    event.nextHop,
    event.securityCount,
    event.reservationSlot,
    event.reservationCounter,
    event.detail,
    event.statistic,
    event.value,
  };
  for (std::size_t index = 0; index < sizeof (fields) / sizeof (fields[0]); ++index)
    {
      if (index != 0)
        {
          g_csrDifferentialTrace << ',';
        }
      g_csrDifferentialTrace << CsrTraceCsvEscape (fields[index]);
    }
  g_csrDifferentialTrace << '\n';
}

/** Write one admission-ledger event when the opt-in surface is enabled. */
inline void
WriteDifferentialAdmissionTrace (const CsrDifferentialTraceEvent &event)
{
  if (!g_csrDifferentialAdmissionTraceEnabled)
    {
      return;
    }
  NS_ABORT_MSG_IF (
    !CsrIsDifferentialAdmissionTraceEvent (event.event),
    "unsupported CSR admission trace event: " << event.event);
  WriteDifferentialTrace (event);
}

/** Write one source-equivalent discrete statistic sample. */
inline void
WriteDifferentialStatisticSample (uint32_t node,
                                  const char *statistic,
                                  double value)
{
  CsrDifferentialTraceEvent event;
  event.event = "statistic_sample";
  event.node = CsrTraceInteger (node);
  event.statistic = statistic;
  event.value = CsrTraceDouble (value);
  WriteDifferentialTrace (event);
}

} // namespace ns3
