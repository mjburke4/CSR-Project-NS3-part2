#include "ns3/core-module.h"
#include "ns3/csr-common.h"
#include "ns3/csr-hop-layer.h"
#include "ns3/csr-net-device.h"
#include "ns3/csr-nwk-layer.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

using namespace ns3;

namespace
{

constexpr const char *SCENARIO_SCHEMA = "csr-opnet-scenario-v1";

// br_app's configured Packet Size is not the size handed to br_Network.
// It creates a br_Network packet whose total size is configured_size * 8 - 64
// bits.  Keep that eight-byte source-model exclusion in the modeled packet,
// rather than compensating for an oversized packet when statistics are read.
constexpr uint32_t OPNET_APP_SIZE_EXCLUSION_BYTES = 8;

// RngStream's MRG32k3a state requires seed < m2 (4294944443).  The
// RngSeedManager storage type is wider than the generator's runnable domain.
constexpr uint32_t NS3_RNG_SEED_MAX = 4294944442U;

constexpr const char *FIXED_DESTINATION_MODE = "fixed";
constexpr const char *RANDOM_ROUTE_DESTINATION_MODE =
  "random_route_or_neighbor";
constexpr const char *APP_DIAGNOSTICS_SCHEMA =
  "csr-app-admission-diagnostics-v1";
constexpr const char *CURRENT_SEND_ONLY_PROFILE = "current-send-only";
constexpr const char *LEGACY_SEND_ONLY_PROFILE =
  "legacy-send-only-no-dscp";
constexpr const char *LEGACY_SEND_TO_FROM_PROFILE =
  "legacy-send-to-from-no-dscp";
constexpr const char *CURRENT_MAC_PROFILE = "current-fine-free-slot";
constexpr const char *HIST_ZERO_BASED_MAC_PROFILE =
  "hist-2014-zero-based-rebuild-list";
constexpr const char *HIST_FINE_NO_AVOID_MAC_PROFILE =
  "hist-2015-fine-one-based-table-no-avoid";
constexpr const char *HIST_MODULO_PROBE_MAC_PROFILE =
  "hist-2014-next-tslot-modulo-probe";

class NullStreamBuffer : public std::streambuf
{
protected:
  int overflow (int character) override
  {
    return traits_type::eq_int_type (character, traits_type::eof ())
      ? traits_type::not_eof (character)
      : character;
  }
};

struct ImportedNode
{
  CsrNodeId id {0};
  int forcedReservationSlot {-1};
  std::string name;
  std::string type;
  double xMeters {0.0};
  double yMeters {0.0};
  double heightMeters {1.0};
  CsrRateKey minSpeedKbps {8};
  CsrRateKey maxSpeedKbps {128};
  double minPowerDbm {-36.0};
  double maxPowerDbm {33.0};
  double linkMarginDb {10.0};
  double eccThreshold {0.1};
  double rxFrequencyHz {400.0e6};
  double txFrequencyHz {400.0e6};
};

struct ImportedFlow
{
  CsrNodeId source {0};
  CsrNodeId destination {0};
  double startSeconds {0.0};
  double intervalSeconds {1.0};
  uint32_t packetBytes {1};
  uint8_t dscp {0};
  std::string destinationMode {FIXED_DESTINATION_MODE};
};

struct FlowRuntimeState
{
  uint64_t attempts {0};
  uint64_t admitted {0};
  uint64_t blockedDiscovery {0};
  uint64_t blockedTopology {0};
  uint64_t blockedGatewayRoute {0};
  uint64_t blockedDestination {0};
  uint64_t blockedNsdp {0};
  bool gatewayKnown {false};
  double firstAdmittedSeconds {std::numeric_limits<double>::quiet_NaN ()};
  double lastAdmittedSeconds {std::numeric_limits<double>::quiet_NaN ()};
  Ptr<UniformRandomVariable> destinationRandomVariable;
};

struct ImportedScenario
{
  std::string name;
  std::string applicationProfile {"unspecified"};
  std::string macProfile {"unspecified"};
  double durationSeconds {60.0};
  double reservationControlStartSeconds {0.0};
  uint32_t seed {128};
  bool tmm {false};
  std::vector<ImportedNode> nodes;
  std::vector<ImportedFlow> flows;
};

struct RuntimeNode
{
  ImportedNode configuration;
  Ptr<CsrNetDevice> device;
  Ptr<CsrHopLayer> hop;
  Ptr<CsrNetLayer> network;
};

std::vector<std::string>
ParseCsvLine (const std::string &line)
{
  std::vector<std::string> fields;
  std::string field;
  bool quoted = false;
  for (std::size_t index = 0; index < line.size (); ++index)
    {
      char character = line[index];
      if (quoted)
        {
          if (character == '"')
            {
              if (index + 1 < line.size () && line[index + 1] == '"')
                {
                  field += '"';
                  ++index;
                }
              else
                {
                  quoted = false;
                }
            }
          else
            {
              field += character;
            }
        }
      else if (character == '"')
        {
          quoted = true;
        }
      else if (character == ',')
        {
          fields.push_back (field);
          field.clear ();
        }
      else
        {
          field += character;
        }
    }
  NS_ABORT_MSG_IF (quoted, "unterminated quote in scenario CSV");
  fields.push_back (field);
  return fields;
}

double
ParseDouble (const std::map<std::string, std::string> &row,
             const std::string &field)
{
  auto iterator = row.find (field);
  NS_ABORT_MSG_IF (iterator == row.end () || iterator->second.empty (),
                   "scenario row omits " << field);
  std::size_t parsed = 0;
  double value = std::stod (iterator->second, &parsed);
  NS_ABORT_MSG_IF (parsed != iterator->second.size () || !std::isfinite (value),
                   "scenario field " << field << " is not finite");
  return value;
}

uint64_t
ParseUnsigned (const std::map<std::string, std::string> &row,
               const std::string &field)
{
  auto iterator = row.find (field);
  NS_ABORT_MSG_IF (iterator == row.end () || iterator->second.empty (),
                   "scenario row omits " << field);
  std::size_t parsed = 0;
  unsigned long long value = std::stoull (iterator->second, &parsed, 0);
  NS_ABORT_MSG_IF (parsed != iterator->second.size (),
                   "scenario field " << field << " is not an integer");
  return value;
}

std::string
GetField (const std::map<std::string, std::string> &row,
          const std::string &field)
{
  auto iterator = row.find (field);
  return iterator == row.end () ? std::string () : iterator->second;
}

ImportedScenario
LoadScenario (const std::string &path)
{
  std::ifstream stream (path);
  NS_ABORT_MSG_IF (!stream.is_open (), "cannot open canonical scenario " << path);
  std::string line;
  NS_ABORT_MSG_IF (!std::getline (stream, line), "scenario CSV is empty");
  std::vector<std::string> headers = ParseCsvLine (line);
  NS_ABORT_MSG_IF (headers.empty (), "scenario CSV has no columns");

  ImportedScenario scenario;
  bool sawRun = false;
  std::map<CsrNodeId, bool> nodeIds;
  uint32_t lineNumber = 1;
  while (std::getline (stream, line))
    {
      ++lineNumber;
      if (line.empty ())
        {
          continue;
        }
      std::vector<std::string> fields = ParseCsvLine (line);
      NS_ABORT_MSG_IF (fields.size () != headers.size (),
                       "scenario CSV line " << lineNumber
                       << " has " << fields.size () << " fields; expected "
                       << headers.size ());
      std::map<std::string, std::string> row;
      for (std::size_t index = 0; index < headers.size (); ++index)
        {
          row[headers[index]] = fields[index];
        }
      NS_ABORT_MSG_IF (GetField (row, "schema") != SCENARIO_SCHEMA,
                       "scenario CSV line " << lineNumber
                       << " has an unsupported schema");
      const std::string record = GetField (row, "record");
      if (record == "run")
        {
          NS_ABORT_MSG_IF (sawRun, "scenario CSV contains multiple run rows");
          sawRun = true;
          scenario.name = GetField (row, "scenario");
          if (!GetField (row, "application_profile").empty ())
            {
              scenario.applicationProfile =
                GetField (row, "application_profile");
            }
          if (!GetField (row, "mac_profile").empty ())
            {
              scenario.macProfile = GetField (row, "mac_profile");
            }
          scenario.durationSeconds = ParseDouble (row, "duration_s");
          const uint64_t seed = ParseUnsigned (row, "seed");
          NS_ABORT_MSG_IF (
            seed < 1 || seed > NS3_RNG_SEED_MAX,
            "scenario seed must be in 1..4294944442");
          scenario.seed = static_cast<uint32_t> (seed);
          scenario.tmm = ParseUnsigned (row, "tmm") != 0;
          if (!GetField (row, "reservation_control_start_s").empty ())
            {
              scenario.reservationControlStartSeconds = ParseDouble (
                row, "reservation_control_start_s");
            }
        }
      else if (record == "node")
        {
          ImportedNode node;
          uint64_t nodeId = ParseUnsigned (row, "node_id");
          NS_ABORT_MSG_IF (nodeId > CSR_NODE_ID_MAX,
                           "scenario node ID exceeds 24 bits");
          node.id = static_cast<CsrNodeId> (nodeId);
          NS_ABORT_MSG_IF (nodeIds.find (node.id) != nodeIds.end (),
                           "scenario contains duplicate node ID " << node.id);
          nodeIds[node.id] = true;
          if (!GetField (row, "forced_reservation_slot").empty ())
            {
              uint64_t forcedSlot = ParseUnsigned (
                row, "forced_reservation_slot");
              NS_ABORT_MSG_IF (forcedSlot < 1 || forcedSlot > 255,
                               "forced reservation slot must be in 1..255");
              node.forcedReservationSlot = static_cast<int> (forcedSlot);
            }
          node.name = GetField (row, "name");
          node.type = GetField (row, "node_type");
          node.xMeters = ParseDouble (row, "x_m");
          node.yMeters = ParseDouble (row, "y_m");
          node.heightMeters = ParseDouble (row, "height_m");
          const uint64_t minSpeedKbps =
            ParseUnsigned (row, "min_speed_kbps");
          const uint64_t maxSpeedKbps =
            ParseUnsigned (row, "max_speed_kbps");
          NS_ABORT_MSG_IF (
            minSpeedKbps > std::numeric_limits<CsrRateKey>::max () ||
              maxSpeedKbps > std::numeric_limits<CsrRateKey>::max (),
            "scenario link-control speed exceeds the wire-format range");
          node.minSpeedKbps = static_cast<CsrRateKey> (minSpeedKbps);
          node.maxSpeedKbps = static_cast<CsrRateKey> (maxSpeedKbps);
          NS_ABORT_MSG_IF (
            !CsrIsOperationalRateKey (node.minSpeedKbps) ||
              !CsrIsOperationalRateKey (node.maxSpeedKbps) ||
              node.minSpeedKbps > node.maxSpeedKbps,
            "scenario link-control speed range is invalid");
          node.minPowerDbm = ParseDouble (row, "min_power_dbm");
          node.maxPowerDbm = ParseDouble (row, "max_power_dbm");
          node.linkMarginDb = ParseDouble (row, "link_margin_db");
          node.eccThreshold = ParseDouble (row, "ecc_threshold");
          node.rxFrequencyHz = ParseDouble (row, "rx_frequency_hz");
          node.txFrequencyHz = ParseDouble (row, "tx_frequency_hz");
          scenario.nodes.push_back (std::move (node));
        }
      else if (record == "flow")
        {
          ImportedFlow flow;
          uint64_t source = ParseUnsigned (row, "flow_src");
          uint64_t destination = ParseUnsigned (row, "flow_dst");
          NS_ABORT_MSG_IF (source > CSR_NODE_ID_MAX ||
                           destination > CSR_NODE_ID_MAX,
                           "scenario flow endpoint exceeds 24 bits");
          flow.source = static_cast<CsrNodeId> (source);
          flow.destination = static_cast<CsrNodeId> (destination);
          flow.startSeconds = ParseDouble (row, "flow_start_s");
          flow.intervalSeconds = ParseDouble (row, "flow_interval_s");
          const uint64_t packetBytes =
            ParseUnsigned (row, "flow_packet_bytes");
          const uint64_t dscp = ParseUnsigned (row, "flow_dscp");
          NS_ABORT_MSG_IF (
            packetBytes > std::numeric_limits<uint32_t>::max (),
            "scenario flow_packet_bytes exceeds the uint32 packet-size limit");
          NS_ABORT_MSG_IF (dscp > 7,
                           "scenario flow_dscp must be in 0..7");
          const uint32_t minimumConfiguredBytes =
            OPNET_APP_SIZE_EXCLUSION_BYTES +
            CsrNetHeader ().GetSerializedSize ();
          NS_ABORT_MSG_IF (
            packetBytes < minimumConfiguredBytes,
            "scenario flow_packet_bytes must be at least "
              << minimumConfiguredBytes
              << " bytes (8-byte br_app exclusion plus the CSR NWK header)");
          flow.packetBytes = static_cast<uint32_t> (packetBytes);
          flow.dscp = static_cast<uint8_t> (dscp);
          const std::string destinationMode =
            GetField (row, "flow_destination_mode");
          if (!destinationMode.empty ())
            {
              flow.destinationMode = destinationMode;
            }
          NS_ABORT_MSG_IF (
            flow.destinationMode != FIXED_DESTINATION_MODE &&
              flow.destinationMode != RANDOM_ROUTE_DESTINATION_MODE,
            "scenario flow_destination_mode must be fixed or "
              "random_route_or_neighbor");
          NS_ABORT_MSG_IF (
            flow.destinationMode == RANDOM_ROUTE_DESTINATION_MODE &&
              flow.source != flow.destination,
            "random_route_or_neighbor flow must use its source as the "
              "placeholder flow_dst");
          NS_ABORT_MSG_IF (flow.startSeconds < 0.0 ||
                           flow.intervalSeconds <= 0.0,
                           "scenario flow contains an invalid value");
          scenario.flows.push_back (flow);
        }
      else
        {
          NS_ABORT_MSG ("scenario CSV line " << lineNumber
                        << " has unknown record type " << record);
        }
    }

  NS_ABORT_MSG_IF (!sawRun, "scenario CSV has no run row");
  NS_ABORT_MSG_IF (scenario.durationSeconds <= 0.0,
                   "scenario duration must be positive");
  NS_ABORT_MSG_IF (scenario.reservationControlStartSeconds < 0.0 ||
                   scenario.reservationControlStartSeconds >
                     scenario.durationSeconds,
                   "reservation control start must be within the run");
  NS_ABORT_MSG_IF (scenario.nodes.empty (), "scenario CSV has no nodes");
  std::sort (scenario.nodes.begin (), scenario.nodes.end (),
             [] (const ImportedNode &first, const ImportedNode &second) {
               return first.id < second.id;
             });
  for (const ImportedFlow &flow : scenario.flows)
    {
      NS_ABORT_MSG_IF (nodeIds.find (flow.source) == nodeIds.end () ||
                       nodeIds.find (flow.destination) == nodeIds.end (),
                       "scenario flow references an unknown node");
    }
  NS_ABORT_MSG_IF (
    scenario.applicationProfile != "unspecified" &&
      scenario.applicationProfile != CURRENT_SEND_ONLY_PROFILE &&
      scenario.applicationProfile != LEGACY_SEND_ONLY_PROFILE &&
      scenario.applicationProfile != LEGACY_SEND_TO_FROM_PROFILE,
    "scenario run row has an unsupported application_profile");
  NS_ABORT_MSG_IF (
    scenario.macProfile != "unspecified" &&
      scenario.macProfile != CURRENT_MAC_PROFILE &&
      scenario.macProfile != HIST_ZERO_BASED_MAC_PROFILE &&
      scenario.macProfile != HIST_FINE_NO_AVOID_MAC_PROFILE &&
      scenario.macProfile != HIST_MODULO_PROBE_MAC_PROFILE,
    "scenario run row has an unsupported mac_profile");
  if (scenario.applicationProfile == LEGACY_SEND_ONLY_PROFILE ||
      scenario.applicationProfile == LEGACY_SEND_TO_FROM_PROFILE)
    {
      std::vector<CsrNodeId> gateways;
      for (const ImportedNode &node : scenario.nodes)
        {
          if (node.type == "gateway")
            {
              gateways.push_back (node.id);
            }
        }
      NS_ABORT_MSG_IF (
        gateways.size () != 1,
        "legacy application profiles require exactly one gateway");
      const CsrNodeId gateway = gateways.front ();
      uint32_t dynamicGatewayFlows = 0;
      for (const ImportedFlow &flow : scenario.flows)
        {
          NS_ABORT_MSG_IF (
            flow.dscp != 0,
            "legacy no-DSCP application profiles require flow_dscp=0");
          if (flow.destinationMode == RANDOM_ROUTE_DESTINATION_MODE)
            {
              dynamicGatewayFlows++;
              NS_ABORT_MSG_IF (
                flow.source != gateway,
                "legacy dynamic destination flow must originate at the gateway");
            }
          else
            {
              NS_ABORT_MSG_IF (
                flow.source == gateway || flow.destination != gateway,
                "legacy fixed flows must originate outside and terminate at "
                  "the gateway");
            }
        }
      const uint32_t expectedDynamic =
        scenario.applicationProfile == LEGACY_SEND_TO_FROM_PROFILE ? 1 : 0;
      NS_ABORT_MSG_IF (
        dynamicGatewayFlows != expectedDynamic,
        "legacy application profile has the wrong gateway-origin flow count");
    }
  return scenario;
}

CsrNodeType
ParseNodeType (const std::string &type)
{
  if (type == "ordinary")
    {
      return CsrNodeType::Ordinary;
    }
  if (type == "routable")
    {
      return CsrNodeType::Routable;
    }
  if (type == "gateway")
    {
      return CsrNodeType::Gateway;
    }
  NS_ABORT_MSG ("unsupported imported CSR node type " << type);
  return CsrNodeType::Ordinary;
}

CsrMacCore::SlotSelectionProfile
ParseMacProfile (const std::string &profile)
{
  if (profile == "unspecified" || profile == CURRENT_MAC_PROFILE)
    {
      return CsrMacCore::SlotSelectionProfile::NS3_CURRENT_FINE_FREE_SLOT;
    }
  if (profile == HIST_ZERO_BASED_MAC_PROFILE)
    {
      return CsrMacCore::SlotSelectionProfile::
        HIST_2014_ZERO_BASED_REBUILD_LIST;
    }
  if (profile == HIST_FINE_NO_AVOID_MAC_PROFILE)
    {
      return CsrMacCore::SlotSelectionProfile::
        HIST_2015_FINE_ONE_BASED_TABLE_NO_AVOID;
    }
  if (profile == HIST_MODULO_PROBE_MAC_PROFILE)
    {
      return CsrMacCore::SlotSelectionProfile::
        HIST_2014_NEXT_TSLOT_MODULO_PROBE;
    }
  NS_ABORT_MSG ("unsupported imported MAC profile " << profile);
  return CsrMacCore::SlotSelectionProfile::NS3_CURRENT_FINE_FREE_SLOT;
}

void
ConnectStack (RuntimeNode &node)
{
  node.hop->SetNodeId (node.configuration.id);
  node.hop->SetMac (&node.device->GetMac ());
  node.network->SetNodeId (node.configuration.id);
  node.network->SetHop (node.hop);
  node.network->SetNodeType (ParseNodeType (node.configuration.type));
  node.network->ConfigureLinkControl (
    node.configuration.minSpeedKbps,
    node.configuration.maxSpeedKbps,
    node.configuration.minPowerDbm,
    node.configuration.maxPowerDbm,
    node.configuration.linkMarginDb);
  node.device->GetMac ().SetRxCallback (
    MakeCallback (&CsrHopLayer::ReceiveFromMac, node.hop));
}

void
EnableReservationControl (Ptr<CsrNetDevice> device, int slot)
{
  device->GetMac ().SetReservationSlotOverrideForDifferentialRun (slot);
  CsrDifferentialTraceEvent event;
  event.event = "reservation_control";
  event.node = CsrTraceInteger (device->GetId ());
  event.reservationSlot = CsrTraceSignedInteger (slot);
  event.detail = "enabled";
  WriteDifferentialTrace (event);
}

void
RecordScenarioNode (const RuntimeNode &node)
{
  CsrDifferentialTraceEvent event;
  event.event = "scenario_node";
  event.node = CsrTraceInteger (node.configuration.id);
  event.rateKbps = CsrTraceInteger (node.configuration.maxSpeedKbps);
  event.detail = node.configuration.name + ":" + node.configuration.type;
  WriteDifferentialTrace (event);
}

void
RecordScenarioLink (const RuntimeNode &first,
                    const RuntimeNode &second,
                    double distanceMeters)
{
  CsrDifferentialTraceEvent event;
  event.event = "scenario_link";
  event.node = CsrTraceInteger (first.configuration.id);
  event.peer = CsrTraceInteger (second.configuration.id);
  event.detail = CsrTraceDouble (distanceMeters);
  WriteDifferentialTrace (event);
}

void
SendFlowPacket (Ptr<CsrNetLayer> network,
                ImportedFlow flow,
                double stopSeconds,
                uint64_t flowLimit,
                bool opnetAppGating,
                std::shared_ptr<FlowRuntimeState> state)
{
  if (flowLimit != 0 && state->admitted >= flowLimit)
    {
      return;
    }

  // br_app schedules its next generator interrupt before applying these
  // gates.  A discovery/no-route attempt is therefore not traffic generated
  // and does not stop later attempts from occurring.
  double next = Simulator::Now ().GetSeconds () + flow.intervalSeconds;
  if (next <= stopSeconds)
    {
      Simulator::Schedule (Seconds (flow.intervalSeconds),
                           &SendFlowPacket,
                           network,
                           flow,
                           stopSeconds,
                           flowLimit,
                           opnetAppGating,
                           state);
    }

  state->attempts++;
  bool admitted = true;
  CsrNodeId destination = flow.destination;
  if (opnetAppGating && network->IsDiscoveryActive ())
    {
      state->blockedDiscovery++;
      admitted = false;
    }
  else if (opnetAppGating && !network->HasApplicationTopologyKnowledge ())
    {
      state->blockedTopology++;
      admitted = false;
    }

  if (admitted && flow.destinationMode == RANDOM_ROUTE_DESTINATION_MODE)
    {
      std::vector<CsrNodeId> candidates =
        network->GetApplicationDestinationCandidates ();
      if (candidates.empty ())
        {
          state->blockedDestination++;
          admitted = false;
        }
      else
        {
          NS_ABORT_MSG_IF (state->destinationRandomVariable == nullptr,
                           "random gateway flow has no random variable");
          double outcome = state->destinationRandomVariable->GetValue (
            0.0, static_cast<double> (candidates.size ()));
          std::size_t index = std::min<std::size_t> (
            static_cast<std::size_t> (outcome), candidates.size () - 1);
          destination = candidates[index];
        }
    }
  else if (admitted && opnetAppGating && !state->gatewayKnown)
    {
      if (network->HasRelayRoute (destination))
        {
          // Historical br_app caches gateway_node_id after the first route
          // lookup.  Later application attempts do not repeat a stricter ARL
          // validity/freshness check; NWK queues packets if forwarding stalls.
          state->gatewayKnown = true;
        }
      else
        {
          state->blockedGatewayRoute++;
          admitted = false;
        }
    }

  if (admitted && opnetAppGating &&
      !network->CanAdmitApplicationPacket (destination))
    {
      state->blockedNsdp++;
      admitted = false;
    }
  if (admitted)
    {
      state->admitted++;
      const double now = Simulator::Now ().GetSeconds ();
      if (!std::isfinite (state->firstAdmittedSeconds))
        {
          state->firstAdmittedSeconds = now;
        }
      state->lastAdmittedSeconds = now;
      // The recovered br_app creates a total br_Network packet of
      // configured_size * 8 - 64 bits.  CsrNetLayer::Send adds our seven-byte
      // CsrNetHeader, so subtract both that header and br_app's eight-byte
      // source-model exclusion from the configured size here.  This makes the
      // packet carried by NWK exactly flow.packetBytes - 8 bytes on the wire.
      const uint32_t networkPacketBytes =
        flow.packetBytes - OPNET_APP_SIZE_EXCLUSION_BYTES;
      const uint32_t networkHeaderBytes =
        CsrNetHeader ().GetSerializedSize ();
      NS_ABORT_MSG_IF (networkPacketBytes < networkHeaderBytes,
                       "configured flow packet is too small for CSR NWK");
      Ptr<Packet> payload =
        Create<Packet> (networkPacketBytes - networkHeaderBytes);
      CsrDifferentialAppTag appTag (payload->GetUid ());
      payload->AddPacketTag (appTag);
      CsrDifferentialTraceEvent event;
      event.event = "app_send";
      event.node = CsrTraceInteger (flow.source);
      event.peer = CsrTraceInteger (destination);
      event.packetType = "data";
      event.source = CsrTraceInteger (flow.source);
      event.destination = CsrTraceInteger (destination);
      // Packet UIDs are observation-only correlation keys.  Packet::Copy()
      // preserves them through the CSR stack, allowing exact delay matching
      // without adding bytes to the modeled packet.
      event.sequence = CsrTraceInteger (appTag.GetSequence ());
      // app_send and nwk_delivery both report the actual total br_Network
      // packet size, including the NWK header and excluding br_app's 64 bits.
      event.sizeBytes = CsrTraceInteger (networkPacketBytes);
      event.detail = "dscp=" + CsrTraceInteger (flow.dscp);
      WriteDifferentialTrace (event);
      network->Send (destination, flow.dscp, payload, true);
    }
}

void
WriteApplicationDiagnostics (
  const std::string &path,
  const ImportedScenario &scenario,
  const std::vector<std::shared_ptr<FlowRuntimeState>> &states)
{
  if (path.empty ())
    {
      return;
    }
  NS_ABORT_MSG_IF (states.size () != scenario.flows.size (),
                   "application diagnostics flow/state count mismatch");
  std::ofstream stream (path, std::ios::out | std::ios::trunc);
  NS_ABORT_MSG_IF (!stream.is_open (),
                   "cannot open application diagnostics: " << path);
  stream
    << "schema,scenario,application_profile,flow_index,source,"
    << "configured_destination,"
    << "destination_mode,attempts,admitted,blocked_discovery,"
    << "blocked_topology,blocked_gateway_route,blocked_destination,"
    << "blocked_nsdp,first_admitted_s,last_admitted_s\n";
  for (std::size_t index = 0; index < scenario.flows.size (); ++index)
    {
      const ImportedFlow &flow = scenario.flows[index];
      const FlowRuntimeState &state = *states[index];
      stream << APP_DIAGNOSTICS_SCHEMA << ','
             << CsrTraceCsvEscape (scenario.name) << ','
             << CsrTraceCsvEscape (scenario.applicationProfile) << ','
             << index << ','
             << flow.source << ','
             << flow.destination << ','
             << flow.destinationMode << ','
             << state.attempts << ','
             << state.admitted << ','
             << state.blockedDiscovery << ','
             << state.blockedTopology << ','
             << state.blockedGatewayRoute << ','
             << state.blockedDestination << ','
             << state.blockedNsdp << ',';
      if (std::isfinite (state.firstAdmittedSeconds))
        {
          stream << CsrTraceDouble (state.firstAdmittedSeconds);
        }
      stream << ',';
      if (std::isfinite (state.lastAdmittedSeconds))
        {
          stream << CsrTraceDouble (state.lastAdmittedSeconds);
        }
      stream << '\n';
    }
}

} // namespace

int
main (int argc, char *argv[])
{
  Time::SetResolution (Time::NS);
  std::string scenarioPath;
  std::string tracePath = "csr-differential-ns3.csv";
  std::string appDiagnosticsPath;
  double stopSeconds = 0.0;
  bool dutyCycling = true;
  bool opnetAlignedDutyCycle = true;
  bool gatewayDiscovery = true;
  bool opnetAppGating = true;
  bool aggregateTraceOnly = false;
  bool quietModelLogs = false;
  uint64_t flowLimit = 0;

  CommandLine command (__FILE__);
  command.AddValue ("scenario", "Canonical scenario CSV", scenarioPath);
  command.AddValue ("trace", "Canonical ns-3 event CSV", tracePath);
  command.AddValue (
    "appDiagnostics",
    "Optional compact application admission diagnostics CSV",
    appDiagnosticsPath);
  command.AddValue ("stop", "Stop time in seconds (0 uses imported duration)", stopSeconds);
  command.AddValue ("dutyCycling", "Enable OPNET duty cycling", dutyCycling);
  command.AddValue (
    "opnetAlignedDutyCycle",
    "Use common phase zero and first WAKE at 0.988 s",
    opnetAlignedDutyCycle);
  command.AddValue ("gatewayDiscovery", "Schedule gateway startup discovery", gatewayDiscovery);
  command.AddValue (
    "opnetAppGating",
    "Suppress generated traffic during discovery or without a route",
    opnetAppGating);
  command.AddValue (
    "aggregateTraceOnly",
    "Write only app/delivery/drop events needed for aggregate comparison",
    aggregateTraceOnly);
  command.AddValue (
    "quietModelLogs",
    "Suppress per-event model stdout while retaining the final summary",
    quietModelLogs);
  command.AddValue ("flowLimit", "Maximum packets per flow (0 is unlimited)", flowLimit);
  command.Parse (argc, argv);

  NS_ABORT_MSG_IF (scenarioPath.empty (), "--scenario is required");
  ImportedScenario scenario = LoadScenario (scenarioPath);
  NS_ABORT_MSG_IF (scenario.tmm,
                   "imported TMM terrain scenarios are not supported yet");
  if (stopSeconds == 0.0)
    {
      stopSeconds = scenario.durationSeconds;
    }
  NS_ABORT_MSG_IF (!std::isfinite (stopSeconds) || stopSeconds <= 0.0,
                   "scenario stop time must be positive");
  RngSeedManager::SetSeed (scenario.seed);
  RngSeedManager::SetRun (1);
  SetDifferentialTraceAggregateOnly (aggregateTraceOnly);
  OpenDifferentialTraceCsv (tracePath);

  NullStreamBuffer nullStreamBuffer;
  std::streambuf *originalCoutBuffer = nullptr;
  if (quietModelLogs)
    {
      originalCoutBuffer = std::cout.rdbuf (&nullStreamBuffer);
    }

  std::map<CsrNodeId, RuntimeNode> nodes;
  for (const ImportedNode &configuration : scenario.nodes)
    {
      RuntimeNode runtime;
      runtime.configuration = configuration;
      runtime.device = CreateObject<CsrNetDevice> (configuration.id);
      runtime.device->GetMac ().SetSlotSelectionProfile (
        ParseMacProfile (scenario.macProfile));
      if (configuration.forcedReservationSlot > 0)
        {
          Simulator::Schedule (
            Seconds (scenario.reservationControlStartSeconds),
            &EnableReservationControl,
            runtime.device,
            configuration.forcedReservationSlot);
        }
      runtime.hop = CreateObject<CsrHopLayer> ();
      runtime.network = CreateObject<CsrNetLayer> ();

      CsrPhyProfile profile;
      profile.txPowerDbm = configuration.maxPowerDbm;
      profile.txBaseFrequencyHz = configuration.txFrequencyHz;
      profile.rxBaseFrequencyHz = configuration.rxFrequencyHz;
      profile.txBwHz = 1.0e6;
      profile.rxBwHz = 1.0e6;
      profile.txHeightMeters = configuration.heightMeters;
      profile.rxHeightMeters = configuration.heightMeters;
      profile.eccThreshold = configuration.eccThreshold;
      profile.propagationModel = CsrPropagationModel::OPNET_THREE_PATH;
      runtime.device->GetPhy ().SetProfile (profile);
      if (dutyCycling && opnetAlignedDutyCycle)
        {
          runtime.device->EnableOpnetAlignedDutyCycling (true);
        }
      else
        {
          runtime.device->EnableDutyCycling (dutyCycling);
        }
      ConnectStack (runtime);
      RecordScenarioNode (runtime);
      nodes.emplace (configuration.id, std::move (runtime));
    }

  for (auto first = nodes.begin (); first != nodes.end (); ++first)
    {
      for (auto second = std::next (first); second != nodes.end (); ++second)
        {
          first->second.device->AddPeer (second->second.device);
          second->second.device->AddPeer (first->second.device);
          double dx = first->second.configuration.xMeters -
                      second->second.configuration.xMeters;
          double dy = first->second.configuration.yMeters -
                      second->second.configuration.yMeters;
          double distance = std::hypot (dx, dy);
          for (auto &entry : nodes)
            {
              entry.second.device->GetPhy ().SetLinkDistanceMeters (
                first->first, second->first, distance);
            }
          RecordScenarioLink (first->second, second->second, distance);
        }
    }

  if (gatewayDiscovery)
    {
      for (auto &entry : nodes)
        {
          if (entry.second.network->GetNodeType () == CsrNodeType::Gateway)
            {
              entry.second.network->ScheduleGatewayStartupDiscovery (
                Seconds (10.0), Seconds (30.0));
            }
        }
    }

  std::vector<std::shared_ptr<FlowRuntimeState>> flowStates;
  flowStates.reserve (scenario.flows.size ());
  for (const ImportedFlow &flow : scenario.flows)
    {
      auto state = std::make_shared<FlowRuntimeState> ();
      if (flow.destinationMode == RANDOM_ROUTE_DESTINATION_MODE)
        {
          state->destinationRandomVariable =
            CreateObject<UniformRandomVariable> ();
        }
      flowStates.push_back (state);
      if (flow.startSeconds > stopSeconds)
        {
          continue;
        }
      Simulator::Schedule (Seconds (flow.startSeconds),
                           &SendFlowPacket,
                           nodes.at (flow.source).network,
                           flow,
                           stopSeconds,
                           flowLimit,
                           opnetAppGating,
                           state);
    }

  Simulator::Stop (Seconds (stopSeconds));
  Simulator::Run ();
  WriteApplicationDiagnostics (appDiagnosticsPath, scenario, flowStates);
  Simulator::Destroy ();
  CloseDifferentialTraceCsv ();
  if (originalCoutBuffer != nullptr)
    {
      std::cout.rdbuf (originalCoutBuffer);
    }
  std::cout << "CSR differential scenario complete: " << scenario.name
            << " nodes=" << nodes.size ()
            << " flows=" << scenario.flows.size ()
            << " stop=" << stopSeconds
            << " trace=" << tracePath
            << std::endl;
  return 0;
}
