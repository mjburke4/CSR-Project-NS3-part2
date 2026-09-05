#ifndef CSR_OPNET_TRACE_H
#define CSR_OPNET_TRACE_H

/*
 * Header-only CSV instrumentation for the recovered Modeler 17.1 sources.
 * Every translation unit appends and flushes independently because OPNET
 * pipeline stages and process models are compiled as separate sources.
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef struct CsrOpnetTraceEvent
{
    double time_s;
    const char *event;
    int node;
    int has_peer;
    int peer;
    const char *packet_type;
    int has_src;
    int src;
    int has_dst;
    int dst;
    int has_sequence;
    int sequence;
    int has_rate;
    int rate_kbps;
    int has_size;
    int size_bytes;
    int has_success;
    int success;
    const char *reason;
    int has_pathloss;
    double pathloss_db;
    int has_rx_power;
    double rx_power_dbm;
    int has_noise;
    double noise_dbm;
    int has_snr;
    double snr_db;
    int has_jsr;
    double jsr_db;
    int has_header_errors;
    int header_errors;
    int has_payload_errors;
    int payload_errors;
    int has_total_errors;
    int total_errors;
    int has_reservation_slot;
    int reservation_slot;
    int has_reservation_counter;
    int reservation_counter;
    const char *detail;
} CsrOpnetTraceEvent;

static FILE *csr_opnet_trace_file = OPC_NIL;
static int csr_opnet_trace_reset_done = 0;

static const char *
csr_opnet_trace_path (void)
{
    const char *path = getenv ("CSR_OPNET_TRACE");
    if (path == OPC_NIL || path[0] == '\0')
        return "csr-opnet-reservation-collision.csv";
    return path;
}

static void
csr_opnet_trace_write_header (FILE *stream)
{
    fprintf (stream,
        "schema,event_index,time_s,event,node,peer,packet_type,src,dst,"
        "sequence,rate_kbps,size_bytes,success,reason,pathloss_db,"
        "rx_power_dbm,noise_dbm,snr_db,jsr_db,header_errors,"
        "payload_errors,total_errors,route_cost,next_hop,security_count,"
        "reservation_slot,reservation_counter,detail\n");
}

static void
csr_opnet_trace_reset_once (void)
{
    FILE *stream;
    if (csr_opnet_trace_reset_done)
        return;
    stream = fopen (csr_opnet_trace_path (), "w");
    if (stream == OPC_NIL)
        {
        fprintf (stderr, "CSR trace: cannot reset %s\n", csr_opnet_trace_path ());
        return;
        }
    csr_opnet_trace_write_header (stream);
    fclose (stream);
    csr_opnet_trace_reset_done = 1;
}

static FILE *
csr_opnet_trace_open (void)
{
    FILE *probe;
    int has_content = 0;
    if (csr_opnet_trace_file != OPC_NIL)
        return csr_opnet_trace_file;
    probe = fopen (csr_opnet_trace_path (), "r");
    if (probe != OPC_NIL)
        {
        has_content = fgetc (probe) != EOF;
        fclose (probe);
        }
    csr_opnet_trace_file = fopen (csr_opnet_trace_path (), "a");
    if (csr_opnet_trace_file == OPC_NIL)
        {
        fprintf (stderr, "CSR trace: cannot open %s\n", csr_opnet_trace_path ());
        return OPC_NIL;
        }
    if (!has_content)
        csr_opnet_trace_write_header (csr_opnet_trace_file);
    return csr_opnet_trace_file;
}

static void
csr_opnet_trace_csv_text (FILE *stream, const char *value)
{
    const char *cursor;
    int quote = 0;
    if (value == OPC_NIL)
        return;
    for (cursor = value; *cursor != '\0'; ++cursor)
        if (*cursor == ',' || *cursor == '"' || *cursor == '\r' || *cursor == '\n')
            quote = 1;
    if (!quote)
        {
        fputs (value, stream);
        return;
        }
    fputc ('"', stream);
    for (cursor = value; *cursor != '\0'; ++cursor)
        {
        if (*cursor == '"')
            fputc ('"', stream);
        fputc (*cursor, stream);
        }
    fputc ('"', stream);
}

static void
csr_opnet_trace_optional_int (FILE *stream, int present, int value)
{
    if (present)
        fprintf (stream, "%d", value);
    fputc (',', stream);
}

static void
csr_opnet_trace_optional_double (FILE *stream, int present, double value)
{
    if (present)
        fprintf (stream, "%.17g", value);
    fputc (',', stream);
}

static void
csr_opnet_trace_emit (CsrOpnetTraceEvent *event)
{
    FILE *stream = csr_opnet_trace_open ();
    if (stream == OPC_NIL)
        return;
    fprintf (stream, "csr-differential-trace-v1,,%.17g,", event->time_s);
    csr_opnet_trace_csv_text (stream, event->event);
    fprintf (stream, ",%d,", event->node);
    csr_opnet_trace_optional_int (stream, event->has_peer, event->peer);
    csr_opnet_trace_csv_text (stream, event->packet_type);
    fputc (',', stream);
    csr_opnet_trace_optional_int (stream, event->has_src, event->src);
    csr_opnet_trace_optional_int (stream, event->has_dst, event->dst);
    csr_opnet_trace_optional_int (stream, event->has_sequence, event->sequence);
    csr_opnet_trace_optional_int (stream, event->has_rate, event->rate_kbps);
    csr_opnet_trace_optional_int (stream, event->has_size, event->size_bytes);
    csr_opnet_trace_optional_int (stream, event->has_success, event->success);
    csr_opnet_trace_csv_text (stream, event->reason);
    fputc (',', stream);
    csr_opnet_trace_optional_double (stream, event->has_pathloss, event->pathloss_db);
    csr_opnet_trace_optional_double (stream, event->has_rx_power, event->rx_power_dbm);
    csr_opnet_trace_optional_double (stream, event->has_noise, event->noise_dbm);
    csr_opnet_trace_optional_double (stream, event->has_snr, event->snr_db);
    csr_opnet_trace_optional_double (stream, event->has_jsr, event->jsr_db);
    csr_opnet_trace_optional_int (
        stream, event->has_header_errors, event->header_errors);
    csr_opnet_trace_optional_int (
        stream, event->has_payload_errors, event->payload_errors);
    csr_opnet_trace_optional_int (
        stream, event->has_total_errors, event->total_errors);
    fputs (",,,", stream); /* route_cost,next_hop,security_count */
    csr_opnet_trace_optional_int (
        stream, event->has_reservation_slot, event->reservation_slot);
    csr_opnet_trace_optional_int (
        stream, event->has_reservation_counter, event->reservation_counter);
    csr_opnet_trace_csv_text (stream, event->detail);
    fputc ('\n', stream);
    fflush (stream);
}

static void
csr_opnet_trace_clear (CsrOpnetTraceEvent *event)
{
    memset (event, 0, sizeof (*event));
}

#if defined (CSR_OPNET_TRACE_APP)
static void
csr_opnet_trace_app_send (
    double time_s, int node, int destination, int size_bytes, int dscp)
{
    CsrOpnetTraceEvent event;
    char detail[32];
    csr_opnet_trace_clear (&event);
    event.time_s = time_s;
    event.event = "app_send";
    event.node = node;
    event.has_peer = 1;
    event.peer = destination;
    event.packet_type = "data";
    event.has_src = 1;
    event.src = node;
    event.has_dst = 1;
    event.dst = destination;
    event.has_size = 1;
    event.size_bytes = size_bytes;
    sprintf (detail, "dscp=%d", dscp);
    event.detail = detail;
    csr_opnet_trace_emit (&event);
}
#endif

#if defined (CSR_OPNET_TRACE_MAC)
static const double csr_opnet_control_start_s =
    /* CSR_CONTROL_START_SECONDS */;

static double
csr_opnet_controlled_slot_epoch (
    double time_s, double holdoff_s, double slot_s)
{
    const double epoch_s = 0.1;
    return ceil ((time_s + holdoff_s + slot_s) / epoch_s) * epoch_s;
}

static int
csr_opnet_forced_slot (int node, int fallback)
{
    if (op_sim_time () < csr_opnet_control_start_s)
        return fallback;
    switch (node)
        {
        /* CSR_FORCE_SLOT_CASES */
        default: return fallback;
        }
}

static int
csr_opnet_has_forced_slot (int node)
{
    return csr_opnet_forced_slot (node, -1) > 0;
}

static void
csr_opnet_trace_reservation (
    double time_s, const char *name, int node, int slot, int counter,
    const char *reason)
{
    CsrOpnetTraceEvent event;
    csr_opnet_trace_clear (&event);
    event.time_s = time_s;
    event.event = name;
    event.node = node;
    event.reason = reason;
    event.has_reservation_slot = 1;
    event.reservation_slot = slot;
    event.has_reservation_counter = 1;
    event.reservation_counter = counter;
    csr_opnet_trace_emit (&event);
}

static void
csr_opnet_trace_tx_start (
    double time_s, int node, int destination, int sequence, int rate_kbps,
    int size_bytes, const char *preamble, int next_slot)
{
    CsrOpnetTraceEvent event;
    char detail[32];
    csr_opnet_trace_clear (&event);
    event.time_s = time_s;
    event.event = "tx_start";
    event.node = node;
    event.has_peer = 1;
    event.peer = destination;
    event.has_src = 1;
    event.src = node;
    event.has_dst = 1;
    event.dst = destination;
    event.has_sequence = 1;
    event.sequence = sequence;
    event.has_rate = 1;
    event.rate_kbps = rate_kbps;
    event.has_size = 1;
    event.size_bytes = size_bytes;
    sprintf (detail, "preamble=%s", preamble == OPC_NIL ? "" : preamble);
    event.detail = detail;
    event.has_reservation_slot = 1;
    event.reservation_slot = next_slot;
    event.has_reservation_counter = 1;
    event.reservation_counter = next_slot;
    csr_opnet_trace_emit (&event);
}
#endif

#if defined (CSR_OPNET_TRACE_ECC)
static void
csr_opnet_trace_ecc (
    double time_s, Packet *pkptr, int node, int accept, const char *reason,
    BrT_Rxch_State_Info *rxch_state_ptr)
{
    CsrOpnetTraceEvent event;
    double watts;
    int value;
    char detail[48];
    csr_opnet_trace_clear (&event);
    event.time_s = time_s;
    event.event = accept ? "rx_accept" : "rx_drop";
    event.node = node;
    event.has_success = 1;
    event.success = accept ? 1 : 0;
    event.reason = reason;
    if (op_pk_nfd_exists (pkptr, "Tx Node ID") &&
        op_pk_nfd_is_set (pkptr, "Tx Node ID"))
        {
        op_pk_nfd_get_int32 (pkptr, "Tx Node ID", &value);
        event.has_peer = 1;
        event.peer = value;
        event.has_src = 1;
        event.src = value;
        }
    if (op_pk_nfd_exists (pkptr, "Speed") && op_pk_nfd_is_set (pkptr, "Speed"))
        {
        op_pk_nfd_get_int32 (pkptr, "Speed", &value);
        event.has_rate = 1;
        event.rate_kbps = value;
        }
    if (op_pk_nfd_exists (pkptr, "Length") && op_pk_nfd_is_set (pkptr, "Length"))
        {
        op_pk_nfd_get_int32 (pkptr, "Length", &value);
        event.has_size = 1;
        event.size_bytes = value / 8;
        }
    if (op_td_is_set (pkptr, OPC_TDA_RA_RCVD_POWER))
        {
        watts = op_td_get_dbl (pkptr, OPC_TDA_RA_RCVD_POWER);
        if (watts > 0.0)
            {
            event.has_rx_power = 1;
            event.rx_power_dbm = 10.0 * log10 (watts * 1000.0);
            }
        }
    if (op_td_is_set (pkptr, OPC_TDA_RA_NOISE_ACCUM) &&
        op_td_is_set (pkptr, OPC_TDA_RA_BKGNOISE))
        {
        watts = op_td_get_dbl (pkptr, OPC_TDA_RA_NOISE_ACCUM) +
            op_td_get_dbl (pkptr, OPC_TDA_RA_BKGNOISE);
        if (watts > 0.0)
            {
            event.has_noise = 1;
            event.noise_dbm = 10.0 * log10 (watts * 1000.0);
            }
        }
    if (op_td_is_set (pkptr, OPC_TDA_RA_SNR))
        {
        event.has_snr = 1;
        event.snr_db = op_td_get_dbl (pkptr, OPC_TDA_RA_SNR);
        }
    value = op_td_is_set (pkptr, OPC_TDA_RA_NUM_COLLS)
        ? op_td_get_int (pkptr, OPC_TDA_RA_NUM_COLLS) : 0;
    if (value > 0)
        {
        event.has_jsr = 1;
        event.jsr_db = rxch_state_ptr->JSR;
        }
    sprintf (detail, "collisions=%d", value);
    event.detail = detail;
    if (op_td_is_set (pkptr, BR_TDA_RA_HEADER_NUM_ERRS))
        {
        event.has_header_errors = 1;
        event.header_errors = op_td_get_int (pkptr, BR_TDA_RA_HEADER_NUM_ERRS);
        }
    if (op_td_is_set (pkptr, BR_TDA_RA_PAYLOAD_NUM_ERRS))
        {
        event.has_payload_errors = 1;
        event.payload_errors = op_td_get_int (pkptr, BR_TDA_RA_PAYLOAD_NUM_ERRS);
        }
    if (op_td_is_set (pkptr, OPC_TDA_RA_NUM_ERRORS))
        {
        event.has_total_errors = 1;
        event.total_errors = op_td_get_int (pkptr, OPC_TDA_RA_NUM_ERRORS);
        }
    csr_opnet_trace_emit (&event);
}
#endif

#endif
