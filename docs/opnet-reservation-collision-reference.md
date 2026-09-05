# Deterministic OPNET reservation/collision reference run

Updated: 2026-08-27

## Evidence goal

This run produces the first packet-level reference for the MAC reservation
lifecycle and an equal-slot collision. An authoritative reference is a CSV
written by a licensed Modeler run of the recovered sources. A copied ns-3 trace
or a hand-authored CSV can test the plumbing, but it is not reference evidence.

Ten complete historical `*.ov` result databases are now decoded and compared
by the separate aggregate workflow. They are authoritative for the bucketed
statistics they contain, but they do not contain chronological reservation,
transmit, receive, or ECC events. They therefore cannot satisfy this run's
event-order evidence goal or replace the licensed Modeler CSV described below.

The repository contains everything needed before that licensed run:

- `utils/testdata/csr-reservation-collision-scenario.csv`, the canonical ns-3
  scenario;
- `utils/instrument-opnet-reservation-trace.py`, an anchor- and SHA-checked
  transformation of the recovered OPNET sources;
- `utils/opnet/csr-opnet-trace.h`, the canonical CSV writer and controlled
  reservation hook;
- `utils/validate-opnet-reservation-collision.py`, the source-side evidence
  validator; and
- the existing scenario runner and differential comparator.

## Controlled scenario

The recovered `CSR_Validation-3_Nodes` network is used without changing its
geometry or radio profile.

| Setting | Value |
| --- | --- |
| Modeler scenario | `CSR_Validation-3_Nodes` |
| Duration / seed / TMM | 65 s / 128 / disabled |
| Gateway | node 1 at (0, 0) |
| Sender 2 | node 2 at approximately (600, -1040) m |
| Sender 3 | node 3 at (1200, 0) m |
| Application traffic | one 600-byte, DSCP-5 packet from each sender to gateway 1 at 60 s |
| Next packet interval | 1000 s, placing it beyond the run |
| Controlled reservation | slot and counter 5 for nodes 2 and 3 |
| Control activation | 59.9 s, after gateway discovery/convergence |
| Comparison window | 59.8 through 61.216 s |

The control is inactive during network convergence. At transmit preparation it
loads the known slot/counter, preserves the recovered holdoff gate, and puts
the first countdown tick on a common 100-ms epoch. `prep_tx()` still starts a
300-ms holdoff on an Idle-to-Search transmit path; an arrival already in Search
still reuses the completed holdoff, as the source does. The trace labels these
paths `controlled_wait` and `controlled_ready`. The subsequent 13-ms countdown,
OTA transmit, receiver interference, BER allocation, and ECC outcome execute
their normal source paths.

The common epoch is necessary. Equal reservation numbers alone do not force a
collision when nodes have different local slot phases: the first preamble can
set `SYNC_PRES` and freeze the other sender before its last tick.

## Prepare the instrumented OPNET sources

Run this against either bare exported source names or the numbered uploaded
copies. The script rejects any recovered source whose SHA-256 does not match
the audited input.

```bash
python3 utils/instrument-opnet-reservation-trace.py \
  project_sources opnet-reservation-collision \
  --force-reservation-slot 2:5 \
  --force-reservation-slot 3:5 \
  --activation-time 59.9
```

The output contains instrumented `br_app.pr.c`, `br_mac.pr.c`, `br_ecc.ps.c`,
`csr-opnet-trace.h`, and `opnet-instrumentation-manifest.json`. The manifest
records the exact original and generated hashes, the generator identity, and
the behavioral control. It also binds the recovered `br_support.h` and
`br_txdel.ps.c` used for the independent airtime contract. Keep all four
generated files beside the manifest: the validator recomputes their hashes,
the generated header, the header-template identity, and the generator identity
before accepting a trace. Every emitted `tx_start` also records `preamble=long` or
`preamble=short` in its `detail` field. `preamble=unknown` is an instrumentation
alarm and the validator rejects it.

The instrumenter rejects an output directory that is the source directory, an
ancestor or descendant of it, or a filesystem alias of it. `--overwrite`
allows replacement of existing bundle files only; it never permits an output
to alias a recovered input, the header template, or another managed output. A
lone existing manifest also requires `--overwrite`.

Install the three generated C sources and header in the Modeler project and
rebuild those process/pipeline models with the normal local source-build
workflow. The recovered media contains generated C rather than the originating
`*.pr.m` models. If a local workflow regenerates C from process models, apply
the equivalent hooks there first; regeneration would otherwise remove the
instrumentation.

## Configure and run Modeler

In `CSR_Validation-3_Nodes`:

1. Set the DES duration to 65 s, seed to 128, and TMM simulation to false.
2. For nodes 2 and 3, set application Start Time to 60 s, Packet Size to 600
   bytes, and Packet Interarrival Time to `constant (1000)`.
3. Leave node 1 without an application transmission in this interval. Its
   recovered default start of 300 s is already outside the run.
4. Set `CSR_OPNET_TRACE` to an absolute writable CSV path before launching the
   run. If it is unset, the trace is written as
   `csr-opnet-reservation-collision.csv` in the Modeler run directory.
5. Run once and retain the CSV with the generated instrumentation manifest.

For a Windows Modeler shell, for example:

```bat
set CSR_OPNET_TRACE=C:\csr-reference\opnet-reservation-collision.csv
```

## Validate the exported evidence

The validator requires an ordered lifecycle for each sender: controlled
prepare, a ready holdoff gate or a 300-ms wait, counters 4 through -1,
reservation advertisement, and transmit. These relevant events must be
contiguous, with exactly one controlled prepare in the comparison window.
Every prepare/holdoff/advertise/transmit slot and counter is bound to the forced
slot. The first tick must match the generated 100-ms control epoch, subsequent
ticks must be 13 ms apart, and advertisement and transmit must coincide with
the `-1` tick within 1 microsecond. It also requires simultaneous
transmissions. Each transmit must name gateway 1 as its peer, have a positive
payload size, name a long or short preamble, and use one of the source-exact
rates: 8, 16, 32, 64, or 128 kbps. The separate 500- and 1000-kbps extension
smokes are deliberately outside this source-exact validator and are rejected
here. Transmit source, destination, and sequence are also required and checked;
the gateway outcome must repeat the transmitting source identity.

The expected receive-completion time is derived independently from recovered
`br_txdel` arithmetic. The preamble plus the 48-bit SOF/speed/length header use
the S0 nibble duration, while the payload plus 32-bit FCS use the selected
rate's nibble duration:

```text
airtime = ((preamble_bits + 48) / 4) * 0.000510
        + ((payload_bytes * 8 + 32) / 4) * selected_nibble_duration
```

Long and short preambles are 7888 and 104 bits. The exact S0 through S4 nibble
durations for 8, 16, 32, 64, and 128 kbps are 0.000510, 0.000254, 0.000126,
0.000062, and 0.000030 seconds respectively.

For each sender, exactly one gateway outcome must match the peer, rate, and
payload size and occur within 1 ms of that derived completion. That same row
must be an `rx_drop` with `success=0` and a positive `collisions` count.
Consequently, a later unrelated collision, duplicate outcomes, or an accept
row combined with a separate drop row cannot satisfy the case.

```bash
python3 utils/validate-opnet-reservation-collision.py \
  opnet-reservation-collision.csv \
  --manifest opnet-instrumentation-manifest.json \
  --receiver 1 \
  --transmitters 2,3 \
  --slot 5 \
  --time-start 59.8 \
  --time-stop 61.3 \
  --slot-tolerance 0.000001 \
  --completion-tolerance 0.001 \
  --report opnet-reservation-collision-validation.json
```

Do not use a failed export as the differential reference. A failure usually
identifies a source-build problem, application attribute mismatch, missing
route convergence, control activation at the wrong time, or a provenance
mismatch. The report path must be distinct from the trace and manifest; direct,
symbolic-link, and hard-link aliases are rejected before any report write.

## Run the differential harness

```bash
python3 utils/run-opnet-differential.py \
  --scenario utils/testdata/csr-reservation-collision-scenario.csv \
  --runner ../ns-3-dev/build/scratch/ns3-dev-csr-opnet-scenario-runner-default \
  --opnet-trace opnet-reservation-collision.csv \
  --opnet-manifest opnet-instrumentation-manifest.json \
  --output-dir reservation-collision-differential \
  --stop 65 \
  --flow-limit 1 \
  --key-fields event,node,peer \
  --select app_send:2 \
  --select app_send:3 \
  --select reservation_prepare:2 \
  --select reservation_prepare:3 \
  --select reservation_tick:2 \
  --select reservation_tick:3 \
  --select reservation_advertise:2 \
  --select reservation_advertise:3 \
  --select tx_start:2 \
  --select tx_start:3 \
  --select rx_accept:1 \
  --select rx_drop:1 \
  --time-start 59.8 \
  --time-stop 61.216 \
  --coincident-tolerance 0.000001 \
  --tolerance time_s=0.013 \
  --ignore-field reason \
  --ignore-field sequence \
  --ignore-field size_bytes \
  --ignore-field header_errors \
  --ignore-field payload_errors \
  --ignore-field total_errors
```

Error counts are excluded from the first lifecycle comparison because Modeler
and ns-3 do not share a random-number implementation. They remain in both raw
traces for distribution-level calibration. The lifecycle, slot/counter values,
transmit/receive identities, collision counts, rate, and mutually observed PHY
quantities remain comparison evidence.

The provenance checks are deliberately strong but not a signature. The
reference-only `br_support.h` and `br_txdel.ps.c` records prove what the
instrumenter inspected, not which objects a licensed Modeler installation
ultimately linked, and the manifest is not cryptographically embedded in the
CSV. Preserve Modeler build logs and binary identities with the authoritative
run if that stronger chain of custody is required.

## Local checkpoint

The ns-3 side currently passes the validator. Both nodes report the existing
Search holdoff as ready, count `5 -> 4 -> 3 -> 2 -> 1 -> 0 -> -1`, start a
long-preamble transmission at exactly 60.165 s, and produce two gateway drops
with `collisions=1` at approximately 61.215304 s. A same-trace plumbing run
through the differential harness selects 22 events and passes with no missing,
extra, replaced, or field-mismatched rows. That plumbing check is not the
authoritative OPNET comparison; the licensed Modeler CSV is the remaining
input for this event-order case. Historical `*.ov` comparisons remain useful
aggregate calibration evidence and are documented separately in
`docs/opnet-aggregate-comparison.md`.
