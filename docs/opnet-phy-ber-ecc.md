# OPNET PHY BER, ECC, and closure parity

Updated: 2026-08-26

## Scope

This increment completes the recovered receive pipeline after the deterministic
power/noise front end:

1. `br_closure.ps.c` visibility gating before channel and receive processing;
2. `br_ber.ps.c` standard and collision modulation-table selection;
3. `br_error_all_stats.ps.c` interval-by-interval realized error allocation;
4. `br_ecc.ps.c` full protected-packet correction threshold;
5. final accept/drop ordering before MAC delivery, neighbor state, or ACKs.

The implementation uses the supplied `Modulation_tables` archive directly.
The raw OPNET files are not required at runtime: a reproducible importer emits
an exact, compact C++ representation of the tables.

## Exact modulation tables

The archive contains one standard `csr` curve, one `dqpsk` curve, and 4,910 CSR
collision curves. The supplied C receive pipeline references `csr` and the
collision family but does not reference `dqpsk`. The project owner confirmed
that the actual radio has 500- and 1000-kbit/s modes, so the extended runtime
profile maps those two rates to the exact recovered `dqpsk` curve. That mapping
is intentional extension behavior, not a source-exact claim. The collision
inventory is complete for every supplied spread rate, JSR bucket, and half-chip
offset:

| Rate key | Offset range | Curves per JSR | Total curves |
| ---: | ---: | ---: | ---: |
| 8 | 0.0-254.5 chips | 510 | 2,550 |
| 16 | 0.0-126.5 chips | 254 | 1,270 |
| 32 | 0.0-62.5 chips | 126 | 630 |
| 64 | 0.0-30.5 chips | 62 | 310 |
| 128 | 0.0-14.5 chips | 30 | 150 |

The DQPSK table contains 161 samples from -20 through 20 dB in 0.25-dB
increments. No DQPSK-specific JSR/offset collision tables were supplied.

The DQPSK file passes the same binary-integrity checks as the remaining table
family: its metadata, sample count, file length, and zero trailer are valid;
all samples are finite and nonnegative; its 146 nonzero values decrease
strictly and are followed by 15 trailing zeroes. It is less robust than `csr`
at every shared positive grid point where the DQPSK curve remains nonzero;
both curves are zero in the high-SNR tail. Representative DQPSK penalties
relative to `csr` are 1.31 dB at BER 0.1, 2.68 dB at BER 1e-2, and 4.01 dB at
BER 1e-5.
Regenerating the C++ data from the supplied ZIP is byte-identical to the
committed include.

The DQPSK curve reaches 0.6585 at -20 dB and remains above 0.5 through about
-7.88 dB. That is unusual for conventional bit BER, but it is smooth, the
file's own display range extends to 0.7, and `br_error_all_stats` explicitly
implements probabilities above 0.5. The table is therefore treated as an
exact recovered OPNET table input for this extension, while its independent
physical bit-error semantics remain unverified.

`utils/import-opnet-modulation-tables.py` accepts either the original ZIP or
an extracted directory. It validates the binary header, big-endian numeric
grid, curve count, filenames, and trailing word. It emits every IEEE-754 BER
sample as an exact hexadecimal literal. Because every supplied curve has only
trailing zeroes after its last nonzero sample, the generated data stores the
nonzero prefix and reconstructs the remaining zeroes during lookup.

The source API `op_tbl_mod_ber()` is opaque. Off-grid lookup therefore uses
the strongest portable match: endpoint sample clamping and arithmetic linear
interpolation. On-grid samples remain exact and are tested independently.
The two display-range values in each OPNET file are not treated as BER clamps.

## Rate profiles and table selection

The source model derives the five spread-rate bit rates from four-bit payload
intervals rather than rounded nominal keys:

| Key | Four-bit payload interval | Exact payload bit rate | Processing gain at 1 MHz |
| ---: | ---: | ---: | ---: |
| 8 | 510 us | 7,843.137255 bps | 18.044801891 dB |
| 16 | 254 us | 15,748.031496 bps | 15.017437296 dB |
| 32 | 126 us | 31,746.031746 bps | 11.972805581 dB |
| 64 | 62 us | 64,516.129032 bps | 8.893017025 dB |
| 128 | 30 us | 133,333.333333 bps | 5.740312677 dB |

The owner-confirmed extended profile adds two bit rates. Their four-bit
accounting intervals are derived from those rates:

| Key | Four-bit payload interval | Exact payload bit rate | Processing gain at 1 MHz |
| ---: | ---: | ---: | ---: |
| 500 | 8 us | 500,000 bps | 0 dB |
| 1000 | 4 us | 1,000,000 bps | -3.010299957 dB |

The recovered files do not establish the physical DQPSK symbol framing, so the
8/4-us values are described only as four-bit payload intervals. The compact
ns-3 compatibility headers still serialize speed in one byte; reserved codes
encode these two keys without changing the established header size. Raw
operational keys 129 and 130 are rejected because they would alias those
reserved codes. The fixed OTA header continues to account for the source
packet's 16-bit Speed field.

`LEGACY_SOURCE_EXACT` remains the default and caps both automatic and explicit
transmission at 128 kbit/s. `EXTENDED_DQPSK` opts into both high rates and
propagates from NWK through HOP to MAC:

```cpp
nwk->SetRateProfile (CsrRateProfile::EXTENDED_DQPSK);
```

Scenarios can retain the extended behavior while capping the advertised and
selected maximum at 500 kbit/s with `SetMaxSpeedKbps(500)`. The MAC's automatic
and explicit transmission paths, HOP link control, NWK link-cost calculation,
routing INFO maximum, OTA duration, BER intervals, and ECC decision then use
the same configured ceiling.

For raw SNR `snr`, receive bandwidth `bw`, and selected bit rate `r`, the table
input is:

```text
effective_snr = snr + 10*log10(bw/(2*r))
```

This processing-gain equation is source-exact for the 8-128-kbit/s spread
rates. Reusing it for the DQPSK extension is an explicit integration
assumption, not an independently recovered high-rate modem equation.

The header always uses the S0 rate. Table selection is:

| Interval state | Header curve | Payload curve |
| --- | --- | --- |
| No collision, 8-128 kbit/s | `csr` | `csr` |
| No collision, extended 500/1000 kbit/s | `csr` | `dqpsk` |
| Same-rate collision, 8-128 kbit/s | 8-kbps JSR/offset curve | matching-rate JSR/offset curve |
| Same-rate collision, extended 500/1000 kbit/s | 8-kbps JSR/offset curve | `dqpsk`, with the recorded jammer treated as payload noise |
| Different-rate collision | 8-kbps JSR/offset curve | rate's standard curve, with jammer watts included in noise |

JSR uses the asymmetric source boundaries at -10.5, -7.5, -4.5, and -1.5 dB.
For 8-128-kbit/s collision tables, signed timing is reduced modulo the
applicable payload interval and rounded to the nearest one-microsecond
half-chip; the endpoint wraps to zero. Receiver `same_speed`, JSR, and timing
state intentionally retain the source's shared, last-collision semantics. The
DQPSK payload fallback does not quantize or use that time offset. Instead, the
recorded JSR supplies jammer power because the archive has no corresponding
DQPSK collision family. This jammer-as-noise rule is a documented
extended-profile fallback, not a recovered DQPSK collision equation.

## Interval error accounting

Every active signal carries a BER interval. Before a signal begins or ends,
the runtime performs this order:

1. close the elapsed interval with its old noise, collision, JSR, and offset;
2. allocate errors for just the bits transmitted during that interval;
3. apply the new interference state;
4. begin the next interval at the same simulation time.

For receive start `t0`, the tested regions are:

```text
header_start  = t0 + preamble_bits / S0_rate
payload_start = t0 + (preamble_bits + 48) / S0_rate
```

The preamble is never error-allocated. Header BER applies to the 48 fixed bits
for SOF, speed, and length. Payload BER applies to the inherited payload and
32-bit FCS. Each interval's floating bit duration is converted with truncation,
matching the source's C cast; split intervals can consequently account for one
fewer bit than one uninterrupted interval.

For every nontrivial region, the model consumes one uniform random value and
inverts the binomial cumulative mass function using the source log-gamma
formula. BER zero, an empty region, and BER one consume no random value.
Header allocation precedes payload allocation when one callback crosses the
boundary. Error allocation continues even after the eventual ECC limit has
already been exceeded.

## ECC and final ordering

The supplied build sets `ECC_HEADER_ONLY` to zero. The protected denominator
is therefore:

```text
protected_bits = total_OTA_bits - preamble_bits
correctable     = int(ecc_threshold * protected_bits)
accepted        = total_errors <= correctable
```

The comparison is inclusive and the threshold product is truncated. The
recovered scenario threshold is 0.1. FCS errors count as payload errors; there
is no additional simulated CRC mutation or check.

An earlier state rejection, receiver lock, or failed node remains rejected and
is not counted as an ECC drop. Otherwise an over-threshold packet increments
the receiver's ECC-drop counter. Only a finally accepted tracked packet reaches
MAC/HOP delivery, neighbor observation, or ACK generation.

The earlier hard `collided` gate is removed when the recovered modulation
tables are active. A selected collided packet can now succeed when its sampled
errors are within ECC capacity. The existing injectable BER hook retains the
hard gate so older MAC-only tests can continue to isolate deterministic state
transitions from stochastic PHY outcomes.

## Closure

Closure is evaluated before propagation delay, channel matching, power,
interference, synchronization, or receive callbacks. A failed closure therefore
has no MAC counters, neighbor state, delivery, or ACK effects.

The runtime exposes never-occluded, spherical-Earth line-of-sight, and delegated
closure modes. All supplied DES configurations disable TMM. The exact external
OPNET `generic_earth_LOS_closure()` implementation and TMM propagation library
were not supplied, so the built-in Earth mode uses the standard geometric
horizon and the delegate is the authoritative extension point for a captured
OPNET or terrain-specific decision. This boundary is explicit rather than
claiming an unrecovered TMM equation.

## Verification

`csr-phy-ber-ecc-smoke.cc` covers:

- exact rates and processing gains for five source-exact keys plus two
  owner-confirmed extension keys;
- JSR and circular half-chip boundary cases;
- independent standard, DQPSK, and all-five-rate collision golden values;
- interpolation and endpoint behavior;
- standard, same-rate, and different-rate table selection;
- preamble exclusion, FCS inclusion, interval splitting, truncation, and cap;
- independent inverse-CDF binomial outcomes;
- inclusive ECC, prior rejection, receiver lock, node failure, and zero length;
- never, Earth-LOS, delegated, and pre-pipeline failed closure;
- live selected-collision acceptance and ECC rejection.

`csr-live-high-rate-dqpsk-smoke.cc` additionally covers:

- default legacy and opt-in extended profile propagation from NWK to MAC;
- explicit 500-kbit/s scenario ceilings;
- live HOP link-control selection at 128, 500, and 1000 kbit/s;
- exact 500/1000 OTA duration and production DQPSK receive decisions;
- low-SNR DQPSK error allocation above probability 0.5 and final ECC drop;
- live equal-power same-rate DQPSK overlap through jammer-as-noise BER/ECC;
- safe one-frame progress when no high-rate concatenation limit is available;
- reserved compact-code non-operational and non-encodable classification.

The full regression currently contains 28 focused CSR programs plus the demo.

## Remaining PHY boundary

- Source-distributed synchronization-threshold variance around -11 dB.
- An authoritative probe of OPNET's off-grid `op_tbl_mod_ber()` interpolation.
- Exact external spherical-Earth/TMM closure if those support libraries or
  captured decisions become available.
- Scenario-level import of promoted frequency, height, power, and link-margin
  attributes instead of setting them programmatically.
- Differential event traces under matched OPNET/ns-3 seeds and topologies.
