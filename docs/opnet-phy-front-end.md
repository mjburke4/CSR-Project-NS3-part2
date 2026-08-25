# OPNET PHY front-end parity

Updated: 2026-08-25

## Scope

This increment ports the deterministic radio stages that precede BER and ECC:

1. `br_rxgroup.ps.c` receiver qualification and state initialization;
2. `br_power.ps.c` channel overlap, propagation, antenna gain, and received
   power;
3. `br_inoise.ps.c` different-rate additive noise and same-rate JSR/time
   offset classification;
4. `br_snr.ps.c` linear-power noise accumulation and SNR;
5. distance-based propagation delay feeding the existing MAC signal lifetime.

The source's modulation-table BER, header/payload error accounting, ECC,
closure, and final error statistics remain a separate increment.

## Recovered project configuration

The supplied binary node and network models contain real radio attributes.
They are not output vectors, but they establish the configuration behind the
validation scenarios.

| Attribute | `br_node_v1.nd.m` default | Supplied validation scenarios |
| --- | ---: | ---: |
| Tx/Rx minimum frequency | 30 MHz | 400 MHz |
| Tx/Rx bandwidth | 1,000 kHz | inherited 1,000 kHz |
| Receiver noise factor | 5.01187233627 (7 dB) | inherited |
| Nominal noise at 1 MHz | -106.975 dBm | inherited |
| Minimum Tx power | -36 dBm | -36 dBm |
| Maximum Tx power | 33 dBm | 33 dBm |
| ECC threshold | external/promoted | 0.1 |
| Link margin | 12 dB | 6 or 12 dB, depending on scenario |
| Node altitude | 1 m default | typically 1.05 m; relays may differ |

`CsrPhyProfile` exposes frequency, bandwidth, gains, heights, nominal noise,
the noise-reference bandwidth, synchronization threshold, and propagation
mode. The default profile represents the node model. A scenario should apply
its promoted 400-MHz frequency and per-node altitude when reproducing the
validation network.

All supplied DES environment files set `tmm_simulate` to `false`. Therefore,
the non-TMM equations in `br_power.ps.c` are authoritative for these files.
Terrain/TMM path-gain injection is not claimed by this increment.

## Received-power equations

For transmitter center frequency $f_c$, wavelength $\lambda=c/f_c$, distance
$d$, transmitter height $h_t$, and receiver height $h_r$, `br_power` computes
three linear gains:

$$
G_{fs}=\frac{\lambda^2}{16\pi^2d^2},\qquad
G_{2r}=\frac{h_t^2h_r^2}{d^4},\qquad
G_{wlan}=\frac{h_t^2h_r^2\,10^{18}}{d^4f_c^2}.
$$

The selected propagation gain is the smallest of $1$, $G_{fs}$, $G_{2r}$,
and $G_{wlan}$. This is a minimum-gain rule, not a conventional choice of the
least-loss model.

The transmitter power is first reduced to the fraction inside the receiver
passband:

$$
P_{tx,in}=P_{tx}\frac{B_{overlap}}{B_{tx}}.
$$

Received power is then:

$$
P_{rx}=P_{tx,in}G_{tx}G_{path}G_{rx}.
$$

The runtime keeps all intermediate values in linear watts and converts to dBm
only for diagnostics and HOP observations. Zero channel overlap fails receiver
qualification before acquisition.

## Noise and interference ordering

Background noise uses the source's -106.975-dBm value at a 1-MHz reference
bandwidth and scales linearly when receiver bandwidth changes. Overlapping
signals are handled by payload rate:

| Overlap | Runtime treatment | Downstream use |
| --- | --- | --- |
| Different rate | Add the other received power in watts to `NOISE_ACCUM`; track peak total noise and minimum SNR | Current error hook and future interval BER accounting |
| Same rate | Do not add power to noise; retain strongest jammer-to-signal ratio and signed start-time offset | Future `csr_*dBJSR_*ChipOff` modulation-table lookup |

This preserves the otherwise surprising split in `br_inoise`: a same-rate
jammer can corrupt a frame without changing the SNR reported by `br_snr`.

Until the supplied modulation tables are connected, the MAC keeps its
deterministic 10.5-dB same-rate collision/capture boundary. That boundary is
an explicit compatibility approximation and is not counted as completed BER
parity. Different-rate interference no longer enters that hard-collision path.

## Runtime diagnostics

Every completed tracked signal now retains a `CsrRxDecision` with:

- channel match and selected propagation model;
- path loss and received power;
- peak noise and minimum SNR;
- same-rate classification, JSR, and time offset;
- error probability and final delivery decision.

The receive CSV contains the same fields, including band overlap and path-model
identifier. This gives the later differential-trace tooling direct
TX-power-to-accept/drop evidence even though the supplied `.ov` files contain
only aggregate receiver-drop vectors rather than per-packet PHY intermediates.

## Verification

`csr-phy-front-end-smoke.cc` covers:

- project/source defaults and dBm/watt conversions;
- independent free-space/flat-earth/ad-hoc-WLAN selection vectors;
- exact 100-m received power, path loss, noise, and SNR;
- partial and zero channel overlap;
- transmitter and receiver antenna gains;
- receiver-bandwidth noise scaling;
- live different-rate additive interference;
- live same-rate JSR and time-offset metadata;
- runtime channel-mismatch rejection.

## Remaining PHY boundary

- Load all `csr` and `csr_<rate>_<JSR>_<chip-offset>` modulation tables.
- Apply processing gain and separate preamble/header versus payload BER.
- Integrate errors over each interference/SNR time interval rather than using
  the current peak-noise compatibility value.
- Port `br_error_all_stats`, `br_ecc`, and `br_closure` accept/drop ordering.
- Reproduce the normal synchronization-threshold variance around -11 dB.
- Apply the promoted Min/Max Power, Link Margin, and ECC threshold through a
  scenario-level configuration importer or explicit ns-3 scenario setup.
