# OPNET MAC receive/contention parity

Updated: 2026-08-24

## Scope

This increment ports the observable receive and contention behavior from
`br_mac.pr.c` and the source-derived transmission duration from
`br_txdel.ps.c`. It replaces independent per-link delivery callbacks with an
explicit signal lifetime at each receiver.

The boundary is deliberate: MAC acquisition, state, overlap, capture/collision,
half duplex, slot freezing, and duty-cycle behavior are implemented here. The
exact probabilistic radio pipeline remains a separate PHY calibration step.

## State and timing mapping

| OPNET behavior | ns-3 behavior |
| --- | --- |
| `Idle_st` | Duty-cycled receiver is asleep and cannot acquire a signal. |
| `Search_st` | Receiver observes eligible preambles and may schedule acquisition. |
| `Track_st` | One selected signal remains locked until packet completion; new preambles are rejected as acquisition candidates but still interfere. |
| `Tx_st` | Radio is half duplex for the entire OTA transmission and returns to Search afterward. |
| `SYNC2TRACK_MIN = 0.00663` s | Acquisition runs 6.63 ms after synchronization begins. |
| Synchronization threshold mean `-11` dB | Deterministic -11-dB SNR threshold is used until the source variance is calibrated. |
| `TSLOT = 0.013` s | Local Tx countdown and neighbor reservation decay advance on 13-ms ticks. |
| `WAKE_CYCLE = 0.988` s | Duty-cycled nodes search during their periodic wake window. |
| `DSP_RX_BOOTTIME + NO_SIG_SRH_TIME` | Awake Search window is 1.1 ms + 7.8 ms. |

## Signal selection and interference

Each receiver stores every active signal with its start, preamble end, packet
end, received power, and arrival order. In Search, the earliest candidate is
the initial selection. A later candidate replaces it only when it has existed
longer than 6.63 ms and is stronger, matching the strict comparison in the
legacy process.

At acquisition, all other active preambles are rejected. The tracked signal is
marked collided when its power margin over the strongest overlap is below
10.5 dB. That boundary corresponds to the `br_ber` transition into the
-12-dB JSR bucket. It is intentionally deterministic for now; the complete
JSR/time-offset BER table has not yet been ported.

After Track begins, a later signal can never capture the receiver. It can be
weak enough for the tracked signal to survive, or strong enough to corrupt it.
During Tx, all overlapping receives are missed because the radio is half
duplex.

## Airtime and slot ordering

`br_txdel.ps.c` sends the preamble, SOF, speed, and length fields at S0, with a
0.00051-second duration per nibble. The payload and 32-bit FCS are then sent at
the selected payload rate. The ns-3 duration therefore uses:

- 7,888 long-preamble bits or 104 short-preamble bits;
- 16-bit SOF, speed, and length fields at S0;
- exact OPNET envelope bytes plus 32-bit FCS at the payload rate.

Both the pending local Tx countdown and learned neighbor reservation counters
advance only while the MAC is in Search and no synchronization preamble is
present. Idle, Track, Tx, and Search-with-SYNC keep the timer alive but freeze
the counters.

The longer airtime exposed a discovery ordering race that the old placeholder
duration hid. Discovery completion now waits for queued/in-flight MAC control
to clear and then observes one quiet response interval. The existing scheduled
DiscoveryStop remains the hard lifecycle bound.

## Duty-cycle behavior

A deterministic wake phase is available for repeatable tests. A signal that
starts while the receiver is in Idle can still be acquired if its preamble
survives through the next wake plus the 6.63-ms acquisition delay. This lets a
long preamble bridge sleep, while a short preamble that expires first is
recorded as a miss.

## Verification

`csr-mac-receive-contention-smoke.cc` covers:

- Search to Track to Search timing;
- equal-power simultaneous collision;
- strongest mature preamble capture;
- rejection and interference from a later stronger signal after Track;
- RX-induced local Tx-countdown freeze and resume;
- long-preamble wake versus short-preamble sleep miss;
- half-duplex loss during Tx.

The complete regression baseline is 25 focused smoke tests plus
`csr-mac-demo-split`, all passing (26/26).

## Remaining PHY boundary

The following behavior is not claimed by this increment:

- stochastic synchronization threshold variance;
- receiver-group qualification and received-power calibration;
- accumulated noise from multiple or different-rate interferers;
- exact JSR/time-offset BER tables and header-versus-payload treatment;
- ECC, closure, and error-statistics pipelines;
- calibrated propagation and the 500/1000-kbps modes.
