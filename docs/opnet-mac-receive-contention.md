# OPNET MAC receive/contention parity

Updated: 2026-09-02

## Scope

This increment ports the observable receive and contention behavior from
`br_mac.pr.c` and the source-derived transmission duration from
`br_txdel.ps.c`. It replaces independent per-link delivery callbacks with an
explicit signal lifetime at each receiver.

The boundary is deliberate: MAC acquisition, state, overlap, capture/collision,
half duplex, slot freezing, and duty-cycle behavior are implemented here. The
received-power/noise front end, exact modulation-table BER, interval errors,
ECC, closure, and final accept/drop ordering are now implemented separately.

## State and timing mapping

| OPNET behavior | ns-3 behavior |
| --- | --- |
| `Idle_st` | Duty-cycled receiver is asleep and cannot acquire a signal. |
| `Search_st` | Receiver observes eligible preambles and may schedule acquisition. |
| `Track_st` | One selected signal remains locked until packet completion; new preambles are rejected for that Track epoch but still interfere and may be reacquired after return to Search. |
| `Tx_st` | Radio is half duplex for the entire OTA transmission and returns to Search afterward. |
| `SYNC2TRACK_MIN = 0.00663` s | Acquisition runs 6.63 ms after synchronization begins. |
| `normal(SYNC_SNR_MIN, SYNC_SNR_VAR)` | One threshold is drawn for every channel-matched SYNC attempt from a normal distribution with mean -11 dB and variance 0.25 dB squared (standard deviation 0.5 dB). |
| `TSLOT = 0.013` s | Local Tx countdown and neighbor reservation decay advance on 13-ms ticks. |
| `WAKE_CYCLE = 0.988` s | Duty-cycled nodes search during their periodic wake window. |
| `DSP_RX_BOOTTIME + NO_SIG_SRH_TIME` | Awake Search window is 1.1 ms + 7.8 ms. |
| `Track_st --DONE_RX_PK--> Search_st` | A packet delivered by the receiver pipeline returns immediately; a pipeline-suppressed packet uses the fallback at `END_RX + TIC` (28 ns). With no Tx/ACK preparation or post-Tx ACK wait, `back2search()` schedules sleep after `NO_SIG_SRH_TIME = 7.8` ms; a new acquisition cancels that sleep on entry to Track. |

## Signal selection and interference

Each receiver stores every active signal with its start, preamble end, packet
end, received power, and arrival order. In Search, the earliest candidate is
the initial selection. A later candidate replaces it only when it has existed
longer than 6.63 ms and is stronger, matching the strict comparison in the
legacy process.

At acquisition, all other active preambles are rejected. Same-rate overlaps
retain exact JSR and time offset for modulation-table lookup. The tracked
signal records a collision when its power margin over the strongest same-rate
overlap is below 10.5 dB, but production delivery is decided later by
BER/error allocation and ECC rather than a hard collision drop. Different-rate
overlaps accumulate in linear watts and affect SNR. The old deterministic hard
gate remains only for tests that explicitly install the compatibility error
hook to isolate MAC state transitions.

After Track begins, a later signal cannot capture the receiver during that
Track epoch. It can be weak enough for the tracked signal to survive, or strong
enough to corrupt it. If its synchronization preamble remains active through
the return to Search, `back2search()` schedules a fresh 6.63-ms acquisition and
`start_track()` explicitly accepts the replacement. During Tx, all overlapping
receives are missed because the radio is half duplex.

### Synchronization-threshold stream

`br_support.h` names the second parameter `SYNC_SNR_VAR`, and
`br_mac.pr.c` passes it as the second parameter to OPNET's `normal`
distribution before calling `op_dist_outcome()` once in `mark_sync()`. The
ns-3 model therefore treats 0.25 as variance, not standard deviation. It owns
a dedicated `NormalRandomVariable`, so duty-phase and PHY error-realization
draws from the pre-existing uniform stream cannot perturb the threshold
sequence. `CsrNetDevice::AssignStreams()` assigns that device-uniform stream
first and the synchronization stream second for stable seed/run/stream replay.

The `SyncDecision` trace source reports transmitter, sequence, observed SNR,
sampled threshold, and the resulting decision for every channel-matched
attempt. `SetSyncSnrThresholdOverrideDb()` supplies a controlled-test
threshold. Setting `CsrPhyProfile::stochasticSyncThreshold` false is the
explicit deterministic compatibility mode; source-exact scenarios leave it
true. This establishes the source distribution and ns-3 replay stability; it
does not claim a draw-for-draw match to Modeler's unavailable RNG stream.

## Airtime and slot ordering

`br_txdel.ps.c` sends the preamble, SOF, speed, and length fields at S0, with a
0.00051-second duration per nibble. The payload and 32-bit FCS are then sent at
the selected payload rate. The ns-3 duration therefore uses:

- 7,888 long-preamble bits or 104 short-preamble bits;
- 16-bit SOF, speed, and length fields at S0;
- exact OPNET envelope bytes plus 32-bit FCS at the payload rate.

Both the pending local Tx countdown and learned neighbor reservation counters
advance only while the MAC is in Search and no synchronization preamble is
present. Track, Tx, and Search-with-SYNC keep the timer alive but freeze the
counters. Idle cancels the persistent timer. An ordinary HOP arrival queues
there, after which Idle's enter executive schedules an RTS on the next
simulation-wide 13-ms boundary unless it lies within half a slot of WAKE.
That RTS runs `prep_tx()` and continues the global phase; a periodic WAKE runs
`start_search()` and establishes a wake-relative phase.

The 300-ms transmit holdoff is independent from that shared slot clock and is
restarted only by a real Idle-to-Search activation. Every transmitted frame
selects and advertises the next slot, and the sender retains the same live
counter even when its queues drain. Traffic arriving during Tx, during
post-Tx Search, or after Track therefore reuses an unexpired advertisement;
an expired Search-state reservation selects a new slot without another
holdoff. Search activation deliberately occurs after that tick's decrement,
while Track-to-Search activation occurs immediately as in `back2search()`.
The Idle-RTS branch redraws a counter at zero because `prep_tx()` tests
`<= 0`; Search activation redraws only after a counter has expired below zero.

The longer airtime exposed a discovery ordering race that the old placeholder
duration hid. The later route-library audit resolved that boundary directly:
local discovery broadcasts at relative `+0`, `+5`, and `+10` seconds and ends
on the fourth callback at `+15`, without waiting for queued/in-flight MAC
control or an extra quiet interval. The independently scheduled 30-second
DiscoveryStop is only a fallback.

## Duty-cycle behavior

A deterministic wake phase is available for repeatable tests. A signal that
starts while the receiver is in Idle can still be acquired if its preamble
survives through the next wake plus the 6.63-ms acquisition delay. This lets a
long preamble bridge sleep, while a short preamble that expires first is
recorded as a miss.

After a tracked packet completes, MAC first forwards the decoded frame to HOP
and then follows the source `back2search()` transition. A valid ACKable frame
can therefore enqueue its authenticated ACK before the transition observes
the queues; `PREP_TX` keeps Search active until the controlled slot. A
malformed or authentication-rejected addressed frame creates no ACK and
receives only the source's 7.8-ms post-Track Search. The former fixed 350-ms
pre-authentication receive hold was unsupported by the supplied source and has
been removed. This timing change is classified as source-exact MAC behavior;
HOP authentication itself remains the separately documented security layer.

The source post-Tx wait is a separate `WAIT_FOR_ACK` event, not a generic wake
extension. It begins at local `DONE_TX` only when both transmit queues are
empty. Expiry in Search runs `dsp_off()` even inside a periodic awake window;
expiry in Track only clears the wait, so the later `back2search()` schedules
its own 7.8-ms no-signal Search. Transmit preparation or a new Tx cancels the
pending wait through the corresponding source paths.

## Verification

`csr-mac-receive-contention-smoke.cc` covers:

- Search to Track to Search timing;
- equal-power simultaneous collision;
- strongest mature preamble capture;
- rejection and interference from a later stronger signal during Track,
  followed by reacquisition when its preamble survives;
- RX-induced local Tx-countdown freeze and resume;
- long-preamble wake versus short-preamble sleep miss;
- half-duplex loss during Tx;
- one normal threshold draw per SYNC attempt;
- -11-dB sample mean and 0.25-dB-squared sample variance;
- exact seed/run/stream replay and isolation from the duty-phase stream; and
- controlled accept/reject boundaries plus deterministic compatibility mode.

`csr-mac-reservation-lifecycle-smoke.cc` additionally covers:

- Idle queueing until its globally aligned RTS TSLOT, `prep_tx()` activation,
  and the independent 300-ms holdoff;
- next-slot advertisement, first-contact ordering, known-peer learning, and
  persistent local countdown;
- reservation reuse for arrivals during Tx, post-Tx Search, and Track return;
- Search expiration/redraw ordering and Idle-RTS redraw at counter zero;
- Search-with-SYNC activation with counter freeze;
- a real HOP retry that consumes the preceding advertised slot;
- a controlled authenticated DATA-to-ACK chain proving ACK preparation
  supersedes the 7.8-ms post-receive sleep without changing slot-13 timing;
- over-air malformed and bad-authentication ACKable DATA proving Track returns
  to Search, no ACK is queued, and Idle begins exactly after 7.8 ms;
- a replacement preamble rejected during Track and reacquired 6.63 ms after
  the first packet completes;
- post-Tx wait expiry in Search and Track, including Search-to-Idle expiry
  inside a periodic awake window;
- an ECC/pipeline rejection held in Track until `END_RX + TIC`; and
- cancellation of a delayed packing retry on entry to Idle.

The complete regression baseline is 36 focused smoke tests plus
`csr-mac-demo-split` and the imported-scenario runner, all passing (38/38).

## Remaining PHY boundary

The following behavior is not claimed by this increment:

- promoted scenario-attribute import;
- authoritative off-grid OPNET interpolation and external Earth-LOS/TMM;
- authoritative high-rate DPSK/DQPSK collision curves. The opt-in 500/1000
  kbit/s profile currently reconstructs the recorded jammer as payload noise.
