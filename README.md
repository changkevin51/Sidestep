# Decentralized V2V collision-avoidance PoC

This project runs the safety decision independently on two Raspberry Pi 5s and
uses one laptop for scenario input and visualization. There is no broker,
central coordinator, or decision server.

```text
laptop sensor injector ──UDP──> CAR1 Pi ──┐
                                         ├─ telemetry + matching proposals
laptop sensor injector ──UDP──> CAR2 Pi ──┘
                    all UDP traffic ──> laptop canvas
```

The laptop broadcasts both configured `SIMULATED` streams, but each Pi accepts
only the stream matching its own `--id` as local sensor input. Peer data must be
rebroadcast by the other Pi. This keeps collision detection, plan selection,
consensus, timeout handling, and execution state on the Pis.

## Network preparation

Connect all three devices to the same Wi-Fi subnet. The access point must allow
client-to-client traffic and UDP broadcast; disable wireless/AP isolation. Allow
UDP `12345` in each host firewall. The laptop also needs local TCP `8000` and
WebSocket TCP `8080`.

The default destination is `255.255.255.255`. If the Wi-Fi adapter or router
drops limited broadcasts, set `V2V_BROADCAST_ADDRESS` on the laptop to the
subnet-directed address (for example, `192.168.1.255`) and make the equivalent
address change in `udp_network.cpp` before building the Pis.

## 1. Build and start both Pi brains

Copy `qnx-brain/` to each Pi. With a configured QNX 8 SDP shell:

```sh
cd qnx-brain
make clean
make CXX=q++ QNX_VARIANT=gcc_ntoaarch64le
```

The same source can be smoke-tested on Linux with `make CXX=g++`. Run `make
clean` when switching host/target toolchains because they share object names.
QNX builds automatically select the Pi 5 AArch64 target, define QNX/POSIX
features, and link `libsocket` when `QNX_TARGET` is set.

The initial QNX SDP 8.0 image does not bundle an audio framework. For the alert
hook, either deploy an executable `/home/qnx/bin/v2v_alert` that accepts
`brake`, `swerve`, or `emergency_stop` (a GPIO buzzer/LED is sufficient), or
port/install an `aplay`-compatible player and place warning files at:

```text
/home/qnx/assets/brake_warning.wav
/home/qnx/assets/swerve_warning.wav
/home/qnx/assets/emergency_warning.wav
```

Then start one identity on each Pi:

```sh
# Pi 1
./v2v_brain --id CAR1 --peer CAR2

# Pi 2
./v2v_brain --id CAR2 --peer CAR1
```

Use `--no-audio` for a bench test without either alert backend. IDs are
case-sensitive and must match the simulator's `CAR1` and `CAR2` labels.

## 2. Start the laptop bridge and visualizer

Install Node.js 18 or newer, then:

```sh
cd laptop-visualizer
npm install
npm start
```

Open <http://localhost:8000>, configure both cars, and select **Start
simulation**. The default crossing scenario exercises the under-three-second
deterministic path. Increase the starting distance to exercise the smart-model
stub, or use a closer/higher-speed crossing to exercise opposite swerves.

The bridge forwards UDP packets to the browser unchanged. Its only active role
is carrying browser-configured sensor samples back to UDP; it never computes or
approves an evasion plan.

## Wire protocol

All messages are UTF-8 CSV datagrams on UDP `12345`:

```text
TELEMETRY,car_id,latitude,longitude,heading,speed,width,length,state_or_scenario
PROPOSAL,sender_id,car_1_acceleration,car_1_steering_rate,car_1_duration,car_2_acceleration,car_2_steering_rate,car_2_duration
EXECUTION,car_id,acceleration,steering_rate,duration,state,ttc
RESET,scenario_id
```

`car_1` and `car_2` in a proposal are the lexicographically ordered IDs, so both
Pis calculate the same right/left assignment. Proposals repeat at 20 Hz until a
match or the deadline. Deterministic decisions use 150 ms; Gemini decisions use
20.5 seconds to tolerate the two peers' independent cloud calls with a 20-second
request timeout. A match enters `EXECUTE`; a timeout enters `EMERGENCY_STOP`.
Execution packets repeat for reliable visualization despite UDP packet loss.

## Decision behavior

- TTC is found with a 10-second, 50 ms OBB/SAT sweep in a local east/north
  coordinate frame.
- Every Start/Restart uses a new scenario tag. Each Pi clears its prior
  proposal, execution action, and cached car pair before evaluating that tag.
- At TTC below 3 seconds, both cars brake only when both stopping-distance
  checks pass. Otherwise canonical car 1 receives a positive steering rate and
  car 2 a negative steering rate for 2.2 seconds.
- At TTC of 3 seconds or more,
  `evaluateSmartDecision(Car, Car, ttc_seconds)` is the explicit
  model integration point. The default build returns conservative braking. A
  build with `ENABLE_GEMINI=1` maps the live canonical car pair into the Gemini
  client and maps its structured response back into `JointActions`.
- Matching proposals trigger the local execution state and control-specific
  warning audio. The browser applies that same execution packet to its simulated
  motion.

## Optional Gemini integration

The normal build has no cloud dependency and uses the deterministic smart-path
fallback. On the development Mac, build against the adjacent Gemini client with:

```bash
cd qnx-brain
make clean
make ENABLE_GEMINI=1 GEMINI_DIR="../../../Gemma 3"
GEMINI_API_KEY="your-key" ./v2v_brain --id CAR1 --peer CAR2
```

The enabled build passes the live canonical car pair and computed TTC directly
to `GeminiClient`; it does not run the demo `collision_ai` executable because
that executable currently contains a hardcoded vehicle scenario. Gemini calls
use a 20-second request timeout and fall back to deterministic braking on missing
credentials, timeout, client failure, or controls rejected by SideStep.

The provided default include and link flags target Homebrew on Apple Silicon.
A QNX build requires QNX-compatible libcurl and nlohmann-json installations and
appropriate `GEMINI_CPPFLAGS` and `GEMINI_LDLIBS` overrides. Cloud latency and
independent model nondeterminism must be tested before treating this pathway as
safety-capable; SideStep still requires matching peer proposals before execute.

This is a demonstration, not a production automotive safety controller. It has
no authenticated transport, clock synchronization, redundant sensing, vehicle
actuator driver, functional-safety case, or certified real-time bounds.
