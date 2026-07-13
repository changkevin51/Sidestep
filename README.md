# SideStep: decentralized V2V collision-avoidance PoC

SideStep demonstrates a decentralized vehicle-to-vehicle collision-avoidance
proof of concept across two Raspberry Pi 5s and one laptop. There is no broker,
central coordinator, or decision server.

```text
laptop sensor injector ──UDP──> CAR1 Pi ──┐
                                         ├─ telemetry + matching proposals
laptop sensor injector ──UDP──> CAR2 Pi ──┘
                    all UDP traffic ──> laptop canvas
```

The laptop broadcasts both configured simulated streams, but each Pi accepts
only the stream matching its own `--id` as local sensor input. Peer data must be
rebroadcast by the other Pi so collision detection, plan selection, consensus,
timeout handling, and execution state remain on the devices.

## Repository layout

- [qnx-brain/](qnx-brain/): C++ implementation for the on-device brains
  - [qnx-brain/main_p2p.cpp](qnx-brain/main_p2p.cpp): two-peer runtime and state machine
  - [qnx-brain/car_collision.cpp](qnx-brain/car_collision.cpp): TTC and collision logic
  - [qnx-brain/udp_network.cpp](qnx-brain/udp_network.cpp): UDP transport
  - [qnx-brain/Makefile](qnx-brain/Makefile): Linux/QNX build rules
- [laptop-visualizer/](laptop-visualizer/): Node.js bridge and browser visualization UI
  - [laptop-visualizer/bridge.js](laptop-visualizer/bridge.js): UDP/WebSocket bridge
  - [laptop-visualizer/simulation.html](laptop-visualizer/simulation.html): telemetry injector and visualizer
  - [laptop-visualizer/ai-service.js](laptop-visualizer/ai-service.js): optional Gemini AI decision service

## Network preparation

Connect all three devices to the same Wi-Fi subnet. The access point must allow
client-to-client traffic and UDP broadcast; disable wireless/AP isolation if the
router enforces it. Allow UDP `12345` through each host firewall. The laptop
also needs TCP `8000` for the web UI and TCP `8080` for the WebSocket bridge.

The bridge and the Pi runtimes can both use a broadcast target. The default in
[laptop-visualizer/bridge.js](laptop-visualizer/bridge.js) is the current subnet
broadcast address, and it can be overridden with `V2V_BROADCAST_ADDRESS` if the
network needs a different directed broadcast.

## 1. Build and start both Pi brains

Copy [qnx-brain/](qnx-brain/) to each Pi. In a configured QNX 8 SDP shell, build with:

```sh
cd qnx-brain
make clean
make CXX=q++ QNX_VARIANT=gcc_ntoaarch64le
```

The same source can be smoke-tested on Linux with:

```sh
cd qnx-brain
make clean
make CXX=g++
```

Run `make clean` when switching toolchains because the object names are shared.
QNX builds select the Pi 5 AArch64 target, define QNX/POSIX features, and link
`libsocket` when `QNX_TARGET` is set.

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

Install Node.js 18 or newer, then run:

```sh
cd laptop-visualizer
npm install
npm start
```

Open <http://localhost:8000>, configure both cars, and select Start simulation.
The default crossing scenario exercises the deterministic under-three-second
path. Increase the starting distance to exercise the smart-model stub, or use a
closer/higher-speed crossing to exercise opposite swerves.

The bridge forwards incoming UDP packets to the browser unchanged. Its role is
limited to carrying browser-configured sensor samples back to UDP; it never
computes or approves an evasion plan.

If you want to try the browser-side Gemini workflow, create a local
[laptop-visualizer/config.json](laptop-visualizer/config.json) file with your
Gemini API settings before starting the bridge. The visualizer will expose
`/api/ai-decision` and `/api/ai-config` for that flow.

## Wire protocol

All messages are UTF-8 CSV datagrams on UDP `12345`:

```text
TELEMETRY,car_id,latitude,longitude,heading,speed,width,length,source_state_or_scenario
PROPOSAL,sender_id,car1_action,car2_action
EXECUTION,car_id,action,state,ttc
RESET,scenario_id
```

`car1_action` and `car2_action` in a proposal are emitted in canonical
vehicle-ID order (`CAR1`, then `CAR2`), so both Pis calculate the same
right/left assignment. Proposals repeat at 20 Hz until a match or the 150 ms
consensus deadline. A match transitions into `EXECUTE`; a timeout transitions
into `EMERGENCY_STOP`. Execution packets repeat for reliable visualization
despite UDP packet loss.

## Decision behavior

- TTC is found with a 10-second, 50 ms OBB/SAT sweep in a local east/north
  coordinate frame.
- Every Start/Restart uses a new scenario tag. Each Pi clears its prior
  proposal, execution action, and cached car pair before evaluating that tag.
- At TTC below 3 seconds, both cars brake only when both stopping-distance
  checks pass. Otherwise canonical car 1 swerves right and car 2 swerves left.
- At TTC of 3 seconds or more, `evaluateSmartDecision(Car, Car)` is the explicit
  onboard-model integration point. The PoC stub returns conservative braking for
  both cars.
- Matching proposals trigger the local execution state and action-specific
  warning audio. The browser applies the same execution packet to its simulated
  motion.

This is a demonstration, not a production automotive safety controller. It has
no authenticated transport, clock synchronization, redundant sensing, vehicle
actuator driver, functional-safety case, or certified real-time bounds.
