# Laptop visualizer

This folder contains the laptop half of the V2V demo:

- `bridge.js` forwards UDP datagrams from port `12345` to WebSocket clients on port `8080` without changing the CSV text.
- Browser messages take the reverse path once, from WebSocket to a UDP broadcast. This is used only by the telemetry injector.
- `simulation.html` configures two simulated cars, emits their telemetry at 20 Hz, and passively visualizes all V2V packets heard by the bridge.

## Run

Install Node.js 18 or newer, then run:

```sh
npm install
npm start
```

Open <http://localhost:8000>. The visualizer connects to the same hostname on
WebSocket port `8080`. When using a custom WebSocket port, open (for example)
`http://localhost:8000/?wsPort=8081`.

The default UDP destination is the limited broadcast address `255.255.255.255:12345`. The operating-system firewall must allow inbound and outbound UDP port `12345`. WebSocket injection accepts only loopback browser clients with the visualizer's HTTP origin and only validated `TELEMETRY,CAR1|CAR2,...,SIMULATED` frames.

Optional bridge environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `V2V_UDP_PORT` | `12345` | V2V UDP receive and destination port |
| `V2V_WS_PORT` | `8080` | WebSocket server port |
| `V2V_HTTP_PORT` | `8000` | Local visualizer web server port |
| `V2V_BROADCAST_ADDRESS` | `255.255.255.255` | UDP destination; a subnet-directed broadcast can be used if required by the Wi-Fi adapter |

## Packet assumptions

The browser emits one packet for each car every 50 ms:

```text
TELEMETRY,car_id,latitude,longitude,heading,speed,width,length,SIMULATED
```

Coordinates are decimal degrees, heading is degrees clockwise from north, speed is meters per second, and dimensions are meters. `SIMULATED` lets each Pi distinguish laptop-injected sensor data from its own telemetry.

The primary packets consumed from the Pis are:

```text
PROPOSAL,sender_id,car_1_action,car_2_action
EXECUTION,car_id,action,state,ttc
```

Proposal actions are in canonical vehicle-ID order (`CAR1`, then `CAR2`). Two recent, identical proposals from different senders display as matched consensus. An execution packet immediately applies its action to the named canvas vehicle. `BRAKE` and `EMERGENCY_STOP` decelerate and turn the vehicle red; `SWERVE_LEFT` and `SWERVE_RIGHT` animate opposite heading changes.

For compatibility with early PoC builds, the parser also tolerates `EXECUTE`, `STATUS`, `STATE`, `TTC`, and joint `EXECUTION,sender_id,car_1_action,car_2_action` packets. Unknown or malformed fields are ignored rather than interpreted.

## Loop behavior

The bridge never sends a UDP-received packet back to UDP, and it never sends a browser-received packet directly to WebSocket clients. A browser-injected datagram can naturally be received once by the laptop's UDP socket and displayed, but there is no recursive echo path.
