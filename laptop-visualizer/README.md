# Laptop visualizer

This folder contains the laptop half of the V2V demo:

- `bridge.js` forwards UDP datagrams from port `12345` to WebSocket clients on port `8080` without changing the CSV text.
- Browser messages take the reverse path once, from WebSocket to a UDP broadcast. This is used only by the telemetry injector.
- `simulation.html` configures two simulated cars, emits their telemetry at 20 Hz, and passively visualizes all V2V packets heard by the bridge.
- `driver-alert.html` is the phone speaker client. An authenticated CAR1/CAR2
  socket receives only that Pi's ElevenLabs MP3 alert.

## Run

Install Node.js 18 or newer, then run:

```sh
npm install
npm start
```

Open <http://localhost:8000>. The visualizer connects to the same hostname on
WebSocket port `8080`. When using a custom WebSocket port, open (for example)
`http://localhost:8000/?wsPort=8081`.

Reset stops the telemetry scheduler, clears both vehicles and all decision
state, and ignores incoming scenario packets until Start is pressed again.

The default UDP destination is the Windows-hotspot subnet broadcast address
`192.168.137.255:12345`. Set `V2V_BROADCAST_ADDRESS` for another subnet. The
operating-system firewall must allow inbound and outbound UDP port `12345`.
WebSocket injection accepts only loopback browser clients with the visualizer's
HTTP origin and only validated tagged `TELEMETRY` or `RESET` frames.

Optional bridge environment variables:

| Variable | Default | Purpose |
| --- | --- | --- |
| `V2V_UDP_PORT` | `12345` | V2V UDP receive and destination port |
| `V2V_WS_PORT` | `8080` | WebSocket server port |
| `V2V_HTTP_PORT` | `8000` | Local visualizer web server port |
| `V2V_HTTP_BIND_ADDRESS` | `0.0.0.0` | HTTP bind address so Pis and phones can reach the bridge |
| `V2V_BROADCAST_ADDRESS` | `192.168.137.255` | UDP destination; change this to the hotspot's subnet-directed broadcast |
| `V2V_ALERT_TOKEN` | empty (alerts disabled) | Shared random secret required by Pi uploads and phone sockets |

For the two-phone setup, libcurl/ElevenLabs configuration, media formats, and
firewall instructions, see [Driver audio setup](../AUDIO_SETUP.md).

## Packet assumptions

The browser emits one packet for each car every 50 ms:

```text
TELEMETRY,car_id,latitude,longitude,heading,speed,width,length,SIMULATED_scenario
```

Reset emits `RESET,scenario_id` so both Pi runtimes immediately clear cached
telemetry, proposals, and execution state. The per-run telemetry suffix provides
the same boundary again on Start/Restart if that UDP reset packet was lost.

Coordinates are decimal degrees, heading is degrees clockwise from north, speed is meters per second, and dimensions are meters. The `SIMULATED_` prefix lets each Pi distinguish laptop-injected sensor data, while the per-run suffix makes Start/Restart clear any prior proposal and execution state before the new values are evaluated.

The primary packets consumed from the Pis are:

```text
PROPOSAL,sender_id,car_1_action,car_2_action
EXECUTION,car_id,action,state,ttc
```

Proposal actions are in canonical vehicle-ID order (`CAR1`, then `CAR2`). Two recent, identical proposals from different senders display as matched consensus. An execution packet immediately applies its action to the named canvas vehicle. `BRAKE` and `EMERGENCY_STOP` decelerate and turn the vehicle red; `SWERVE_LEFT` and `SWERVE_RIGHT` animate opposite heading changes.

For compatibility with early PoC builds, the parser also tolerates `EXECUTE`, `STATUS`, `STATE`, `TTC`, and joint `EXECUTION,sender_id,car_1_action,car_2_action` packets. Unknown or malformed fields are ignored rather than interpreted.

## Loop behavior

The bridge never sends a UDP-received packet back to UDP, and it never sends a browser-received packet directly to WebSocket clients. If the host receives its own browser-injected UDP broadcast, the bridge suppresses that delayed copy instead of forwarding it back to the browser. External UDP packets continue to be forwarded normally.
