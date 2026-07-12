'use strict';

const dgram = require('dgram');
const fs = require('fs');
const http = require('http');
const path = require('path');
const { WebSocket, WebSocketServer } = require('ws');

const UDP_PORT = Number.parseInt(process.env.V2V_UDP_PORT || '12345', 10);
const WS_PORT = Number.parseInt(process.env.V2V_WS_PORT || '8080', 10);
const HTTP_PORT = Number.parseInt(process.env.V2V_HTTP_PORT || '8000', 10);
const BROADCAST_ADDRESS = process.env.V2V_BROADCAST_ADDRESS || '255.255.255.255';
const MAX_PACKET_BYTES = 4096;
const MAX_WEBSOCKET_BACKLOG_BYTES = 64 * 1024;
const MAX_INSTRUCTION_BODY_BYTES = 8 * 1024;

function validPort(value) {
  return Number.isInteger(value) && value > 0 && value <= 65535;
}

if (![UDP_PORT, WS_PORT, HTTP_PORT].every(validPort)) {
  throw new Error('V2V_UDP_PORT, V2V_WS_PORT, and V2V_HTTP_PORT must be valid TCP/UDP ports.');
}

function isLoopback(address) {
  return address === '127.0.0.1' || address === '::1' || address === '::ffff:127.0.0.1';
}

function validBrowserTelemetry(rawMessage) {
  if (typeof rawMessage !== 'string' || rawMessage.length === 0
      || Buffer.byteLength(rawMessage, 'utf8') > MAX_PACKET_BYTES
      || rawMessage.includes('\0') || rawMessage.includes('\n') || rawMessage.includes('\r')) {
    return false;
  }

  const fields = rawMessage.split(',');
  if (fields.length !== 9 || fields.some((field) => field.length === 0 || field !== field.trim())
      || fields[0] !== 'TELEMETRY' || !/^(CAR1|CAR2)$/.test(fields[1])
      || fields[8] !== 'SIMULATED') {
    return false;
  }

  const numbers = fields.slice(2, 8).map(Number);
  return numbers.every(Number.isFinite)
    && numbers[0] >= -90 && numbers[0] <= 90
    && numbers[1] >= -180 && numbers[1] <= 180
    && numbers[2] >= 0 && numbers[2] < 360
    && numbers[3] >= 0 && numbers[3] <= 120
    && numbers[4] > 0 && numbers[4] <= 10
    && numbers[5] > 0 && numbers[5] <= 30;
}

function validIdentifier(value) {
  return typeof value === 'string' && /^[A-Za-z0-9_-]{1,32}$/.test(value);
}

function validInstructionText(value) {
  return typeof value === 'string'
    && value.length > 0
    && value.length <= 128
    && !value.includes(',')
    && !value.includes('\0')
    && !value.includes('\n')
    && !value.includes('\r');
}

function parseInstructionRequest(bodyText) {
  let parsed;
  try {
    parsed = JSON.parse(bodyText);
  } catch (err) {
    return null;
  }

  if (!parsed || typeof parsed !== 'object') {
    return null;
  }

  const seq = Number(parsed.seq);
  const ttlMs = Number(parsed.ttlMs);
  const targetId = parsed.targetId;
  const action = parsed.action;
  const reason = typeof parsed.reason === 'string' ? parsed.reason.trim() : '';

  if (!Number.isInteger(seq) || seq <= 0 || !Number.isSafeInteger(seq)
      || !Number.isInteger(ttlMs) || ttlMs <= 0 || ttlMs > 60000
      || !validIdentifier(targetId) || !validIdentifier(action)
      || (reason.length > 0 && !validInstructionText(reason))) {
    return null;
  }

  return {
    seq,
    targetId,
    action,
    ttlMs,
    reason,
  };
}

function formatInstructionPacket(instruction) {
  return [
    'INSTRUCTION',
    String(instruction.seq),
    instruction.targetId,
    instruction.action,
    String(instruction.ttlMs),
    instruction.reason || 'gemini',
  ].join(',');
}

function readRequestBody(request) {
  return new Promise((resolve, reject) => {
    let body = '';
    request.on('data', (chunk) => {
      body += chunk;
      if (Buffer.byteLength(body, 'utf8') > MAX_INSTRUCTION_BODY_BYTES) {
        reject(new Error('request body too large'));
        request.destroy();
      }
    });
    request.on('end', () => resolve(body));
    request.on('error', reject);
  });
}

function broadcastInstruction(instruction) {
  const packet = Buffer.from(formatInstructionPacket(instruction), 'utf8');
  return new Promise((resolve, reject) => {
    udpSender.send(packet, UDP_PORT, BROADCAST_ADDRESS, (err) => {
      if (err) {
        reject(err);
        return;
      }
      resolve();
    });
  });
}

const allowedOrigins = new Set([
  `http://localhost:${HTTP_PORT}`,
  `http://127.0.0.1:${HTTP_PORT}`,
  `http://[::1]:${HTTP_PORT}`,
]);
const udpReceiver = dgram.createSocket({ type: 'udp4', reuseAddr: true });
const udpSender = dgram.createSocket('udp4');
const webSockets = new WebSocketServer({
  port: WS_PORT,
  maxPayload: MAX_PACKET_BYTES,
  verifyClient: (info) => isLoopback(info.req.socket.remoteAddress)
    && (!info.origin || allowedOrigins.has(info.origin)),
});

let udpSenderReady = false;
let shuttingDown = false;

function log(message) {
  process.stdout.write(`[bridge] ${message}\n`);
}

function error(message, detail) {
  const suffix = detail && detail.message ? `: ${detail.message}` : '';
  process.stderr.write(`[bridge] ${message}${suffix}\n`);
}

// UDP packets travel in one direction only: UDP -> WebSocket. They are never
// retransmitted to UDP here, which prevents a broadcast echo loop.
udpReceiver.on('message', (packet) => {
  if (packet.length === 0 || packet.length > MAX_PACKET_BYTES) {
    return;
  }

  // V2V packets are UTF-8 CSV strings. No envelope, prefix, or field changes
  // are introduced by the bridge.
  const rawMessage = packet.toString('utf8');
  for (const client of webSockets.clients) {
    if (client.readyState === WebSocket.OPEN
        && client.bufferedAmount <= MAX_WEBSOCKET_BACKLOG_BYTES) {
      client.send(rawMessage);
    }
  }
});

udpReceiver.on('error', (err) => {
  error('UDP receiver error', err);
});

udpReceiver.bind(UDP_PORT, '0.0.0.0', () => {
  const address = udpReceiver.address();
  log(`listening for UDP on ${address.address}:${address.port}`);
});

udpSender.on('error', (err) => {
  error('UDP sender error', err);
});

udpSender.bind(0, '0.0.0.0', () => {
  try {
    udpSender.setBroadcast(true);
    udpSenderReady = true;
    log(`browser telemetry broadcasts to ${BROADCAST_ADDRESS}:${UDP_PORT}`);
  } catch (err) {
    error('could not enable UDP broadcasting', err);
  }
});

webSockets.on('listening', () => {
  log(`WebSocket server listening on ws://localhost:${WS_PORT}`);
});

webSockets.on('connection', (socket, request) => {
  const peer = request.socket.remoteAddress || 'unknown client';
  log(`visualizer connected (${peer})`);

  // WebSocket packets travel in one direction only: browser -> UDP. The UDP
  // receiver may naturally observe the host's own broadcast once, but neither
  // side rebroadcasts a received packet, so it cannot form a loop.
  socket.on('message', (data, isBinary) => {
    if (isBinary) {
      return;
    }

    const rawMessage = data.toString('utf8');
    if (!udpSenderReady || !validBrowserTelemetry(rawMessage)) return;
    const packet = Buffer.from(rawMessage, 'utf8');

    udpSender.send(packet, UDP_PORT, BROADCAST_ADDRESS, (err) => {
      if (err) {
        error('could not broadcast browser telemetry', err);
      }
    });
  });

  socket.on('error', (err) => {
    error(`WebSocket client error (${peer})`, err);
  });

  socket.on('close', () => {
    log(`visualizer disconnected (${peer})`);
  });
});

webSockets.on('error', (err) => {
  error('WebSocket server error', err);
});

// A tiny static server keeps setup to one command and avoids browser file://
// restrictions. It deliberately serves only the visualizer page.
const visualizerPath = path.join(__dirname, 'simulation.html');
const httpServer = http.createServer(async (request, response) => {
  const requestPath = (request.url || '/').split('?')[0];
  if (request.method === 'POST' && requestPath === '/instruction') {
    try {
      const body = await readRequestBody(request);
      const instruction = parseInstructionRequest(body);
      if (!udpSenderReady) {
        response.writeHead(503, { 'Content-Type': 'application/json; charset=utf-8' });
        response.end(JSON.stringify({ ok: false, error: 'UDP sender is not ready yet' }));
        return;
      }
      if (!instruction) {
        response.writeHead(400, { 'Content-Type': 'application/json; charset=utf-8' });
        response.end(JSON.stringify({ ok: false, error: 'invalid instruction payload' }));
        return;
      }

      await broadcastInstruction(instruction);
      response.writeHead(202, { 'Content-Type': 'application/json; charset=utf-8' });
      response.end(JSON.stringify({ ok: true }));
    } catch (err) {
      error('could not broadcast instruction', err);
      response.writeHead(500, { 'Content-Type': 'application/json; charset=utf-8' });
      response.end(JSON.stringify({ ok: false, error: 'could not broadcast instruction' }));
    }
    return;
  }

  if (requestPath === '/favicon.ico') {
    response.writeHead(204);
    response.end();
    return;
  }

  if (requestPath !== '/' && requestPath !== '/simulation.html') {
    response.writeHead(404, { 'Content-Type': 'text/plain; charset=utf-8' });
    response.end('Not found');
    return;
  }

  fs.readFile(visualizerPath, (err, page) => {
    if (err) {
      error('could not read simulation.html', err);
      response.writeHead(500, { 'Content-Type': 'text/plain; charset=utf-8' });
      response.end('Visualizer page unavailable');
      return;
    }

    response.writeHead(200, {
      'Cache-Control': 'no-store',
      'Content-Type': 'text/html; charset=utf-8',
    });
    response.end(page);
  });
});

httpServer.on('error', (err) => {
  error('HTTP server error', err);
});

httpServer.listen(HTTP_PORT, '127.0.0.1', () => {
  log(`visualizer available at http://localhost:${HTTP_PORT}`);
});

function shutdown(signal) {
  if (shuttingDown) {
    return;
  }
  shuttingDown = true;
  log(`received ${signal}; shutting down`);

  for (const client of webSockets.clients) {
    client.close(1001, 'Bridge shutting down');
  }

  httpServer.close();
  webSockets.close();
  udpReceiver.close();
  udpSender.close();
}

process.on('SIGINT', () => shutdown('SIGINT'));
process.on('SIGTERM', () => shutdown('SIGTERM'));
