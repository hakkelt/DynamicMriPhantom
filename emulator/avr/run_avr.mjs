// ATmega328P emulator (avr8js) for the segment_player firmware.
//
//   node run_avr.mjs --hex build/avr-atmega328p/segment_player.hex
//                    [--port 0] [--trace trace.txt] [--realtime] [--max-seconds 60]
//
// * The firmware's UART is bridged to a TCP server on 127.0.0.1 (pyserial URL
//   socket://127.0.0.1:<port>). The chosen port is printed as "LISTENING <port>".
// * Every change of DIR (D2), STEP (D3), BUTTON (D5) and MRI (D6) is recorded
//   with its CPU cycle (16 cycles = 1 us) and appended to --trace on exit/"flush"
//   (nothing is recorded without --trace).
// * Commands on stdin, one per line:
//     ttl [us]      MRI pin high for us microseconds (default 100), then low
//     button [ms]   hold the button (D5, active low) for ms milliseconds (default 20)
//     time          print "TIME <cycle>"
//     flush         append new trace events to the file, print "FLUSHED <total events>"
//     quit          write the trace and exit
// * Without --realtime the simulation runs as fast as it can.
import { appendFileSync, readFileSync, writeFileSync } from 'node:fs';
import net from 'node:net';
import readline from 'node:readline';
import {
  CPU, avrInstruction, AVRTimer, timer0Config, timer1Config, timer2Config,
  AVRUSART, usart0Config, AVRIOPort, portBConfig, portCConfig, portDConfig,
} from 'avr8js';

const F_CPU = 16_000_000;
const PINS = { 2: 'dir', 3: 'step', 5: 'button', 6: 'mri' };

function parseArgs(argv) {
  const a = { port: 0, realtime: false, maxSeconds: 0, trace: null, hex: null };
  for (let i = 0; i < argv.length; i++) {
    const k = argv[i];
    if (k === '--hex') a.hex = argv[++i];
    else if (k === '--port') a.port = Number(argv[++i]);
    else if (k === '--trace') a.trace = argv[++i];
    else if (k === '--realtime') a.realtime = true;
    else if (k === '--max-seconds') a.maxSeconds = Number(argv[++i]);
    else throw new Error(`unknown argument ${k}`);
  }
  if (!a.hex) throw new Error('--hex is required');
  return a;
}

function loadHex(text, target) {
  let base = 0;
  for (const line of text.split(/\r?\n/)) {
    if (!line.startsWith(':')) continue;
    const bytes = Buffer.from(line.slice(1), 'hex');
    const [len, hi, lo, type] = bytes;
    if (type === 0) target.set(bytes.subarray(4, 4 + len), base + (hi << 8) + lo);
    else if (type === 2) base = ((bytes[4] << 8) | bytes[5]) << 4;
    else if (type === 4) base = ((bytes[4] << 8) | bytes[5]) << 16;
  }
}

const args = parseArgs(process.argv.slice(2));
const flash = new Uint8Array(0x8000);
loadHex(readFileSync(args.hex, 'utf8'), flash);
const cpu = new CPU(new Uint16Array(flash.buffer));
new AVRTimer(cpu, timer0Config);
new AVRTimer(cpu, timer1Config);
new AVRTimer(cpu, timer2Config);
const usart = new AVRUSART(cpu, usart0Config, F_CPU);
new AVRIOPort(cpu, portBConfig);
new AVRIOPort(cpu, portCConfig);
const portD = new AVRIOPort(cpu, portDConfig);

// avr8js 0.21.1 drops a pending RX-complete interrupt on every UCSR0B write
// that keeps RXEN set, and the Arduino core writes UCSR0B (UDRIE0) for every
// transmitted byte, so received bytes get lost while the firmware is sending.
// Real hardware keeps RXC pending until UDR0 is read: restore it.
{
  const rxVector = usart0Config.rxCompleteInterrupt;
  const ucsrbHook = cpu.writeHooks[usart0Config.UCSRB];
  cpu.writeHooks[usart0Config.UCSRB] = (value, oldValue, addr, mask) => {
    const pending = cpu.pendingInterrupts[rxVector];
    const flag = cpu.data[usart0Config.UCSRA] & 0x80;
    const result = ucsrbHook(value, oldValue, addr, mask);
    if (pending && !cpu.pendingInterrupts[rxVector] && (value & 0x80)) {
      cpu.data[usart0Config.UCSRA] |= flag;
      cpu.queueInterrupt(pending);
    }
    return result;
  };
}

// --- pin trace ------------------------------------------------------------
// Outputs are traced by the port listener, inputs where this harness drives them.
// Events are buffered until the next flush and then appended to the file.
let pending = [];
let traced = 0;
const record = args.trace
  ? (bit, level) => pending.push(`${cpu.cycles} ${PINS[bit]} ${level}`)
  : () => {};
if (args.trace) writeFileSync(args.trace, '');
let lastD = 0;
portD.addListener((value) => {
  const changed = value ^ lastD;
  lastD = value;
  if (changed & 0x04) record(2, (value >> 2) & 1);
  if (changed & 0x08) record(3, (value >> 3) & 1);
});
function drive(bit, level) {
  portD.setPin(bit, level);
  record(bit, level ? 1 : 0);
}
portD.setPin(5, true); // button released (pull-up)
portD.setPin(6, false);

function writeTrace() {
  if (!args.trace || !pending.length) return;
  appendFileSync(args.trace, pending.join('\n') + '\n');
  traced += pending.length;
  pending = [];
}

// --- serial over TCP ---------------------------------------------------------
const rxQueue = [];
let txPending = [];
let client = null;

function feedRx() {
  if (rxQueue.length && usart.writeByte(rxQueue[0])) rxQueue.shift();
}
usart.onRxComplete = feedRx;
usart.onByteTransmit = (b) => {
  if (client) client.write(Buffer.from([b]));
  else txPending.push(b);
};

const server = net.createServer((sock) => {
  if (client) client.destroy();
  client = sock;
  sock.setNoDelay(true);
  if (txPending.length) {
    sock.write(Buffer.from(txPending));
    txPending = [];
  }
  sock.on('data', (buf) => {
    for (const b of buf) rxQueue.push(b);
    feedRx();
  });
  sock.on('close', () => { if (client === sock) client = null; });
  sock.on('error', () => {});
});
server.listen(args.port, '127.0.0.1', () => {
  console.log(`LISTENING ${server.address().port}`);
});

// --- stdin control -----------------------------------------------------------
let quitting = false;
const scheduled = []; // [cycle, fn]
function at(cycle, fn) { scheduled.push([cycle, fn]); scheduled.sort((x, y) => x[0] - y[0]); }

readline.createInterface({ input: process.stdin }).on('line', (line) => {
  const [cmd, arg] = line.trim().split(/\s+/);
  if (cmd === 'ttl') {
    const us = arg ? Number(arg) : 100;
    drive(6, true);
    at(cpu.cycles + us * 16, () => drive(6, false));
    console.log(`TTL ${cpu.cycles}`);
  } else if (cmd === 'button') {
    const ms = arg ? Number(arg) : 20;
    drive(5, false);
    at(cpu.cycles + ms * 16000, () => drive(5, true));
    console.log(`BUTTON ${cpu.cycles}`);
  } else if (cmd === 'time') {
    console.log(`TIME ${cpu.cycles}`);
  } else if (cmd === 'flush') {
    writeTrace();
    console.log(`FLUSHED ${traced}`);
  } else if (cmd === 'quit') {
    quitting = true;
  }
}).on('close', () => { quitting = true; });

// --- main loop ---------------------------------------------------------------
const SLICE = 1600; // 100 us of simulated time between event-loop turns
const t0 = process.hrtime.bigint();

async function main() {
  while (!quitting) {
    const end = cpu.cycles + SLICE;
    while (cpu.cycles < end) {
      avrInstruction(cpu);
      cpu.tick();
    }
    while (scheduled.length && scheduled[0][0] <= cpu.cycles) scheduled.shift()[1]();
    if (args.maxSeconds && cpu.cycles >= args.maxSeconds * F_CPU) break;
    if (args.realtime) {
      const ahead = cpu.cycles / F_CPU - Number(process.hrtime.bigint() - t0) / 1e9;
      if (ahead > 0.002) {
        await new Promise((r) => setTimeout(r, Math.floor(ahead * 1000)));
        continue;
      }
    }
    await new Promise((r) => setImmediate(r));
  }
  writeTrace();
  console.log(`EXIT ${cpu.cycles}`);
  if (client) client.end();
  server.close();
  process.exit(0);
}
main();
