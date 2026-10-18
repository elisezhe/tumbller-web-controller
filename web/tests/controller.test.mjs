import test from 'node:test';
import assert from 'node:assert/strict';

import {
  COMMANDS,
  TumbllerBluetooth,
  UART_CHARACTERISTIC_UUID,
  UART_SERVICE_UUID,
  encodeCommand,
} from '../controller.js';

test('encodeCommand accepts every firmware command as one byte', () => {
  for (const command of Object.values(COMMANDS)) {
    assert.deepEqual([...encodeCommand(command)], [command.charCodeAt(0)]);
  }
});

test('glow mode is exposed as the dedicated g firmware command', () => {
  assert.equal(COMMANDS.glow, 'g');
  assert.deepEqual([...encodeCommand(COMMANDS.glow)], ['g'.charCodeAt(0)]);
});

test('speed controls expose dedicated slower and faster firmware commands', () => {
  assert.equal(COMMANDS.slower, '-');
  assert.equal(COMMANDS.faster, '+');
  assert.deepEqual([...encodeCommand(COMMANDS.slower)], ['-'.charCodeAt(0)]);
  assert.deepEqual([...encodeCommand(COMMANDS.faster)], ['+'.charCodeAt(0)]);
});

test('encodeCommand rejects commands outside the firmware protocol', () => {
  assert.throws(() => encodeCommand('x'), /Unsupported Tumbller command/);
  assert.throws(() => encodeCommand('ff'), /Unsupported Tumbller command/);
});

test('connect requests ELEGOO BT16 and resolves the UART characteristic', async () => {
  const calls = [];
  const characteristic = { startNotifications: async () => calls.push('notify') };
  const service = {
    getCharacteristic: async (uuid) => {
      calls.push(['characteristic', uuid]);
      return characteristic;
    },
  };
  const server = {
    getPrimaryService: async (uuid) => {
      calls.push(['service', uuid]);
      return service;
    },
  };
  const device = {
    name: 'ELEGOO BT16',
    gatt: { connect: async () => server },
    addEventListener: () => {},
  };
  const bluetooth = {
    requestDevice: async (options) => {
      calls.push(['request', options]);
      return device;
    },
  };

  const client = new TumbllerBluetooth(bluetooth);
  await client.connect();

  assert.equal(client.connected, true);
  assert.deepEqual(calls[0], [
    'request',
    { filters: [{ name: 'ELEGOO BT16' }], optionalServices: [UART_SERVICE_UUID] },
  ]);
  assert.deepEqual(calls[1], ['service', UART_SERVICE_UUID]);
  assert.deepEqual(calls[2], ['characteristic', UART_CHARACTERISTIC_UUID]);
  assert.equal(calls[3], 'notify');
});

test('send writes a firmware command without a response', async () => {
  const writes = [];
  const characteristic = {
    writeValueWithoutResponse: async (value) => writes.push([...value]),
  };
  const client = new TumbllerBluetooth({});
  client.characteristic = characteristic;

  await client.send(COMMANDS.stop);

  assert.deepEqual(writes, [[COMMANDS.stop.charCodeAt(0)]]);
});

test('send refuses to run before a BLE connection exists', async () => {
  const client = new TumbllerBluetooth({});
  await assert.rejects(client.send(COMMANDS.forward), /Robot is not connected/);
});
