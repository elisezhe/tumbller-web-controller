export const DEVICE_NAME = 'ELEGOO BT16';
export const UART_SERVICE_UUID = '0000ffe0-0000-1000-8000-00805f9b34fb';
export const UART_CHARACTERISTIC_UUID = '0000ffe1-0000-1000-8000-00805f9b34fb';

export const COMMANDS = Object.freeze({
  forward: 'f',
  backward: 'b',
  left: 'l',
  right: 'i',
  stop: 's',
  followWide: '0',
  follow: '1',
  obstacle: '2',
  cycleLights: '3',
  glow: 'g',
  slower: '-',
  faster: '+',
  standUp: '4',
  initialize: '5',
  resetBrightness: '6',
  dim: '7',
  brighten: '8',
  lightsOff: '9',
});

const VALID_COMMANDS = new Set(Object.values(COMMANDS));
const encoder = new TextEncoder();
const decoder = new TextDecoder();

export function encodeCommand(command) {
  if (typeof command !== 'string' || command.length !== 1 || !VALID_COMMANDS.has(command)) {
    throw new Error(`Unsupported Tumbller command: ${String(command)}`);
  }
  return encoder.encode(command);
}

export class TumbllerBluetooth {
  constructor(bluetooth = globalThis.navigator?.bluetooth) {
    this.bluetooth = bluetooth;
    this.device = null;
    this.characteristic = null;
    this.onConnectionChange = () => {};
    this.onTelemetry = () => {};
    this.handleDisconnected = this.handleDisconnected.bind(this);
    this.handleNotification = this.handleNotification.bind(this);
  }

  get connected() {
    return Boolean(this.characteristic && (!this.device?.gatt || this.device.gatt.connected !== false));
  }

  async connect() {
    if (!this.bluetooth?.requestDevice) {
      throw new Error('Web Bluetooth is unavailable. Open this page in Chrome or Edge on localhost.');
    }

    this.device = await this.bluetooth.requestDevice({
      filters: [{ name: DEVICE_NAME }],
      optionalServices: [UART_SERVICE_UUID],
    });
    this.device.addEventListener?.('gattserverdisconnected', this.handleDisconnected);

    const server = await this.device.gatt.connect();
    const service = await server.getPrimaryService(UART_SERVICE_UUID);
    this.characteristic = await service.getCharacteristic(UART_CHARACTERISTIC_UUID);
    this.characteristic.addEventListener?.('characteristicvaluechanged', this.handleNotification);
    if (this.characteristic.startNotifications) {
      await this.characteristic.startNotifications();
    }
    this.onConnectionChange(true, this.device.name || DEVICE_NAME);
    return this.device;
  }

  async disconnect() {
    if (this.characteristic?.stopNotifications) {
      await this.characteristic.stopNotifications().catch(() => {});
    }
    this.device?.gatt?.disconnect?.();
    this.handleDisconnected();
  }

  async send(command) {
    if (!this.characteristic) {
      throw new Error('Robot is not connected.');
    }
    const value = encodeCommand(command);
    if (this.characteristic.writeValueWithoutResponse) {
      await this.characteristic.writeValueWithoutResponse(value);
    } else {
      await this.characteristic.writeValue(value);
    }
  }

  handleNotification(event) {
    const value = event.target?.value;
    if (!value) return;
    const bytes = new Uint8Array(value.buffer, value.byteOffset, value.byteLength);
    this.onTelemetry(decoder.decode(bytes));
  }

  handleDisconnected() {
    this.characteristic = null;
    this.onConnectionChange(false, this.device?.name || DEVICE_NAME);
  }
}
