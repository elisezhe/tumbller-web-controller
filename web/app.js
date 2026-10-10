import { COMMANDS, TumbllerBluetooth } from './controller.js';
import { commandForKey, isMotionCommand } from './input.js';

const client = new TumbllerBluetooth();
const connectButton = document.querySelector('#connect-button');
const disconnectButton = document.querySelector('#disconnect-button');
const statusDot = document.querySelector('#status-dot');
const statusText = document.querySelector('#status-text');
const deviceText = document.querySelector('#device-text');
const telemetry = document.querySelector('#telemetry');
const commandText = document.querySelector('#last-command');
const controls = [...document.querySelectorAll('[data-command]')];
let heldMotion = null;

function setStatus(connected, label = 'ELEGOO BT16') {
  document.body.dataset.connected = String(connected);
  statusDot.classList.toggle('online', connected);
  statusText.textContent = connected ? 'Connected' : 'Disconnected';
  deviceText.textContent = connected ? label : 'No robot selected';
  connectButton.hidden = connected;
  disconnectButton.hidden = !connected;
  controls.forEach((control) => {
    control.disabled = !connected;
  });
}

function log(message, kind = 'info') {
  const entry = document.createElement('div');
  entry.className = `log-entry ${kind}`;
  entry.innerHTML = `<time>${new Date().toLocaleTimeString()}</time><span></span>`;
  entry.querySelector('span').textContent = message;
  telemetry.prepend(entry);
  while (telemetry.children.length > 30) telemetry.lastElementChild.remove();
}

async function send(command, label) {
  try {
    await client.send(command);
    commandText.textContent = `${label} · ${command}`;
    log(`Sent ${label} (${command})`, 'sent');
  } catch (error) {
    log(error.message, 'error');
  }
}

async function stopHeldMotion() {
  if (!heldMotion || !client.connected) return;
  heldMotion = null;
  await send(COMMANDS.stop, 'Stop');
  document.querySelectorAll('.is-active').forEach((button) => button.classList.remove('is-active'));
}

client.onConnectionChange = (connected, name) => {
  setStatus(connected, name);
  log(connected ? `Connected to ${name}` : 'Robot disconnected', connected ? 'success' : 'warning');
};
client.onTelemetry = (data) => log(data.trim() || '[binary data]', 'received');

connectButton.addEventListener('click', async () => {
  connectButton.disabled = true;
  connectButton.textContent = 'Connecting…';
  try {
    await client.connect();
  } catch (error) {
    log(error.message, 'error');
  } finally {
    connectButton.disabled = false;
    connectButton.textContent = 'Connect robot';
  }
});

disconnectButton.addEventListener('click', () => client.disconnect());

document.querySelector('#emergency-stop').addEventListener('click', () => {
  heldMotion = null;
  send(COMMANDS.stop, 'Emergency stop');
});

for (const control of controls) {
  const command = control.dataset.command;
  const label = control.dataset.label || control.textContent.trim();
  const hold = control.dataset.hold === 'true';

  if (hold) {
    control.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      heldMotion = command;
      control.classList.add('is-active');
      send(command, label);
      control.setPointerCapture?.(event.pointerId);
    });
    control.addEventListener('pointerup', stopHeldMotion);
    control.addEventListener('pointercancel', stopHeldMotion);
    control.addEventListener('lostpointercapture', stopHeldMotion);
  } else {
    control.addEventListener('click', () => {
      if (command === COMMANDS.standUp && !window.confirm('Run the stand-up maneuver? Keep hands clear of the wheels.')) return;
      send(command, label);
    });
  }
}

document.addEventListener('keydown', (event) => {
  if (event.repeat || !client.connected) return;
  const command = commandForKey(event.key);
  if (!command) return;
  event.preventDefault();
  if (isMotionCommand(command)) heldMotion = command;
  send(command, event.code.replace('Key', ''));
});

document.addEventListener('keyup', (event) => {
  const command = commandForKey(event.key);
  if (command && isMotionCommand(command)) {
    event.preventDefault();
    stopHeldMotion();
  }
});

window.addEventListener('blur', stopHeldMotion);
window.addEventListener('pointerup', stopHeldMotion);

if (!navigator.bluetooth) {
  log('Web Bluetooth is unavailable. Use Chrome or Edge and open this page through localhost.', 'error');
  connectButton.disabled = true;
}
setStatus(false);
