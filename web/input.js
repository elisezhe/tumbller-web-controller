const KEY_COMMANDS = Object.freeze({
  ArrowUp: 'f',
  w: 'f',
  ArrowDown: 'b',
  s: 'b',
  ArrowLeft: 'l',
  a: 'l',
  ArrowRight: 'i',
  d: 'i',
  ' ': 's',
});

const MOTION_COMMANDS = new Set(['f', 'b', 'l', 'i']);

export function commandForKey(key) {
  return KEY_COMMANDS[key] ?? KEY_COMMANDS[key.toLowerCase?.()] ?? null;
}

export function isMotionCommand(command) {
  return MOTION_COMMANDS.has(command);
}
