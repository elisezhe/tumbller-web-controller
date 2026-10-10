import test from 'node:test';
import assert from 'node:assert/strict';

import { commandForKey, isMotionCommand } from '../input.js';

test('keyboard controls map arrows and WASD to firmware motion commands', () => {
  assert.equal(commandForKey('ArrowUp'), 'f');
  assert.equal(commandForKey('w'), 'f');
  assert.equal(commandForKey('ArrowDown'), 'b');
  assert.equal(commandForKey('a'), 'l');
  assert.equal(commandForKey('ArrowRight'), 'i');
  assert.equal(commandForKey(' '), 's');
});

test('keyboard controls ignore unrelated keys and normalize letter case', () => {
  assert.equal(commandForKey('D'), 'i');
  assert.equal(commandForKey('Enter'), null);
});

test('motion command detection excludes stop and mode commands', () => {
  assert.equal(isMotionCommand('f'), true);
  assert.equal(isMotionCommand('i'), true);
  assert.equal(isMotionCommand('s'), false);
  assert.equal(isMotionCommand('2'), false);
});
