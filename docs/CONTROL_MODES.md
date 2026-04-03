# Control Modes

## 1. Auto wall-follow mode

`MODE_AUTO_WALL`

The robot reads front and right ToF distance measurements and switches between:

- obstacle response when the front wall is too close
- wall-following when the right wall is present
- search behavior when no wall is detected

## 2. Manual WiFi mode

`MODE_MANUAL`

The robot accepts browser commands for:

- forward
- backward
- turn left
- turn right
- stop

## 3. Servo modes

### Hold
`SERVO_HOLD`

The attack arm stays at a fixed angle.

### Attack
`SERVO_ATTACK`

The servo sweeps between configured angle limits to strike or hook targets.

## 4. Tower patterns

Three open-loop scripted tower routines are available:

- `TOWER_LOW`
- `TOWER_HIGH`
- `TOWER_HIGH_RED`

Each sequence applies timed motor outputs to drive toward known field objectives.

## 5. Vive GoTo mode

When enabled, the robot:

- maps a 0–100 target coordinate into calibrated Vive coordinates
- estimates heading from recent motion
- uses a simple steering law to drive toward the target

## 6. TopHat safety override

All motion commands pass through a TopHat safety check.

If TopHat health reaches zero:

- motor motion is stopped
- tower routines are canceled
- motion remains locked out for 15 seconds
