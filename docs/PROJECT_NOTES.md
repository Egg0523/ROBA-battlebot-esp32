# Project Notes

This repository is a cleaned packaging of the final project materials for a MEAM 5100 ROBA robot.

## Design goals

The robot was designed to be:

- hard to push
- mechanically simple
- capable of zero-turn motion
- effective at wall-following
- capable of attacking whisker switches
- compliant with TopHat health reporting requirements

## What worked well

- Differential-drive mobility
- ToF-based wall following
- Strong drivetrain and weight distribution
- Scripted tower motions
- TopHat integration logic in the final code

## What was less reliable

- Vive localization under noisy field conditions
- The structural stiffness of the servo attack arm
- Open-loop positioning accuracy without Vive

## Why this repo is organized this way

The original project code was delivered as one `.ino` sketch plus a separate final report. For GitHub, those materials are easier to understand when grouped into:

- `src/` for executable code
- `docs/` for architecture and usage notes
- `bom/` for hardware purchasing information
