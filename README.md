# Self-Driving Maze Robot

An Arduino robot that drives itself through a maze using three ultrasonic sensors and two continuous-rotation servo wheels. It follows the walls, takes the more open path at a junction, and turns around at dead ends.

Built in C++ for Arduino.

## How it works

Every loop the robot pings its front, left, and right sensors, takes the median of three pings per sensor to reject stray echoes, and decides what to do:

| Situation | What the robot does |
| --- | --- |
| Path ahead is clear (30 cm or more) | Drives forward, nudging to keep an even gap to the side walls |
| Wall ahead, left blocked | Turns right |
| Wall ahead, right blocked | Turns left |
| Wall ahead, both sides open | Turns toward the side with more room |
| Wall ahead and on both sides | Backs up and turns around |

A sensor that hears no echo within range reports 0 from NewPing. The sketch treats that as open space, not as a wall.

## Hardware

| Part | Pin |
| --- | --- |
| Front ultrasonic sensor (single-pin mode) | A2 |
| Left ultrasonic sensor | A0 |
| Right ultrasonic sensor | A1 |
| Right wheel, continuous-rotation servo | D12 |
| Left wheel, continuous-rotation servo | D13 |
| Arm servo (parked at startup) | D11 |

The board is an Arduino Uno or compatible. Each sensor uses one pin for both trigger and echo.

## Running it

1. Install the **NewPing** library in the Arduino IDE (Sketch → Include Library → Manage Libraries, search "NewPing").
2. Open `self_driving_robot/self_driving_robot.ino`.
3. Select your board and port, then upload.
4. Open the Serial Monitor at 9600 baud to watch the sensor readings and decisions.

## Tuning

All the numbers you would want to change are at the top of the sketch:

- `FRONT_STOP_CM`: how close a wall can get before the robot stops to decide.
- `SIDE_OPEN_CM`: how far a side wall must be to count as an opening.
- `SIDE_TARGET_CM`: the gap to keep from a wall while driving.
- `TURN_90_MS` and `TURN_180_MS`: how long a turn lasts. These depend on battery charge and floor surface.
- `RUN_DIRECTION_TEST`: set to `true` to run each motion once at startup and confirm the wheels move the way the function names say.

## History

`original_2022/` holds the first version of the sketch, written for a timed maze challenge. The current sketch keeps its motor timings and its rule of turning away from a blocked left side, and adds median filtering, open-space handling, wall centering, and dead-end recovery.
