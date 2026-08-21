# Drone autonomy

This is the drone's **decision-making code**. It is not the motor controller,
camera software, or Pixhawk firmware.

The code can be tested on a laptop with fake GPS, ultrasonic, and camera data.
It does not talk to a real drone yet.

## The parts and their jobs

```text
GPS, IMU, compass, barometer
            ↓
        Pixhawk
 keeps the drone level and controls motors
            ↕
        Raspberry Pi 4B
 decides the next safe mission step
       ↙       ↓       ↘
 bottom     side       top
 range      range      range
 sensor     sensor     sensor
            ↑
 separate OpenCV project
 marker finding only
```

| Part | Job |
| --- | --- |
| Pixhawk | Keeps the drone stable, controls motors, reads normal flight sensors, and runs its own failsafes. |
| Raspberry Pi 4B | Runs this project. It reads mission data and asks Pixhawk for high-level actions. |
| GPS | Gets the drone close to the mission location outdoors. |
| Side ultrasonic sensor | Checks if the direction the drone is facing is clear before a short forward move. |
| Top ultrasonic sensor | Stops takeoff or climb when there is not enough room above the drone. |
| Bottom ultrasonic sensor | Helps check ground distance during the final descent. |
| Separate OpenCV project | Finds the landing or medical-payload marker near the destination. It is not copied into this project. |
| Radio controller / override switch | Lets a human immediately take over. This must always win over autonomy. |

The Pi must **never** send motor commands directly. It can only ask a future
Pixhawk adapter for things such as `hold`, `takeoff`, `yaw_to`, `forward`,
`release_payload`, and `return_home`.

## What the code does now

The state machine has these steps:

```text
IDLE
  → PREFLIGHT
  → TAKEOFF
  → GPS_NAVIGATE
  → SEARCH_MOVE
  → SEARCH
  → TARGET_ACQUIRED
  → ALIGN
  → DROP_READY
  → DROP_PAYLOAD
  → RETURN_HOME
```

Other safe steps are:

```text
GPS_NAVIGATE or SEARCH_MOVE → YAW_SCAN → continue travel
ALIGN or DROP_READY → TARGET_LOST → ALIGN or RETURN_HOME
any active step → HOLD       (manual override or sensor problem)
GPS payload mission failure → RETURN_HOME
```

### What each step means

| State | What the code checks or requests |
| --- | --- |
| `IDLE` | Nothing is running yet. |
| `PREFLIGHT` | Checks fresh GPS, heading, and top-sensor data before takeoff. |
| `TAKEOFF` | Requests the configured 5 m search height. It stops if the top sensor is not clear. |
| `GPS_NAVIGATE` | Calculates GPS direction to the target, turns toward it, checks the side sensor, then asks for one short forward step. |
| `YAW_SCAN` | Holds, then checks the target heading and nearby yaw angles until the side sensor finds a clear direction. |
| `SEARCH_MOVE` | Travels to one point in the configured 10 m × 10 m search grid. |
| `SEARCH` | Holds briefly at that grid point while OpenCV looks for the marker, then moves to the next point. |
| `TARGET_ACQUIRED` | Waits briefly to make sure the marker remains stable and visible. |
| `ALIGN` | Makes small image-based marker-correction requests. |
| `DROP_READY` | Holds and checks that the marker stays centered for the final payload-drop wait. |
| `DROP_PAYLOAD` | Requests the servo release exactly once. No retry is requested. |
| `TARGET_LOST` | Holds immediately when the visual marker disappears. It aborts if the marker does not return in time. |
| `HOLD` | Requests that Pixhawk hold position. Manual override and missing safety data lead here. |
| `RETURN_HOME` | Requests Pixhawk's configured return-to-home action. It is used after payload release and most GPS-mission failures. |

## GPS navigation algorithm

The GPS algorithm is intentionally simple:

```text
1. Read the drone GPS position and heading.
2. Calculate the distance and compass bearing to the target GPS point.
3. If close enough to the point, stop GPS travel and begin marker search.
4. Turn toward the target bearing.
5. Check the side sensor in the direction now being faced.
6. If clear, request one short, slow forward step.
7. If blocked, hold and enter YAW_SCAN.
8. Repeat.
```

`YAW_SCAN` checks these headings by default:

```text
target direction
target direction - 30 degrees
target direction + 30 degrees
target direction - 60 degrees
target direction + 60 degrees
```

It chooses the first clear direction. It then makes only one short forward
step before calculating the GPS direction again. If none are clear, it returns
home instead of guessing.

## Payload search and release algorithm

The payload mission uses these default values, all configurable in
`LandingConfig`:

| Setting | Default |
| --- | --- |
| Search height | 5 m |
| Search area | 10 m × 10 m, centred on the target GPS point |
| Grid spacing | 2 m |
| Battery return-home level | 40% or below |
| Maximum search time | 10 minutes |
| Release type | Servo request |
| Release retry | None |

The grid is a back-and-forth path, like mowing a lawn. At each grid point, the
drone holds for a short configurable time. If no marker is found after every
point, it returns home.

For a drop, the marker must be stable, visible, and centred. The drone then
holds for the final configured confirmation time, sends one servo-release
request, and returns home. There is currently no physical release confirmation
and no retry: your servo mechanism must be designed so that one correctly
timed command releases the payload safely.

This is **basic obstacle checking**, not full obstacle avoidance. One side
sensor can only see one direction at a time. It cannot see behind the drone,
objects outside its narrow beam, thin objects, wires, moving objects, or every
obstacle between yaw angles. If all tested directions are blocked, the code
holds and waits for a human decision.

## Marker and OpenCV rules

OpenCV is only for landing-marker or payload-marker work near the mission area.
GPS handles normal travel.

A marker is used only when it is:

- Present
- Stable
- Visible in the current camera frame
- Made of valid numeric values

An old tracked marker with `visible=False` is never used for movement. The CV
confidence value is kept as information; it is not treated as a guarantee.

Image error means “left/right/up/down in the camera image.” It does **not** yet
mean metres or drone-body movement. Camera calibration and camera-to-drone
alignment are required before image guidance can control a real aircraft.

## Sensor safety rules already in code

- A GPS mission cannot start without a navigation-data provider.
- Missing or stale GPS/heading data during travel causes `HOLD`.
- The top sensor must be fresh and clear before takeoff.
- The side sensor must be fresh and clear before each forward request.
- A blocked side sensor starts a yaw scan instead of moving forward.
- Missing bottom-sensor data during descent causes `HOLD`.
- Marker loss during descent causes `TARGET_LOST`, which immediately holds.
- Low battery at or below 40% during a GPS payload mission requests return home.
- Stale Pi↔Pixhawk telemetry requests return home.
- A Pixhawk-reported hard failsafe requests HOLD so Pixhawk's own failsafe can
  stay in charge.
- Manual override has first priority.
- All movement values, timeouts, distances, and yaw choices live in
  `LandingConfig` and can be changed for simulation.

The shown defaults are not approved real-flight values. They are deliberately
small mock-test values and must be measured and tested for your drone.

## Important code files

| File | What it contains |
| --- | --- |
| `src/drone_autonomy/models.py` | Target, GPS, range-sensor, and state data. |
| `src/drone_autonomy/interfaces.py` | Small contracts for CV input, navigation input, and high-level vehicle requests. |
| `src/drone_autonomy/state_machine.py` | All mission states, transitions, GPS bearing/distance math, yaw scan, and safety gates. |
| `src/drone_autonomy/mocks.py` | Fake inputs and vehicle that only record requested actions. |
| `src/drone_autonomy/simulation.py` | Runs a list of fake flight moments and returns a readable timeline. |
| `tests/` | Safety and mission tests. |

Every production class and function has hover documentation in a Python-aware
editor. Data fields are described in their class documentation.

## What you must do manually before real hardware use

### 1. Identify the exact hardware

Record the model number and data sheet for:

- Pixhawk board
- GPS/compass module
- Each HC-SR04 ultrasonic sensor
- ESCs, motors, battery, power module, and radio receiver
- Payload-release hardware, if a payload will be dropped

HC-SR04 sensors are inexpensive but have important limits: their beam is
narrow, their readings can be wrong on angled/soft surfaces, and they are not
waterproof. Their `echo` output is normally 5 V and must not be wired directly
to Raspberry Pi GPIO.

### 2. Build safe power and wiring

- Give Pixhawk and the Pi proper regulated power supplies sized for their load.
- Do not power the Pi through a random Pixhawk telemetry connector.
- Connect Pi and Pixhawk through a correctly wired MAVLink telemetry link.
- Use common ground only as required by the chosen wiring design.
- Check all signal voltages before connecting them.
- Raspberry Pi GPIO is 3.3 V. HC-SR04 echo is normally 5 V, so use a proper
  level shifter or voltage divider before every Pi GPIO echo pin.
- Keep ultrasonic, GPS/compass, Pi, and power wires away from noisy motor/ESC
  wiring as much as possible.

### 3. Mount and measure the sensors

- Mount the side sensor so yaw points its beam in the direction being checked.
- Mount the top sensor where props, frame parts, and payload cannot block it.
- Mount the bottom sensor with a clear view of the ground.
- Measure each sensor's minimum and maximum reliable distance.
- Test on grass, dirt, concrete, water, dark surfaces, angled surfaces, and
  likely mission obstacles. Ultrasonic readings can fail on bad surfaces.
- Measure sensor readings while motors are off and while the drone is running.
- Set `side_clearance_m`, `top_clearance_m`, and `landing_distance_m` from
  measurements, not guesses.

### 4. Configure and prove Pixhawk first

Before connecting autonomy:

- Calibrate IMU, compass, radio, power module, and GPS.
- Confirm GPS lock, heading, position hold, takeoff, landing, and return home.
- Configure and test battery, radio-loss, geofence, estimator, and companion-
  computer-link failsafes.
- Verify a physical/manual override switch can immediately give control back to
  the pilot.
- Fly manually in a safe legal test area before autonomous tests.

### 5. Write the real input adapters

This repository still needs small hardware-specific pieces:

- A Pixhawk/MAVLink adapter that reads GPS, heading, altitude, armed state,
  flight mode, battery, and failsafe status.
- Ultrasonic drivers that timestamp readings, reject noise, mark stale values,
  and create `NavigationReadings`.
- A CV adapter that converts the separate CV project's tracked output into
  `VisualTarget` without copying any OpenCV pipeline code here.
- A future `VehicleInterface` adapter that can send only approved high-level
  requests to Pixhawk after checking mode, health, and limits.

Do not write the real vehicle adapter until hardware-in-the-loop simulation and
props-off testing are complete.

### 6. Make the missing mission decisions

You need to decide and document:

- Maximum mission height, speed, travel distance, wind, and battery limits.
- What happens if one ultrasonic sensor fails.
- What happens if Pi freezes or loses MAVLink connection.
- What happens if GPS accuracy is poor or GPS is lost.
- Whether the single servo-release command needs an additional software or
  physical safety lock before flight.

### 7. Add calibration before real visual alignment

For real `ALIGN` and `DESCEND`, measure:

- Camera lens calibration
- Camera angle relative to the drone body
- Camera position relative to the drone centre
- How camera left/right/up/down maps to Pixhawk movement directions

Do not use image error for real movement until this is tested safely.

### 8. Test in the right order

1. Run unit tests and fake simulations.
2. Read GPS and ultrasonic values on a desk.
3. Read Pixhawk telemetry with props removed.
4. Run Pixhawk simulation/hardware-in-the-loop tests.
5. Test failsafes and manual override with props removed.
6. Test tethered, low-height flight with an experienced pilot.
7. Test takeoff, hold, and GPS travel separately.
8. Test marker search, then marker alignment.
9. Test short descent last.
10. Review logs after every test.

Never make the first outdoor test a full autonomous mission or autonomous
landing.

## Not implemented yet

- Real Pixhawk/MAVLink connection
- Real ultrasonic GPIO drivers and filtering
- Real OpenCV adapter
- Full obstacle avoidance or map building
- GPS accuracy/quality checks beyond fresh valid coordinates
- Camera calibration and image-to-body conversion
- Real servo payload-release adapter and physical safety lock
- Flight logs, telemetry recording, and a ground-station interface
- Hardware-in-the-loop simulation

## Run the tests

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

The tests use only fake inputs. They cannot fly a drone.

## Run a fake mission and read the output

Run this from the repository folder:

```bash
PYTHONPATH=src python examples/payload_mission_demo.py
```

It prints a fake successful mission. The important columns are:

| Output | Meaning |
| --- | --- |
| `time` | Fake time in seconds. It is not real waiting. |
| `state` | The current decision step. |
| `reason` | Why the state changed. |
| `actions` | High-level requests the code made. They are recorded only. |

The last section, `Recorded requests`, shows the full list of fake requests,
including takeoff, hold, servo release, and return home. Nothing is sent to
Pixhawk, a servo, or the motors.
