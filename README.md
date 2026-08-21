# Drone autonomy

This is the drone's **decision-making code**. It is not the motor controller,
camera software, or Pixhawk firmware.

It runs in **two modes**:

| Mode | Where | What it does |
| --- | --- | --- |
| **Simulation** | Your laptop | Plays scripted fake flights. No hardware, no dependencies. |
| **Hardware** | Raspberry Pi 4B | Talks to a real PX4 Pixhawk, real HC-SR04 sensors, a real servo, and the separate OpenCV project. |

Both modes build the **same** state machine from the **same** configuration.
Only the things standing behind the interfaces change. That is the point: what
you watch in simulation is the same logic that flies.

```bash
# On your laptop
PYTHONPATH=src python -m drone_autonomy sim

# On the Raspberry Pi, read-only, commands nothing
PYTHONPATH=src python -m drone_autonomy check --config config/mission.toml

# On the Raspberry Pi, the real mission
PYTHONPATH=src python -m drone_autonomy fly --config config/mission.toml --i-have-completed-bench-testing
```

## The parts and their jobs

```text
GPS, IMU, compass, barometer
            ↓
        Pixhawk  (PX4)
 keeps the drone level and controls motors
            ↕ MAVLink over a serial telemetry port
        Raspberry Pi 4B
 decides the next safe mission step
       ↙       ↓       ↘        ↘
 bottom     side       top      servo
 range      range      range    payload release
 sensor     sensor     sensor   (Pi GPIO)
            ↑
 separate OpenCV project
 marker finding only, over local UDP
```

| Part | Job |
| --- | --- |
| Pixhawk (PX4) | Keeps the drone stable, controls motors, reads normal flight sensors, and runs its own failsafes. |
| Raspberry Pi 4B | Runs this project. It reads mission data and asks Pixhawk for high-level actions. |
| GPS | Gets the drone close to the mission location outdoors. |
| Side ultrasonic sensor | Checks if the direction the drone is facing is clear before a short forward move. |
| Top ultrasonic sensor | Stops takeoff or climb when there is not enough room above the drone. |
| Bottom ultrasonic sensor | Helps check ground distance during the final descent. |
| Payload servo | Opens the release mechanism once, driven from a Pi GPIO pin. |
| Separate OpenCV project | Finds the landing or medical-payload marker near the destination. It is not copied into this project. |
| Radio controller / override switch | Lets a human immediately take over. This must always win over autonomy. |

The Pi must **never** send motor commands directly. It only asks Pixhawk for
things such as `hold`, `takeoff`, `yaw_to`, `forward`, `release_payload`, and
`return_home`. Pixhawk is free to refuse, limit, or override any of them.

## Simulation mode

Simulation needs nothing installed. It plays scripted fake flights and prints
what the drone brain decided at each moment.

```bash
PYTHONPATH=src python -m drone_autonomy sim --list          # show the fake flights
PYTHONPATH=src python -m drone_autonomy sim                 # run all of them
PYTHONPATH=src python -m drone_autonomy sim nominal         # run one
```

The output looks like this:

```text
time | state | reason | actions
 0.0 | takeoff | preflight_passed | takeoff
 1.0 | gps_navigate | takeoff_settled | -
 2.0 | search_move | gps_target_reached | -
 3.0 | search | search_point_reached | -
 4.0 | target_acquired | fresh_stable_target | hold
 5.0 | align | acquisition_confirmed | -
 6.0 | drop_ready | alignment_confirmed | velocity
 7.0 | drop_payload | payload_drop_authorized | hold
 8.0 | return_home | payload_release_requested | release_payload, return_home
```

| Output | Meaning |
| --- | --- |
| `time` | Fake time in seconds. It is not real waiting. |
| `state` | The current decision step. |
| `reason` | Why the state changed. |
| `actions` | High-level requests the code made. They are recorded only. |

Each fake flight says which state it should end in, and the runner tells you if
one ended somewhere else. The same check runs as a test, so a change that
breaks a safety path fails the test suite.

### The fake flights

| Scenario | What it proves |
| --- | --- |
| `nominal` | A complete delivery: travel, search, marker, one release, return home. |
| `blocked_path` | The side sensor blocks a step, a yaw scan finds a clear heading, travel resumes. |
| `no_clear_direction` | Every scanned heading is blocked, so it returns home instead of guessing. |
| `marker_never_found` | The whole grid is searched with no marker, so it returns home. |
| `search_timeout` | The search runs out of time across several grid points. |
| `low_battery` | Battery at or below 40% during flight requests return home. |
| `gps_lost` | Stale GPS or heading holds instead of guessing a direction. |
| `top_blocked` | An overhead obstacle stops takeoff. |
| `not_armed` | Autonomy refuses to start on a disarmed aircraft. |
| `unstable_marker` | Unstable and remembered-but-invisible markers are ignored. |
| `marker_lost_before_drop` | Losing the marker before release holds, then comes home without releasing. |
| `manual_override` | The pilot taking control beats every autonomous decision. |
| `pixhawk_failsafe` | A Pixhawk failsafe makes this code stand down. |
| `link_lost` | Stale Pi-to-Pixhawk telemetry requests return home. |

Add your own in [scenarios.py](src/drone_autonomy/runtime/scenarios.py).

## Hardware mode

### Install on the Raspberry Pi

```bash
sudo apt install pigpio
sudo systemctl enable --now pigpiod
pip install -e ".[hardware]"          # pymavlink and pigpio
cp config/mission.example.toml config/mission.toml
```

Then edit `config/mission.toml`. Every setting is explained in the file.

### Step 1: read-only check

```bash
PYTHONPATH=src python -m drone_autonomy check --config config/mission.toml
```

This connects to Pixhawk, reads telemetry, reads all three ultrasonic sensors,
and listens for marker messages. It **commands nothing**. Run it on the bench
and again with propellers removed. It prints one line per second:

```text
mode=POSCTL      armed=False link=ok    batt= 96.0% alt=  0.0m gps=ok  hdg=  91.4 bottom= 0.42m side= 2.85m top=  --   marker=no
```

`top=  --  ` means that sensor has no usable fresh reading. Fix that before
flying: the mission logic treats a missing reading as unsafe, not as clear.

To test the perception link without a camera, run the fake sender alongside it:

```bash
PYTHONPATH=src python examples/fake_cv_sender.py --pattern approach
```

### Step 2: the real mission

```bash
PYTHONPATH=src python -m drone_autonomy fly --config config/mission.toml --i-have-completed-bench-testing
```

The long flag is deliberate. Without it the command refuses to start and tells
you to run the read-only check first.

Every decision cycle is written to `logs/mission-<date>-<time>.jsonl`, one JSON
object per line, with the state, the reason, the requests made, and the
telemetry and sensor values at that moment. Read the log after every flight.

### The three interlocks

Three settings are **off** in the example configuration and must stay off until
the matching manual work is genuinely finished:

| Setting | While it is false | Turn it on only after |
| --- | --- | --- |
| `servo.enabled` | Release requests are refused and logged. Nothing opens. | The release mechanism is built and tested on the bench, ideally with a physical lock. |
| `mavlink.allow_arm` | Autonomy never arms; a human must arm first. | You have decided a computer may arm this aircraft and have tested it props-off. |
| `mavlink.allow_image_guidance` | Marker image error never moves the aircraft. | Camera calibration and camera-to-body alignment are measured and tested. |

`allow_image_guidance` matters most. Image error means "left/right/up/down in
the camera picture." It is not metres and it is not drone-body directions.
Until the camera is calibrated against the airframe, letting it move a real
aircraft is guessing.

## What the code does

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
any active step → RETURN_HOME (mission failure, low battery, timeout)
```

### What each step means

| State | What the code checks or requests |
| --- | --- |
| `IDLE` | Nothing is running yet. |
| `PREFLIGHT` | Checks that the vehicle is armed and healthy and that GPS, heading, and top-sensor data are fresh. |
| `TAKEOFF` | Requests the configured 5 m search height. It stops if the top sensor is not clear. |
| `GPS_NAVIGATE` | Calculates GPS direction to the target, turns toward it, checks the side sensor, then asks for one short forward step. |
| `YAW_SCAN` | Holds, then checks the target heading and nearby yaw angles until the side sensor finds a clear direction. |
| `SEARCH_MOVE` | Travels to one point in the configured 10 m × 10 m search grid. |
| `SEARCH` | Holds briefly at that grid point while OpenCV looks for the marker, then moves to the next point. |
| `TARGET_ACQUIRED` | Waits briefly to make sure the marker remains stable and visible. |
| `ALIGN` | Makes small image-based marker-correction requests. |
| `DROP_READY` | Holds and checks that the marker stays centered for the final payload-drop wait. |
| `DROP_PAYLOAD` | Requests the servo release exactly once. No retry is requested. |
| `TARGET_LOST` | Holds immediately when the visual marker disappears. It comes home if the marker does not return in time. |
| `HOLD` | Requests that Pixhawk hold position. Manual override and missing safety data lead here. |
| `RETURN_HOME` | Requests Pixhawk's configured return-to-home action. It is used after payload release and most mission failures. |

`HOLD` is a stopping point on purpose: its only exit is `ABORT`. Once the pilot
has taken control or a required sensor has failed, this code does not decide by
itself that things are fine again. Restart the mission deliberately.

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

This is **basic obstacle checking**, not full obstacle avoidance. One side
sensor can only see one direction at a time. It cannot see behind the drone,
objects outside its narrow beam, thin objects, wires, moving objects, or every
obstacle between yaw angles. If all tested directions are blocked, the code
comes home rather than guessing.

## Payload search and release algorithm

| Setting | Default |
| --- | --- |
| Search height | 5 m |
| Search area | 10 m × 10 m, centred on the target GPS point |
| Grid spacing | 2 m |
| Battery return-home level | 40% or below |
| Maximum search time | 10 minutes, across the whole search |
| Maximum mission time | Off by default; set `mission_timeout_s` |
| Release type | Servo request from a Pi GPIO pin |
| Release retry | None |

The grid is a back-and-forth path, like mowing a lawn. At each grid point, the
drone holds for a short configurable time. If no marker is found after every
point, it returns home.

For a drop, the marker must be stable, visible, and centred. The drone then
holds for the final configured confirmation time, sends one servo-release
request, and returns home. There is no physical release confirmation and no
retry: your servo mechanism must be designed so that one correctly timed
command releases the payload safely.

## Marker and OpenCV rules

OpenCV is only for landing-marker or payload-marker work near the mission area.
GPS handles normal travel.

A marker is used only when it is:

- Present
- Stable
- Visible in the current camera frame
- Made of valid numeric values
- Recent enough (older than `cv.max_age_s` is ignored)

An old tracked marker with `visible=False` is never used for movement. The CV
confidence value is kept as information; it is not treated as a guarantee.

The perception project sends one JSON message per frame to a local UDP port:

```json
{
  "track_id": 1,
  "target_point": [320.0, 240.0],
  "horizontal_error": -0.12,
  "vertical_error": 0.05,
  "normalized_radius": 0.18,
  "marker_confidence": 0.82,
  "stable": true,
  "visible": true,
  "sent_at_s": 12345.678
}
```

`sent_at_s` is optional and must come from the same clock, which in practice
means both programs run on the same Pi. Without it, arrival time is used. A
malformed message is dropped whole; it is never partly believed. `stable` and
`visible` default to `false` when missing, so a message that forgets them can
never move the aircraft.

UDP was chosen so a crash or a slow frame in perception cannot stall a decision
cycle here, and so old frames are dropped instead of queueing up behind fresh
ones. To use a different transport, replace `UdpTargetProvider` only.

## How the PX4 adapter works

PX4's Offboard mode is how a companion computer moves the aircraft. Two things
about it shape this adapter:

- Offboard needs a **continuous setpoint stream** faster than 2 Hz, both to
  enter the mode and to stay in it. A background thread streams the current
  setpoint at 20 Hz by default.
- If that stream stops, PX4 leaves Offboard and runs its own failsafe. That is
  exactly what should happen if this program crashes, so the adapter never
  tries to work around it. Stopping cleanly with Ctrl-C is handled: the mission
  ends, then the stream stops on purpose.

| Mission request | What the adapter sends |
| --- | --- |
| `takeoff` | `AUTO.TAKEOFF` mode plus `MAV_CMD_NAV_TAKEOFF` |
| `yaw_to` | Offboard setpoint: zero velocity, commanded heading |
| `forward` | Offboard setpoint: body-frame forward velocity that **expires** after the step, then hovers |
| `hold` | Offboard setpoint: still hover, so a short pause does not drop out of Offboard |
| `descend` | Offboard setpoint: slow downward velocity |
| `velocity` | Refused unless `allow_image_guidance` is on |
| `release_payload` | Pi GPIO servo, behind the `servo.enabled` interlock |
| `return_home` | Stops the stream, then `AUTO.RTL` |
| `land` | Stops the stream, then `AUTO.LAND` |

**Manual override** on PX4 does not arrive as a separate signal. When the pilot
takes the radio, the flight mode simply changes to one they fly. The adapter
treats any pilot-flown mode (`MANUAL`, `STABILIZED`, `ALTCTL`, `POSCTL`,
`ACRO`) as manual override, and treats *any* unexpected mode change as a
failsafe. Either way autonomy stands down.

If the adapter cannot do what the mission asked — PX4 refuses Offboard, a
command fails to send, no servo is fitted — it records a fault, and that fault
is reported to the mission as a failsafe. The mission then holds or comes home
instead of assuming the request worked.

## Sensor safety rules already in code

- A GPS mission cannot start without a navigation-data provider.
- Preflight refuses to continue on a disarmed aircraft.
- Missing or stale GPS/heading data during travel causes `HOLD`.
- GPS is only trusted with a 3D fix, enough satellites, and good reported accuracy.
- The top sensor must be fresh and clear before takeoff.
- The side sensor must be fresh and clear before each forward request.
- A blocked side sensor starts a yaw scan instead of moving forward.
- Missing bottom-sensor data during descent causes `HOLD`.
- Marker loss during descent causes `TARGET_LOST`, which immediately holds.
- Low battery at or below 40% during flight requests return home.
- Stale Pi↔Pixhawk telemetry requests return home.
- A Pixhawk-reported hard failsafe requests HOLD so Pixhawk's own failsafe can
  stay in charge.
- An ultrasonic reading is dropped if it is out of range, if the echo timed
  out, or if the sensor has repeated one value too many times. A failed sensor
  reads as **missing**, never as **clear**.
- Ultrasonic readings are median-filtered and go stale if updates stop.
- The three sensors are triggered one at a time, so one cannot hear another's
  pulse and report a confident wrong distance.
- Manual override has first priority.
- All movement values, timeouts, distances, and yaw choices live in the
  configuration file and can be changed without touching the safety logic.

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
| `src/drone_autonomy/runtime/config.py` | The one configuration object both modes are built from. |
| `src/drone_autonomy/runtime/loop.py` | The real-time mission loop and the flight log. |
| `src/drone_autonomy/runtime/scenarios.py` | The library of scripted fake flights. |
| `src/drone_autonomy/runtime/sim_mode.py` | Simulation mode. |
| `src/drone_autonomy/runtime/hardware_mode.py` | Hardware mode: read-only check and real flight. |
| `src/drone_autonomy/hardware/px4.py` | PX4 telemetry decoding, mode encoding, and the Offboard adapter. |
| `src/drone_autonomy/hardware/ultrasonic.py` | HC-SR04 driver, filtering, and the combined navigation source. |
| `src/drone_autonomy/hardware/servo.py` | Payload-release servo behind its interlock. |
| `src/drone_autonomy/hardware/cv_link.py` | UDP receiver for marker messages from the OpenCV project. |
| `config/mission.example.toml` | Every setting, explained, with safe defaults. |
| `tests/` | Safety, mission, adapter, and scenario tests. |

Every production class and function has hover documentation in a Python-aware
editor. Data fields are described in their class documentation.

Each hardware adapter is split in two: the rules that decide what may be
trusted are plain functions and small classes tested on a laptop, and the
input/output work is a thin shell around them. That is why the whole test suite
runs without pymavlink, pigpio, a Pixhawk, or a camera.

## Run the tests

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

The tests use only fake inputs. They cannot fly a drone.

## What you must do manually before real hardware use

### 1. Identify the exact hardware

Record the model number and data sheet for:

- Pixhawk board
- GPS/compass module
- Each HC-SR04 ultrasonic sensor
- ESCs, motors, battery, power module, and radio receiver
- Payload-release servo and mechanism

HC-SR04 sensors are inexpensive but have important limits: their beam is
narrow, their readings can be wrong on angled/soft surfaces, and they are not
waterproof. Their `echo` output is normally 5 V and must not be wired directly
to Raspberry Pi GPIO.

### 2. Build safe power and wiring

- Give Pixhawk and the Pi proper regulated power supplies sized for their load.
- Do not power the Pi through a random Pixhawk telemetry connector.
- Connect Pi and Pixhawk through a correctly wired MAVLink telemetry link,
  usually `TELEM2`. Set the matching PX4 serial parameters and baud rate.
- Give the payload servo its own regulated supply sharing ground with the Pi. A
  servo drawing stall current through a Pi 5 V pin can brown out the Pi.
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
- Use `check` mode to watch the filtered readings while you do all of this.

### 4. Configure and prove Pixhawk first

Before connecting autonomy:

- Calibrate IMU, compass, radio, power module, and GPS.
- Confirm GPS lock, heading, position hold, takeoff, landing, and return home.
- Configure and test battery, radio-loss, geofence, estimator, and
  Offboard-loss failsafes. Decide what PX4 should do when Offboard setpoints
  stop, because that is what happens if this program dies.
- Verify a physical/manual override switch can immediately give control back to
  the pilot.
- Fly manually in a safe legal test area before autonomous tests.

### 5. Build and test the release mechanism

- Build the servo mechanism so the payload is held **mechanically** at rest,
  not by continuous servo torque. If the Pi loses power, the signal stops.
- Test the release on the bench, on the ground, with `servo.enabled = true` and
  the aircraft disarmed, before it is ever tested in the air.
- Decide whether it needs an additional physical safety lock for flight.

### 6. Make the missing mission decisions

You need to decide and document:

- Maximum mission height, speed, travel distance, wind, and battery limits.
- What happens if one ultrasonic sensor fails.
- What happens if the Pi freezes or loses MAVLink connection.
- What happens if GPS accuracy is poor or GPS is lost.
- Whether a computer may arm this aircraft at all (`mavlink.allow_arm`).

### 7. Add calibration before real visual alignment

For real `ALIGN`, measure:

- Camera lens calibration
- Camera angle relative to the drone body
- Camera position relative to the drone centre
- How camera left/right/up/down maps to Pixhawk movement directions

Only then set `allow_image_guidance = true`, and set
`image_guidance_scale` from that measurement.

### 8. Test in the right order

1. Run the tests and every simulation scenario.
2. Run `check` on a desk with the sensors and the fake CV sender.
3. Run `check` with Pixhawk connected and propellers removed.
4. Run PX4 software-in-the-loop and point `mavlink.connection` at it.
5. Test failsafes and manual override with props removed.
6. Test tethered, low-height flight with an experienced pilot.
7. Test takeoff, hold, and GPS travel separately.
8. Test marker search, then marker alignment.
9. Test the payload release last.
10. Read the mission log after every test.

Never make the first outdoor test a full autonomous mission.

## Not implemented yet

- Full obstacle avoidance or map building
- Camera calibration and image-to-body conversion, so image guidance is off
- A physical payload-release confirmation sensor, and release retry
- A ground-station display or live telemetry downlink
- Wind, geofence, and airspace checks inside this code; PX4's geofence is the
  only boundary
- Hardware-in-the-loop testing has not been done by this project
