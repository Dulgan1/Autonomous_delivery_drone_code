# Drone autonomy

This is the drone decision-making project. It does not do computer vision. It receives a simple target message from the separate CV project and asks for safe, high-level actions.

There is no camera code, motor control, Pixhawk, or MAVLink code here.

## Safety model

`LandingStateMachine` decides which step comes next. It only uses a target when it is present, stable, visible, and has valid values. Confidence is kept as information; it is not treated as a chance of success.

`VehicleInterface` is a small list of requests: `takeoff`, `hold`, `velocity`, `descend`, and `land`. `MockVehicle` only records those requests for tests. It cannot fly a real drone.

The left/right and up/down guidance values come from the image. They are not metres and are not drone-body directions. This project is for tests and simulation until camera calibration and safe flight-controller integration are added.

## State flow

```text
IDLE -> TAKEOFF -> SEARCH -> TARGET_ACQUIRED -> ALIGN -> DESCEND -> LAND
                     ^             |               |          |
                     |             +-- loss -------+----------+--> TARGET_LOST -> ALIGN | ABORT
                     +---------------- acquisition/search timeout -----------------------> ABORT

Any state -- manual override --> HOLD
Any state -- explicit abort --> ABORT
```

When the target is lost, the drone asks to hold right away. It can only go back to `ALIGN` after seeing a fresh valid target. If it cannot find one quickly, it goes to `ABORT` and holds. It never keeps descending without a target. `LAND` needs a valid centered target and a low altitude reported by the mock vehicle.

| Transition | Explicit condition |
| --- | --- |
| `IDLE → TAKEOFF` | operator starts autonomy |
| `TAKEOFF → SEARCH` | takeoff wait finishes; timeout aborts |
| `SEARCH → TARGET_ACQUIRED` | a fresh, stable, visible target appears; timeout aborts |
| `TARGET_ACQUIRED → ALIGN` | target stays valid long enough; loss/timeout returns to search |
| `ALIGN → DESCEND` | target stays near the image centre long enough |
| `ALIGN/DESCEND → TARGET_LOST` | target is missing/invalid, or a timeout occurs |
| `TARGET_LOST → ALIGN` | target returns before its timeout |
| `DESCEND → LAND` | target is centered and altitude is low |
| any active state → `HOLD` | manual override |
| any active state → `ABORT` | operator abort or timeout that cannot recover |

## Run tests

```bash
PYTHONPATH=src python -m unittest discover -s tests -v
```

## Try a fake flight

`SimulationRunner` lets you play a list of fake moments through the state
machine. Each moment can include a target, altitude, manual override, or
abort. It returns a timeline showing the state, why it changed, and the action
requested. It does not use a camera or control a drone.

```python
from drone_autonomy import LandingConfig, LandingStateMachine, ScenarioStep, SimulationRunner
from drone_autonomy.mocks import MockTargetProvider, MockVehicle

targets = MockTargetProvider()
vehicle = MockVehicle()
machine = LandingStateMachine(targets, vehicle, LandingConfig(takeoff_settle_s=0))
timeline = SimulationRunner(machine, targets, vehicle).run([
    ScenarioStep(time_s=0, altitude_m=1.0),
    # Add more steps with a VisualTarget to simulate seeing the marker.
])
```
