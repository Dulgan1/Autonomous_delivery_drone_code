from src.drone_autonomy.models import LandingConfig, LandingStateMachine, ScenarioStep, SimulationRunner
from drone_autonomy.mocks import MockTargetProvider, MockVehicle

targets = MockTargetProvider()
vehicle = MockVehicle()
machine = LandingStateMachine(targets, vehicle, LandingConfig(takeoff_settle_s=0))
timeline = SimulationRunner(machine, targets, vehicle).run([
    ScenarioStep(time_s=0, altitude_m=1.0),
    # Add more steps with a VisualTarget to simulate seeing the marker.
])