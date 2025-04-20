# IoT HTL - Component-Level Functions (Python Implementation Skeletons)

import random
import time
import statistics
from enum import Enum

# Mock sensor data classes
class SensorData:
    def __init__(self, sensor_id, data, timestamp):
        self.sensor_id = sensor_id
        self.data = data
        self.timestamp = timestamp

class Sensor:
    def __init__(self, sensor_id, sensor_type):
        self.sensor_id = sensor_id
        self.sensor_type = sensor_type

    def read_data(self):
        # Simulate sensor reading with dummy values
        return SensorData(self.sensor_id, random.uniform(0, 100), time.time())

    def self_test(self):
        return random.choice([True, True, True, False])  # Mostly true

# 5.3.1 SensorModule
class SensorModule:
    def __init__(self):
        self.sensors = [
            Sensor("lidar01", "LiDAR"),
            Sensor("cam01", "Camera"),
            Sensor("radar01", "Radar")
        ]

    def collect_data(self):
        """Gathers raw sensor data from all physical sensors."""
        data = {}
        for sensor in self.sensors:
            reading = sensor.read_data()
            print(f"Collected data from {sensor.sensor_id}: {reading.data:.2f}")
            data[reading.sensor_id] = reading
        return data

    def self_test(self):
        """Performs diagnostic check on sensor hardware."""
        results = {sensor.sensor_id: sensor.self_test() for sensor in self.sensors}
        for sid, passed in results.items():
            print(f"Sensor {sid} self-test: {'PASSED' if passed else 'FAILED'}")
        return all(results.values())

# 5.3.2 SensorFusion
class SensorFusion:
    def __init__(self):
        self.sensors = []
        self.data_map = {}  # sensor_id -> list of SensorData
        self.environment_map = {}

    def validate_integrity(self):
        """Checks consistency of combined sensor data."""
        for sensor_id, readings in self.data_map.items():
            if not readings:
                print(f"Integrity check failed: No data for sensor {sensor_id}")
                return False
        print("All sensor data passed integrity check.")
        return True

    def handle_sensor_reading(self, event):
        """Triggered by new sensor reading."""
        if event.sensor_id not in self.data_map:
            self.data_map[event.sensor_id] = []
        self.data_map[event.sensor_id].append(event)
        print(f"Received reading from {event.sensor_id}: {event.data:.2f}")

    def aggregate_data(self):
        """Selects most recent data from each sensor."""
        latest_data = {}
        for sensor_id, readings in self.data_map.items():
            if readings:
                latest = max(readings, key=lambda r: r.timestamp)
                latest_data[sensor_id] = latest
                print(f"Latest reading from {sensor_id}: {latest.data:.2f} at {latest.timestamp}")
        return latest_data

    def fuse_sensor_data(self):
        """Executes fusion logic to update environment map."""
        aggregated_data = self.aggregate_data()
        values = [reading.data for reading in aggregated_data.values()]
        if not values:
            print("No data to fuse.")
            return

        fused_average = statistics.mean(values)
        print(f"Fused data average: {fused_average:.2f}")
        
        self.environment_map = {
            "fused_average": fused_average,
            "individual_readings": {sid: r.data for sid, r in aggregated_data.items()}
        }

# 5.3.3 PerceptionModule
class PerceptionModule:
    def __init__(self):
        self.fused_data = None  # Will hold the latest fused sensor data (from SensorFusion)

    def handle_fused_data(self, event):
        """Handles the fused data from the SensorFusion module."""
        self.fused_data = event
        print("Fused data received and processed.")

    def detect_objects(self):
        """Detects objects in the environment based on fused data."""
        if not self.fused_data:
            print("No fused data available to detect objects.")
            return []
        # Simulating object detection by randomly returning objects
        detected_objects = ['Car', 'Pedestrian', 'Bicycle', 'Truck']
        print(f"Detected objects: {', '.join(detected_objects)}")
        return detected_objects

    def detect_lanes(self):
        """Detects lanes from the fused sensor data."""
        if not self.fused_data:
            print("No fused data available to detect lanes.")
            return []
        # Simulating lane detection
        lanes = ['Left Lane', 'Right Lane']
        print(f"Detected lanes: {', '.join(lanes)}")
        return lanes

    def classify_traffic_sign(self):
        """Classifies traffic signs based on the fused data."""
        if not self.fused_data:
            print("No fused data available to classify traffic signs.")
            return None
        # Simulating traffic sign classification
        traffic_sign = "Stop Sign"
        print(f"Classified traffic sign: {traffic_sign}")
        return traffic_sign

    def track_objects(self):
        """Tracks the detected objects over time."""
        if not self.fused_data:
            print("No fused data available to track objects.")
            return []
        # Simulating object tracking (e.g., by ID)
        tracked_objects = ['Car ID: 001', 'Pedestrian ID: 002']
        print(f"Tracking objects: {', '.join(tracked_objects)}")
        return tracked_objects

# 5.3.4 PlanningModule
class PlanningModule:
    def __init__(self):
        self.perception_data = None  # Will hold perception updates (from PerceptionModule)
        self.vehicle_state = None  # Will hold the current vehicle state (e.g., speed, position)

    def handle_perception_update(self, event):
        """Handles perception updates from the PerceptionModule."""
        self.perception_data = event
        print("Perception update received.")

    def handle_vehicle_state(self, event):
        """Handles vehicle state update."""
        self.vehicle_state = event
        print(f"Vehicle state updated: {event}")

    def decide_behavior(self):
        """Decides the vehicle's behavior based on perception data and vehicle state."""
        if not self.perception_data:
            print("No perception data available to decide behavior.")
            return "Stop"
        detected_objects = self.perception_data.detect_objects()
        lanes = self.perception_data.detect_lanes()
        
        # Basic decision-making simulation
        if 'Pedestrian' in detected_objects:
            print("Pedestrian detected. Deciding to stop.")
            return "Stop"
        if 'Car' in detected_objects and lanes:
            print("Car detected. Deciding to overtake.")
            return "Overtake"
        return "Continue"

    def plan_trajectory(self):
        """Plans the vehicle's trajectory."""
        if not self.vehicle_state:
            print("No vehicle state available to plan trajectory.")
            return None
        # Simulating trajectory planning
        trajectory = {"start": self.vehicle_state, "end": "Destination"}
        print(f"Planned trajectory: {trajectory}")
        return trajectory

    def evaluate_safety(self):
        """Evaluates the safety of the current plan."""
        if not self.perception_data:
            print("No perception data available to evaluate safety.")
            return "Unsafe"
        # Simulating safety check based on detected objects and lanes
        detected_objects = self.perception_data.detect_objects()
        if 'Pedestrian' in detected_objects:
            print("Safety evaluation: Unsafe due to pedestrian.")
            return "Unsafe"
        print("Safety evaluation: Safe.")
        return "Safe"

    def check_for_collision(self, trajectory):
        """Checks for potential collisions along the planned trajectory."""
        if not trajectory:
            print("No trajectory available to check for collisions.")
            return False
        # Simulating collision check
        detected_objects = self.perception_data.detect_objects()
        if 'Car' in detected_objects:
            print("Collision risk detected with car.")
            return True
        print("No collision detected.")
        return False

    def detect_lane_change(self):
        """Detects if a lane change is required."""
        if not self.perception_data:
            print("No perception data available to detect lane change.")
            return False
        # Simulating lane change detection
        lanes = self.perception_data.detect_lanes()
        if 'Left Lane' in lanes:
            print("Lane change detected: Moving to Left Lane.")
            return True
        print("No lane change required.")
        return False

    def plan_route(self):
        """Plans the overall route."""
        if not self.vehicle_state:
            print("No vehicle state available to plan route.")
            return None
        # Simulating route planning (e.g., to a destination)
        route = {"start": self.vehicle_state, "end": "Destination"}
        print(f"Planned route: {route}")
        return route

    def publish_plan(self, bus):
        """Publishes the planned trajectory or route."""
        if not self.vehicle_state:
            print("No vehicle state available to publish plan.")
            return
        # Simulating plan publishing
        print(f"Publishing plan: {self.vehicle_state} -> Destination")
        bus.publish(self.vehicle_state)  # Assuming a bus object to publish to

# Supporting classes for simulation
class PIDController:
    def __init__(self, kp=1.0, ki=0.0, kd=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

class VehicleState:
    def __init__(self, speed=0.0, angle=0.0):
        self.speed = speed
        self.steering_angle = angle

class TrajectoryPlanEvent:
    def __init__(self, trajectory_points):
        self.trajectory_points = trajectory_points

class EventBus:
    def publish(self, event_name, data):
        print(f"Event Published: {event_name} -> {data}")

class ReceivedMessage:
    def __init__(self, raw_data, source):
        self.raw_data = raw_data
        self.source = source


# 5.3.5 ControlModule
class ControlModule:
    def __init__(self):
        self.steeringController = PIDController(kp=0.8)
        self.speedController = PIDController(kp=1.0)
        self.currentVehicleState = VehicleState()
        self.acceleration = 0.0
        self.brakingForce = 0.0
        self.steeringAngle = 0.0
        self.trajectory = []

    def handle_plan(self, event):
        self.trajectory = event.trajectory_points
        print(f"Trajectory received with {len(self.trajectory)} points.")

    def execute_trajectory(self):
        for point in self.trajectory:
            self.adjust_steering_angle(point['angle'])
            self.adjust_speed(point['speed'])
            self.read_vehicle_sensors()

    def adjust_steering_angle(self, target_angle):
        control_signal = self.steeringController.compute(target_angle, self.currentVehicleState.steering_angle)
        self.steeringAngle = control_signal
        print(f"Steering adjusted to: {self.steeringAngle:.2f}")

    def adjust_speed(self, target_speed):
        control_signal = self.speedController.compute(target_speed, self.currentVehicleState.speed)
        if control_signal >= 0:
            self.acceleration = control_signal
            self.brakingForce = 0
        else:
            self.apply_brakes(-control_signal)
        print(f"Speed control applied: Acceleration={self.acceleration:.2f}, Braking={self.brakingForce:.2f}")

    def apply_brakes(self, force):
        self.brakingForce = force
        self.acceleration = 0
        print(f"Brakes applied with force: {force:.2f}")

    def read_vehicle_sensors(self):
        # Simulate reading sensor data
        self.currentVehicleState.speed += self.acceleration - self.brakingForce
        self.currentVehicleState.steering_angle = self.steeringAngle
        print(f"Sensor Readings -> Speed: {self.currentVehicleState.speed:.2f}, Angle: {self.currentVehicleState.steering_angle:.2f}")

    def detect_override(self):
        # Simulated driver override detection
        override_detected = False  # This could be dynamic based on external flags
        print(f"Override detected: {override_detected}")
        return override_detected

    def publish_vehicle_state(self, bus):
        bus.publish("VehicleStateUpdate", {
            "speed": self.currentVehicleState.speed,
            "steering_angle": self.currentVehicleState.steering_angle
        })


# 5.3.6 CommunicationModule
class CommunicationModule:
    def __init__(self):
        self.v2xStack = {}
        self.cloudClient = {}
        self.connectionStatus = True
        self.securityContext = {}

    def broadcast_message(self, message):
        if self.connectionStatus:
            print(f"Broadcasting V2X message: {message}")
        else:
            print("V2X broadcast failed: No connection")

    def receive_message(self):
        # Simulate receiving a message
        message = ReceivedMessage("Traffic update: Congestion ahead", "infrastructure")
        print(f"Received message from {message.source}: {message.raw_data}")
        return message

    def parse_and_dispatch(self, msg):
        print(f"Parsing message: {msg.raw_data}")
        if "Traffic" in msg.raw_data:
            print("Dispatching event: TrafficAlert")

    def send_data_to_cloud(self, data):
        if self.connectionStatus:
            print(f"Sending data to cloud: {data}")
        else:
            print("Cloud upload failed: No connection")

# 5.3.7 SoftwareUpdateManager
class SoftwareUpdateManager:
    def __init__(self, update_server_url):
        self.updateServerUrl = update_server_url
        self.currentVersion = "1.0.0"
        self.status = "Idle"
        self.updateHistory = []

    def check_for_updates(self):
        print(f"Checking for updates from {self.updateServerUrl}...")
        # Simulated update info
        if self.currentVersion != "1.1.0":
            return {"version": "1.1.0", "filePath": "/tmp/update_1_1_0.pkg"}
        return None

    def download_update(self):
        print("Downloading update...")
        success = True  # Simulate successful download
        self.status = "Downloaded" if success else "Download Failed"
        return success

    def validate_update(self, file_path):
        print(f"Validating update at {file_path}...")
        # Simulated validation check
        is_valid = "update" in file_path
        print("Validation", "succeeded." if is_valid else "failed.")
        return is_valid

    def install_update(self, file_path):
        if self.validate_update(file_path):
            print(f"Installing update from {file_path}...")
            self.currentVersion = "1.1.0"
            self.updateHistory.append(self.currentVersion)
            self.status = "Installed"
            print("Update installed successfully.")
            return True
        else:
            print("Installation failed. Rolling back.")
            self.rollback()
            return False

    def rollback(self):
        if self.updateHistory:
            previous = self.updateHistory[-2] if len(self.updateHistory) > 1 else "1.0.0"
            self.currentVersion = previous
            self.status = "Rolled back"
            print(f"Rolled back to version {self.currentVersion}")
            return True
        print("No previous version to rollback to.")
        return False

# 5.3.8 InterfaceManager
class HMIState(Enum):
    DRIVER_VIEW_ACTIVE = 1
    TECHNICIAN_DIAGNOSTICS_ACTIVE = 2

class InterfaceManager:
    def __init__(self, driver_display, technician_ui):
        self.driverDisplay = driver_display
        self.technicianUI = technician_ui
        self.hmiState = HMIState.DRIVER_VIEW_ACTIVE

    def handle_system_event(self, event):
        print(f"Handling system event: {event}")
        if event.get("type") == "alert":
            self.driverDisplay.show_alert(event["message"])
        elif event.get("type") == "diagnostic":
            self.technicianUI.show_diagnostics(event["details"])

    def render_dashboard(self):
        print("Rendering driver dashboard...")
        self.driverDisplay.update()

    def render_technician_ui(self, ui_type):
        print(f"Rendering technician UI: {ui_type}")
        self.technicianUI.render(ui_type)

    def handle_user_input(self, event):
        print(f"User input received: {event}")
        # Translate into actions or commands
        if event.get("action") == "switch_view":
            self.hmiState = HMIState.TECHNICIAN_DIAGNOSTICS_ACTIVE if self.hmiState == HMIState.DRIVER_VIEW_ACTIVE else HMIState.DRIVER_VIEW_ACTIVE

# 5.3.9 Abstract Sensor Class
class SensorStatus(Enum):
    OFFLINE = 0
    INITIALIZING = 1
    ONLINE = 2
    ERROR = 3

class BaseSensor:
    def __init__(self, sensor_id, sensor_type):
        self.sensorID = sensor_id
        self.sensorType = sensor_type
        self.status = SensorStatus.OFFLINE
        self.reading = None

    def initialize(self):
        """Perform necessary setup and initialization for sensor hardware."""
        raise NotImplementedError("initialize() must be implemented by subclasses.")

    def get_status(self):
        """Retrieve the current operational status of the sensor."""
        return self.status

    def read_data(self):
        """Read the latest data from the sensor."""
        raise NotImplementedError("read_data() must be implemented by subclasses.")

    def run_selftest(self):
        """Initiate a self-diagnostic test on the sensor hardware."""
        raise NotImplementedError("run_selftest() must be implemented by subclasses.")
# 5.3.10 Specific Sensor Subclasses
class LiDAR(BaseSensor):
    def __init__(self, sensor_id, config):
        super().__init__(sensor_id, "LiDAR")
        self.hardwareConfig = config

    def initialize(self):
        print(f"Initializing LiDAR sensor {self.sensorID}...")
        self.status = SensorStatus.INITIALIZING
        self.status = SensorStatus.ONLINE

    def run_selftest(self):
        print(f"Running LiDAR self-test...")
        return True

    def read_data(self):
        self.reading = {"distance_map": [1.0, 2.5, 3.7]}
        return self.reading

    def publish_reading(self):
        print(f"LiDAR {self.sensorID} reading: {self.reading}")

class Camera(BaseSensor):
    def __init__(self, sensor_id, config):
        super().__init__(sensor_id, "Camera")
        self.hardwareConfig = config

    def initialize(self):
        print(f"Initializing Camera sensor {self.sensorID}...")
        self.status = SensorStatus.INITIALIZING
        self.status = SensorStatus.ONLINE

    def run_selftest(self):
        print("Camera self-test passed.")
        return True

    def read_data(self):
        self.reading = {"frame": "image_data"}
        return self.reading

    def publish_reading(self):
        print(f"Camera {self.sensorID} reading: {self.reading}")

class Radar(BaseSensor):
    def __init__(self, sensor_id, config):
        super().__init__(sensor_id, "Radar")
        self.hardwareConfig = config

    def initialize(self):
        print(f"Initializing Radar sensor {self.sensorID}...")
        self.status = SensorStatus.INITIALIZING
        self.status = SensorStatus.ONLINE

    def run_selftest(self):
        print("Radar self-test successful.")
        return True

    def read_data(self):
        self.reading = {"velocity_vector": [5.5, 0.2]}
        return self.reading

    def publish_reading(self):
        print(f"Radar {self.sensorID} reading: {self.reading}")
