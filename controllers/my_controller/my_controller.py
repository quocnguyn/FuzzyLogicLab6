from controller import Robot
import skfuzzy as fuzz
import skfuzzy.control as ctrl
import numpy as np

# Constants
SENSOR_NAMES = ['ps0', 'ps1', 'ps6', 'ps7']
MOTOR_NAMES = ['left wheel motor', 'right wheel motor']
MAX_SPEED = 6.28
BASE_SPEED = 5.0
TIMESTEP = 64  # Default Webots timestep

# Fuzzy control ranges
SENSOR_RANGE = np.arange(0, 2000, 1)
TURN_RANGE = np.arange(-3.0, 3, 0.2)
GO_RANGE = np.arange(-3.0, 3, 0.2)

def initialize_robot():
    """Initialize robot devices and return handle"""
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    
    # Initialize sensors
    sensors = []
    for name in SENSOR_NAMES:
        sensor = robot.getDevice(name)
        sensor.enable(timestep)
        sensors.append(sensor)
    
    # Initialize motors
    motors = []
    for name in MOTOR_NAMES:
        motor = robot.getDevice(name)
        motor.setPosition(float('inf'))
        motor.setVelocity(0.0)
        motors.append(motor)
    
    return robot, sensors, motors

def create_fuzzy_system():
    """Create and configure fuzzy control system"""
    # Antecedents (inputs)
    left_sensor = ctrl.Antecedent(SENSOR_RANGE, 'left')
    right_sensor = ctrl.Antecedent(SENSOR_RANGE, 'right')
    
    # Consequents (outputs)
    turn = ctrl.Consequent(TURN_RANGE, 'turn')
    go = ctrl.Consequent(GO_RANGE, 'go')

    # Membership functions
    left_sensor['far'] = fuzz.trapmf(left_sensor.universe, [0, 60, 60, 80])
    left_sensor['near'] = fuzz.trapmf(left_sensor.universe, [80, 200, 1000, 1000])
    right_sensor['far'] = fuzz.trapmf(right_sensor.universe, [0, 60, 60, 80])
    right_sensor['near'] = fuzz.trapmf(right_sensor.universe, [80, 200, 1000, 1000])
    
    turn['left'] = fuzz.trapmf(turn.universe, [-2.8, -2.2, -1.8, -1.2])
    turn['straight'] = fuzz.trapmf(turn.universe, [-0.1, 0, 0, 0.1])
    turn['right'] = fuzz.trapmf(turn.universe, [1.2, 1.8, 2.2, 2.8])
    
    go['stop'] = fuzz.trimf(go.universe, [0, 0, 0.2])
    go['forward_slow'] = fuzz.trimf(go.universe, [0.0, 0.2, 0.5])
    go['forward_fast'] = fuzz.trapmf(go.universe, [0.3, 0.7, 1.0, 1.0])

    # Rule base
    rules = [
        ctrl.Rule(left_sensor['far'] & right_sensor['far'], (turn['straight'], go['forward_fast'])),
        ctrl.Rule(left_sensor['near'] & right_sensor['far'], (turn['right'], go['forward_slow'])),
        ctrl.Rule(left_sensor['far'] & right_sensor['near'], (turn['left'], go['forward_slow'])),
        ctrl.Rule(left_sensor['near'] & right_sensor['near'], (turn['left'], go['stop']))
    ]
    
    # Control system
    control_system = ctrl.ControlSystem(rules)
    return ctrl.ControlSystemSimulation(control_system)

def read_sensor_values(sensors):
    """Read and average sensor values"""
    left_val = (sensors[0].getValue() + sensors[1].getValue()) / 2
    right_val = (sensors[2].getValue() + sensors[3].getValue()) / 2
    return left_val, right_val

def calculate_motor_speeds(direction, speed_factor, base_speed=BASE_SPEED):
    """Calculate motor speeds based on fuzzy outputs"""
    # Handle stop condition
    if speed_factor < -0.1: 
        return speed_factor * base_speed, speed_factor * base_speed
    
    # Normal movement calculation
    left_speed = speed_factor * base_speed * (1 - direction)
    right_speed = speed_factor * base_speed * (1 + direction)
    
    # Constrain speeds to valid range
    left_speed = np.clip(left_speed, -MAX_SPEED, MAX_SPEED)
    right_speed = np.clip(right_speed, -MAX_SPEED, MAX_SPEED)
    
    return left_speed, right_speed

def main():
    """Main control loop"""
    robot, sensors, [left_motor, right_motor] = initialize_robot()
    fuzzy_system = create_fuzzy_system()
    
    while robot.step(TIMESTEP) != -1:
        # Sensor reading
        left_val, right_val = read_sensor_values(sensors)
        
        # Fuzzy inference
        fuzzy_system.input['left'] = left_val
        fuzzy_system.input['right'] = right_val
        fuzzy_system.compute()
        
        # Get outputs
        direction = fuzzy_system.output['turn']
        speed_factor = fuzzy_system.output['go']
        
        # Calculate motor speeds
        left_speed, right_speed = calculate_motor_speeds(direction, speed_factor)
        
        # Actuate motors
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
        
        # Debug output
        print(f"Sensors: L={left_val:.1f} R={right_val:.1f} | "
              f"Outputs: Turn={direction:.2f} Go={speed_factor:.2f} | "
              f"Motors: L={left_speed:.2f} R={right_speed:.2f}")

if __name__ == "__main__":
    main()