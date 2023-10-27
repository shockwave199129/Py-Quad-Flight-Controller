import smbus2
import RPi.GPIO as GPIO
import time
import math

# Define I2C address for MPU6050
MPU_ADDRESS = 0x68

# Define MPU6050 registers
PWR_MGMT_1 = 0x6B
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0x1C
CONFIG = 0x1A

# Define MPU6050 sensitivity values
GYRO_SENSITIVITY = 500  # ±500°/s
ACCEL_SENSITIVITY = 8  # ±8g

# Define GPIO pins for ESC control
ESC_PINS = [4, 5, 6, 7]

# Define the GPIO pins for each channel
CHANNEL_PINS = [17, 18, 22, 23]

# Initialize I2C bus
bus = smbus2.SMBus(1)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
for pin in ESC_PINS:
    GPIO.setup(pin, GPIO.OUT)

# Set up GPIO pins as input
for pin in CHANNEL_PINS:
    GPIO.setup(pin, GPIO.IN)


# Constants
SSF_GYRO = 65.5  # Sensitivity Scale Factor for Gyro
X = 0
Y = 1
Z = 2

CHANNEL1 = 0
CHANNEL2 = 1
CHANNEL3 = 2
CHANNEL4 = 3

STOPPED = 0
STARTING = 1
STARTED = 2

YAW = 0
PITCH = 1
ROLL = 2
THROTTLE = 3

# Variables
pid_set_points = [0, 0, 0]
angular_motions = [0, 0, 0]
errors = [0, 0, 0]
delta_err = [0, 0, 0]
previous_error = [0, 0, 0]
error_sum = [0, 0, 0]
pulse_length = [1500, 1500, 1000, 1500]

status = STOPPED

"""
 * Real measures on 3 axis calculated from gyro AND accelerometer in that order : Yaw, Pitch, Roll
 *  - Left wing up implies a positive roll
 *  - Nose up implies a positive pitch
 *  - Nose right implies a positive yaw
"""
measures = [0, 0, 0]

# Define the arrays for accelerometer and gyro data
acc_raw = [0, 0, 0]
gyro_raw = [0, 0, 0]

# Define gyro_offset as a list
gyro_offset = [0, 0, 0]

# Initialize gyro angles
gyro_angle = [0, 0, 0]

# Initialize accelerometer angles
acc_angle = [0, 0, 0]

# PID constants
Kp = [0.0, 0.0, 0.0]
Ki = [0.0, 0.0, 0.0]
Kd = [0.0, 0.0, 0.0]

mode_mapping = [0, 0, 0, 0]

# Initialize loop_timer
loop_timer = time.time()

# Sampling frequency
FREQ = 250  # 250Hz

# Sampling period
period = 1 / FREQ

# ESC pulse length values
pulse_length_esc1 = 1000
pulse_length_esc2 = 1000
pulse_length_esc3 = 1000
pulse_length_esc4 = 1000

# Compensate for battery voltage drop
battery_voltage = 1240

def setupMpu6050Registers():
    # Setup MPU6050 registers
    bus.write_byte_data(MPU_ADDRESS, PWR_MGMT_1, 0x00)  # Configure power management
    bus.write_byte_data(MPU_ADDRESS, GYRO_CONFIG, 0x08)  # Configure gyro sensitivity
    bus.write_byte_data(MPU_ADDRESS, ACCEL_CONFIG, 0x10)  # Configure accelerometer sensitivity
    bus.write_byte_data(MPU_ADDRESS, CONFIG, 0x03)  # Set Digital Low Pass Filter about ~43Hz


def calibrateMpu6050():
    max_samples = 2000

    for i in range(max_samples):
        # Read sensor data (make sure you have the readSensor function)
        readSensor()

        gyro_offset[X] += gyro_raw[X]
        gyro_offset[Y] += gyro_raw[Y]
        gyro_offset[Z] += gyro_raw[Z]

        # Generate low throttle pulse to initialize ESC and prevent them from beeping
        # Set GPIO pins for ESC control to HIGH (make sure you have defined ESC_PINS)
        for pin in ESC_PINS:
            GPIO.output(pin, GPIO.HIGH)

        # Wait for 1000 microseconds (1 millisecond)
        time.sleep(0.001)

        # Set GPIO pins for ESC control to LOW
        for pin in ESC_PINS:
            GPIO.output(pin, GPIO.LOW)

        # Wait for 3 milliseconds before the next loop
        time.sleep(0.003)

    # Calculate average offsets
    gyro_offset[X] /= max_samples
    gyro_offset[Y] /= max_samples
    gyro_offset[Z] /= max_samples

def readRedioData():
    pulse_length[CHANNEL1] = GPIO.input(CHANNEL_PINS[CHANNEL1])
    pulse_length[CHANNEL2] = GPIO.input(CHANNEL_PINS[CHANNEL2])
    pulse_length[CHANNEL3] = GPIO.input(CHANNEL_PINS[CHANNEL3])
    pulse_length[CHANNEL4] = GPIO.input(CHANNEL_PINS[CHANNEL4])

def read_analog(pin):
    GPIO.setup(pin, GPIO.IN)

    return GPIO.input(pin)

def configureChannelMapping():
    mode_mapping[YAW]      = CHANNEL4
    mode_mapping[PITCH]    = CHANNEL2
    mode_mapping[ROLL]     = CHANNEL1
    mode_mapping[THROTTLE] = CHANNEL3

def min_max(value, min_value, max_value):
    if value > max_value:
        value = max_value
    elif value < min_value:
        value = min_value

    return value

def resetGyroAngles():
    gyro_angle[X] = acc_angle[X]
    gyro_angle[Y] = acc_angle[Y]

def calculate_set_point(angle, channel_pulse):
    level_adjust = angle * 15  # Value 15 limits maximum angle value to ±32.8°
    set_point = 0.0

    # Need a dead band of 16µs for better results
    if channel_pulse > 1508:
        set_point = channel_pulse - 1508
    elif channel_pulse < 1492:
        set_point = channel_pulse - 1492

    set_point -= level_adjust
    set_point /= 3.0

    return set_point

def calculate_yaw_set_point(yaw_pulse, throttle_pulse):
    set_point = 0.0

    # Do not yaw when turning off the motors
    if throttle_pulse > 1050:
        # There is no notion of angle on this axis as the quadcopter can turn on itself
        set_point = calculate_set_point(0, yaw_pulse)

    return set_point

def resetPidController():
    errors[YAW]   = 0
    errors[PITCH] = 0
    errors[ROLL]  = 0

    error_sum[YAW]   = 0
    error_sum[PITCH] = 0
    error_sum[ROLL]  = 0

    previous_error[YAW]   = 0
    previous_error[PITCH] = 0
    previous_error[ROLL]  = 0

def readSensor():
    # Start communicating with the MPU-6050
    bus.write_byte(MPU_ADDRESS, 0x3B)
    
    # Request 14 bytes from the MPU-6050
    data = bus.read_i2c_block_data(MPU_ADDRESS, 0, 14)
    
    # Extract and store the sensor data
    acc_raw[X] = (data[0] << 8 | data[1])  # X accelerometer
    acc_raw[Y] = (data[2] << 8 | data[3])  # Y accelerometer
    acc_raw[Z] = (data[4] << 8 | data[5])  # Z accelerometer
    temperature = (data[6] << 8 | data[7])  # Temperature
    gyro_raw[X] = (data[8] << 8 | data[9])  # X gyro
    gyro_raw[Y] = (data[10] << 8 | data[11])  # Y gyro
    gyro_raw[Z] = (data[12] << 8 | data[13])  # Z gyro

def calculateGyroAngles():
    global gyro_angle
    # Subtract offsets
    gyro_raw[X] -= gyro_offset[X]
    gyro_raw[Y] -= gyro_offset[Y]
    gyro_raw[Z] -= gyro_offset[Z]

    # Angle calculation using integration
    gyro_angle[X] += (gyro_raw[X] / (FREQ * SSF_GYRO))
    gyro_angle[Y] += (-gyro_raw[Y] / (FREQ * SSF_GYRO))  # Change sign to match the accelerometer's one

    # Transfer roll to pitch if IMU has yawed
    gyro_angle[Y] += gyro_angle[X] * math.sin(gyro_raw[Z] * (math.pi / (FREQ * SSF_GYRO * 180)))
    gyro_angle[X] -= gyro_angle[Y] * math.sin(gyro_raw[Z] * (math.pi / (FREQ * SSF_GYRO * 180)))

def calculateAccelerometerAngles():
    global acc_angle
    # Calculate total 3D acceleration vector: √(X² + Y² + Z²)
    acc_total_vector = math.sqrt(pow(acc_raw[X], 2) + pow(acc_raw[Y], 2) + pow(acc_raw[Z], 2))

    # To prevent asin from producing a NaN, make sure the input value is within [-1;+1]
    if abs(acc_raw[X]) < acc_total_vector:
        acc_angle[X] = math.asin(float(acc_raw[Y]) / acc_total_vector) * (180 / math.pi)  # asin gives angle in radians. Convert to degrees by multiplying by 180/pi

    if abs(acc_raw[Y]) < acc_total_vector:
        acc_angle[Y] = math.asin(float(acc_raw[X]) / acc_total_vector) * (180 / math.pi)


def calculateAngles():
    calculateGyroAngles()
    calculateAccelerometerAngles()

    global initialized

    if initialized:
        # Correct the drift of the gyro with the accelerometer
        gyro_angle[X] = gyro_angle[X] * 0.9996 + acc_angle[X] * 0.0004
        gyro_angle[Y] = gyro_angle[Y] * 0.9996 + acc_angle[Y] * 0.0004
    else:
        # At very first start, init gyro angles with accelerometer angles
        resetGyroAngles()
        initialized = True

    # To dampen the pitch and roll angles, a complementary filter is used
    measures[ROLL] = measures[ROLL] * 0.9 + gyro_angle[X] * 0.1
    measures[PITCH] = measures[PITCH] * 0.9 + gyro_angle[Y] * 0.1
    measures[YAW] = -gyro_raw[Z] / SSF_GYRO  # Store the angular motion for this axis

    # Apply low-pass filter (10Hz cutoff frequency)
    angular_motions[ROLL] = 0.7 * angular_motions[ROLL] + 0.3 * gyro_raw[X] / SSF_GYRO
    angular_motions[PITCH] = 0.7 * angular_motions[PITCH] + 0.3 * gyro_raw[Y] / SSF_GYRO
    angular_motions[YAW] = 0.7 * angular_motions[YAW] + 0.3 * gyro_raw[Z] / SSF_GYRO


def calculateSetPoints():
    readRedioData()
    # Implement code to calculate PID set points on the YAW, PITCH, and ROLL axes
    # Calculate set points for YAW, PITCH, and ROLL
    pid_set_points[YAW] = calculate_yaw_set_point(pulse_length[mode_mapping[YAW]], pulse_length[mode_mapping[THROTTLE]])
    pid_set_points[PITCH] = calculate_set_point(measures[PITCH], pulse_length[mode_mapping[PITCH]])
    pid_set_points[ROLL] = calculate_set_point(measures[ROLL], pulse_length[mode_mapping[ROLL]])

def calculateErrors():
    # Implement code to calculate errors used by the PID controller
    # Calculate current errors
    errors[YAW] = angular_motions[YAW] - pid_set_points[YAW]
    errors[PITCH] = angular_motions[PITCH] - pid_set_points[PITCH]
    errors[ROLL] = angular_motions[ROLL] - pid_set_points[ROLL]

    # Calculate sum of errors : Integral coefficients
    error_sum[YAW] += errors[YAW]
    error_sum[PITCH] += errors[PITCH]
    error_sum[ROLL] += errors[ROLL]

    # Keep values in the acceptable range
    error_sum[YAW] = min_max(error_sum[YAW], -400 / Ki[YAW], 400 / Ki[YAW])
    error_sum[PITCH] = min_max(error_sum[PITCH], -400 / Ki[PITCH], 400 / Ki[PITCH])
    error_sum[ROLL] = min_max(error_sum[ROLL], -400 / Ki[ROLL], 400 / Ki[ROLL])

    # Calculate error delta : Derivative coefficients
    delta_err[YAW] = errors[YAW] - previous_error[YAW]
    delta_err[PITCH] = errors[PITCH] - previous_error[PITCH]
    delta_err[ROLL] = errors[ROLL] - previous_error[ROLL]

    # Save current error as previous_error for the next time
    previous_error[YAW] = errors[YAW]
    previous_error[PITCH] = errors[PITCH]
    previous_error[ROLL] = errors[ROLL]

def stopAll():
    pulse_length_esc1 = 1000
    pulse_length_esc2 = 1000
    pulse_length_esc3 = 1000
    pulse_length_esc4 = 1000

def isStarted():
    # When left stick is moved in the bottom left corner
    if (status == STOPPED) and (pulse_length[mode_mapping[YAW]] <= 1012) and (pulse_length[mode_mapping[THROTTLE]]) <= 1012: 
        status = STARTING

    # When left stick is moved back in the center position
    if (status == STARTING) and (pulse_length[mode_mapping[YAW]] == 1500) and (pulse_length[mode_mapping[THROTTLE]] <= 1012):
        status = STARTED

        # Reset PID controller's variables to prevent bump start
        resetPidController()

        resetGyroAngles()

    # When left stick is moved in the bottom right corner
    if (status == STARTED) and (pulse_length[mode_mapping[YAW]] >= 1988) and (pulse_length[mode_mapping[THROTTLE]] <= 1012):
        status = STOPPED
        # Make sure to always stop motors when status is STOPPED
        stopAll()
    

    return status == STARTED

def pidController():
    """
    * Calculate motor speed for each motor of an X quadcopter depending on received instructions and measures from sensor
    * by applying PID control.
    *
    * (A) (B)     x
    *   \ /     z ↑
    *    X       \|
    *   / \       +----→ y
    * (C) (D)
    *
    * Motors A & D run clockwise.
    * Motors B & C run counter-clockwise.
    *
    * Each motor output is considered as a servomotor. As a result, value range is about 1000µs to 2000µs
    """
    # Implement code to calculate motor speed for each motor using PID control
    # Use the errors, error_sum, PID constants, throttle, and mode_mapping
    # Calculate motor speed and update pulse_length_esc1, pulse_length_esc2, pulse_length_esc3, and pulse_length_esc4
    yaw_pid      = 0
    pitch_pid    = 0
    roll_pid     = 0

    throttle = pulse_length[mode_mapping[THROTTLE]]

    # Initialize motor commands with throttle
    pulse_length_esc1 = throttle
    pulse_length_esc2 = throttle
    pulse_length_esc3 = throttle
    pulse_length_esc4 = throttle

    # Do not calculate anything if throttle is 0
    if throttle >= 1012:
        # PID = e.Kp + ∫e.Ki + Δe.Kd
        yaw_pid = (errors[YAW] * Kp[YAW]) + (error_sum[YAW] * Ki[YAW]) + (delta_err[YAW] * Kd[YAW])
        pitch_pid = (errors[PITCH] * Kp[PITCH]) + (error_sum[PITCH] * Ki[PITCH]) + (delta_err[PITCH] * Kd[PITCH])
        roll_pid = (errors[ROLL] * Kp[ROLL]) + (error_sum[ROLL] * Ki[ROLL]) + (delta_err[ROLL] * Kd[ROLL])

        # Keep values within the acceptable range (TODO: export hard-coded values in variables/const)
        yaw_pid = min_max(yaw_pid, -400, 400)
        pitch_pid = min_max(pitch_pid, -400, 400)
        roll_pid = min_max(roll_pid, -400, 400)

        # Calculate pulse duration for each ESC
        pulse_length_esc1 = throttle - roll_pid - pitch_pid + yaw_pid
        pulse_length_esc2 = throttle + roll_pid - pitch_pid - yaw_pid
        pulse_length_esc3 = throttle - roll_pid + pitch_pid - yaw_pid
        pulse_length_esc4 = throttle + roll_pid + pitch_pid + yaw_pid

    # Prevent out-of-range-values
    pulse_length_esc1 = min_max(pulse_length_esc1, 1100, 2000)
    pulse_length_esc2 = min_max(pulse_length_esc2, 1100, 2000)
    pulse_length_esc3 = min_max(pulse_length_esc3, 1100, 2000)
    pulse_length_esc4 = min_max(pulse_length_esc4, 1100, 2000)

def compensateBatteryDrop():
    # Implement code to compensate for battery voltage drop
    # Use the battery_voltage and ESC pulse length values
    # Adjust the pulse_length_esc1, pulse_length_esc2, pulse_length_esc3, and pulse_length_esc4
    if is_battery_connected():
        factor = (1240 - battery_voltage) / 3500.0
        pulse_length_esc1 += pulse_length_esc1 * factor
        pulse_length_esc2 += pulse_length_esc2 * factor
        pulse_length_esc3 += pulse_length_esc3 * factor
        pulse_length_esc4 += pulse_length_esc4 * factor

def is_battery_connected():
    global battery_voltage

    # Reduce noise with a low-pass filter (10Hz cutoff frequency)
    battery_voltage = battery_voltage * 0.92 + (read_analog(0) + 65) * 0.09853

    return 800 < battery_voltage < 1240

def applyMotorSpeed():
    # Implement code to apply motor speed to the ESCs
    # Use the ESC_PINS and pulse_length_esc1, pulse_length_esc2, pulse_length_esc3, pulse_length_esc4
    # Set the PWM signal to the ESC pins based on pulse_length values
    while time.time() - loop_timer < period:
        pass

    # Update loop timer
    loop_timer = time.time()

    # Set pins #4 #5 #6 #7 HIGH
    for pin in ESC_PINS:
        GPIO.output(pin, GPIO.HIGH)

    # Wait until all pins #4 #5 #6 #7 are LOW
    while any(GPIO.input(pin) == GPIO.HIGH for pin in ESC_PINS):
        now = time.time()
        difference = now - loop_timer

        if difference >= pulse_length_esc1:
            GPIO.output(4, GPIO.LOW)  # Set pin #4 LOW
        if difference >= pulse_length_esc2:
            GPIO.output(5, GPIO.LOW)  # Set pin #5 LOW
        if difference >= pulse_length_esc3:
            GPIO.output(6, GPIO.LOW)  # Set pin #6 LOW
        if difference >= pulse_length_esc4:
            GPIO.output(7, GPIO.LOW)  # Set pin #7 LOW

if __name__ == "__main__":

    # Setup
    setupMpu6050Registers()
    calibrateMpu6050()
    configureChannelMapping()

    # Main loop
    while True:
        try:
            # Read raw values from MPU-6050 (implement your own readSensor function)
            readSensor()

            # Calculate angles from gyro & accelerometer's values (implement your own calculateAngles function)
            calculateAngles()

            # Calculate set points of PID controller (implement your own calculateSetPoints function)
            calculateSetPoints()

            # Calculate errors comparing angular motions to set points (implement your own calculateErrors function)
            calculateErrors()

            # Check if the quadcopter is started (implement your own isStarted function)
            if isStarted():
                # Calculate motors speed with PID controller (implement your own pidController function)
                pidController()

                # Compensate for battery voltage drop (implement your own compensateBatteryDrop function)
                compensateBatteryDrop()

            # Apply motors speed (implement your own applyMotorSpeed function)
            applyMotorSpeed()
        except:
            pass
        finally:
            # Calculate loop execution time
            execution_time = time.time() - loop_timer

            # Delay to maintain the desired loop frequency
            if execution_time < period:
                time.sleep(period - execution_time)

            # Update loop timer
            loop_timer = time.time()

    # Cleanup GPIO
    GPIO.cleanup()