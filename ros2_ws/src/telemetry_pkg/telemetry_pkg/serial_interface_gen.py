#!/usr/bin/env python3

#  Telemetry Project:
#  Renato and Michael.

# -------------IMPORTS-------------------
import rclpy
from rclpy.node import Node
from time import time
import os

from telemetry_interfaces.msg import (
    Bool, Int, String, Buzzer, PlayBuzzer, Imu, Encoders, LineSensors, ProximitySensors, Motors, Zumo, SamplingRate, EnableRead
)

from telemetry_pkg.utilities import RepeatedTimer, TelemetryLogger, generate_str_timestamp, log_msg

# -------------GLOBAL PARAMETERS-----------
PERIOD_INIT_SENSORS = 500 # msec
ENABLE_INIT_SENSORS = True
PUBLISH_PERIOD = 0.02 # sec
KEEP_ALIVE_PERIOD = 2 # sec
SENSOR_SETTINGS_PERIOD = 2 # sec
ARDUINO_SENSORS = ["buzzer", "imu", "encoders", "line_sensors", "proximity_sensors"]

PLAY_BUZZER_TIME = 5 # sec
IMU_TIME = 0.1 # sec
ERROR_IMU_TIME = 5 # sec
ENCODERS_TIME = 0.1 # sec
ERROR_ENCODERS_TIME = 5 # sec
LINE_SENSORS_TIME = 0.1 # sec
PROXIMITY_SENSORS_TIME = 0.1 # sec

class SerialInterfaceGeneratorNode(Node):
    def __init__(self):
        super().__init__("serial_interface")

        self.init_log()

        self.period_subscriber_ = self.create_subscription(SamplingRate, "set_period", self.callback_set_period, 10)
        self.enable_subscriber_ = self.create_subscription(EnableRead, "enable_read", self.callback_enable_read, 10)
        self.motors_subscriber_ = self.create_subscription(Motors, "motors", self.callback_motors, 10)

        self.data_zumo_publisher_ = self.create_publisher(Zumo, "zumo_sensor_settings", 10)
        self.keep_alive_publisher_ = self.create_publisher(String, "keep_alive", 10)

        self.period_sensor_dict = {}
        self.enable_sensor_dict = {}
        for sen in ARDUINO_SENSORS:
            self.period_sensor_dict[sen] = PERIOD_INIT_SENSORS
            self.enable_sensor_dict[sen] = ENABLE_INIT_SENSORS

        self.init_buzzer()
        self.init_imu()
        self.init_encoders()
        self.init_line_sensors()
        self.init_proximity_sensors()

        self.logger.info("Started Serial Interface Generator")

    # -------------LOGGER------------
    def init_log(self):
        log_dir = os.path.join(os.path.expanduser('~'), "ros2_ws/telemetry_logs")
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        log_file_path = os.path.join(log_dir, f"serial_interface_generator_log_{generate_str_timestamp()}.txt")

        self.logger = TelemetryLogger('logger', log_file_path).logger
        self.log_level_subscriber_ = self.create_subscription(Int, "log_level", self.callback_set_log_level, 10)

        self.telemetry_data_logger = TelemetryLogger('telemetry_data_logger', log_file_path, console=False, disabled=True).logger                
        self.telemetry_data_log_enable_subscriber_ = self.create_subscription(Bool, "telemetry_data_log_enable", self.callback_telemetry_data_log_enable,10)

    def callback_set_log_level(self, msg):
        self.logger.setLevel(msg.data)

    def callback_telemetry_data_log_enable(self, msg):
        if msg.data:
            self.telemetry_data_logger.disabled = False
        else:
            self.telemetry_data_logger.disabled = True

   # ----------------BUZZER-------------------
    def init_buzzer(self):
        self.is_playing = False
        self.buzzer_last_play_time = 0
        self.buzzer_last_sample_time = 0
        self.read_buzzer_publisher_ = self.create_publisher(Buzzer, "read_buzzer", 10)
        self.play_buzzer_subscriber_ = self.create_subscription(PlayBuzzer, "play_buzzer", self.callback_play_buzzer, 10)

    def callback_play_buzzer(self, msg):
        self.is_playing = bool(msg.play_buzzer)
        self.buzzer_last_play_time = time()

    def playing_buzzer(self):
        if self.is_playing:
            if (time() * 1000 - self.buzzer_last_play_time) >= PLAY_BUZZER_TIME:
                self.is_playing = False

# ----------------IMU------------------
    def init_imu(self):
        self.accelerometer = [0, 0, 0]
        self.magnetometer = [0, 0, 0]
        self.gyro = [0, 0, 0]
        self.imu_error_read = False
        self.imu_error_init = False
        self.imu_last_sample_time = 0
        self.read_imu_publisher_ = self.create_publisher(Imu, "read_imu", 10)

    def imu_gen(self):
        for idx, acc in enumerate(self.accelerometer):
            self.accelerometer[idx] = (acc + (idx+1)*11)
            if self.accelerometer[idx] >= 30000:
                self.accelerometer[idx] = -30000

        for idx, mag in enumerate(self.magnetometer):
            self.magnetometer[idx] = (mag + (idx+1)*17)
            if self.magnetometer[idx] >= 30000:
                self.magnetometer[idx] = -30000

        for idx, gyr in enumerate(self.gyro):
            self.gyro[idx] = (gyr + (idx+1)*31)
            if self.gyro[idx] >= 30000:
                self.gyro[idx] = -30000

    def imu_error_gen(self):
        self.imu_error_read = not self.imu_error_read

    def imu_error_init_gen(self):
        self.imu_error_init = not self.imu_error_init

# ---------------ENCODERS----------------
    def init_encoders(self):
        self.encoders_right = 0
        self.encoders_left = 0
        self.encoders_error_right = False
        self.encoders_error_left = False
        self.encoders_last_sample_time = 0
        self.read_encoders_publisher_ = self.create_publisher(Encoders, "read_encoders", 10)

    def encoders_gen(self):
        self.encoders_right += 10
        if self.encoders_right >= 2000:
            self.encoders_right = -2000
        self.encoders_left += 20
        if self.encoders_left >= 2000:
            self.encoders_left = -2000

    def encoders_error_left_gen(self):
        self.encoders_error_left = not self.encoders_error_left

    def encoders_error_right_gen(self):
        self.encoders_error_right = not self.encoders_error_right

# ---------------LINE SENSORS----------------
    def init_line_sensors(self):
        self.line_sensors = [0, 0, 0]
        self.line_sensors_last_sample_time = 0
        self.read_line_sensors_publisher_ = self.create_publisher(LineSensors, "read_line_sensors", 10)        

    def line_sensors_gen(self):
        for idx, line_sen in enumerate(self.line_sensors):
            self.line_sensors[idx] = (line_sen + (idx+1)*10) % 2000

# ----------------PROXIMITY SENSORS------------------
    def init_proximity_sensors(self):
        self.left_sensor = [0, 0]
        self.front_sensor = [0, 0]
        self.right_sensor = [0, 0]
        self.read_basic = [0, 0, 0]
        self.proximity_sensors_last_sample_time = 0
        self.read_proximity_sensors_publisher_ = self.create_publisher(ProximitySensors, "read_proximity_sensors", 10)

    def proximity_sensors_gen(self):
        for idx, left in enumerate(self.left_sensor):
            self.left_sensor[idx] = (left + (idx+1)*1) % 70

        for idx, front in enumerate(self.front_sensor):
            self.front_sensor[idx] = (front + (idx+1)*2) % 70

        for idx, right in enumerate(self.right_sensor):
            self.right_sensor[idx] = (right + (idx+1)*3) % 70

    def proximity_sensors_basic_left_gen(self):
        self.read_basic[0] = (self.read_basic[0] + 1) % 2

    def proximity_sensors_basic_front_gen(self):
        self.read_basic[1] = (self.read_basic[1] + 1) % 2

    def proximity_sensors_basic_right_gen(self):
        self.read_basic[2] = (self.read_basic[2] + 1) % 2

    def callback_set_period(self, msg):
        if msg.sensor not in ARDUINO_SENSORS:
            return
        self.period_sensor_dict[msg.sensor] = msg.period_msec
        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_enable_read(self, msg):
        if msg.sensor not in ARDUINO_SENSORS:
            return
        self.enable_sensor_dict[msg.sensor] = bool(msg.enable_read)
        log_msg(self.telemetry_data_logger, "recieved", msg)

    def callback_motors(self, msg):
        log_msg(self.telemetry_data_logger, "recieved", msg)

    def publish(self):
        if self.enable_sensor_dict['buzzer']:
            if (time() * 1000 - self.buzzer_last_sample_time) >= self.period_sensor_dict['buzzer']:
                self.buzzer_last_sample_time = time() * 1000
                msg = Buzzer()
                msg.is_playing = self.is_playing
                self.read_buzzer_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

        if self.enable_sensor_dict['imu']:
            if (time() * 1000 - self.imu_last_sample_time) >= self.period_sensor_dict['imu']:
                self.imu_last_sample_time = time() * 1000
                msg = Imu()
                msg.error_read = self.imu_error_read
                msg.error_init = self.imu_error_init
                if not msg.error_read and not msg.error_init: # no errors, take data
                    msg.accelerometer = self.accelerometer
                    msg.magnetometer = self.magnetometer
                    msg.gyro = self.gyro
                self.read_imu_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

        if self.enable_sensor_dict['encoders']:
            if (time() * 1000 - self.encoders_last_sample_time) >= self.period_sensor_dict['encoders']:
                self.encoders_last_sample_time = time() * 1000
                msg = Encoders()
                msg.error_right = self.encoders_error_right
                msg.error_left = self.encoders_error_left
                if not msg.error_right: # no errors, take data
                    msg.right = self.encoders_right
                if not msg.error_left: # no errors, take data
                    msg.left = self.encoders_left
                self.read_encoders_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

        if self.enable_sensor_dict['line_sensors']:
            if (time() * 1000 - self.line_sensors_last_sample_time) >= self.period_sensor_dict['line_sensors']:
                self.line_sensors_last_sample_time = time() * 1000
                msg = LineSensors()
                msg.data = self.line_sensors
                self.read_line_sensors_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)

        if self.enable_sensor_dict['proximity_sensors']:
            if (time() * 1000 - self.proximity_sensors_last_sample_time) >= self.period_sensor_dict['proximity_sensors']:
                self.proximity_sensors_last_sample_time = time() * 1000
                msg = ProximitySensors()
                msg.left_sensor = self.left_sensor
                msg.front_sensor = self.front_sensor
                msg.right_sensor = self.right_sensor
                msg.read_basic = self.read_basic
                self.read_proximity_sensors_publisher_.publish(msg)
                log_msg(self.telemetry_data_logger, "published", msg)
    
    def publish_sensor_settings(self):
        msg = Zumo()
        msg.buzzer = [self.period_sensor_dict['buzzer'], int(self.enable_sensor_dict['buzzer'])]
        msg.imu = [self.period_sensor_dict['imu'], int(self.enable_sensor_dict['imu'])]
        msg.encoders = [self.period_sensor_dict['encoders'], int(self.enable_sensor_dict['encoders'])]
        msg.line_sensors = [self.period_sensor_dict['line_sensors'], int(self.enable_sensor_dict['line_sensors'])]
        msg.proximity_sensors = [self.period_sensor_dict['proximity_sensors'], int(self.enable_sensor_dict['proximity_sensors'])]
        self.data_zumo_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

    def keep_alive_publish(self):
        msg = String()
        msg.data = "serial_interface"
        self.keep_alive_publisher_.publish(msg)
        log_msg(self.telemetry_data_logger, "published", msg)

def main(args=None):
    rclpy.init(args=args) # Initialize ROS2 libraries
    node = SerialInterfaceGeneratorNode()
    try:
        publish_timer = RepeatedTimer(PUBLISH_PERIOD, node.publish) # Parse from serial and publish to topics
        keep_alive_timer = RepeatedTimer(KEEP_ALIVE_PERIOD, node.keep_alive_publish)
        timer_sensor_settings = RepeatedTimer(SENSOR_SETTINGS_PERIOD, node.publish_sensor_settings)
        # --BUZZER-- #
        play_buzzer_timer = RepeatedTimer(PLAY_BUZZER_TIME, node.playing_buzzer)
        # --IMU-- #
        imu_timer = RepeatedTimer(IMU_TIME, node.imu_gen)
        imu_error_timer = RepeatedTimer(ERROR_IMU_TIME, node.imu_error_gen)
        imu_error_init_timer = RepeatedTimer(1.5*ERROR_IMU_TIME, node.imu_error_init_gen)
        # --ENCODERS-- #
        encoders_timer = RepeatedTimer(ENCODERS_TIME, node.encoders_gen)
        encoders_error_left_timer = RepeatedTimer(1.5*ERROR_ENCODERS_TIME, node.encoders_error_left_gen)
        encoders_error_right_timer = RepeatedTimer(ERROR_ENCODERS_TIME, node.encoders_error_right_gen)
        # --LINE SENSORS-- #
        line_sensors_timer = RepeatedTimer(LINE_SENSORS_TIME, node.line_sensors_gen)
        # --PROXIMITY SENSORS-- #
        proximity_sensors_timer = RepeatedTimer(PROXIMITY_SENSORS_TIME, node.proximity_sensors_gen)
        proximity_sensors_basic_left_timer = RepeatedTimer(20*PROXIMITY_SENSORS_TIME, node.proximity_sensors_basic_left_gen)
        proximity_sensors_basic_front_timer = RepeatedTimer(10*PROXIMITY_SENSORS_TIME, node.proximity_sensors_basic_front_gen)
        proximity_sensors_basic_right_timer = RepeatedTimer(15*PROXIMITY_SENSORS_TIME, node.proximity_sensors_basic_right_gen)
        rclpy.spin(node) # Make node run in background

    except KeyboardInterrupt:
        node.logger.info("Serial Interface Generator got Keyboard Interrupt")
        pass

    finally:
        publish_timer.stop() 
        keep_alive_timer.stop()
        timer_sensor_settings.stop()
        play_buzzer_timer.stop()
        imu_timer.stop()
        imu_error_timer.stop()
        imu_error_init_timer.stop()
        encoders_timer.stop()
        encoders_error_left_timer.stop()
        encoders_error_right_timer.stop()
        line_sensors_timer.stop()
        proximity_sensors_timer.stop()
        proximity_sensors_basic_left_timer.stop()
        proximity_sensors_basic_front_timer.stop()
        proximity_sensors_basic_right_timer.stop()
        node.logger.info("Shutting down Serial Interface Generator")
        node.destroy_node()

if __name__ == "__main__":
    main()