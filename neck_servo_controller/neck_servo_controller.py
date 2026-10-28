# Publish the angle to topic /neck_angle, data type is !!!string!!!
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS


class NeckServoController(Node):

    # XM430-W350-T control table
    ADDR_TORQUE_ENABLE = 64
    ADDR_OPERATING_MODE = 11
    ADDR_GOAL_POSITION = 116
    ADDR_PROFILE_VELOCITY = 112

    # Servo configuration
    DXL_ID = 1
    BAUDRATE = 57600
    PORT = "/dev/ttyUSB0"

    # Preset angles relative to home
    PRESET_ANGLES = {
        "0": 0,
        "30": 30,
        "45": 45,
        "90": 15,
    }

    HOME_TICK = 2048
    DIRECTION = 1

    def __init__(self):
        super().__init__("neck_servo_controller")

        self.port = PortHandler(self.PORT)
        self.packet = PacketHandler(2.0)
        self.connected = False
        self.torque_enabled = False

        if not self.port.openPort():
            raise RuntimeError("Failed to open serial port")

        self.connected = True

        try:
            if not self.port.setBaudRate(self.BAUDRATE):
                raise RuntimeError("Failed to set baud rate")

            model, result, error = self.packet.ping(
                self.port, self.DXL_ID
            )
            self.check_result(result, error)

            self.get_logger().info(
                f"Dynamixel connected, model={model}"
            )

            # Disable torque before changing operating mode
            self.write1(self.ADDR_TORQUE_ENABLE, 0)
            self.write1(self.ADDR_OPERATING_MODE, 3)

            # Set profile velocity
            self.write4(self.ADDR_PROFILE_VELOCITY, 30)

            # Enable torque
            self.write1(self.ADDR_TORQUE_ENABLE, 1)
            self.torque_enabled = True

            self.subscription = self.create_subscription(
                String,
                "/neck_angle",
                self.command_callback,
                10
            )

            self.get_logger().info(
                "Neck controller ready. "
                "Listening to /neck_angle"
            )

        except Exception:
            self.shutdown()
            raise

    def check_result(self, result, error):
        if result != COMM_SUCCESS:
            raise RuntimeError(
                self.packet.getTxRxResult(result)
            )
        if error != 0:
            raise RuntimeError(
                self.packet.getRxPacketError(error)
            )

    def write1(self, address, value):
        result, error = self.packet.write1ByteTxRx(
            self.port, self.DXL_ID, address, value
        )
        self.check_result(result, error)

    def write4(self, address, value):
        result, error = self.packet.write4ByteTxRx(
            self.port, self.DXL_ID, address, value
        )
        self.check_result(result, error)

    def angle_to_tick(self, angle):
        tick = self.HOME_TICK + round(
            self.DIRECTION * angle * 4096 / 360
        )

        if not 0 <= tick <= 4095:
            raise ValueError(
                f"Target tick {tick} is outside position range"
            )

        return tick

    def command_callback(self, msg):
        command = msg.data.strip().lower()

        if command not in self.PRESET_ANGLES:
            self.get_logger().warn(
                f"Unknown neck command: {command}"
            )
            return

        angle = self.PRESET_ANGLES[command]
        goal_tick = self.angle_to_tick(angle)

        try:
            self.write4(self.ADDR_GOAL_POSITION, goal_tick)

            self.get_logger().info(
                f"Moving neck: {command}, "
                f"angle={angle} deg, tick={goal_tick}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to move neck: {e}"
            )

    def shutdown(self):
        if self.connected:
            if self.torque_enabled:
                try:
                    self.write1(self.ADDR_TORQUE_ENABLE, 0)
                except Exception as e:
                    self.get_logger().error(str(e))

                self.torque_enabled = False

            self.port.closePort()
            self.connected = False


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = NeckServoController()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f"Controller error: {e}")

    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
