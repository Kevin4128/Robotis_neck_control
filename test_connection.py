from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = "/dev/ttyUSB0"
BAUDRATE = 115200
DXL_ID = 1

port = PortHandler(PORT)
packet = PacketHandler(2.0)

if not port.openPort():
    raise RuntimeError("Failed to open serial port")

try:
    if not port.setBaudRate(BAUDRATE):
        raise RuntimeError("Failed to set baud rate")

    model, result, error = packet.ping(port, DXL_ID)

    if result != COMM_SUCCESS:
        print(packet.getTxRxResult(result))
    elif error != 0:
        print(packet.getRxPacketError(error))
    else:
        print(f"Servo connected! ID={DXL_ID}, Model={model}")
finally:
    port.closePort()
