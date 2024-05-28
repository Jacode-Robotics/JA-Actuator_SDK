import os

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import *                 # Uses Dynamixel SDK library

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Define the proper baudrate to search DYNAMIXELs. Note that XL320's baudrate is 1 M bps.
BAUDRATE                = 2000000

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 1
NEW_ID                      = 0

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = 'COM15'

#THE data of calibrate the zero position is 17
ZERO_POSITION               = 17

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

def MODEL_NUMBER(ID,NUMBER):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, 0, NUMBER)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("connected succeeded")

def Torque_Enable(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, 512, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    if DATA == 0:
        print('disable succeeded')
    else:
        print('Ensable  succeeded')


def opsition_control(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, 564, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("CONTROL Succeeded")

def velocity_control(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, 552, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

def Control_Mode(ID,DATA):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, 11, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Mode switching Succeeded")


if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

DXL_ID = []
NUMBER = []
for i in range(1,21):
    dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
    if dxl_comm_result != COMM_SUCCESS:
        continue
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID [%d] ping Succeeded" % i)
        print('number:%d' % dxl_model_number)
        DXL_ID.append(i)
        NUMBER.append(dxl_model_number)

ID = DXL_ID[0]
NUM = NUMBER[0]

MODEL_NUMBER(ID,NUM)
time.sleep(0.3)

Torque_Enable(ID, 1)
time.sleep(0.5)

opsition_control(ID,0)
time.sleep(3)

Torque_Enable(ID,0)
time.sleep(0.5)

Control_Mode(ID,1)
time.sleep(0.5)

Torque_Enable(ID, 1)
time.sleep(0.5)

while 1:
    velocity_control(ID, 100)
    if getch() == chr(0x0d):
        break
print("Press 'Enter' to continue!")
    
velocity_control(ID, 0)
time.sleep(1)

while 1:
    if getch() == chr(0x0d):
        break
    print("Press 'Enter' to continue!")

Torque_Enable(ID,0)
time.sleep(1)

Control_Mode(ID,4)
time.sleep(1)

Torque_Enable(ID, 1)
time.sleep(1)

opsition_control(ID,0)
time.sleep(3)

Torque_Enable(ID, 0)
time.sleep(0.5)

# Close port
portHandler.closePort()








