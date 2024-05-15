

import os
import struct

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

from dynamixel_sdk import *                    # Uses Dynamixel SDK library

# Control table address
ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
ADDR_GOAL_POSITION          = 564
LEN_GOAL_POSITION           = 4          # Data Byte Length
ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4          # Data Byte Length
DXL_MINIMUM_POSITION_VALUE  = -5000      # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 5000       # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 2000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Make sure that each DYNAMIXEL ID should have unique ID.
DXL_ID = [1, 2, 3, 4, 5, 6]


# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'


TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

index = 0
dxl_goal_position = [0, 0 ,0,0,0,0]         # Goal position

# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

# Initialize GroupSyncRead instace for Present Position
groupSyncRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

# Open port
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

# set drive mode to time_position
for i in range(0, len(DXL_ID)):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], 10, 4)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID[%d] time open" % DXL_ID[i])
time.sleep(0.1)

# set profile time
for i in range(0, len(DXL_ID)):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL_ID[i], 522, 5000)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID[%d] set time success" % DXL_ID[i])

for i in range(0, len(DXL_ID)):
    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL_ID[i])

    # Add parameter storage for Dynamixel present position value
    dxl_addparam_result = groupSyncRead.addParam(DXL_ID[i])
    if dxl_addparam_result != True:
        print("[ID:%03d] groupSyncRead addparam failed" % DXL_ID[i])
        quit()

for i in range(0, len(DXL_ID)):
    # Write home position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, 0)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0]
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], 0, dxl_present_position))

        if not abs(dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break 

while 1:
    print("Press any key to continue! (or press ESC to quit!)")
    if getch() == chr(0x1b):
        break
    
    

    pointion1 = [0, 0, -90, 0, -90, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))

    for i in range(0, len(DXL_ID)):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])),\
                               DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        arrived_joint_num = 0
        for i in range(0, len(DXL_ID)):
            # Get Dynamixel present position value
            dxl_present_position = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position))

            if not (abs(dxl_goal_position[i] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                arrived_joint_num += 1

        if arrived_joint_num >= len(DXL_ID):
            print("Reaching zero position")
            break
        time.sleep(0.5)


    pointion1 = [-20, -45, -70, -20, -70, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))

    for i in range(0, len(DXL_ID)):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])),\
                            DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        arrived_joint_num = 0
        for i in range(0, len(DXL_ID)):
            # Get Dynamixel present position value
            dxl_present_position = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position))

            if not (abs(dxl_goal_position[i] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                arrived_joint_num += 1

        if arrived_joint_num >= len(DXL_ID):
            print("Reaching position1")
            break
        time.sleep(3)



    pointion1 = [0, 0, -90, 0, -90, 0]
    for i in range(0,len(pointion1)):
        dxl_goal_position[i] = int(float(pointion1[i])  / 360 * pow(2 , 15))

    for i in range(0, len(DXL_ID)):
        # Allocate goal position value into byte array
        param_goal_position = [DXL_LOBYTE(DXL_LOWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_LOWORD(dxl_goal_position[i])),\
                            DXL_LOBYTE(DXL_HIWORD(dxl_goal_position[i])), DXL_HIBYTE(DXL_HIWORD(dxl_goal_position[i]))]
        # Add Dynamixel goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL_ID[i], param_goal_position)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL_ID[i])
            quit()

    # Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    # Clear syncwrite parameter storage
    groupSyncWrite.clearParam()

    while 1:
        # Syncread present position
        dxl_comm_result = groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        arrived_joint_num = 0
        for i in range(0, len(DXL_ID)):
            # Get Dynamixel present position value
            dxl_present_position = groupSyncRead.getData(DXL_ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];

            print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], dxl_goal_position[i], dxl_present_position))

            if not (abs(dxl_goal_position[i] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                arrived_joint_num += 1

        if arrived_joint_num >= len(DXL_ID):
            print("Reaching position1")
            break
        time.sleep(0.5)



# Clear syncread parameter storage
groupSyncRead.clearParam()

for i in range(0, len(DXL_ID)):
    # Write home position
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID[i], ADDR_GOAL_POSITION, 0)
    
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

    while 1:
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID[i], ADDR_PRESENT_POSITION)
        dxl_present_position = struct.unpack('i', struct.pack('I', dxl_present_position))[0];
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL_ID[i], 0, dxl_present_position))

        if not abs(0 - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
            break    
        
    # Disable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID[i], ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()
