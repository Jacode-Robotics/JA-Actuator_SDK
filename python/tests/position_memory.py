import os
import struct
import time
import csv

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

from dynamixel_sdk import *

PROTOCOL_VERSION            = 2.0
BAUDRATE                = 2000000

# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

ADDR_PRESENT_POSITION       = 580
LEN_PRESENT_POSITION        = 4
ADDR_GOAL_POSITION          = 564
LEN_GOAL_POSITION           = 4

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
positionRead = GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)


if portHandler.openPort() == 0:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE) == 0:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

ID = []
def operation_preparation():
    for i in range(1,21):
        dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
        if dxl_comm_result != COMM_SUCCESS:
            continue
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("ID [%d] ping Succeeded" % i)
            print('number:%d' % dxl_model_number)
            ID.append(i)
        time.sleep(0.2)
    time.sleep(0.2)
    for i in range(0, len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 11, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Change operation mode succeeded" )
        time.sleep(0.2)
    time.sleep(0.1)

    for i in range(0, len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 512, 1)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("torque enable succeeded" )
        time.sleep(0.2)

def addparam():
    for i in range(0, len(ID)):
        dxl_addparam_result = positionRead.addParam(ID[i])
        if dxl_addparam_result != True:
            print("[ID:%03d] positionRead addparam failed" % ID[i])
            quit()

def get_position():
    POSITION = []
    dxl_comm_result = positionRead.txRxPacket()
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

    for i in range(0, len(ID)):
        present_position = positionRead.getData(ID[i], ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        present_position = struct.unpack('i', struct.pack('I', present_position))[0]
        print("[ID:%03d] present_position:%03d" % (ID[i] ,present_position))
        POSITION.append(present_position)

        time.sleep(0.2)

    return POSITION

def log_positions(filename='arm_positions.csv'):
    try:
        with open(filename, mode='a', newline='') as file:
            flag = 1
            writer = csv.writer(file)
            while flag:
                positions = get_position()
                writer.writerow(positions)
                print(f"Logged positions: {positions}")
                print('')
                print('ESC to exit and any other key to continue')
                if getch() == chr(0x1b):
                    flag = 0
    except KeyboardInterrupt:
        print("The program has been manually stopped")

def read_positions(filename='arm_positions.csv'):
    positions_list = []
    with open(filename, mode='r', newline='') as file:
        reader = csv.reader(file)
        for row in reader:
            positions_list.append([int(value) for value in row])
    return positions_list, len(positions_list)

def read_single_position_row(filename='arm_positions.csv', row_number=0):
    if row_number < 0:
        raise ValueError("row_number must be a non-negative integer")

    with open(filename, mode='r', newline='') as file:
        reader = csv.reader(file)
        for i, row in enumerate(reader):
            if i == row_number:
                return [int(value) for value in row]
        raise IndexError("row_number out of range")

if __name__ == '__main__':
##############################################################################################################################
########记录位置###############################################################################################################
    
    operation_preparation()
    addparam()
    log_positions()
    print("Recording Complete")
    '''  '''
########记录位置###############################################################################################################
##############################################################################################################################

##############################################################################################################################
########复刻位置###############################################################################################################

    # for i in range(1,21):
    #     dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         continue
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         print("ID [%d] ping Succeeded" % i)
    #         print('number:%d' % dxl_model_number)
    #         ID.append(i)
    #     time.sleep(0.1)

    # for i in range(0,len(ID)):
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 11, 4)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         print("Change operation mode succeeded" )
    #     time.sleep(0.2)
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 10, 4)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         print("Change drive mode succeeded" )
    #     dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 512, 1)
    #     if dxl_comm_result != COMM_SUCCESS:
    #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         print("%s" % packetHandler.getRxPacketError(dxl_error))
    #     time.sleep(0.3)
    # positions_data, data_length = read_positions()

    # for i in range(0,data_length):
    #     flag = 1
    #     positions_data = read_single_position_row(row_number = i)
    #     print(positions_data)
    #     position = positions_data[0]

##############################################################################################################################
########单电机#################################################################################################################
        
        # for i in range(0,len(ID)):
        #     dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID[i], 564, position)
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #     elif dxl_error != 0:
        #         print("%s" % packetHandler.getRxPacketError(dxl_error))
        #     else:
        #         print("%d successed"%position)

        #     while flag:
        #         dxl_present_velocity, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, 2, 571)
        #         if dxl_present_velocity == 1:
        #             flag = 0

########单电机#################################################################################################################
##############################################################################################################################

##############################################################################################################################
########多电机#################################################################################################################

        # for i in range(0, len(ID)): 
        #     param_goal_position = [DXL_LOBYTE(DXL_LOWORD(positions_data[i])), DXL_HIBYTE(DXL_LOWORD(positions_data[i])), DXL_LOBYTE(DXL_HIWORD(positions_data[i])), DXL_HIBYTE(DXL_HIWORD(positions_data[i]))]
        #     dxl_addparam_result = groupSyncWrite.addParam(ID[i], param_goal_position)
        #     if dxl_addparam_result != True:
        #         print("[ID:%03d] groupSyncWrite addparam failed" % ID[i])
        #         quit()

        #     dxl_comm_result = groupSyncWrite.txPacket()
        #     if dxl_comm_result != COMM_SUCCESS:
        #         print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        #     # Clear syncwrite parameter storage
        #     groupSyncWrite.clearParam()
        #     time.sleep(0.3)

        # STATUS = [0]*len(ID)
        # while 1:
        #     for i in range(0,len(ID)):
        #         moving_status, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(portHandler, ID[i], 571)
        #         moving_status = struct.unpack('i', struct.pack('I', moving_status))[0]
        #         if dxl_comm_result != COMM_SUCCESS:
        #             print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        #         elif dxl_error != 0:
        #             print("%s" % packetHandler.getRxPacketError(dxl_error))
        #         else:
        #             #print("[ID:%03d]  moving_status:%03d" % (ID[i],  moving_status))
        #             STATUS[i] = moving_status
        #             time.sleep(0.1)

        #     if all(element == 1 for element in STATUS):
        #         break

########多电机#################################################################################################################
##############################################################################################################################

########复刻位置###############################################################################################################
##############################################################################################################################

    for i in range(0,len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 512, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    for i in range(0, len(ID)):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 11, 3)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Change operation mode succeeded" )
        time.sleep(0.2)

        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID[i], 10, 0)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            print("Change drive mode succeeded" )

portHandler.closePort()






