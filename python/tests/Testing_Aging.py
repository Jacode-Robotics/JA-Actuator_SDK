import os
import struct
import time
import keyboard
import ctypes

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

SOURCE                      = 0
# Initialize PortHandler instance
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
portHandler = PortHandler(DEVICENAME)

# Initialize PacketHandler instance
# Set the protocol version
# Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
packetHandler = PacketHandler(PROTOCOL_VERSION)

groupSyncRead = GroupSyncRead(portHandler, packetHandler, 0, 2)

'''def hex_to_decimal(hex_string):
    decimal_value = int(hex_string, 16)
    return decimal_value'''

'''def READ_MODEL_NUMBER(ID):
    data_read = groupSyncRead.getData(ID, 0, 2)
    data_read = struct.unpack('i', struct.pack('I', data_read))[0]
    print(data_read)
    if result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(result))
    elif error != 0:
        print("%s" % packetHandler.getRxPacketError(error))
'''
    
    #NUMBER = hex_to_decimal(data_read)
    #return NUMBER

def MODEL_NUMBER(ID,NUMBER):
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, ID, 0, NUMBER)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("ID:[%d],[%d]connected succeeded" % (ID, NUMBER))

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
        print("CONTROL Succeeded,%d" %DATA)

def profile_velocity(ID, DATA):
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, ID, 560, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("velocity modification Succeeded")

def Control_Mode(ID,DATA):
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, ID, 11, DATA)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Mode switching Succeeded")

flag = True  
def on_key_pressed(event):
    global flag
    if event.name == 'enter':
        print("quit succeeded")
        flag = False  

keyboard.on_press(on_key_pressed)

def Power_judgment():

    for i in range(1,30):
        dxl_model_number,dxl_comm_result, dxl_error = packetHandler.ping(portHandler, i)
        if dxl_comm_result != COMM_SUCCESS:
            continue
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        else:
            global SOURCE
            SOURCE = 1
            print("Succeeded to open the port")
            print("Succeeded to change the baudrate") 
            break

def is_port_online(port_name):
    if os.name == 'nt':
        device_path = f"\\\\.\\{port_name}"
        handle = ctypes.windll.kernel32.CreateFileW(device_path,0x80000000,0,None,3,0x10000000,None)
        if handle != -1:
            ctypes.windll.kernel32.CloseHandle(handle)
            return True
        else:
            return False
    else:
        device_path = f"{port_name}"
        return os.path.exists(device_path)

if is_port_online(DEVICENAME):
    print(f"{DEVICENAME} on line")
else:
    print(f"{DEVICENAME} not online")

# Open port
if portHandler.openPort() == 0:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()


# Set port baudrate
if portHandler.setBaudRate(BAUDRATE) == 0:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

Power_judgment()

if SOURCE:
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
            
        time.sleep(0.5)


    for i in range(0,len(DXL_ID)):
        MODEL_NUMBER(DXL_ID[i],NUMBER[i])
        time.sleep(0.3)
        Torque_Enable(DXL_ID[i],0)
        time.sleep(0.2)
        profile_velocity(DXL_ID[i],500)
        time.sleep(0.2)
        Torque_Enable(DXL_ID[i],1)
    
        #READ_MODEL_NUMBER(i)
        #NUMBER = READ_MODEL_NUMBER(i)
        #MODEL_NUMBER(NUMBER)

    #time.sleep(1)
    while 1:
        id_number = 0
        for i in range(0,len(DXL_ID)):
            opsition_control(DXL_ID[i],0)
            id_number += 1
        if id_number >= len(DXL_ID):
            break

    time.sleep(4)

    while flag:
        for i in range(0,len(DXL_ID)):

            opsition_control(DXL_ID[i],5000)

        time.sleep(4)

        for i in range(0,len(DXL_ID)):
            opsition_control(DXL_ID[i],-5000)
        
        print('succeeded')

        time.sleep(4)
        print('press Enter to quit!')

        time.sleep(0.5)


    '''
    stop = False
    while not stop:
        while 1:
            print('press Enter to quit!')
            time.sleep(0.5)
            if keyboard.is_pressed('1'):
                for i in range(0,5):
                    keyboard.is_pressed('1')
                    print('quit succeeded')
                    stop =  True
                    break
            else:
                for i in range(0,len(DXL_ID)):
                    opsition_control(DXL_ID[i],5000)

                time.sleep(4)

                for i in range(0,len(DXL_ID)):
                    opsition_control(DXL_ID[i],-5000)
            
                print('succeeded')

                time.sleep(4)
                print('press Enter to quit!')

            time.sleep(1)
            if keyboard.is_pressed('1'):
                for i in range(0,5):
                    keyboard.is_pressed('1')
                    print('quit succeeded')
                    stop =  True
                    break
            time.sleep(1)
    '''
        
    '''    for i in range(0,len(DXL_ID)):
            opsition_control(DXL_ID[i],5000)
        time.sleep(4)
        for i in range(0,len(DXL_ID)):
            opsition_control(DXL_ID[i],-5000)
        print('succeeded')
        time.sleep(4)
        time.sleep(1.5)
        print('press Enter to quit!')
        time.sleep(0.5)
        if keyboard.is_pressed('1'):
            print('quit succeeded')
            break
        time.sleep(1)'''

    ''' a = getch()   
        print('press Enter to quit!')
        time.sleep(1)
        if keyboard.is_pressed('enter'):
            print('quit succeeded')
            break
        else:'''

    for i in range(0,len(DXL_ID)):
        opsition_control(DXL_ID[i],0)
    time.sleep(4)

    for i in range(0,len(DXL_ID)):
        Torque_Enable(DXL_ID[i],0)


    # Close port
    portHandler.closePort()

else:
    print("Not powered on/not connected to equipment")





