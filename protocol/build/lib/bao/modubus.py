import time
from .port_pack import *

ID                  = 0
FUNCTION            = 1
ADDRESS             = 2
DATA_CONTENT        = 4
DATA_LENGTH         = 4
CRC                 = 8
READ                = 3
WRITE               = 6

#CRC计算
def crc16(buffer):

    temp = 0xFFFF 
    for byte in buffer:  
        temp ^= byte
        for _ in range(8):  
            if (temp & 0x01) == 0:
                temp >>= 1
            else:
                temp >>= 1
                temp ^= 0xA001
    return temp

#将整数转换成元素为四个二位十六进制数的列表
def format_hex_last(num):

    hex_str = f"{num:08x}"
    parts = [hex_str[i:i+2] for i in range(0, 8, 2)]
    parts = parts[:-1] + [parts[-1]]
    formatted_parts = [f"0x{part}" for part in parts]
    return formatted_parts

#写命令
def write(port,id,address,data):
    pack = [0] * 10

    pack[ID]                = id
    pack[FUNCTION]          = WRITE

    location = address.zfill(4)
    location_int = int(location, 16)
    address_1 = (location_int >> 8) & 0xff
    address_2 = location_int & 0xff

    pack[ADDRESS]           = address_1
    pack[ADDRESS + 1]       = address_2

    if data >= 0:
        hex_data = format_hex_last(data)
        for i in range(0,len(hex_data)):
            hex_data_int = int(hex_data[i],16)

            pack[DATA_CONTENT + i] = hex_data_int

    elif data < 0:  
        max_uint32 = (1 << 32) - 1  
        complement = max_uint32 + 1 + data
        hex_data = hex(complement)[2:]
        data_32bit_int = int(hex_data,16)
        processing_data =[
        (data_32bit_int >> 24) & 0xff,
        (data_32bit_int >> 16) & 0xff,
        (data_32bit_int >> 8) & 0xff,
        data_32bit_int & 0xff
        ]
        final_data = [f"0x{byte:02x}" for byte in processing_data]
        for i in range(0,len(final_data)):
            final_data_int = int(final_data[i],16)

            pack[DATA_CONTENT + i] = final_data_int

    crc_array = [0] * 8
    crc_array = pack[:-2]
    crc_int = crc16(crc_array)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2
    try:
        crc_array = pack[:-2]
        crc_int = crc16(crc_array)
        pack[CRC] = crc_int >> 8
        pack[CRC + 1] = crc_int & 0xff
    except Exception as e:
        print(f"CRC计算失败: {e}")
        return None
    
    port.writePort(pack)
    response = port.readPort(10)
    if response is not None:
        response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
    else:
        print("没有接收到响应。")

    return response_array

#读命令
def read(port,id,address):
    pack = [0] * 10

    pack[ID]                = id
    pack[FUNCTION]          = READ

    location = address.zfill(4)
    location_int = int(location, 16)
    address_1 = (location_int >> 8) & 0xff
    address_2 = location_int & 0xff

    pack[ADDRESS]               = address_1
    pack[ADDRESS + 1]           = address_2

    pack[DATA_LENGTH]           = 0X00
    pack[DATA_LENGTH + 1]       = 0X00
    pack[DATA_LENGTH + 2]       = 0X00
    pack[DATA_LENGTH + 3]       = 0X02

    crc_array = [0] * 8
    crc_array = pack[:-2]
    crc_int = crc16(crc_array)
    crc1 = (crc_int >> 8) & 0xff
    crc2 = crc_int & 0xff

    pack[CRC]               = crc1
    pack[CRC + 1]           = crc2
    try:
        crc_array = pack[:-2]
        crc_int = crc16(crc_array)
        pack[CRC] = crc_int >> 8
        pack[CRC + 1] = crc_int & 0xff
    except Exception as e:
        print(f"CRC计算失败: {e}")
        return None
    
    port.writePort(pack)
    response = port.readPort(10)
    if response is not None:
        response_array = [f"0x{response.hex()[i:i+2]}" for i in range(0, len(response.hex()), 2)]
    else:
        print("没有接收到响应。")

    read_data1 = response_array[DATA_CONTENT]
    read_data2 = response_array[DATA_CONTENT + 1]
    read_data3 = response_array[DATA_CONTENT + 2]
    read_data4 = response_array[DATA_CONTENT + 3]

    read_data1 = read_data1[2:]
    read_data2 = read_data2[2:]
    read_data3 = read_data3[2:]
    read_data4 = read_data4[2:]

    hex_read_data = '0x' + read_data1 + read_data2 + read_data3 + read_data4
    read_data = int(hex_read_data,16)

    return read_data,response_array
