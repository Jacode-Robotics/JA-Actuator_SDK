import serial

class SerialPort:
    def __init__(self, port_name, baudrate=115200):
        self.port_name = port_name
        self.baudrate = baudrate
        self.ser = None
        self.is_open = False

    def openPort(self):
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1
            )
            self.is_open = True
            print("端口已打开。")
        except serial.SerialException as e:
            print(f"打开端口时出错: {e}")
            return False
        return True

    def closePort(self):
        if self.is_open and self.ser is not None:
            self.ser.close()
            self.is_open = False
            print("端口已关闭。")

    def readPort(self, length):
        if self.ser and self.is_open:
            return self.ser.read(length)
        else:
            print("端口未打开或不存在。")
            return None

    def writePort(self, data):
        if self.ser and self.is_open:
            return self.ser.write(data)
        else:
            print("端口未打开或不存在。")
            return None

    def setBaudRate(self, baudrate):
        if self.ser and self.is_open:
            self.ser.baudrate = baudrate
            self.baudrate = baudrate
            print(f"波特率已设置为 {baudrate}。")
        else:
            print("端口未打开或不存在，无法设置波特率。")
