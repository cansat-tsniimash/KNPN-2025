import RPi.GPIO as GPIO
import serial
import struct



# настроить пины
radio_M0 = 24
radio_M1 = 23
GPIO.setmode(GPIO.BCM)
GPIO.setup(radio_M0, GPIO.OUT)
GPIO.setup(radio_M1, GPIO.OUT)

# настроить порт UART
port = "/dev/ttyUSB0"    #"COM1"
baudrate = 9600
timeout = 0.1
ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)

# настроить режим работы малинки (0 - писать, 1 - писать/читать, 2 - читать, 3 - настройки)
def operating_mode (mode):
    if mode == 0:
        GPIO.output(radio_M0, GPIO.LOW)
        GPIO.output(radio_M1, GPIO.LOW)
    elif mode == 1:
        GPIO.output(radio_M0, GPIO.HIGH)
        GPIO.output(radio_M1, GPIO.LOW)
    elif mode == 2:
        GPIO.output(radio_M0, GPIO.LOW)
        GPIO.output(radio_M1, GPIO.HIGH)
    elif mode == 3:
        GPIO.output(radio_M0, GPIO.HIGH) 
        GPIO.output(radio_M1, GPIO.HIGH)
    else:
        print("Error. Number is not defined")

# отправить комманду для нстройки малинки
def command (adress, length, data):
    ser.write([0xC0, adress, length, data])

# настройка малинки
operating_mode(3)
addr_datarate = 0x02
data_rate = 0b1100101
addr_channel = 0x04
channel = 0xA
command(addr_datarate, 0x01, data_rate)
command(addr_channel, 0x01, channel)
operating_mode(2)

# читаем
buffer = bytes()
flug_0 = 0xAA
flug_1 = 0xBB
while True: 
    buffer += ser.read(100)
    flug_cond = 0
    if flug_cond == flug_0:
        while len(buffer) >= 51:
            flug_cond = struct.unpack("B", buffer[0])
            crc_cond = 0
            for i in buffer[0:48]:        
                crc_cond = crc_cond + i

            crc = struct.unpack("H", buffer[48:50])
            crc_origin = crc[0]
            if crc_cond == crc_origin:
                unpack_data = struct.unpack("BHI10hIh3fBhH", buffer[:50])
                print ("Number", unpack_data[1])
                print ("Time_ms", unpack_data[2])
                print ("Accelerometer x", unpack_data[3]*488/1000/1000)
                print ("Accelerometer y", unpack_data[4]*488/1000/1000)
                print ("Accelerometer z", unpack_data[5]*488/1000/1000)
                print ("Gyroscope x", unpack_data[6]*70/1000)
                print ("Gyroscope y", unpack_data[7]*70/1000)
                print ("Gyroscope z", unpack_data[8]*70/1000)
                print ("Magnetometer x", unpack_data[9]/1711)
                print ("Magnetometer y", unpack_data[10]/1711)
                print ("Magnetometer z", unpack_data[11]/1711)
                print ("Bme_temp", unpack_data[12])
                print ("Bme_press", unpack_data[13])
                print ("Bme_humidity", unpack_data[14])
                print ("Bme_height", unpack_data[15])
                print ("Lux_board", unpack_data[16])
                print ("Lux_sp", unpack_data[17])
                print ("State", unpack_data[18])
                print ("Lidar", unpack_data[19])
                print ("crc", unpack_data[20])
                print ("\n")
                buffer = buffer[50:]
            else:
                buffer = buffer[1:]
                continue
    elif flug_cond == flug_1:
        while len(buffer) >= 54:
            crc_cond = 0
            for i in buffer[0:51]:        
                crc_cond = crc_cond + i

            crc = struct.unpack("H", buffer[51:53])
            crc_origin = crc[0]
            if crc_cond == crc_origin:
                unpack_data = struct.unpack("BHIh3fIf7HhIhH", buffer[:53])
                print ("Number", unpack_data[1])
                print ("Time_ms", unpack_data[2])
                print ("Fix", unpack_data[3])
                print ("Lat", unpack_data[4])
                print ("Lon", unpack_data[5])
                print ("Alt", unpack_data[6])
                print ("GPS_time_s", unpack_data[7])
                print ("Current", unpack_data[8])
                print ("Bus_voltage", unpack_data[9])
                print ("MICS_5524", unpack_data[10])
                print ("MICS_CO", unpack_data[11])
                print ("MICS_NO2", unpack_data[12])
                print ("MICS_NH3", unpack_data[13])
                print ("CCS_CO2", unpack_data[14])
                print ("CCS_TVOC", unpack_data[15])
                print ("Bme_temp_g", unpack_data[16])
                print ("Bme_press_g", unpack_data[17])
                print ("Bme_humidity_g", unpack_data[18])
                print ("crc", unpack_data[19])
                print ('\n') 
                buffer = buffer[53:]  
            else:
                buffer = buffer[1:]
                continue 
    else:
        buffer = buffer[1:]
        continue