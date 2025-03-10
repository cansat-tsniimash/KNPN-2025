import RPi.GPIO as GPIO
import serial
import struct
import time
GPIO.cleanup()
# настроить пины
radio_M0 = 6
radio_M1 = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(radio_M0, GPIO.OUT)
GPIO.setup(radio_M1, GPIO.OUT)

# настроить порт UART
port = "/dev/ttyUSB1"    #"COM1"
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
    ser.write(struct.pack("<4B", 0xC0, adress, length, data))

# расчёт контрольной суммы
def crc16(data : bytearray, offset=0, length=-1):
    if length < 0:
        length = len(data)
    
    if data is None or offset < 0 or offset > len(data)- 1 and offset+length > len(data):
        return 0

    crc = 0xFFFF
    for i in range(0, length):
        crc ^= data[offset + i] << 8
        for j in range(0, 8):
            if (crc & 0x8000) > 0:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
        crc = crc & 0xFFFF
    return crc & 0xFFFF

# настройка малинки
operating_mode(3)
addr_datarate = 0x02
data_rate = 98
addr_channel = 0x04
channel = 0x17
command(addr_datarate, 0x01, data_rate)
time.sleep(1)
command(addr_channel, 0x01, channel)
time.sleep(1)
read_register = ser.read(120)

ser.write([0xC1, 0x00, 0x09])
read_register = ser.read(12)

print(struct.unpack("<3BH7B", read_register[:12]))

operating_mode(0)

# читаем
buf = bytes()
flug_0 = 0xAA
flug_1 = 0xBB
while True: 
    portion = ser.read(100)
    buf += portion
    print (portion)

    while len(buf) > 0:

        flug_cond = struct.unpack("B", buf[:1])
        if flug_cond[0] == flug_0:
            print("buf: ", buf)
            if len(buf) >= 50:
                
                crc_cond = crc16(buf[:48])
    
                crc = struct.unpack("H", buf[48:50])
                if crc_cond == crc[0]:
                    unpack_data = struct.unpack("<BHI10hIh3fBhH", buf[:50])
                    print ("Number", unpack_data[1])
                    print ("Time_ms", unpack_data[2])
                    #print ("Accelerometer x", unpack_data[3]*488/1000/1000)
                    #print ("Accelerometer y", unpack_data[4]*488/1000/1000)
                    #print ("Accelerometer z", unpack_data[5]*488/1000/1000)
                    #print ("Gyroscope x", unpack_data[6]*70/1000)
                    #print ("Gyroscope y", unpack_data[7]*70/1000)
                    #print ("Gyroscope z", unpack_data[8]*70/1000)
                    #print ("Magnetometer x", unpack_data[9]/1711)
                    #print ("Magnetometer y", unpack_data[10]/1711)
                    #print ("Magnetometer z", unpack_data[11]/1711)
                    print ("Accelerometer x", unpack_data[3])
                    print ("Accelerometer y", unpack_data[4])
                    print ("Accelerometer z", unpack_data[5])
                    print ("Gyroscope x", unpack_data[6])
                    print ("Gyroscope y", unpack_data[7])
                    print ("Gyroscope z", unpack_data[8])
                    print ("Magnetometer x", unpack_data[9])
                    print ("Magnetometer y", unpack_data[10])
                    print ("Magnetometer z", unpack_data[11])
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

                    print ("crc_ground", crc16(buf[:48]))
                    buf = buf[50:]
                else:
                    buf = buf[1:]
                    break

            else:
                buf = buf[1:]
                break
        elif flug_cond[0] == flug_1:
            if len(buf) >= 53:
                crc_cond = crc16(buf[:51])
    
                crc = struct.unpack("H", buf[51:53])

                if crc_cond == crc[0]:
                    unpack_data = struct.unpack("<BHIh3fIf7HhIhH", buf[:53])
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


                    print ("crc_ground", crc16(buf[:51]))
                    buf = buf[53:]  
                else:
                    buf = buf[1:]
                    break
            else:
                buf = buf[1:]
                break 
        else:
            buf = buf[1:]
            continue