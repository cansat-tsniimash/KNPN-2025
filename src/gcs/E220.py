import RPi.GPIO as GPIO
import serial
import struct
import time
import datetime
import argparse
import sys
import socket
import math	

GPIO.cleanup()
# настроить пины

radio_M0 = 23
radio_M1 = 24
GPIO.setmode(GPIO.BCM)
GPIO.setup(radio_M0, GPIO.OUT)
GPIO.setup(radio_M1, GPIO.OUT)

# настроить порт UART
port = "/dev/ttyRF1"    #"COM1"
baudrate = 9600
timeout = 0.1
ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
baudrate = 38400

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

#Настройка записи в сsv и bin
def generate_logfile_name():
	now = datetime.datetime.utcnow().replace(microsecond=0)
	isostring = now.isoformat()  # string 2021-04-27T23:17:31
	isostring = isostring.replace("-", "")  # string 20210427T23:17:31
	isostring = isostring.replace(":", "")  # string 20210427T231731, òî ÷òî íàäî
	return "./log/knpn_binary" + isostring + ".bin"

def generate_csv_name(text):
	now = datetime.datetime.utcnow().replace(microsecond=0)
	isostring = now.isoformat()  # string 2021-04-27T23:17:31
	isostring = isostring.replace("-", "")  # string 20210427T23:17:31
	isostring = isostring.replace(":", "")  # string 20210427T231731, òî ÷òî íàäî
	return text + isostring + ".csv"

filename_f = generate_logfile_name()
filename_f_ = filename_f + ".super"
filename_f1 = generate_csv_name("./log/1_knpn")
filename_f2 = generate_csv_name("./log/2_knpn")
f = open(filename_f, 'wb')
f.flush()
f_ = open(filename_f_, 'wb')
f_.flush()
f1 = open(filename_f1, 'w')
f1.write('"Number";"Time_ms";"Accel x";"Accel y";"Accel z";"Gyro x";"Gyro y";"Gyro z";"Mag x";"Mag y";"Mag z";"Bme_temp";"Bme_press";"Bme_humidity";"Bme_height";"Lux_board";"Lux_sp";"State";"Lidar";"crc"\n')
f1.flush()
f2 = open(filename_f2, 'w')
f2.write('"Number";"Time_ms";"Fix";"Lat";"Lon";"Alt";"GPS_time_s";"Current";"Bus_voltage";"MICS_5524";"MICS_CO";"MICS_NO2";"MICS_NH3";"CCS_CO2";"CCS_TVOC";"Bme_temp_g";"Bme_press_g";"Bme_humidity_g";"crc"\n')
f2.flush()

# настройка малинки
operating_mode(3)
addr_datarate = 0x02
data_rate = 0xA5
addr_channel = 0x04
channel = 0x28
time.sleep(1)
command(addr_datarate, 0x01, data_rate)
time.sleep(1)
command(addr_channel, 0x01, channel)
time.sleep(1)
read_register = ser.read(120)
print(read_register)

ser.write([0xC1, 0x00, 0x09])
read_register = ser.read(13)

try:
	print(struct.unpack("<3BH7B", read_register[:12]))
except Exception as e:
	print(e)

operating_mode(0)
time.sleep(1)
ser.close()
ser = serial.Serial(port, baudrate=baudrate, timeout=timeout)
time.sleep(1)
print("F")
host = '0.0.0.0'
port = 22000
addr_udp = None
udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_socket.setblocking(False)
udp_socket.settimeout(0.001)
print("F")
udp_socket.bind((host, port))
print("F")


#print("F")
# читаем
buf = bytes()
flug_0 = 0xAA
flug_1 = 0xBB
while True: 
	portion = ser.read(1000)
	if portion:
		f_.write(portion)
		f_.flush()

	buf += portion

	#if len(portion) == 0:
	#	print("NO DATA")
	#else:
	#    print (portion)
	try:
		conn, addr = udp_socket.recvfrom(1)
		if len(conn) > 0:
			print("Connected to ", addr)
			addr_udp = addr
	except TimeoutError:
		pass


	try:
		while len(buf) > 0:
			flug_cond = struct.unpack("B", buf[:1])
			if flug_cond[0] == flug_0:
			#    print("buf: ", buf)
				if len(buf) >= 50:
					
					crc_cond = crc16(buf[:48])
		
					crc = struct.unpack("H", buf[48:50])
					if crc_cond == crc[0]:
						print("==== Пакет тип 1 ====")
						unpack_data = struct.unpack("<BHI10hIh3fBhH", buf[:50])
						if addr_udp is not None:
							udp_socket.sendto(buf[:50], addr_udp)
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
					#    print ("Accelerometer x", unpack_data[3])
					#    print ("Accelerometer y", unpack_data[4])
					#    print ("Accelerometer z", unpack_data[5])
					#    print ("Gyroscope x", unpack_data[6])
					#    print ("Gyroscope y", unpack_data[7])
					#    print ("Gyroscope z", unpack_data[8])
					#    print ("Magnetometer x", unpack_data[9])
					#    print ("Magnetometer y", unpack_data[10])
					#    print ("Magnetometer z", unpack_data[11])
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

						f.write(buf[:50])
						f.flush()

						for i in range(1,21):
							f1.write(str(unpack_data[i]))
							f1.write(";")
						f1.write('\n')
						f1.flush()

					   # print ("crc_ground", crc16(buf[:48]))
						buf = buf[50:]
					else:
						buf = buf[1:]
						break

				else:
					buf = buf[1:]
					break
			elif flug_cond[0] == flug_1:
				if len(buf) >= 53:
					R0_5524 = 4444
					R0_6814_CO = 418151
					R0_6814_NO2 = 29600
					R0_6814_NH3 = 30650
					crc_cond = crc16(buf[:51])
		
					crc = struct.unpack("H", buf[51:53])

					if crc_cond == crc[0]:
						print("==== Пакет тип 2 ====")
						unpack_data = struct.unpack("<BHIh3fIf7HhIhH", buf[:53])
						if addr_udp is not None:
							udp_socket.sendto(buf[:53], addr_udp)
						print ("Number", unpack_data[1])
						print ("Time_ms", unpack_data[2])	
						print ("Fix", unpack_data[3])
						print ("Lat", unpack_data[4])
						print ("Lon", unpack_data[5])
						print ("Alt", unpack_data[6])
						print ("GPS_time_s", unpack_data[7])
						print ("Current", unpack_data[8])
						print ("Bus_voltage", unpack_data[9])
						
						print ("MICS_5524_U", unpack_data[10])
						print ("MICS_CO_U", unpack_data[11])
						print ("MICS_NO2_U", unpack_data[12])
						print ("MICS_NH3_U", unpack_data[13])

						print ("CCS_CO2", unpack_data[14])
						print ("CCS_TVOC", unpack_data[15])
						print ("Bme_temp_g", unpack_data[16])
						print ("Bme_press_g", unpack_data[17])
						print ("Bme_humidity_g", unpack_data[18])
						print ("crc", unpack_data[19])

						y_CO_5524 = ((10000 * unpack_data[10])/(3.3 - (unpack_data[10] / 1000)))/R0_5524
						y_CO_6814 = ((10000 * unpack_data[11])/(3.3 - (unpack_data[10] / 1000)))/R0_6814_CO
						print ("MICS_5524_ppm", 4.5094139420749839545/y_CO_5524**1.1729399483804680851)
						print ("MICS_CO_ppm", 4.5094139420749839545/y_CO_6814**1.1729399483804680851)
						y_NO2 = ((10000 * unpack_data[12])/(3.3 - (unpack_data[12] / 1000)))/R0_6814_NO2
						print ("MICS_NO2_ppm", 0.16240122845736334614*y_NO2**0.97846644245673148509)
						y_NH3 = ((10000 * unpack_data[13])/(3.3 - (unpack_data[13] / 1000)))/R0_6814_NH3
						print ("MICS_NH3_ppm", 0.63849852577435507858/y_NH3**1.860835538460201907)
						print ('\n') 

						f.write(buf)
						f.flush()

						for i in range(1,20):
							f2.write(str(unpack_data[i]))
							f2.write(";")
						f2.write('\n')
						f2.flush()


					  #  print ("crc_ground", crc16(buf[:51]))
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
	except Exception as e:
		print("ERROR: %s", e)
		print ('\n')
		buf = bytes()

