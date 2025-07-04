import struct
import time
import datetime
import argparse
import sys

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

f = open(sys.argv[1], 'rb')
f1 = open(sys.argv[1] + "_p1.csv", "w")
f1.write('"Number";"Time_ms";"Accel x";"Accel y";"Accel z";"Gyro x";"Gyro y";"Gyro z";"Mag x";"Mag y";"Mag z";"Bme_temp";"Bme_press";"Bme_humidity";"Bme_height";"Lux_board";"Lux_sp";"State";"Lidar";"crc"\n')
f2 = open(sys.argv[1] + "_p1.csv", "w")
f2.write('"Number";"Time_ms";"Fix";"Lat";"Lon";"Alt";"GPS_time_s";"Current";"Bus_voltage";"MICS_5524";"MICS_CO";"MICS_NO2";"MICS_NH3";"CCS_CO2";"CCS_TVOC";"Bme_temp_g";"Bme_press_g";"Bme_humidity_g";"crc"\n')

flug_0 = 0xAA
flug_1 = 0xBB
buf = f.read(10000)
while len(buf) >= 50: 
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
				crc_cond = crc16(buf[:51])
	
				crc = struct.unpack("H", buf[51:53])

				if crc_cond == crc[0]:
					print("==== Пакет тип 2 ====")
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
	portion = f.read(10000)
	buf += portion

f.close()
f1.close()
f2.close()