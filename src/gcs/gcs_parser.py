import sys
import argparse
import time
import struct
import datetime


def generate_logfile_name():
    now = datetime.datetime.utcnow().replace(microsecond=0)
    isostring = now.isoformat()  # string 2021-04-27T23:17:31
    isostring = isostring.replace("-", "")  # string 20210427T23:17:31
    isostring = isostring.replace(":", "")  # string 20210427T231731, òî ÷òî íàäî
    return "./log/knpn_Mandarinas" + isostring + ".bin"


if __name__ == '__main__':
    filename_f = "knpn_Mandarinas20230623T040946.bin"
    filename_f2 = "./log/knpn_2.csv"
    filename_f3 = "./log/knpn_3.csv"
    f = open(filename_f, 'rb')
    f1 = open(filename_f1, 'w')
    f1.write('"Time Pack";"Number";"Temp BME";"Pressure";"Height BME";Current";"Bus voltage";"State";"Photo"\n')
    f1.flush()
    f2 = open(filename_f2, 'w')
    f2.write('"Time Pack";"Number";"Mag x";"Mag y";"Mag z";"Accel x";"Accel y";"Accel z";"Gyro x";"Gyro y";"Gyro z"\n')
    f2.flush()
    f3 = open(filename_f3, 'w')
    f3.write('"Time Pack";"Number";"Temp DS";"Latitude";"Longitude";"Height";"Time, s";"Time, mks";"Fix"\n')
    f3.flush()


    while True:
    	biter = f.read(1)
    	packet_size = struct.unpack(">B", biter)
    	data = f.read(packet_size)

        try:
            if data[0] == 0xaa:
                pass
                print("==== Пакет тип 1 ====")
                unpack_data = struct.unpack("<BIHhIffhbIH", data[:30])
                print ("Temperature BME", unpack_data[3]/100)
                print ("Pressure", unpack_data[4])
                print ("Height BME", unpack_data[5])
                print ("Bus voltage", unpack_data[7]/1000)
                print ("Current", unpack_data[6]/1000)
                print ("Number", unpack_data[2])
                print ("Photo", unpack_data[9]/100)
                print ("State", unpack_data[8])
                print ("Time", unpack_data[1])

                print ('\n')

                for i in range(1,10):
                    f1.write(str(unpack_data[i]))
                    f1.write(";")
                    f1.flush()
                f1.write('\n')

            elif data[0] == 187:
                #continue
                print("==== Пакет тип 2 ====")
                unpack_data = struct.unpack("<BIH9hH", data[:27])
                print ("Accelerometer x", unpack_data[6]/1000)
                print ("Accelerometer y", unpack_data[7]/1000)
                print ("Accelerometer z", unpack_data[8]/1000)
                print ("Gyroscope x", unpack_data[9]/1000)
                print ("Gyroscope y", unpack_data[10]/1000)
                print ("Gyroscope z", unpack_data[11]/1000)
                print ("Magnetometer x", unpack_data[3]/1000)
                print ("Magnetometer y", unpack_data[4]/1000)
                print ("Magnetometer z", unpack_data[5]/1000)
                print ("Number", unpack_data[2])
                print ("Time", unpack_data[1])

                print ('\n')

                for i in range(1,12):
                    f2.write(str(unpack_data[i]))
                    f2.write(";")
                    f2.flush()
                f2.write('\n')

            elif data[0] == 204: 
                #continue
                print("==== Пакет тип 3 ====")
                unpack_data = struct.unpack("<BIHh3f2IbH", data[:32])
                print ("Latitude", unpack_data[4])
                print ("Longitude", unpack_data[5])
                print ("Height", unpack_data[6])
                print ("Time, s", unpack_data[7]) 
                print ("Time, mks", unpack_data[8])
                print ("Fix", unpack_data[9])
                print ("Number", unpack_data[2])
                print ("Time", unpack_data[1])
                print ("Temperature DS", unpack_data[3])

                print ('\n')

                for i in range(1,10):
                    f3.write(str(unpack_data[i]))
                    f3.write(";")
                    f3.flush()
                f3.write('\n')
            else:
                print('unknown flag ', data[0])
        except Exception as e:
            print(e)
