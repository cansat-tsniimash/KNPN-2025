import sys
import argparse
import time
import struct
import datetime

from RF24 import RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
from RF24 import RF24_1MBPS, RF24_250KBPS, RF24_2MBPS
from RF24 import RF24_CRC_16, RF24_CRC_8, RF24_CRC_DISABLED
from RF24 import RF24 as RF24_CLASS
from RF24 import RF24_CRC_DISABLED
from RF24 import RF24_CRC_8 
from RF24 import RF24_CRC_16

radio2=RF24_CLASS(22, 1)
#radio2=RF24_CLASS(24, 1)
#radio2=RF24_CLASS(22, 0)


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


if __name__ == '__main__':
    static_payload_size = None

    radio2.begin()

    radio2.openReadingPipe(1, b'\xac\xac\xac\xac\xac')

    radio2.setCRCLength(RF24_CRC_8)
    radio2.setAddressWidth(5)
    radio2.channel = 10
    radio2.setDataRate(RF24_250KBPS)
    radio2.setAutoAck(False)


    if static_payload_size is not None:
        radio2.disableDynamicPayloads()
        radio2.payloadSize = static_payload_size
    else:
        radio2.enableDynamicPayloads()

    #radio2.enableAckPayload()
    #radio2.enableDynamicAck()
    #radio2.setCRCLength(RF24_CRC_DISABLED)
 
    radio2.startListening()
    radio2.printDetails()


    filename_f = generate_logfile_name()
    filename_f1 = generate_csv_name("./log/1_knpn")
 #   filename_f2 = "./log/knpn_2.csv"
 #   filename_f3 = "./log/knpn_3.csv"
    f = open(filename_f, 'wb')
    f.flush()
    f1 = open(filename_f1, 'w')
    f1.write('"Number";"Time_ms";"Accel x";"Accel y";"Accel z";"Gyro x";"Gyro y";"Gyro z";"Mag x";"Mag y";"Mag z";"crc"\n')
    f1.flush()
  #  f2 = open(filename_f2, 'w')
  #  f2.write('"Time Pack";"Number";"Mag x";"Mag y";"Mag z";"Accel x";"Accel y";"Accel z";"Gyro x";"Gyro y";"Gyro z"\n')
  #  f2.flush()
  #  f3 = open(filename_f3, 'w')
  #  f3.write('"Time Pack";"Number";"Temp DS";"Latitude";"Longitude";"Height";"Time, s";"Time, mks";"Fix"\n')
  #  f3.flush()


    while True:
        has_payload, pipe_number = radio2.available_pipe()
        #print(f'has_payload-{has_payload}, pipe_number={pipe_number}')

        if has_payload:
            payload_size = static_payload_size
            if payload_size is None:
                payload_size = radio2.getDynamicPayloadSize()

            data = radio2.read(payload_size)
            #print('got data %s' % data)
            packet = data
            packet_size = len(packet)
            biter = struct.pack(">B", packet_size)
            record = biter + packet
            if len(data) <= 0:
                continue

            try:
                
                if data[0] == 170:
                    #continue
                    print("==== Пакет тип 1 ====")
                    unpack_data = struct.unpack("<BHIhhhhhhhhhH", data[:27])
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
                    print ("crc", unpack_data[12])
                    # print ("Number", unpack_data[2])
                    # print ("Time", unpack_data[1])
                    print ('\n')

                    for i in range(1,13):
                        f1.write(str(unpack_data[i]))
                        f1.write(";")
                    f1.write('\n')
                    f1.flush()

               #if data[0] == 0xaa:
               #    #pass
               #    print("==== Пакет тип 1 ====")
               #    unpack_data = struct.unpack("<BIHhIffhbIH", data[:30])
               #    print ("Temperature BME", unpack_data[3]/100)
               #    print ("Pressure", unpack_data[4])
               #    print ("Height BME", unpack_data[5])
               #    print ("Bus voltage", unpack_data[7]/1000)
               #    print ("Current", unpack_data[6]/1000)
               #    print ("Number", unpack_data[2])
               #    print ("Photo", unpack_data[9]/100)
               #    print ("State", unpack_data[8])
               #    print ("Time", unpack_data[1])

               #    print ('\n')

               #    for i in range(1,10):
               #        f1.write(str(unpack_data[i]))
               #        f1.write(";")
               #        f1.flush()
               #    f1.write('\n')

               #elif data[0] == 187:
               #    #continue
               #    print("==== Пакет тип 2 ====")
               #    unpack_data = struct.unpack("<BIH9hH", data[:27])
               #    print ("Accelerometer x", unpack_data[6]/1000)
               #    print ("Accelerometer y", unpack_data[7]/1000)
               #    print ("Accelerometer z", unpack_data[8]/1000)
               #    print ("Gyroscope x", unpack_data[9]/1000)
               #    print ("Gyroscope y", unpack_data[10]/1000)
               #    print ("Gyroscope z", unpack_data[11]/1000)
               #    print ("Magnetometer x", unpack_data[3]/1000)
               #    print ("Magnetometer y", unpack_data[4]/1000)
               #    print ("Magnetometer z", unpack_data[5]/1000)
               #    print ("Number", unpack_data[2])
               #    print ("Time", unpack_data[1])

               #    print ('\n')

               #    for i in range(1,12):
               #        f2.write(str(unpack_data[i]))
               #        f2.write(";")
               #        f2.flush()
               #    f2.write('\n')

               #elif data[0] == 204: 
               #    #continue
               #    print("==== Пакет тип 3 ====")
               #    unpack_data = struct.unpack("<BIHh3f2IbH", data[:32])
               #    print ("Latitude", unpack_data[4])
               #    print ("Longitude", unpack_data[5])
               #    print ("Height", unpack_data[6])
               #    print ("Time, s", unpack_data[7]) 
               #    print ("Time, mks", unpack_data[8])
               #    print ("Fix", unpack_data[9])
               #    print ("Number", unpack_data[2])
               #    print ("Time", unpack_data[1])
               #    print ("Temperature DS", unpack_data[3])

               #    print ('\n')

               #    for i in range(1,10):
               #        f3.write(str(unpack_data[i]))
               #        f3.write(";")
               #        f3.flush()
               #    f3.write('\n')
                    
                else:
                    print('unknown flag ', data[0], data)
                    radio2.flush_rx()

            except Exception as e:
                print(e)
                
            f.write(record)
            f.flush()
        else:
            #print('got no data')
            pass

        #time.sleep(0.1)
