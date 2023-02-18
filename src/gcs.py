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


#radio1=RF24_CLASS(24, 1)
radio2=RF24_CLASS(22, 0)


def generate_logfile_name():
	now = datetime.datetime.utcnow().replace(microsecond=0)
	isostring = now.isoformat()  # string 2021-04-27T23:17:31
	isostring = isostring.replace("-", "")  # string 20210427T23:17:31
	isostring = isostring.replace(":", "")  # string 20210427T231731, òî ÷òî íàäî
	return "./log/knpn_Mandarinas" + isostring + ".bin"


if __name__ == '__main__':
	static_payload_size = None
	#static_payload_size = 16

	radio2.begin()

	radio2.openReadingPipe(1, b'\x9a\x78\x56\x34\x12')

	radio2.setCRCLength(RF24_CRC_8)
	radio2.setAddressWidth(5)
	radio2.channel = 36
	radio2.setDataRate(NRF24_DATARATE_1000_KBIT)
	radio2.enableAckPayload()
	radio2.enableDynamicAck()
	radio2.setAutoAck(True)


	if static_payload_size is not None:
		radio2.disableDynamicPayloads()
		radio2.payloadSize = static_payload_size
	else:
		radio2.enableDynamicPayloads()

	radio2.startListening()
	radio2.printDetails()

	#filename = 'monolit_malinaS.bin'
	filename = generate_logfile_name()
	f = open(filename, 'wb')





	while True:
		has_payload, pipe_number = radio2.available_pipe()
		#print(f'has_payload-{has_payload}, pipe_number={pipe_number}')

		if has_payload:
			payload_size = static_payload_size
			if payload_size is None:
				payload_size = radio2.getDynamicPayloadSize()

			data = radio2.read(payload_size)
			print('got data %s' % data)
			packet = data
			packet_size = len(packet)
			biter = struct.pack(">B", packet_size)
			record = biter + packet

			try:
                if data[0] == 255:
                    pass
                    print("==== Пакет МА тип 1 ====")
                    unpack_data = struct.unpack("<", data)
                    print ("Температура:", unpack_data[5])
                    print ("Давление", unpack_data[6])

                    print ("Напряжение на шине", unpack_data[8])
                    print ("Ток", unpack_data[7])

                    print ("Номер", unpack_data[1])

                    print ("Сопротивление", unpack_data[10])
                    print ("Соостояние", unpack_data[9])
            elif data[0] == 254:
                    pass
                    print("==== Пакет МА тип 2 ====")
                    unpack_data = struct.unpack("<", data)
                    print ("Акселерометр x", unpack_data[5])
                    print ("Акселерометр y", unpack_data[6])

                    print ("Напряжение на шине", unpack_data[8])
                    print ("Ток", unpack_data[7])

                    print ("Номер", unpack_data[1])

                    print ("Сопротивление", unpack_data[10])
                    print ("Соостояние", unpack_data[9])


			f.write(record)
			f.flush()
		else:
			#print('got no data')
			pass

		time.sleep(5.0)