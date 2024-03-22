import serial.tools.list_ports
import time
import csv

ports = serial.tools.list_ports.comports()
serialInst = serial.Serial()

portsList = []

for onePort in ports:
    portsList.append(str(onePort))
    print(str(onePort))

val = input("Select Port: COM")

for x in range(0, len(portsList)):
    if portsList[x].startswith("COM" + str(val)):
        portVar = "COM" + str(val)
        print(portVar)

serialInst.baudrate = 9600
serialInst.port = portVar
serialInst.open()
rows1 = [['Nikhil', 'COE', '2', '9.0']]
rows2 = [['Nikhil', 'COE', '2', '9.0']]

def csv_Write(rows):
    filename = "O2_DataLog_Caribate.csv"

    with open(filename, 'a') as csvfile:
        # creating a csv writer object
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(rows)
        time.sleep(5)
index = 0
while True:
    if serialInst.in_waiting:
        packet = serialInst.readline()
        raw_data = packet.decode('utf').rstrip('\n')
        # time,CO2_1,val_1,CO2_2,val_2
        csv_Write([str(index),str(raw_data[8:13]), str(raw_data[22:27])])
        index+=1
        print([str(index),str(raw_data[8:13]), str(raw_data[22:27])])
