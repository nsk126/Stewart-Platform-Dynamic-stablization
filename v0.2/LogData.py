import serial
import time
import csv

ser = serial.Serial('COM11',baudrate=115200,timeout=0.1)

b_roll = []
b_pitch = []
p_roll = []
p_pitch = []
err_r = []
err_p = []
uroll = []
upitch = []
time = []

try:
    while 1:
        Data = ser.readline()
        if Data != b'':
            Data = Data.strip()
            Data = Data.decode("utf-8")
            Data = Data.split(",")
            Data = [float(i) for i in Data]
            if len(Data) == 9:
                b_roll.append(Data[0])
                b_pitch.append(Data[1])
                p_roll.append(Data[2])
                p_pitch.append(Data[3])
                err_r.append(Data[4])
                err_p.append(Data[5])
                uroll.append(Data[6])
                upitch.append(Data[7])
                time.append(Data[8])

except KeyboardInterrupt:
    Data = [b_roll,b_pitch,p_roll,p_pitch,err_r,err_p,uroll,upitch,time]
    with open('Stewart_Data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(Data)
        exit()