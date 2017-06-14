import serial
import time
import numpy as np

ser = serial.Serial('COM5')
sensor_num = 4
#operations
opr_init = '020500\r\n'.encode('utf-8')
opr_get = '020202\r\n'.encode('utf-8')
opr_sensor_num = '020304\r\n'.encode('utf-8') #4

#initialization
def init():
    ser.write(opr_sensor_num)
    ser.write(opr_init)


def split_data(datas,sensor_num):
    #intitialize
    data = [0 for i in range (sensor_num)]

    for i in range(sensor_num):
        data[i] = datas[(2 + i*16) : (18 + i*16)]
    return data

def decompose_data(data):
    voltage_x = data[0:4]
    voltage_y = data[4:8]
    voltage_z = data[8:12]
    voltage_h = data[12:16]

    vol_x = int(voltage_x,16)
    vol_y = int(voltage_y,16)
    vol_z = int(voltage_z,16)
    vol_h = int(voltage_h,16)
    return [vol_x, vol_y, vol_z, vol_h]

def split_and_decompose_data(data,sensor_num):
    splited_data = split_data(data,sensor_num)
    vx = np.zeros(4)
    vy = np.zeros(4)
    vz = np.zeros(4)
    vh = np.zeros(4)
    for i in range (sensor_num):
        [vx[i],vy[i],vz[i],vh[i]] = decompose_data(splited_data[i])

    #change the data from [x0,x1,x2,...] to [x0,y0,z0,h0],[...
    v = np.zeros((sensor_num,4))
    for i in range (sensor_num):
        v[i] = np.append(np.append(vx[i],vy[i]),np.append(vz[i],vh[i]))

    return v






if  __name__ == '__main__':
    init()
    print ('通信用ポート番号:', ser.portstr)
    print ('使用センサー数:', sensor_num)

    ser.write(opr_get)

    while True:
        str0 = ser.readline()
        data0 = str0.decode('utf-8').strip()
        v = split_and_decompose_data(data0,sensor_num)

        print(["vx", "vy" ,"vz", "vh"])
        print(v)

        time.sleep(1.0)

    ser.close()
