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

    print (data)
    return [data]

def decompose_data(data):
    voltage_x = data[0:4]
    voltage_y = data[4:8]
    voltage_z = data[8:12]
    voltage_h = data[12:16]
    print(voltage_x)

    vol_x = int(voltage_x,16)
    vol_y = int(voltage_y,16)
    vol_z = int(voltage_z,16)
    vol_h = int(voltage_h,16)
    return [vol_x, vol_y, vol_z, vol_h]

def split_and_decompose_data(data,sensor_num):
    vx = np.zeros(sensor_num)
    vy = np.zeros(sensor_num)
    vz = np.zeros(sensor_num)
    vh = np.zeros(sensor_num)
    print(split_data(data,sensor_num)[0])
    for i in range (sensor_num):
        [vx[i],vy[i],vz[i],vh[i]] = decompose_data(split_data(data,sensor_num)[i])
    return [vx,vy,vz,vh]




if  __name__ == '__main__':
    init()
    print ('通信用ポート番号:', ser.portstr)
    print ('使用センサー数:', sensor_num)

    ser.write(opr_get)

    while True:
        str0 = ser.readline()
        data0 = str0.decode('utf-8').strip()
        [vx,vy,vz,vh] = split_and_decompose_data(data0,sensor_num)
        print(vx)
        print(vy)
        print(vz)
        print(vh)
        print(data0)
        time.sleep(1.0)

    ser.close()
