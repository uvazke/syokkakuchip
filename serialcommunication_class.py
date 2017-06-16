import serial
import time
import numpy as np



class SerialCommunication:
    #initialization
    def __init__(self):
        #operations
        self.ser = serial.Serial('COM5')
        self.sensor_num = 4
        opr_init = '020500\r\n'.encode('utf-8')
        opr_get = '020202\r\n'.encode('utf-8')
        opr_sensor_num = '020304\r\n'.encode('utf-8') #4

        self.ser.write(opr_sensor_num)
        self.ser.write(opr_init)
        time.sleep(0.025 + self.sensor_num * 0.05)
        self.ser.write(opr_get)
        data_init = self.read_data()
        #base voltages
        self.data_init = self.split_and_decompose_data(data_init)

    def read_data(self):
        str0 = self.ser.readline()
        data0 = str0.decode('utf-8').strip()
        return data0

    def convert_data_to_voltage(self):
        data = self.split_and_decompose_data(self.read_data())
        revised_data = data - self.data_init
        v = revised_data * 3.3 / 1023
        return v

    #input the number of sensor you want to detect force as sensor_number(from 0 to self.sensor_num - 1)
    def detect_force(self,sensor_num,matrix):
        v = self.convert_data_to_voltage()
        v_xyz = v[sensor_num][0:3]
        v_h = v[sensor_num][3]
        delV = v_xyz - matrix[:,3]*v_h
        self.F = np.dot(matrix[:,0:3],v_xyz)
        print("F"+str(sensor_num))
        print(self.F)

    def split_data(self,datas):
        #intitialize
        data = [0 for i in range (self.sensor_num)]

        for i in range(self.sensor_num):
            data[i] = datas[(2 + i*16) : (18 + i*16)]
        return data

    def decompose_data(self,data):
        voltage_x = data[0:4]
        voltage_y = data[4:8]
        voltage_z = data[8:12]
        voltage_h = data[12:16]
        vol_x = int(voltage_x,16)
        vol_y = int(voltage_y,16)
        vol_z = int(voltage_z,16)
        vol_h = int(voltage_h,16)
        return [vol_x, vol_y, vol_z, vol_h]

    def split_and_decompose_data(self,data):
        splited_data = self.split_data(data)
        vx = np.zeros(4)
        vy = np.zeros(4)
        vz = np.zeros(4)
        vh = np.zeros(4)
        for i in range (self.sensor_num):
            [vx[i],vy[i],vz[i],vh[i]] = self.decompose_data(splited_data[i])

        #change the data from [x0,x1,x2,...] to [x0,y0,z0,h0],[...
        v = np.zeros((self.sensor_num,4))
        for i in range (self.sensor_num):
            v[i] = np.append(np.append(vx[i],vy[i]),np.append(vz[i],vh[i]))

        return v


if  __name__ == '__main__':
    save_prd = 3
    #2T-38
    matrix0 = np.array([[-2.835 ,-0.478, -1.271, 0.088],[-0.084, -3.414, -0.736, 0.688],[0.872, -0.019, 2.729, 0.53]])
    #2D-36
    matrix1 = np.array([[-1.484, 0.059, -0.382, -0.097],[-0.018, -1.389, -0.118, 0.377],[0.353, -0.037, 1.168, -0.848]])
    #1P-37
    matrix2 = np.array([[-2.079, 0.154, -0.514, -0.457],[0.007, -2.013, -0.513, 0.511],[-0.029, 0.057, 2.257, -0.317]])
    #2R-37
    matrix3 = np.array([[-3.679, 0.194, -0.974, -0.196],[-0.497, -4.767, -1.242, 0.845],[0.094, -0.305, 4.053, -1.573]])
    sercom = SerialCommunication()
    print ('通信用ポート番号:', sercom.ser.portstr)
    print ('使用センサー数:', sercom.sensor_num)
    k = 0
    F0 = []
    F1 = []
    F2 = []
    F3 = []
    while True:
        sercom.detect_force(0,matrix0)
        F0 = np.append(F0, sercom.F)
        sercom.detect_force(1,matrix1)
        F1 = np.append(F1, sercom.F)
        sercom.detect_force(2,matrix2)
        F2 = np.append(F2, sercom.F)
        sercom.detect_force(3,matrix3)
        F3 = np.append(F3, sercom.F)
        #time.sleep(1.0)
        k += 1

        if k % save_prd == 0:
            np.savetxt('../../programming/data/data1.csv', F0, delimiter=' ')
            np.savetxt('../../programming/data/data2.csv', F1, delimiter=' ')
            np.savetxt('../../programming/data/data3.csv', F2, delimiter=' ')
            np.savetxt('../../programming/data/data4.csv', F3, delimiter=' ')


    ser.close()
