# coding: UTF-8
import serial
import time
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns


class SerialCommunication:
    #initialization
    def __init__(self):
        #operations
        self.raw_data = np.array([[""]])
        self.ser = serial.Serial('COM5')
        self.sensor_num = 4
        opr_init = '020500\r\n'.encode('utf-8')
        opr_get = '020202\r\n'.encode('utf-8')
        opr_sensor_num = '020304\r\n'.encode('utf-8') #4
        opr_sampling_rate = '020100\r\n' #4ms
        self.ser.write(opr_sensor_num)
        self.ser.write(opr_init)
        time.sleep(0.025 + self.sensor_num * 0.05)
        self.start_time = time.time()


        self.ser.write(opr_get)
        data_init = self.read_data()
        #base voltages
        self.data_init = self.split_and_decompose_data(data_init)

    def read_data(self):
        str0 = self.ser.readline()
        data0 = str0.decode('utf-8').strip()
        return data0

    def convert_data_to_voltage(self):
        raw_data = self.read_data()
        #save raw_data
        self.raw_data = np.vstack((self.raw_data, raw_data))
        data = self.split_and_decompose_data(raw_data)
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
    save_prd = 1000
    #2T-38
    matrix0 = np.array([[-2.835 ,-0.478, -1.271, 0.088],[-0.084, -3.414, -0.736, 0.688],[0.872, -0.019, 2.729, 0.53]])
    #2D-36
    matrix1 = np.array([[-1.484, 0.059, -0.382, -0.097],[-0.018, -1.389, -0.118, 0.377],[0.353, -0.037, 1.168, -0.848]])
    #1P-37
    #matrix2 = np.array([[-2.079, 0.154, -0.514, -0.457],[0.007, -2.013, -0.513, 0.511],[-0.029, 0.057, 2.257, -0.317]])
    matrix2 = np.array([[-2.37, 0.04, -0.548, -0.144],[0.082, -2.509, -0.879, 0.849],[-0.527, 0.134, 2.269, 0.061]])
    #2R-37
    matrix3 = np.array([[-3.679, 0.194, -0.974, -0.196],[-0.497, -4.767, -1.242, 0.845],[0.094, -0.305, 4.053, -1.573]])
    sercom = SerialCommunication()
    print ('通信用ポート番号:', sercom.ser.portstr)
    print ('使用センサー数:', sercom.sensor_num)
    k = 0
    #row 0 is just the initializer, so you should use the data without row 0
    F0 = np.array([[0.0,0.0,0.0]])
    F1 = np.array([[0.0,0.0,0.0]])
    F2 = np.array([[0.0,0.0,0.0]])
    F3 = np.array([[0.0,0.0,0.0]])
    iter_num = 0
    while iter_num <= save_prd :
        iter_num += 1
        sercom.detect_force(0,matrix0)
        F0 = np.r_[F0, sercom.F[np.newaxis, :]]
        sercom.detect_force(1,matrix1)
        F1 = np.r_[F1, sercom.F[np.newaxis, :]]
        sercom.detect_force(2,matrix2)
        F2 = np.r_[F2, sercom.F[np.newaxis, :]]
        sercom.detect_force(3,matrix3)
        F3 = np.r_[F3, sercom.F[np.newaxis, :]]
        elapsed_time = time.time() - sercom.start_time
        print(elapsed_time)

        k += 1
        if k % save_prd == 0:
            #save_data as csv
            np.savetxt('../../programming/data/data1.csv', F0, delimiter=' ')
            np.savetxt('../../programming/data/data2.csv', F1, delimiter=' ')
            np.savetxt('../../programming/data/data3.csv', F2, delimiter=' ')
            np.savetxt('../../programming/data/data4.csv', F3, delimiter=' ')

            data1=np.loadtxt('../../programming/data/data1.csv')
            data2=np.loadtxt('../../programming/data/data2.csv')
            data3=np.loadtxt('../../programming/data/data3.csv')
            data4=np.loadtxt('../../programming/data/data4.csv')
            #new window
            fig = plt.figure()
            sensor1 = fig.add_subplot(2,2,1)
            sensor2 = fig.add_subplot(2,2,2)
            sensor3 = fig.add_subplot(2,2,3)
            sensor4 = fig.add_subplot(2,2,4)


            #show data
            x = np.linspace(0, 0.004 * save_prd, save_prd)

            label = ["Fx","Fy","Fz"]
            for col in range(3):
                sensor1.plot(x, data1[1:,col],label = label[col])
                sensor2.plot(x, data2[1:,col],label = label[col])
                sensor3.plot(x, data3[1:,col],label = label[col])
                sensor4.plot(x, data4[1:,col],label = label[col])
            sensor1.set_xlabel('elapsed time(s)')
            sensor1.set_ylabel('Force (N)')
            sensor1.set_title('sensor1')

            sensor2.set_xlabel('elapsed time(s)')
            sensor2.set_ylabel('Force (N)')
            sensor2.set_title('sensor2')

            sensor3.set_xlabel('elapsed time(s)')
            sensor3.set_ylabel('Force (N)')
            sensor3.set_title('sensor3')

            sensor4.set_xlabel('elapsed time(s)')
            sensor4.set_ylabel('Force (N)')
            sensor4.set_title('sensor4')

            sensor1.legend(loc="upper left",frameon = True)
            sensor2.legend(loc="upper left",frameon = True)
            sensor3.legend(loc="upper left",frameon = True)
            sensor4.legend(loc="upper left",frameon = True)
            ym = -3.0
            yp = + 3.0
            sensor1.set_ylim(ym,yp)
            sensor2.set_ylim(ym,yp)
            sensor3.set_ylim(ym,yp)
            sensor4.set_ylim(ym,yp)


            plt.tight_layout()
            plt.savefig('../../programming/data/image.png')
            np.savetxt('../../programming/data/raw_data.csv', sercom.raw_data, delimiter=' ')
            plt.show()


    sercom.ser.close()
