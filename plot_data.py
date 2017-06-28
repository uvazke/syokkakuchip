import time
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import scipy.fftpack as scp

# 時系列のサンプルデータ作成
N = 1000                         # データ数
dt = 0.004                       # サンプリング間隔
f = 1/dt                          # サンプリング周波数
t = np.linspace(1, N, N)*dt-dt

save_prd = 1000
data1=np.loadtxt('../../programming/data/data1.csv')
data2=np.loadtxt('../../programming/data/data2.csv')
data3=np.loadtxt('../../programming/data/data3.csv')
data4=np.loadtxt('../../programming/data/data4.csv')
#new window
fig = plt.figure(1)
sensor1 = fig.add_subplot(2,2,1)
sensor2 = fig.add_subplot(2,2,2)
sensor3 = fig.add_subplot(2,2,3)
sensor4 = fig.add_subplot(2,2,4)

fig2 = plt.figure(2)
Fs1 = fig2.add_subplot(2,2,1)
Fs2 = fig2.add_subplot(2,2,2)
Fs3 = fig2.add_subplot(2,2,3)
Fs4 = fig2.add_subplot(2,2,4)

#show data
x = np.linspace(0, 0.004 * save_prd, save_prd)
n = np.linspace(1, N, N)
label = ["Fx","Fy","Fz"]

yf1 = scp.fft(data1[1:])
yf2 = scp.fft(data2[1:])
yf3 = scp.fft(data3[1:])
yf4 = scp.fft(data4[1:])
freqList = scp.fftfreq(N, d=1.0/ f)

for col in range(3):
    sensor1.plot(x, data1[1:,col],label = label[col])
    sensor2.plot(x, data2[1:,col],label = label[col])
    sensor3.plot(x, data3[1:,col],label = label[col])
    sensor4.plot(x, data4[1:,col],label = label[col])
    Fs1.plot(n, np.real(yf1[0:,col]),label = label[col])
    Fs2.plot(n, np.real(yf2[0:,col]),label = label[col])
    Fs3.plot(n, np.real(yf3[0:,col]),label = label[col])
    Fs4.plot(n, np.real(yf4[0:,col]),label = label[col])






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

ym = -6.0
yp = +6.0
sensor1.set_ylim(ym,yp)
sensor2.set_ylim(ym,yp)
sensor3.set_ylim(ym,yp)
sensor4.set_ylim(ym,yp)
sensor1.legend(loc="upper left",frameon = True)
sensor2.legend(loc="upper left",frameon = True)
sensor3.legend(loc="upper left",frameon = True)
sensor4.legend(loc="upper left",frameon = True)



plt.tight_layout()
plt.savefig('../../programming/data/image.png')
plt.show()
