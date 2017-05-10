import matplotlib
matplotlib.use('TkAgg')

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
from time import time

batch = 1


class Scope(object):
    def __init__(self, ax, maxt=5, dt=0.02):
        self.dt = dt
        self.maxt = maxt

        self.ax_hr = ax[0]
        self.ax_txt = ax[1]
        self.ax_rms = ax[2]
        self.ax_xyz = ax[3]

        self.tdata = np.arange(0, self.maxt, self.dt)
        # self.txyzdata = np.array([[i,i,i] for i in np.arange(0, self.maxt, self.dt)])

        # heart rate data
        self.hrdata = np.array([128]*int(self.maxt/self.dt))
        self.hrline = Line2D(self.tdata, self.hrdata)
        self.ax_hr.add_line(self.hrline)
        self.ax_hr.set_ylim(0, 255)
        self.ax_hr.set_xlim(0, self.maxt)
        self.ax_hr.get_yaxis().set_visible(False)

        # accelero xyz data
        self.xyzdata = np.array([[0,0,0]]*int(self.maxt/self.dt))
        self.xline = Line2D(self.tdata, self.xyzdata[:,0], lw=1, color='r', label='x')
        self.yline = Line2D(self.tdata, self.xyzdata[:,1], lw=1, color='g', label='y')
        self.zline = Line2D(self.tdata, self.xyzdata[:,2], lw=1, color='b', label='z')
        self.ax_xyz.add_line(self.xline)
        self.ax_xyz.add_line(self.yline)
        self.ax_xyz.add_line(self.zline)
        self.ax_xyz.set_ylim(-100, 100)
        self.ax_xyz.set_xlim(0, self.maxt)
        self.ax_xyz.legend(loc='upper left', prop={'size':7}, ncol=3)

        # accelero rms data
        self.rmsdata = np.array([1]*int(self.maxt/self.dt))
        self.flitdata = np.array([1]*int(self.maxt/self.dt))
        self.rmsline = Line2D(self.tdata, self.rmsdata, lw=1, color='C0', label='raw')
        self.flitline = Line2D(self.tdata, self.flitdata, color='C1', label='filtered')
        self.ax_rms.add_line(self.rmsline)
        self.ax_rms.add_line(self.flitline)
        self.ax_rms.set_ylim(0, 2.5)
        self.ax_rms.set_xlim(0, self.maxt)
        self.ax_rms.legend(loc='upper left', prop={'size':7}, ncol=1)

        # text
        self.ax_txt.get_xaxis().set_visible(False)
        self.ax_txt.get_yaxis().set_visible(False)
        self.ax_txt.set_ylim(0, 50)
        self.ax_txt.set_xlim(0, 150)
        ax_txt.text(10, 40, r'BPM:', fontsize=15)
        ax_txt.text(10, 20, r'SPM:', fontsize=15)
        self.hr_txt = ax_txt.text(10, 30, str(0), fontsize=15)
        self.sr_txt = ax_txt.text(10, 10, str(0), fontsize=15)

        st.flushInput()

    def update(self, param):
        raw_hr = param[0]
        raw_rms = param[1]
        raw_flit = param[2]
        raw_xyz = param[3]
        hr = param[4]
        sr = param[5]

        if raw_hr.size:
            self.hrdata = np.append(self.hrdata, raw_hr)
            self.hrdata = self.hrdata[len(raw_hr):]
            ymax = max(self.hrdata)
            ymin = min(self.hrdata)
            margin = (ymax-ymin)/20.0
            self.ax_hr.set_ylim(ymin-margin, ymax+margin)
            self.hrline.set_ydata(self.hrdata)

        if raw_rms.size:
            self.rmsdata = np.append(self.rmsdata, raw_rms)
            self.rmsdata = self.rmsdata[len(raw_rms):]
            self.rmsline.set_ydata(self.rmsdata)

        if raw_flit.size:
            self.flitdata = np.append(self.flitdata, raw_flit)
            self.flitdata = self.flitdata[len(raw_flit):]
            self.flitline.set_ydata(self.flitdata)

        if raw_xyz.size:
            self.xyzdata = np.vstack([self.xyzdata, raw_xyz])
            self.xyzdata = self.xyzdata[len(raw_xyz):]
            self.xline.set_ydata(self.xyzdata[:,0])
            self.yline.set_ydata(self.xyzdata[:,1])
            self.zline.set_ydata(self.xyzdata[:,2])

        if hr>=0:
            self.hr_txt.set_text(str(hr))

        if sr>=0:
            self.sr_txt.set_text(str(sr))

        return self.hrline, self.rmsline, self.flitline, self.xline, self.yline, self.zline, self.hr_txt, self.sr_txt


def emitter(p=0.03):
    rate_upd = 0
    while True:
        ## print(time())
        raw_hr = np.array([])
        raw_rms = np.array([])
        raw_flit = np.array([])
        raw_xyz = np.array([]).reshape(0, 3)
        hr = -1
        sr = -1
        hrlist = []
        srlist = []

        l = st.inWaiting()
        if l >= 8:
            for i in range(int(l/8.)):
                # print('before', st.inWaiting())
                rd = st.readline()
                # print([i for i in rd])
                # print('after', st.inWaiting())
                raw_hr = np.append(raw_hr, rd[0])
                raw_xyz = np.vstack([raw_xyz, 128-np.array([i for i in rd[1:4]])])
                raw_rms = np.append(raw_rms, np.linalg.norm(raw_xyz[-1])/31.)
                raw_flit = np.append(raw_flit, (rd[4]-128)/31.)


                g.write(str(raw_hr[-1])+','+str(raw_xyz[-1,0])+','+str(raw_xyz[-1,1])+','+str(raw_xyz[-1,2])+'\n')

                if rate_upd%50 == 0:
                    hr = rd[5]-1 if hr>10 else rd[5]
                    sr = rd[6]-1 if sr>10 else rd[6]
                    hrlist.append(hr)
                    srlist.append(sr)
                    if rate_upd%500 == 0:
                        f.write(str(sum(hrlist)/10)+','+str(sum(srlist)/10)+',\n')
                rate_upd = (rate_upd + 1) % 500
        else:
            # print('no data')
            pass
            
        # print()
            
        yield (raw_hr, raw_rms, raw_flit, raw_xyz, hr, sr)


# change to your own serial name here
# st = serial.Serial('/dev/tty.usbmodem1423', 115200)
# st = serial.Serial('COM3', 115200)
while True:
    try:
        st = serial.Serial('/dev/ttyUSB0', 115200)
        break
    except:
        pass

    try:
        st = serial.Serial('/dev/ttyACM0', 115200)
        break
    except:
        pass

    try:
        st = serial.Serial('/dev/tty.SLAB_USBtoUART', 115200)
        break
    except:
        pass

    try:
        st = serial.Serial('/dev/tty.usbmodem1423', 115200)
        break
    except:
        pass

    try:
        st = serial.Serial('/dev/tty.usbmodem1413', 115200)
        break
    except:
        pass

    try:
        st = serial.Serial('COM3', 115200)
        break
    except:
        break

f = open('rate_data.csv', 'w')
g = open('raw_data.csv', 'w')

fig = plt.figure(figsize=(10, 5))
ax_hr = fig.add_subplot(2, 2, 1)
ax_txt = fig.add_subplot(2, 2, 2)
ax_rms = fig.add_subplot(2, 2, 3)
ax_xyz = fig.add_subplot(2, 2, 4)

ax = [ax_hr, ax_txt, ax_rms, ax_xyz]
scope = Scope(ax)

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=10, blit=True)

plt.show()

st.close()
try:
    f.close()
    g.close()
except:
    pass
