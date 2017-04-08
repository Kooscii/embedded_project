import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

batch = 1


class Scope(object):
    def __init__(self, ax, maxt=5, dt=0.01):
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
        self.xline = Line2D(self.tdata, self.xyzdata[:,0], lw=1, color = 'r', label='x')
        self.yline = Line2D(self.tdata, self.xyzdata[:,1], lw=1, color = 'g', label='y')
        self.zline = Line2D(self.tdata, self.xyzdata[:,2], lw=1, color = 'b', label='z')
        self.ax_xyz.add_line(self.xline)
        self.ax_xyz.add_line(self.yline)
        self.ax_xyz.add_line(self.zline)
        self.ax_xyz.set_ylim(-100, 100)
        self.ax_xyz.set_xlim(0, self.maxt)
        self.ax_xyz.legend(loc='upper left', prop={'size':7}, ncol=3)

        # accelero rms data
        self.rmsdata = np.array([1]*int(self.maxt/self.dt))
        self.rmsline = Line2D(self.tdata, self.rmsdata)
        self.ax_rms.add_line(self.rmsline)
        self.ax_rms.set_ylim(0, 3)
        self.ax_rms.set_xlim(0, self.maxt)

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
        raw_xyz = param[2]
        hr = param[3]
        sr = param[4]

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

        return self.hrline, self.rmsline, self.xline, self.yline, self.zline,


def emitter(p=0.03):
    while True:
        raw_hr = np.array([])
        raw_rms = np.array([])
        raw_xyz = np.array([]).reshape(0,3)
        hr = -1
        sr = -1

        l = st.inWaiting()
        if l >= 7: 
            for i in range(int(l/5)):
                rd = st.readline()
                raw_hr = np.append(raw_hr, rd[0])
                raw_xyz = np.vstack([raw_xyz, 128-np.array([i for i in rd[1:4]])])
                raw_rms = np.append(raw_rms, np.linalg.norm(raw_xyz[-1])/31)
                hr = rd[4]-11
                sr = rd[5]-11
            # f.write(str(data[0])+'\r\n')
            
        yield (raw_hr, raw_rms, raw_xyz, hr, sr)


# change to your own serial name here
# st = serial.Serial('/dev/tty.usbmodem1423', 115200)
# st = serial.Serial('COM3', 115200)
try:
    st = serial.Serial('/dev/ttyUSB0', 115200)
except:
    st = serial.Serial('/dev/ttyACM0', 115200)
# f = open('data.txt', 'w')

fig = plt.figure(figsize=(10,5))
ax_hr = fig.add_subplot(2, 2, 1)
ax_txt = fig.add_subplot(2, 2, 2)
ax_rms = fig.add_subplot(2, 2, 3)
ax_xyz = fig.add_subplot(2, 2, 4)

ax = [ax_hr, ax_txt, ax_rms, ax_xyz]
scope = Scope(ax)

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=1,
                              blit=True)

plt.show()

st.close()
# f.close()