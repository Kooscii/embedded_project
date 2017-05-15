import matplotlib
matplotlib.use('TkAgg')

import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial
import time
from matplotlib.widgets import Button
import os.path

batch = 1


class Scope(object):
    def __init__(self, ax, maxt=3, dt=0.02):
        self.dt = dt
        self.maxt = maxt

        self.ax_heart = ax[0]
        self.ax_txt = ax[1]
        self.ax_step = ax[2]
        self.ax_rate = ax[3]

        self.data_time = np.arange(0, self.maxt, self.dt)
        # self.data_time_xyz_step = np.array([[i,i,i] for i in np.arange(0, self.maxt, self.dt)])

        # heart rate data
        # raw data
        self.data_raw_heart = np.array([128]*int(self.maxt/self.dt))
        self.line_raw_heart = Line2D(self.data_time, self.data_raw_heart, lw=0.5, color='orange', label='raw')
        self.ax_heart.add_line(self.line_raw_heart)
        # filtered data
        self.data_filt_heart = np.array([128]*int(self.maxt/self.dt))
        self.data_thrs_heart = np.array([128]*int(self.maxt/self.dt))
        self.line_filt_heart = Line2D(self.data_time, self.data_filt_heart, lw=1.5, color='C1', label='filtered')
        self.line_thrs_heart = Line2D(self.data_time, self.data_thrs_heart, lw=0.5, color='C0', label='threshold')
        self.ax_heart.add_line(self.line_filt_heart)
        self.ax_heart.add_line(self.line_thrs_heart)
        self.ax_heart.set_ylim(0, 255)
        self.ax_heart.set_xlim(0, self.maxt)
        self.ax_heart.get_yaxis().set_visible(False)
        self.ax_heart.legend(loc='upper left', prop={'size':7}, ncol=3)
        self.ax_heart.set_xlabel('Time (s)')

        # hr, sr data
        self.data_time60 = np.arange(0, 60, self.dt)
        self.data_bpm = np.array([0]*int(60/self.dt))
        self.data_spm = np.array([0]*int(60/self.dt))
        self.line_bpm = Line2D(self.data_time60, self.data_bpm, lw=1, color='C1', label='BPM', marker='.', markersize=0.01)
        self.line_spm = Line2D(self.data_time60, self.data_spm, lw=1, color='C0', label='SPM', marker='.', markersize=0.01)
        self.ax_rate.add_line(self.line_bpm)
        self.ax_rate.add_line(self.line_spm)
        self.ax_rate.set_ylim(0, 200)
        self.ax_rate.set_xlim(0, 60)
        self.ax_rate.legend(loc='upper left', prop={'size':7}, ncol=2)
        self.ax_rate.set_xlabel('Time (s)')


        # accelero rms data
        # raw data
        self.data_rms_step = np.array([1]*int(self.maxt/self.dt))
        self.data_xyz_step = np.array([[0,0,0]]*int(self.maxt/self.dt))
        self.line_rms_step = Line2D(self.data_time, self.data_rms_step, lw=0.5, color='orange', label='rms')
        self.line_x_step = Line2D(self.data_time, self.data_xyz_step[:,0], lw=0.3, color='lightpink') #, label='x')
        self.line_y_step = Line2D(self.data_time, self.data_xyz_step[:,1], lw=0.3, color='lightgreen') #, label='y')
        self.line_z_step = Line2D(self.data_time, self.data_xyz_step[:,2], lw=0.3, color='lightblue') #, label='z')
        self.ax_step.add_line(self.line_rms_step)
        self.ax_step.add_line(self.line_x_step)
        self.ax_step.add_line(self.line_y_step)
        self.ax_step.add_line(self.line_z_step)
        # filtered data
        self.data_filt_step = np.array([1]*int(self.maxt/self.dt))
        self.data_thrs_step = np.array([1]*int(self.maxt/self.dt))
        self.line_filt_step = Line2D(self.data_time, self.data_filt_step, lw=1.5, color='C1', label='filtered')
        self.line_thrs_step = Line2D(self.data_time, self.data_thrs_step, lw=0.5, color='C0', label='threshold')
        self.ax_step.add_line(self.line_filt_step)
        self.ax_step.add_line(self.line_thrs_step)
        self.ax_step.set_ylim(-1, 3)
        self.ax_step.set_xlim(0, self.maxt)
        self.ax_step.legend(loc='upper left', prop={'size':7}, ncol=7)
        self.ax_step.set_xlabel('Time (s)')

        # text
        self.ax_txt.get_xaxis().set_visible(False)
        self.ax_txt.get_yaxis().set_visible(False)
        self.ax_txt.set_ylim(0, 50)
        self.ax_txt.set_xlim(0, 150)
        ax_txt.text(20, 40, r'BPM:', fontsize=15)
        ax_txt.text(20, 15, r'SPM:', fontsize=15)
        ax_txt.text(60, 40, r'Total Beats:', fontsize=15)
        ax_txt.text(60, 15, r'Total Steps:', fontsize=15)
        self.data_beats = 0
        self.data_steps = 0
        self.txt_bpm = ax_txt.text(20, 30, str(0), fontsize=15)
        self.txt_spm = ax_txt.text(20, 5, str(0), fontsize=15)
        self.txt_beats = ax_txt.text(60, 30, str(self.data_beats), fontsize=15)
        self.txt_steps = ax_txt.text(60, 5, str(self.data_steps), fontsize=15)

        st.flushInput()

    def update(self, param):
        (
            data_raw_heart,
            data_thrs_heart,
            data_filt_heart,
            data_xyz_step,
            data_thrs_step,
            data_filt_step,
            data_bpm,
            data_spm,
            data_beats,
            data_steps
        ) = param

        self.data_raw_heart = np.append(self.data_raw_heart, data_raw_heart)
        self.data_raw_heart = self.data_raw_heart[len(data_raw_heart):]
        self.line_raw_heart.set_ydata(self.data_raw_heart)

        self.data_thrs_heart = np.append(self.data_thrs_heart, data_thrs_heart)
        self.data_thrs_heart = self.data_thrs_heart[len(data_thrs_heart):]
        self.line_thrs_heart.set_ydata(self.data_thrs_heart)

        self.data_filt_heart = np.append(self.data_filt_heart, data_filt_heart)
        self.data_filt_heart = self.data_filt_heart[len(data_filt_heart):]
        self.line_filt_heart.set_ydata(self.data_filt_heart)
        # rescale axis y
        ymax = max(self.data_filt_heart)
        ymin = min(self.data_filt_heart)
        if (ymax-ymin) < 20:
            mid = (ymax+ymin)/2
            ymax = mid + 10
            ymin = mid - 10
        margin = (ymax-ymin)/5.0
        self.ax_heart.set_ylim(ymin-margin, ymax+margin)

        self.data_xyz_step = np.vstack([self.data_xyz_step, data_xyz_step])
        self.data_xyz_step = self.data_xyz_step[len(data_xyz_step):]
        self.line_x_step.set_ydata(self.data_xyz_step[:,0])
        self.line_y_step.set_ydata(self.data_xyz_step[:,1])
        self.line_z_step.set_ydata(self.data_xyz_step[:,2])

        self.data_rms_step = np.append(self.data_rms_step, np.linalg.norm(data_xyz_step, axis=1))
        self.data_rms_step = self.data_rms_step[len(data_xyz_step):]
        self.line_rms_step.set_ydata(self.data_rms_step)

        self.data_thrs_step = np.append(self.data_thrs_step, data_thrs_step)
        self.data_thrs_step = self.data_thrs_step[len(data_thrs_step):]
        self.line_thrs_step.set_ydata(self.data_thrs_step)

        self.data_filt_step = np.append(self.data_filt_step, data_filt_step)
        self.data_filt_step = self.data_filt_step[len(data_filt_step):]
        self.line_filt_step.set_ydata(self.data_filt_step)


        self.txt_bpm.set_text(str(data_bpm[-1]))
        self.data_bpm = np.append(self.data_bpm, data_bpm)
        self.data_bpm = self.data_bpm[len(data_bpm):]
        self.line_bpm.set_ydata(self.data_bpm)


        self.txt_spm.set_text(str(data_spm[-1]))
        self.data_spm = np.append(self.data_spm, data_spm)
        self.data_spm = self.data_spm[len(data_spm):]
        self.line_spm.set_ydata(self.data_spm)


        self.txt_beats.set_text(str(data_beats))


        self.txt_steps.set_text(str(data_steps))  

        if log_flag[0]:
            self.txt_bpm.set_color('r')
            self.txt_spm.set_color('r')
            self.txt_beats.set_color('r')
            self.txt_steps.set_color('r')
        else:
            self.txt_bpm.set_color('k')
            self.txt_spm.set_color('k')
            self.txt_beats.set_color('k')
            self.txt_steps.set_color('k')

        # self.txt_beats.set_text(str(self.data_beats))
        # self.txt_steps.set_text(str(self.data_steps))

        return (self.line_raw_heart,
                self.line_thrs_heart,
                self.line_filt_heart,
                self.line_x_step,
                self.line_y_step,
                self.line_z_step,
                self.line_rms_step,
                self.line_thrs_step,
                self.line_filt_step,
                self.txt_bpm,
                self.txt_spm,
                self.txt_beats,
                self.txt_steps,
                self.line_bpm,
                self.line_spm
                )


def emitter(p=0.03):
    log_rate = 100
    log_list = []
    list_bpm = []
    list_spm = []
    data_beats = 0
    prev_beats = 0
    data_steps = 0
    prev_steps = 0
    while True:
        ## print(time())
        data_raw_heart = []
        data_thrs_heart = []
        data_filt_heart = []
        data_xyz_step = []
        data_thrs_step = []
        data_filt_step = []
        data_bpm = []
        data_spm = []

        while (st.inWaiting() < 14):
            pass

        len_inbuf = st.inWaiting()

        for i in range(int(len_inbuf/14.)):
            inbuf = st.readline()
            val = [i-11 for i in inbuf]

            data_raw_heart.append(val[1])
            data_thrs_heart.append(val[2])
            data_filt_heart.append(val[3])

            data_xyz_step.append([(i-128.)/32. for i in val[4:7]])
            data_thrs_step.append(((val[7]-128)/32.-1)*1.1+1)
            data_filt_step.append((val[8]-128)/32.)

            list_bpm.append(val[9])
            list_bpm = list_bpm[-25:]
            data_bpm.append(int(sum(list_bpm)/len(list_bpm)))

            list_spm.append(val[10])
            list_spm = list_spm[-25:]
            data_spm.append(int(sum(list_spm)/len(list_spm)))

            if prev_beats <= val[11]:
                data_beats += (val[11]) - data_beats%100
            else:
                data_beats += (val[11]) + 100 - data_beats%100
            prev_beats = val[11]

            if prev_steps <= val[12]:
                data_steps += (val[12]) - data_steps%100
            else:
                data_steps += (val[12]) + 100 - data_steps%100
            prev_steps = val[12]

            if log_flag[0]:
                f[0].write(inbuf)

        yield (
                data_raw_heart,
                data_thrs_heart,
                data_filt_heart,
                data_xyz_step,
                data_thrs_step,
                data_filt_step,
                data_bpm,
                data_spm,
                data_beats,
                data_steps
            )

def startLog(event, log_flag, f):
    if not log_flag[0]:
        f[0] = open('log-'+time.strftime("%m%d%H%M%S", time.localtime()), 'wb')
        log_flag[0] = True
    else:
        f[0].close()
        log_flag[0] = False


# change to your own serial name here
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

log_flag = [False]
f = [None]

fig = plt.figure(figsize=(10, 5))
ax_heart = fig.add_subplot(2, 2, 1)
ax_txt = fig.add_subplot(2, 2, 2)
ax_step = fig.add_subplot(2, 2, 3)
ax_xyz = fig.add_subplot(2, 2, 4)
fig.canvas.mpl_connect('key_press_event', lambda event: startLog(event, log_flag, f))

ax = [ax_heart, ax_txt, ax_step, ax_xyz]
scope = Scope(ax)

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=10, blit=True)

plt.tight_layout()
plt.show()

st.close()
try:
    f[0].close()
except:
    pass
