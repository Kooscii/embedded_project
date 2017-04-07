import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import serial

batch = 1


class Scope(object):
    def __init__(self, ax, maxt=5, dt=0.01):
        self.ax = ax
        self.dt = dt
        self.maxt = maxt
        self.tdata = []
        self.ydata = []
        self.line = Line2D(self.tdata, self.ydata)
        self.ax.add_line(self.line)
        self.ax.set_ylim(0, 255)
        self.ax.set_xlim(0, self.maxt)
        st.flushInput()

    def update(self, y):
        self.ydata.extend(y)
        if len(self.tdata)<self.maxt/self.dt:
            t = [i*self.dt for i in range(len(self.tdata), len(self.tdata)+batch)]
            self.tdata.extend(t)
        else:
            self.ydata = self.ydata[batch:]
        # ymax = max(self.ydata)
        # ymin = min(self.ydata)
        # margin = (ymax-ymin)/20.0
        # self.ax.set_ylim(ymin-margin, ymax+margin)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


def emitter(p=0.03):
    while True:
        data = st.read(batch)
        # f.write(str(data[0])+'\r\n')
        yield [i for i in data]


# change to your own serial name here
# st = serial.Serial('/dev/tty.usbmodem1423', 115200)
# st = serial.Serial('COM3', 115200)
# st = serial.Serial('/dev/ttyUSB0', 115200)
st = serial.Serial('/dev/ttyACM0', 115200)
# f = open('data.txt', 'w')

fig, ax = plt.subplots(figsize=(15,5))
scope = Scope(ax)

# pass a generator in "emitter" to produce data for the update func
ani = animation.FuncAnimation(fig, scope.update, emitter, interval=1,
                              blit=True)

plt.show()

st.close()
# f.close()