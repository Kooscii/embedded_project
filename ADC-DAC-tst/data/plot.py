import serial
import numpy as np
from matplotlib import pyplot as plt

div=10

st = serial.Serial('/dev/tty.usbmodem1423', 115200)
save_idx = 0;

plt.ion()
ydata = np.array([])
record = np.array([])

data = st.read(int(20000/div))
record = np.append(record.astype(int), [i for i in data])
ydata = record


fig = plt.figure(figsize=(20,5))
line, = plt.plot(ydata)
plt.ylim([0,255])
plt.xlim([-int(100/div),int(20100/div)])

while True: 
	data = st.read(int(1000/div))
	record = np.append(record, [i for i in data])
	ydata = record[-int(20000/div):]
	ymin = float(min(ydata))
	ymax = float(max(ydata))
	margin = (ymax-ymin)/20
	plt.ylim([ymin-margin,ymax+margin])
	line.set_xdata(np.arange(len(ydata)))
	line.set_ydata(ydata)
	plt.pause(0.001)
	# save every 100000 samples
	if (record.size>=int(120000/div)):
		f = open('data'+str(save_idx),'wb')
		np.save(f, record[0:int(100000/div)])
		record = record[-int(20000/div):]
		save_idx += 1
		f.close()
