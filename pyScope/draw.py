import numpy as np
import matplotlib.pyplot as plt

f = open('log-0514153103','rb')
g = open('log-0514162019','rb')

index = []
rawHR = []
threshHR = []
filtHR_val = []
rawStep_x = []
rawStep_y = []
rawStep_z = []
threshStep = []
filtStep_rms = []
outHR = []
outSR = []
outHC = []
outSC = []

for line in f:
    if line[0]-11 > 0:
        index.append(line[0]-11)
        rawHR.append(line[1])
        threshHR.append(line[2])
        filtHR_val.append(line[3])
        rawStep_x.append(line[4]-128)
        rawStep_y.append(line[5]-128)
        rawStep_z.append(line[6]-128)
        threshStep.append(line[7]-128)
        filtStep_rms.append(line[8]-128)
        outHR.append(line[9]-11)
        outSR.append(line[10]-11)
        outHC.append(line[11]-11)
        outSC.append(line[12]-11)

for line in g:
    if line[0]-11 > 0:
        index.append(line[0]-11)
        rawHR.append(line[1])
        threshHR.append(line[2])
        filtHR_val.append(line[3])
        rawStep_x.append(line[4]-128)
        rawStep_y.append(line[5]-128)
        rawStep_z.append(line[6]-128)
        threshStep.append(line[7]-128)
        filtStep_rms.append(line[8]-128)
        outHR.append(line[9]-11)
        outSR.append(line[10]-11)
        outHC.append(line[11]-11)
        outSC.append(line[12]-11)

f.close()
g.close()

rawStep_rms = list(map(lambda x, y, z: (x**2+y**2+z**2)**(1/2), rawStep_x, rawStep_y, rawStep_z))
t = np.array(list(range(len(index))))/50

# plt.figure(figsize=(13, 8),sharex=True)
f, ax = plt.subplots(3, figsize=(13, 8),sharex=True)

ax[0].plot(t, rawHR, lw=0.5, c='orange', label='raw')
ax[0].plot(t, threshHR, lw=0.5, c='C0', label='threshold')
ax[0].plot(t, filtHR_val, lw=1, c='C1', label='filtered')

ax[1].plot(t, rawStep_rms, lw=0.5, c='orange', label='raw')
ax[1].plot(t, threshStep, lw=0.5, c='C0', label='threshold')
ax[1].plot(t, filtStep_rms, lw=1, c='C1', label='filtered')

ax[2].plot(t, outSR, label='SPM')
ax[2].plot(t, outHR, label='BPM')
ax[2].set_xlabel('Time (s)')

plt.tight_layout()
plt.show()

# f = open('expdata.csv', 'w')

# for i in range(len(index)):
#     f.write('%d, %d, %d, %d, %d, %d, \n'%(rawHR[i], rawStep_rms[i], outHR[i], outSR[i], threshHR[i], threshStep[i]))

# f.close()
