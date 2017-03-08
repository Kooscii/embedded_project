import numpy as np

hr = np.load('data1')
f = open('data.h','w')
f.write('const uint16_t hr_data[] = {')

for i in range (0,100000,10):
    f.write(str(hr[i])+',')
    if (i+10)%200==0:
        f.write('\r\n\t\t')

f.write('};')
f.close()
