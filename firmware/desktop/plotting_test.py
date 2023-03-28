import matplotlib.pyplot as plt
import numpy as np
import time


x = np.linspace(0,200,200, dtype = int)
print(x.size)
y = np.zeros(x.size, dtype = float)

plt.ion()

fig = plt.figure()
ax = fig.add_subplot(111)
line1, = ax.plot(x,y,'b')
ax.set_ylim([200,-200])

count = 50.0
index = 0

while True:
    # time.sleep(0.1)
    count-= 0.2
    y[index] = count
    index +=1
    if(index > 199):
        index= 0
    # print(y)
    line1.set_ydata(y)
    fig.canvas.draw()
    fig.canvas.flush_events()

    