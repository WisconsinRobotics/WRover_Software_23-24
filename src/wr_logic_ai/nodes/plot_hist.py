import matplotlib.animation as animation
import matplotlib.pyplot as plt
from matplotlib import style

import pickle


alpha = 3
style.use('fivethirtyeight')
fig = plt.figure()
ax1 = fig.add_subplot(1, 1, 1)

def plot_histogram(x):
    try:
        dataf = open('sectors.data', 'rb')
        sectors = pickle.load(dataf)[10:50]
        angles = [i for i in range(30, 150, alpha)]
        ax1.clear()
        ax1.plot(angles, sectors)
    except EOFError:
        print('EOF ERROR')
    finally:
        dataf.close()


ani = animation.FuncAnimation(fig, plot_histogram, interval = 1)
plt.show()
