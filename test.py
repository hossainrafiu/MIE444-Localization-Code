import matplotlib.pyplot as plt
import numpy as np

points = [[5, 0], [0, 5], [-5, 0], [0, -5]]

plt.figure()
plt.ion()
while True:
    for _, point in enumerate(points):
        x, y = point + np.random.randn(2) * 0.5
        plt.xlim(-10, 10)
        plt.ylim(-10, 10)
        plt.plot(x, y, 'ro')
    plt.show()
    plt.pause(0.1)
    plt.clf()