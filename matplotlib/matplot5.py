import numpy as np
import matplotlib.pyplot as plt
x = np.linspace(-1.0,1.0,10000)
z = np.random.rand(10000)
y = x**2+z
plt.plot(x,y, linestyle='', marker=',', markerfacecolor='blue')
plt.show()
