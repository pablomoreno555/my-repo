import matplotlib.pyplot as plt
import numpy as np

y = [1.9,3.8,5.0,5.8,6.3,9.0,9.9,13.0,14.3,13.8]
x = np.linspace(1,10,10)
plt.xticks(np.linspace(0,12,13))
plt.bar(x,y, align='edge', width=0.35, color='green')

plt.show()
