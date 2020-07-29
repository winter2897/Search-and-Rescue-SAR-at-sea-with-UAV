import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt
from pylab import *

import pandas as pd
df = pd.read_csv("GPS.csv", names=['16.49991017', '112', '132.6557465'])
a = df[['16.49991017']].values
A = []
for i in range(0, len(a)):
    z = float(df[['16.49991017']].values[i])
    A.append(z)

B = []
for i in range(0, len(a)):
    z = float(df[['112']].values[i])
    B.append(z)

# C = []
# for i in range(0, len(a)):
#     z = float(df[['16.49991017']].values[i])
#     C.append(z)

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure()
ax = fig.gca(projection='3d')

x=A
y=B
# z=C  # creating the z array with the same length as th
ax.plot(y, x)  # adding z as an argument for the plot
ax.legend()
plt.show()