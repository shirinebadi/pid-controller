from cmath import sqrt
import numpy as np

rectangle = []
X1 = np.linspace(-2, 2 , 100)

for x in X1:
    rectangle.append([x,3.0])

Y2 = np.linspace(-3, 3 , 100)

for y in Y2:
    rectangle.append([2.0,y])

X3 = np.linspace(-2, 2 , 100)

for x in X3:
    rectangle.append([x,-3.0])

Y4 = np.linspace(-3, 3 , 100)

for y in Y4:
    rectangle.append([-2.0,y])

print(sqrt((2 - 1.6)**2 + ((int(-2.93) - int(-2.86))**2)))