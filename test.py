import math
import numpy as np

A = (24, 34)
B = (34, 34)
dx, dy = B[0] - A[0], B[1] - A[1]
theta_math = np.degrees(np.arctan2(dy, dx)) 
print(theta_math)

A = (24, 34)
B = (24, 56)
dx, dy = B[0] - A[0], B[1] - A[1]
theta_math = np.degrees(np.arctan2(dy, dx)) 
print(theta_math)