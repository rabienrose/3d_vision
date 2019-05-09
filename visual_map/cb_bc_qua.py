import numpy as np
from scipy.spatial.transform import Rotation as R
x = np.array([[0.99994824,  0.01010297,  0.00120024, -0.16778745],[0.0021942,  -0.09895494, -0.9950895,  -0.31369233],[-0.00993459,  0.99504063, -0.09897199, -0.05316503], [0.0, 0.0, 0.0, 1.0]])
y=np.linalg.inv(x)
r=R.from_dcm(y[0:3,0:3])
print(r.as_quat()) #x,y,z,w
print(x[0:4,3])

