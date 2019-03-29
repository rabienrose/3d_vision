import numpy as np
from scipy.spatial.transform import Rotation as R
x = np.array([[-0.99999518, -0.00310315,  0.00007742,  0.05310734],[-0.00124823,  0.42483127,  0.90527169,  0.00929943],[-0.00284208,  0.90526723, -0.4248331,  -0.01557671], [0.0, 0.0, 0.0, 1.0]])
y=np.linalg.inv(x)
r=R.from_dcm(y[0:3,0:3])
r.as_quat()
