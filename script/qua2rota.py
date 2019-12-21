import numpy as np
from autolab_core import RigidTransform
 
orientation = {'y': 0.0182929, 'x': 0.00794676, 'z': 0.000830867, 'w': 0.999801}
position = {'y': -0.26022684372145516, 'x': 0.6453529828252734, 'z': 1.179122068068349}
    
rotation_quaternion = np.asarray([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
translation = np.asarray([position['x'], position['y'], position['z']])
 
T_qua2rota = RigidTransform(rotation_quaternion, translation)

print(T_qua2rota)