import numpy as np

# changed order from 'w, x, y, z' to 'x, y, z, w' because isaac quaternion are (qx, qy, qz, real)
def rotation_matrix_quaterion(q): 
    x, y, z, w = q             
    return np.array([
        [1 - 2*y**2 - 2*z**2, 2*x*y - 2*w*z],
        [2*x*y + 2*w*z, 1 - 2*x**2 - 2*z**2]
    ])

def setup(db):     # used to be db: og.Database for all 3    
    db.internal_state.position = np.zeros(2)
    db.internal_state.velocity = np.zeros(2)
    db.internal_state.prev_velocity = np.zeros(2)
    
    pass

def compute(db):
    # inputs
    acc_vec = np.array([db.inputs.linear_acceleration_v[0], db.inputs.linear_acceleration_v[1]])
    dt = db.inputs.dt
    q = db.inputs.q

    # computations
    R = rotation_matrix_quaterion(q)

    # For velocity output
    db.internal_state.velocity += acc_vec * dt # velocity in local coordinates (x', y') to filter with other local sensors like encoders
    vel_global = np.dot(R, db.internal_state.velocity) # velocity in global coordinates (x, y) to filter with global sensors like lidar 
    db.outputs.linear_velocity = [vel_global[0], vel_global[1], 0]

    # For translation output
    # delta_vel = acc_vec * dt
    # translation = db.internal_state.velocity * dt - delta_vel * dt / 2.0
    translation = (db.internal_state.velocity + db.internal_state.prev_velocity) / 2.0 * dt
    global_translation = np.dot(R, translation)
    # db.internal_state.position += global_translation # translation in global coordinates
    db.internal_state.position += global_translation

    db.outputs.translation = [db.internal_state.position[0], db.internal_state.position[1], 0]

    return True


def cleanup(db):
    #db.internal_state.position = np.zeros(2) # pay attention
    #db.internal_state.velocity = np.zeros(2)
    pass