import numpy as np

def perpendicular(v):
    """ Finds an arbitrary perpendicular vector to *v*."""
    # for two vectors (x, y, z) and (a, b, c) to be perpendicular,
    # the following equation has to be fulfilled
    #     0 = ax + by + cz

    # x = y = z = 0 is not an acceptable solution
    if v[0] == v[1] == v[2] == 0:
        raise ValueError('zero-vector')

    # If one dimension is zero, this can be solved by setting that to
    # non-zero and the others to zero. Example: (4, 2, 0) lies in the
    # x-y-Plane, so (0, 0, 1) is orthogonal to the plane.
    if v[0] == 0:
        return np.array([1, 0, 0])
    if v[1] == 0:
        return np.array([0, 1, 0])
    if v[2] == 0:
        return np.array([0, 0, 1])

    # arbitrarily set a = b = 1
    # then the equation simplifies to
    #     c = -(x + y)/z
    return np.array([1, 1, -1.0 * (v[0] + v[1]) / v[2]])


def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0)) # clip --> limit the values in an array to min and max


def normalize(a):
    return a/np.linalg.norm(a)


def get_orientation(COG, tip):
    v = np.sqrt((COG[0]-tip[0])**2 + (COG[1]-tip[1])**2 + (COG[2]-tip[2])**2) 
    return np.linalg.norm(v)

def get_StemLength(X, X_orient, h_tabletop, theta):
    u = np.array([0, 0, h_tabletop]) # to b changed to: np.array([0, h_tabletop, 0])
    v = X_orient
    # intersection of the 2 directions gives the root point
    # eq of 3d line passing by fruit COG and with direction X_orient
    #y_inter = u[1] - X[1] / v[1]  # since the intersection point lies in the plane of the tabletop 
    #x_inter = (v[0] * y_inter) + X[0]
    #z_inter = (v[2] * y_inter) + X[2]
    deltaZ = u[2] - X[2]
    Lstem = deltaZ / np.cos(theta)
    X_root_vect = v * Lstem
    z_inter = u[2]  # since the intersection point lies in the plane of the tabletop , to b changed to y_inter
    x_inter = X_root_vect[0] + X[0] # from: x_inter - X[0] = vector of unit  X_orient and length z_inter
    y_inter = X_root_vect[1] + X[1]
    inter = np.array([x_inter, y_inter, z_inter]) # root of stem
    #L = np.sqrt((inter[0]-X[0])**2 + (inter[1]-X[1])**2 + (inter[2]-X[2])**2) 
    return Lstem, inter



if __name__ == "__main__":    
    a = np.array([1,2, 5])
    print perpendicular(normalize(a))
