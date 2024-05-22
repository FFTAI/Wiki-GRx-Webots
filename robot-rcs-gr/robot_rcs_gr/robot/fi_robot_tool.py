import numpy
import torch


def radian_tmatrix(alpha, a, d, theta):
    T = numpy.matrix([[numpy.cos(theta), -numpy.sin(theta) * numpy.cos(alpha), numpy.sin(theta) * numpy.sin(alpha),
                       a * numpy.cos(theta)],
                      [numpy.sin(theta), numpy.cos(theta) * numpy.cos(alpha), -numpy.cos(theta) * numpy.sin(alpha),
                       a * numpy.sin(theta)],
                      [0, numpy.sin(alpha), numpy.cos(alpha), d],
                      [0, 0, 0, 1]])

    return T


def radian_rpy_to_tmatrix(rpy):
    R = numpy.matrix(
        [[numpy.cos(rpy[2]) * numpy.cos(rpy[1]),
          numpy.cos(rpy[2]) * numpy.sin(rpy[1]) * numpy.sin(rpy[0]) - numpy.sin(rpy[2]) * numpy.cos(rpy[0]),
          numpy.cos(rpy[2]) * numpy.sin(rpy[1]) * numpy.cos(rpy[0]) + numpy.sin(rpy[2]) * numpy.sin(rpy[0])],
         [numpy.sin(rpy[2]) * numpy.cos(rpy[1]),
          numpy.sin(rpy[2]) * numpy.sin(rpy[1]) * numpy.sin(rpy[0]) + numpy.cos(rpy[2]) * numpy.cos(rpy[0]),
          numpy.sin(rpy[2]) * numpy.sin(rpy[1]) * numpy.cos(rpy[0]) - numpy.cos(rpy[2]) * numpy.sin(rpy[0])],
         [-numpy.sin(rpy[1]),
          numpy.cos(rpy[1]) * numpy.sin(rpy[0]),
          numpy.cos(rpy[1]) * numpy.cos(rpy[0])]])

    return R


# 欧拉角按照XYZ顺序转换为四元数
def radian_rpy_to_quat(rpy):
    r = rpy[0]
    p = rpy[1]
    y = rpy[2]

    cy = numpy.cos(y * 0.5)
    sy = numpy.sin(y * 0.5)
    cr = numpy.cos(r * 0.5)
    sr = numpy.sin(r * 0.5)
    cp = numpy.cos(p * 0.5)
    sp = numpy.sin(p * 0.5)

    qx = cy * sr * cp + sy * cr * sp
    qy = cy * cr * sp - sy * sr * cp
    qz = sy * cr * cp + cy * sr * sp
    qw = cy * cr * cp - sy * sr * sp

    quat = numpy.array([qx, qy, qz, qw])

    return quat


# 欧拉角按照ZYX顺序转换为四元数
def radian_ypr_to_quat(rpy):
    r = rpy[0]
    p = rpy[1]
    y = rpy[2]

    cy = numpy.cos(y * 0.5)
    sy = numpy.sin(y * 0.5)
    cr = numpy.cos(r * 0.5)
    sr = numpy.sin(r * 0.5)
    cp = numpy.cos(p * 0.5)
    sp = numpy.sin(p * 0.5)

    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp
    qw = cy * cr * cp + sy * sr * sp

    quat = numpy.array([qx, qy, qz, qw])

    return quat


# 欧拉角按照ZYX顺序转换为四元数，并确保连贯性
def radian_ypr_to_quat_continuous(rpy, previous_quat):
    r = rpy[0]
    p = rpy[1]
    y = rpy[2]

    cy = numpy.cos(y * 0.5)
    sy = numpy.sin(y * 0.5)
    cr = numpy.cos(r * 0.5)
    sr = numpy.sin(r * 0.5)
    cp = numpy.cos(p * 0.5)
    sp = numpy.sin(p * 0.5)

    qx = cy * sr * cp - sy * cr * sp
    qy = cy * cr * sp + sy * sr * cp
    qz = sy * cr * cp - cy * sr * sp
    qw = cy * cr * cp + sy * sr * sp

    quat = numpy.array([qx, qy, qz, qw])

    # 如果需要保持连贯性，并且存在前一四元数
    if previous_quat is not None:
        # 计算两个四元数之间的点积
        dot_product = numpy.dot(previous_quat, quat)
        # 如果点积为负，则说明两个四元数在四维空间中的指向相反，
        # 因此我们需要取新四元数的反值来保持插值的连续性
        if dot_product < 0:
            quat = -quat

    return quat


def degree_rpy_to_quat(rpy):
    rpy = numpy.radians(rpy)
    quat = radian_rpy_to_quat(rpy)

    return quat


def degree_ypr_to_quat(rpy):
    rpy = numpy.radians(rpy)
    quat = radian_ypr_to_quat(rpy)

    return quat


# 四元数按照XYZ顺序转换为欧拉角
def radian_quat_to_rpy(quat):
    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    r = numpy.arctan2(2 * (qx * qy + qz * qw), 1 - 2 * (qy * qy + qz * qz))
    p = numpy.arcsin(2 * (qx * qz - qy * qw))
    y = numpy.arctan2(2 * (qx * qw + qy * qz), 1 - 2 * (qz * qz + qw * qw))

    rpy = numpy.array([r, p, y])

    return rpy


# 四元数按照ZYX顺序转换为欧拉角
def radian_quat_to_ypr(quat):
    qx = quat[0]
    qy = quat[1]
    qz = quat[2]
    qw = quat[3]

    r = numpy.arctan2(2 * (qx * qy - qz * qw), 1 - 2 * (qy * qy + qz * qz))
    p = numpy.arcsin(2 * (qx * qz + qy * qw))
    y = numpy.arctan2(2 * (qx * qw - qy * qz), 1 - 2 * (qz * qz + qw * qw))

    rpy = numpy.array([r, p, y])

    return rpy


def degree_quat_to_rpy(q):
    rpy = radian_quat_to_rpy(q)
    rpy = numpy.degrees(rpy)

    return rpy


def radian_xyz_to_tmatrix(xyz):
    T = numpy.matrix([[1, 0, 0, xyz[0]],
                      [0, 1, 0, xyz[1]],
                      [0, 0, 1, xyz[2]],
                      [0, 0, 0, 1]])

    return T


def radian_tmatrix_to_xyz(T):
    xyz = numpy.array([T[0, 3], T[1, 3], T[2, 3]])
    return xyz


def radian_tmatrix_to_rpy(T):
    R = T[0:3, 0:3]
    rpy = numpy.array([numpy.arctan2(R[2, 1], R[2, 2]),
                       numpy.arctan2(-R[2, 0], numpy.sqrt(R[2, 1] * R[2, 1] + R[2, 2] * R[2, 2])),
                       numpy.arctan2(R[1, 0], R[0, 0])])

    return rpy


def radian_tmatrix_to_quat(T):
    R = T[0:3, 0:3]
    qw = numpy.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    q = numpy.array([qx, qy, qz, qw])
    return q


def radian_quat_to_tmatrix(q):
    qw = q[3]
    qx = q[0]
    qy = q[1]
    qz = q[2]
    T = numpy.matrix([[1 - 2 * qy * qy - 2 * qz * qz, 2 * qx * qy - 2 * qz * qw, 2 * qx * qz + 2 * qy * qw, 0],
                      [2 * qx * qy + 2 * qz * qw, 1 - 2 * qx * qx - 2 * qz * qz, 2 * qy * qz - 2 * qx * qw, 0],
                      [2 * qx * qz - 2 * qy * qw, 2 * qy * qz + 2 * qx * qw, 1 - 2 * qx * qx - 2 * qy * qy, 0],
                      [0, 0, 0, 1]])
    return T


def radian_tmatrix_to_axis(T):
    R = T[0:3, 0:3]
    theta = numpy.arccos((numpy.trace(R) - 1) / 2)
    if theta == 0:
        axis = numpy.array([0, 0, 1])
    else:
        axis = numpy.array([(R[2, 1] - R[1, 2]) / (2 * numpy.sin(theta)),
                            (R[0, 2] - R[2, 0]) / (2 * numpy.sin(theta)),
                            (R[1, 0] - R[0, 1]) / (2 * numpy.sin(theta))])
    return axis, theta


def radian_axis_to_tmatrix(axis, theta):
    T = numpy.matrix([[numpy.cos(theta) + axis[0] * axis[0] * (1 - numpy.cos(theta)),
                       axis[0] * axis[1] * (1 - numpy.cos(theta)) - axis[2] * numpy.sin(theta),
                       axis[0] * axis[2] * (1 - numpy.cos(theta)) + axis[1] * numpy.sin(theta), 0],
                      [axis[1] * axis[0] * (1 - numpy.cos(theta)) + axis[2] * numpy.sin(theta),
                       numpy.cos(theta) + axis[1] * axis[1] * (1 - numpy.cos(theta)),
                       axis[1] * axis[2] * (1 - numpy.cos(theta)) - axis[0] * numpy.sin(theta), 0],
                      [axis[2] * axis[0] * (1 - numpy.cos(theta)) - axis[1] * numpy.sin(theta),
                       axis[2] * axis[1] * (1 - numpy.cos(theta)) + axis[0] * numpy.sin(theta),
                       numpy.cos(theta) + axis[2] * axis[2] * (1 - numpy.cos(theta)), 0],
                      [0, 0, 0, 1]])
    return T


def degree_axis_to_tmatrix(axis, theta):
    theta = numpy.radians(theta)
    T = radian_axis_to_tmatrix(axis, theta)
    return T


def radian_tmatrix_to_homogeneous(T):
    h = numpy.array(
        [T[0, 0], T[0, 1], T[0, 2], T[1, 0], T[1, 1], T[1, 2], T[2, 0], T[2, 1], T[2, 2], T[0, 3], T[1, 3], T[2, 3]])
    return h


def degree_tmatrix_to_homogeneous(T):
    T = numpy.radians(T)
    h = radian_tmatrix_to_homogeneous(T)
    return h


def radian_homogeneous_to_tmatrix(h):
    T = numpy.matrix([[h[0], h[1], h[2], h[9]],
                      [h[3], h[4], h[5], h[10]],
                      [h[6], h[7], h[8], h[11]],
                      [0, 0, 0, 1]])
    return T


def degree_homogeneous_to_tmatrix(h):
    h = numpy.radians(h)
    T = radian_homogeneous_to_tmatrix(h)
    return numpy.degrees(T)


def radian_rp_to_horizontal_sextant_angle(rp):
    """Converts a roll-pitch vector to a horizontal sextant angle.

    Args:
        rp: A roll-pitch vector.

    Returns:
        The horizontal sextant angle in radians.
    """
    return numpy.arccos(numpy.cos(rp[0]) * numpy.cos(rp[1]))


def degree_rp_to_horizontal_sextant_angle(rp):
    """Converts a roll-pitch vector to a horizontal sextant angle.

    Args:
        rp: A roll-pitch vector.

    Returns:
        The horizontal sextant angle in degrees.
    """
    return numpy.degrees(radian_rp_to_horizontal_sextant_angle(numpy.radians(rp)))


def quat_rotate_inverse(quat, vect):
    q_w = quat[-1]
    q_vec = quat[:3]
    a = vect * (2.0 * q_w ** 2 - 1.0)
    b = numpy.cross(q_vec, vect) * q_w * 2.0
    c = q_vec * numpy.matmul(q_vec.reshape(1, 3), vect.reshape(3, 1)) * 2.0

    return a - b + c


def torch_quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * torch.bmm(q_vec.view(shape[0], 1, 3), v.view(shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c


def normalize(x, eps: float = 1e-9):
    return x / x.norm(p=2, dim=-1).clamp(min=eps, max=None).unsqueeze(-1)


def quat_unit(a):
    return normalize(a)


def quat_from_angle_axis(angle, axis):
    theta = (angle / 2).unsqueeze(-1)
    xyz = normalize(axis) * theta.sin()
    w = theta.cos()
    return quat_unit(numpy.cat([xyz, w], dim=-1))


def wuciX(p, h):
    m = len(p)
    n = m - 1
    T = numpy.zeros(m)
    for j in range(n + 1):
        T[j] = sum(h[0:j])

    # print(T)

    # % % M为初始系数为0矩阵共6n个约束方程
    M = numpy.zeros([6 * n, 6 * n])
    A1 = numpy.array([[0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, ],
                      [h[0] ** 5, h[0] ** 4, h[0] ** 3, h[0] ** 2, h[0], 1, 0, 0, 0, 0, 0, 0],
                      [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],
                      [5 * h[0] ** 4, 4 * h[0] ** 3, 3 * h[0] ** 2, 2 * h[0], 1, 0, 0, 0, 0, 0, -1, 0],
                      [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                      [20 * h[0] ** 3, 12 * h[0] ** 2, 6 * h[0], 2, 0, 0, 0, 0, 0, -2, 0, 0],
                      [60 * h[0] ** 2, 24 * h[0], 6, 0, 0, 0, 0, 0, -6, 0, 0, 0],
                      [120 * h[0], 24, 0, 0, 0, 0, 0, -24, 0, 0, 0, 0]])
    C1 = numpy.array([[0, 0, 0, 0, 0, 1],
                      [h[n - 1] ** 5, h[n - 1] ** 4, h[n - 1] ** 3, h[n - 1] ** 2, h[n - 1], 1],
                      [5 * h[n - 1] ** 4, 4 * h[n - 1] ** 3, 3 * h[n - 1] ** 2, 2 * h[n - 1], 1, 0],
                      [20 * h[n - 1] ** 3, 12 * h[n - 1] ** 2, 6 * h[n - 1], 2, 0, 0]])

    M[0:8, 0:12] = A1
    M[6 * n - 4:6 * n, 6 * n - 6:6 * n] = C1

    # print('n=',n)
    # for i in numpy.arange(2, n, 1):
    #    print('i',i)

    for i in numpy.arange(2, n, 1):
        print(i)
        B = numpy.array([
            [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
            [h[i - 1] ** 5, h[i - 1] ** 4, h[i - 1] ** 3, h[i - 1] ** 2, h[i - 1], 1, 0, 0, 0, 0, 0, 0],
            [5 * h[i - 1] ** 4, 4 * h[i - 1] ** 3, 3 * h[i - 1] ** 2, 2 * h[i - 1], 1, 0, 0, 0, 0, 0, - 1, 0],
            [20 * h[i - 1] ** 3, 12 * h[i - 1] ** 2, 6 * h[i - 1], 2, 0, 0, 0, 0, 0, - 2, 0, 0],
            [60 * h[i - 1] ** 2, 24 * h[i - 1], 6, 0, 0, 0, 0, 0, - 6, 0, 0, 0],
            [120 * h[i - 1], 24, 0, 0, 0, 0, 0, - 24, 0, 0, 0, 0]])
        # % % B为中间段系数矩阵共6(n - 2)个约束方程

        M[6 * i - 4: 6 * i + 2, 6 * i - 6: 6 * i + 6] = B

    Y = numpy.zeros([6 * n, 1])
    Y1 = numpy.array([p[0], p[1], 0, 0, 0, 0, 0, 0])
    Yn = numpy.array([p[n - 1], p[m - 1], 0, 0, ])
    Y[0: 8, 0] = Y1
    Y[6 * n - 4: 6 * n, 0] = Yn

    for i in numpy.arange(2, n, 1):
        Y[6 * i - 4: 6 * i + 2, 0] = numpy.array([p[i - 1], p[i], 0, 0, 0, 0, ])

    X = numpy.dot(numpy.linalg.inv(M), Y)
    X = numpy.array(X).flatten()

    return X, T, n


if __name__ == "__main__":
    q = torch.Tensor([[0, 0, 0, 1]])
    v = torch.Tensor([[1, 1, 1]])

    result = torch_quat_rotate_inverse(q, v)
    print("result = ", result)

    q = numpy.array([0, 0, 0, 1])
    v = numpy.array([1, 1, 1])

    result = quat_rotate_inverse(q, v)
    print("result = ", result)
