import re
import numpy as np
import pyomo.environ as pyo
from collections import Iterable

def get_data(file_name):
    data = []
    T = []
    with open(file_name) as file:
        line = file.readline()

        while True:
            while line is '\n':
                line = file.readline()
            if line is "":
                break
            t_guess = 0
            t_init = 0
            x, y, z, t = [], [], [], []
            while line is not "\n":
                match = re.search(r"x:(?P<x>\-?\d+\.\d+)\ty:(?P<y>\-?\d+\.\d+)\tz:(?P<z>\-?\d+\.\d+)(\tt:(?P<t>\-?\d+\.\d+))?", line)
                x.append(match.group('x'))
                y.append(match.group('y'))
                z.append(match.group('z'))
                t.append(t_guess if not match.group('t') else match.group('t'))

                # Time relative to start
                if len(t) == 1:
                    t_init = float(t[0])
                t[-1] = float(t[-1]) - float(t_init)

                line = file.readline()
                t_guess += 16
            data.append([*x, *y, *z])
            T.append(t)
    return np.array(data, dtype=float).reshape((len(data), len(data[0]))), np.array(T, dtype=float).reshape((len(T), len(T[0])))/1000


def get_features(data, time_array, norm_factor=None, v_range=None, pos_range=None):
    """Given raw data, create feature vector."""
    N_t, N = data.shape[0]//3, data.shape[1]

    # Raw acceleration values
    ax, ay, az = data[:N_t, :], data[N_t:N_t*2, :], data[N_t*2:N_t*3, :]

    # Integrate
    def integrate(x, t):
        """Integrage Using trapezoid Rule."""
        x_i = np.zeros(x.shape)
        x_i[:-1] = (t[1:] - t[:-1]) * (x[:-1] + x[1:]) / 2
        x_i = np.cumsum(x_i, axis=0)
        x_i[-1] = 2*x_i[-2] - x_i[-3]
        return x_i

    vx = integrate(ax, time_array)
    x = integrate(vx, time_array)
    vy = integrate(ay, time_array)
    y = integrate(vy, time_array)
    vz = integrate(az, time_array)
    z = integrate(vz, time_array)

    # Time independent distribution
    def get_dist(x, r, bins=100):
        dist = np.zeros((bins, N))
        for i in range(N):
            dist[:, i] = np.histogram(x[:, i], bins=bins, range=r)[0]
        return dist

    bins = 5
    if v_range is not None: v_range = np.array([
        [vx.min(), vx.max()],
        [vy.min(), vy.max()],
        [vz.min(), vz.max()]
    ])
    if pos_range is not None: pos_range = np.array([
        [x.min(), x.max()],
        [y.min(), y.max()],
        [z.min(), z.max()]
    ])

    vx_dist = get_dist(vx, (v_range[0, 0], v_range[0, 1]), bins=bins)
    vy_dist = get_dist(vx, (v_range[1, 0], v_range[1, 1]), bins=bins)
    vz_dist = get_dist(vx, (v_range[2, 0], v_range[2, 1]), bins=bins)

    x_dist = get_dist(x, (pos_range[0, 0], pos_range[0, 1]), bins=bins)
    y_dist = get_dist(x, (pos_range[1, 0], pos_range[1, 1]), bins=bins)
    z_dist = get_dist(x, (pos_range[2, 0], pos_range[2, 1]), bins=bins)

    X = np.array([
        ax.sum(axis=0),
        ay.sum(axis=0),
        az.sum(axis=0),
        vx.sum(axis=0),
        vy.sum(axis=0),
        vz.sum(axis=0),
        x.sum(axis=0),
        y.sum(axis=0),
        z.sum(axis=0),

        # ax.max(axis=0),
        # ay.max(axis=0),
        # az.max(axis=0),
        # vx.max(axis=0),
        # vy.max(axis=0),
        # vz.max(axis=0),
        # x.max(axis=0),
        # y.max(axis=0),
        # z.max(axis=0),

        # ax.min(axis=0),
        # ay.min(axis=0),
        # az.min(axis=0),
        # vx.min(axis=0),
        # vy.min(axis=0),
        # vz.min(axis=0),
        # x.min(axis=0),
        # y.min(axis=0),
        # z.min(axis=0),

        # time_array[ax.argmax(axis=0)].diagonal(),
        # time_array[ay.argmax(axis=0)].diagonal(),
        # time_array[az.argmax(axis=0)].diagonal(),
        # time_array[vx.argmax(axis=0)].diagonal(),
        # time_array[vy.argmax(axis=0)].diagonal(),
        # time_array[vz.argmax(axis=0)].diagonal(),
        # time_array[x.argmax(axis=0)].diagonal(),
        # time_array[y.argmax(axis=0)].diagonal(),
        # time_array[z.argmax(axis=0)].diagonal(),
        
        # time_array[ax.argmin(axis=0)].diagonal(),
        # time_array[ay.argmin(axis=0)].diagonal(),
        # time_array[az.argmin(axis=0)].diagonal(),
        # time_array[vx.argmin(axis=0)].diagonal(),
        # time_array[vy.argmin(axis=0)].diagonal(),
        # time_array[vz.argmin(axis=0)].diagonal(),
        # time_array[x.argmin(axis=0)].diagonal(),
        # time_array[y.argmin(axis=0)].diagonal(),
        # time_array[z.argmin(axis=0)].diagonal(),

        *vx_dist,
        *vy_dist,
        *vz_dist,

        *x_dist,
        *y_dist,
        *z_dist,
    ]).T

    # Normalise the data
    if not norm_factor: norm_factor = np.linalg.norm(X, axis=1).mean()
    X = X / norm_factor

    return X, norm_factor, v_range, pos_range


def SVM_linear(X, t, C=10, disp=False):
    """Learn SVM from data X, shape (N, d), and labels t. Return weight vector and bias."""

    # Optimise Lagrangian
    (N, d) = X.shape
    model = pyo.ConcreteModel()
    model.a = pyo.Var(np.arange(N), within=pyo.Reals, bounds=(0, C))
    def dual_obj(model):
        return sum(model.a[n] - 0.5*sum(model.a[n]*model.a[m]*t[n]*t[m]*np.dot(X[n], X[m]) for m in range(N)) for n in range(N))

    def lagrangian_con(model):
        return sum(model.a[n] * t[n] for n in range(N)) == 0

    model.lagrangian_con = pyo.Constraint(expr=lagrangian_con)
    model.obj = pyo.Objective(expr=dual_obj, sense=pyo.maximize)
    opt = pyo.SolverFactory("ipopt", tee=True)
    opt.options['halt_on_ampl_error'] = 'yes'
    opt.solve(model)
    if disp: model.display()
    a = np.array([pyo.value(model.a[i]) for i in range(N)]) # Convert data

    # Calculate weight vector
    w = sum(a[n] * t[n] * X[n] for n in range(N))

    # Calculate bias and average over all points
    b = sum(t[n] - np.dot(w, X[n]) for n in range(N)) / N

    # Get indexes of Support Vectors
    svi = np.where(a > 0)[0]

    return w, b, svi


def SVM_nonlinear(X, t, K, C=10, disp=False):
    """Learn SVM using Gaussian radial basis kernel funtions from data X, shape (N, d), and labels t. 
    Return lagrange mutlipliers, bias and support vecor indices."""

    # Optimise Lagrangian
    (N, d) = X.shape
    model = pyo.ConcreteModel()
    model.a = pyo.Var(np.arange(N), within=pyo.NonNegativeReals, bounds=(0, C))
    def dual_obj(model):
        return sum(model.a[n] - 0.5*sum(model.a[n]*model.a[m]*t[n]*t[m]*K(X[n], X[m]) for m in range(N)) for n in range(N))

    def lagrangian_con(model):
        return sum(model.a[n] * t[n] for n in range(N)) == 0

    model.lagrangian_con = pyo.Constraint(expr=lagrangian_con)
    model.obj = pyo.Objective(expr=dual_obj, sense=pyo.maximize)
    opt = pyo.SolverFactory("ipopt")
    opt.solve(model)
    if disp: model.display()
    a = np.array([pyo.value(model.a[i]) for i in range(N)]) # Convert data

    # Calculate bias and average over all points
    b = sum(t[n] - sum(a[n] * t[n] * K(X, n, m) for m in range(N)) for n in range(N)) / N

    # Get indexes of Support Vectors
    svi = np.where(a > 0)[0]

    return a, b, svi


def print_learned_model(X, up_t, up_a, up_b, circle_t, circle_a, circle_b, left_t, left_a, left_b, v_range, pos_range, norm_factor):
    def convert_C_array(x, name=""):

        # Deepest Level
        if not isinstance(x, Iterable):
            return r'extern float {} = {};'.format(name, x) if name else str(x)

        # Construct inside recursively
        s = ", ".join([convert_C_array(val) for val in x])

        # If not at outer depth, return as is
        if not name: return "\n\t" + "{" + s + "}"

        # Else add other bits
        shape_str = "".join(["[" + str(dim) + "]" for dim in x.shape])
        return "extern float " + name + shape_str + " = {" + s + "\n};"

    print("#ifndef MODEL_DATA\n#define MODEL_DATA")
    print(convert_C_array(X, "X"))
    print(convert_C_array(up_t, "up_t"))
    print(convert_C_array(up_a, "up_a"))
    print(convert_C_array(up_b, "up_b"))
    print(convert_C_array(circle_t, "circle_t"))
    print(convert_C_array(circle_a, "circle_a"))
    print(convert_C_array(circle_b, "circle_b"))
    print(convert_C_array(left_t, "left_t"))
    print(convert_C_array(left_a, "left_a"))
    print(convert_C_array(left_b, "left_b"))
    print(convert_C_array(v_range, "v_range"))
    print(convert_C_array(pos_range, "pos_range"))
    print(convert_C_array(norm_factor, "norm_factor"))
    print("#endif")

