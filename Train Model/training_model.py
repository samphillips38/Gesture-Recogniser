import re
import numpy as np
import pyomo.environ as pyo
from collections import Iterable

class NLSVM():

    def __init__(self) -> None:

        # Model parameters
        self.C = 10
        self.gam = 0.8

        # Retrieve data and format
        self.X, self.up_t, self.circle_t, self.left_t = self.get_data()

    def __read_file(self, file_name):
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

    def __K(self, Xn, Xm, use_np=False):
        """Radial Basis kernel function."""
        if use_np:
            return np.exp(-self.gam * np.linalg.norm(Xn - Xm, axis=1)**2)
        else:
            return pyo.exp(-self.gam * np.linalg.norm(Xn - Xm)**2)

    def __test_nonlinear_SVM(self, X_test, test_t, X, t, a, b):
        """Test the nonlinear SVM against model."""
        (N_test, d) = X_test.shape
        N = X.shape[0]
        y = np.zeros(N_test)
        for n in range(N):
            y += a[n] * t[n] * self.__K(X[n], X_test, use_np=True)
        errors = [1 if y[i]*test_t[i] < 0 else 0 for i in range(N_test)]
        print("Samples Tested: ", N_test)
        print("Total Errors: ", sum(errors))
        print("Accuracy: ", (N - sum(errors))/N)

    def __get_features(self, data, time_array, test=False):
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
        if not test: self.v_range = np.array([
            [vx.min(), vx.max()],
            [vy.min(), vy.max()],
            [vz.min(), vz.max()]
        ])
        if not test: self.pos_range = np.array([
            [x.min(), x.max()],
            [y.min(), y.max()],
            [z.min(), z.max()]
        ])

        vx_dist = get_dist(vx, (self.v_range[0, 0], self.v_range[0, 1]), bins=bins)
        vy_dist = get_dist(vx, (self.v_range[1, 0], self.v_range[1, 1]), bins=bins)
        vz_dist = get_dist(vx, (self.v_range[2, 0], self.v_range[2, 1]), bins=bins)

        x_dist = get_dist(x, (self.pos_range[0, 0], self.pos_range[0, 1]), bins=bins)
        y_dist = get_dist(x, (self.pos_range[1, 0], self.pos_range[1, 1]), bins=bins)
        z_dist = get_dist(x, (self.pos_range[2, 0], self.pos_range[2, 1]), bins=bins)

        X = np.array([
            ax.sum(axis=0), ay.sum(axis=0), az.sum(axis=0),
            vx.sum(axis=0), vy.sum(axis=0), vz.sum(axis=0),
            x.sum(axis=0), y.sum(axis=0), z.sum(axis=0),
            *vx_dist, *vy_dist, *vz_dist,
            *x_dist, *y_dist, *z_dist,
        ]).T

        # Normalise the data
        if not test: self.norm_factor = np.linalg.norm(X, axis=1).mean()
        X = X / self.norm_factor

        # Labels
        up_t = np.array([*[-1]*self.N_h, *[1]*self.N_h, *[1]*self.N_h])
        circle_t = np.array([*[1]*self.N_h, *[-1]*self.N_h, *[1]*self.N_h])
        left_t = np.array([*[1]*self.N_h, *[1]*self.N_h, *[-1]*self.N_h])

        return X, up_t, circle_t, left_t

    def __SVM_nonlinear(self, X, t):
        """Learn SVM using Gaussian radial basis kernel funtions from data X, shape (N, d), and labels t. 
        Return lagrange mutlipliers, bias and support vecor indices."""

        # Optimise Lagrangian using pyomo
        model = pyo.ConcreteModel()
        model.a = pyo.Var(np.arange(self.N), within=pyo.NonNegativeReals, bounds=(0, self.C))
        def dual_obj(model):
            return sum(model.a[n] - 0.5*sum(model.a[n]*model.a[m]*t[n]*t[m]*self.__K(X[n], X[m]) for m in range(self.N)) for n in range(self.N))

        def lagrangian_con(model):
            return sum(model.a[n] * t[n] for n in range(self.N)) == 0

        model.lagrangian_con = pyo.Constraint(expr=lagrangian_con)
        model.obj = pyo.Objective(expr=dual_obj, sense=pyo.maximize)
        opt = pyo.SolverFactory("ipopt")
        opt.solve(model)
        a = np.array([pyo.value(model.a[i]) for i in range(self.N)]) # Convert data

        # Calculate bias and average over all points
        b = sum(t[n] - sum(a[n] * t[n] * self.__K(X, n, m) for m in range(self.N)) for n in range(self.N)) / self.N

        # Get indexes of Support Vectors
        svi = np.where(a > 0)[0]

        return a, b, svi

    def get_data(self):
        """Get all of the raw data and extract features."""
        up_data, up_time = self.__read_file("up_data.txt")
        circle_data, circle_time = self.__read_file("circle_data.txt")
        left_data, left_time = self.__read_file("left_data.txt")
        self.raw_data = np.array([*up_data, *circle_data, *left_data]).T
        self.time_array = np.array([*up_time, *circle_time, *left_time]).T

        (self.d, self.N) = self.time_array.shape
        self.N_h = self.N//3

        return self.__get_features(self.raw_data, self.time_array)

    def get_test_data(self):
        """Get test data and extract features."""
        up_test_data, up_time = self.__read_file("up_test_data.txt")
        circle_test_data, circle_time = self.__read_file("circle_test_data.txt")
        left_test_data, left_time = self.__read_file("left_test_data.txt")
        self.test_data = np.array([*up_test_data, *circle_test_data, *left_test_data]).T
        self.test_time_array = np.array([*up_time, *circle_time, *left_time]).T

        (self.test_d, self.test_N) = self.test_time_array.shape
        self.test_N_h = self.test_N//3

        return self.__get_features(self.test_data, self.test_time_array, test=True)

    def learn_model(self):
        """Learn parameters of the model."""
        self.up_a, self.up_b, self.up_svi = self.__SVM_nonlinear(self.X, self.up_t)
        self.circle_a, self.circle_b, self.circle_svi = self.__SVM_nonlinear(self.X, self.circle_t)
        self.left_a, self.left_b, self.left_svi = self.__SVM_nonlinear(self.X, self.left_t)

    def blind_test(self):
        """Test on the training data."""
        print("Up Test: ")
        self.__test_nonlinear_SVM(self.X, self.up_t, self.X, self.up_t, self.up_a, self.up_b)
        print("\nCircle Test: " )
        self.__test_nonlinear_SVM(self.X, self.circle_t, self.X, self.circle_t, self.circle_a, self.circle_b)
        print("\nLeft Test: " )
        self.__test_nonlinear_SVM(self.X, self.left_t, self.X, self.left_t, self.left_a, self.left_b)

    def test_model(self):
        """Test with test data and print out results."""
        self.X_test, self.test_up_t, self.test_circle_t, self.test_left_t = self.get_test_data()

        print("Up Test: ")
        self.__test_nonlinear_SVM(self.X_test, self.test_up_t, self.X, self.up_t, self.up_a, self.up_b)
        print("\nCircle Test: " )
        self.__test_nonlinear_SVM(self.X_test, self.test_circle_t, self.X, self.circle_t, self.circle_a, self.circle_b)
        print("\nLeft Test: " )
        self.__test_nonlinear_SVM(self.X_test, self.test_left_t, self.X, self.left_t, self.left_a, self.left_b)

    def print_learned_model(self):
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
        print(convert_C_array(self.X, "X"))
        print(convert_C_array(self.up_t, "up_t"))
        print(convert_C_array(self.up_a, "up_a"))
        print(convert_C_array(self.up_b, "up_b"))
        print(convert_C_array(self.circle_t, "circle_t"))
        print(convert_C_array(self.circle_a, "circle_a"))
        print(convert_C_array(self.circle_b, "circle_b"))
        print(convert_C_array(self.left_t, "left_t"))
        print(convert_C_array(self.left_a, "left_a"))
        print(convert_C_array(self.left_b, "left_b"))
        print(convert_C_array(self.v_range, "v_range"))
        print(convert_C_array(self.pos_range, "pos_range"))
        print(convert_C_array(self.norm_factor, "norm_factor"))
        print("#endif")