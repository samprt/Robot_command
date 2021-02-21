from roblib import *

xmin, xmax, ymin, ymax = -3, 3, -3, 3


def draw_field(f):
    X, Y, U, V = [], [], [], []
    for i in linspace(xmin, xmax, 20):
        for j in linspace(ymin, ymax, 20):
            x = array([[i], [j]])
            X.append(i)
            Y.append(j)
            U.append(f(x)[0, 0] / norm(f(x)))
            V.append(f(x)[1, 0] / norm(f(x)))
    quiver(X, Y, U, V)


def square():
    a, b = array([[0, 0]]).T, array([[2, 2]]).T
    n1 = array([[0, 1]]).T
    n2 = array([[-1, 0]]).T
    n3 = array([[0, -1]]).T
    n4 = array([[1, 0]]).T

    def field(x):
        return -2 * (n1 @ n1.T @ (x - a) + n2 @ n2.T @ (x - b) + n3 @ n3.T @ (x - b) + n4 @ n4.T @ (x - a))

    plot([a[0, 0], b[0, 0]], [a[1, 0], a[1, 0]], "red")
    plot([b[0, 0], b[0, 0]], [a[1, 0], b[1, 0]], "red")
    plot([b[0, 0], a[0, 0]], [b[1, 0], b[1, 0]], "red")
    plot([a[0, 0], a[0, 0]], [b[1, 0], a[1, 0]], "red")
    draw_field(field)


def inputs():
    def f_circle(x):
        return -(x[0, 0] ** 2 + x[1, 0] ** 2 - 1) * x + array([[-x[1, 0]], [x[0, 0]]])

    a11 = float(input("a11 ="))
    a12 = float(input("a12 ="))
    a21 = float(input("a21 ="))
    a22 = float(input("a22 ="))

    A = array([[a11, a12],
               [a21, a22]])

    def field(x):
        return A @ f_circle(inv(A) @ x)

    draw_field(field)


if __name__ == '__main__':
    # square()
    # pause(0)
    while True:
        inputs()
        pause(0)
