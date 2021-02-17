import sympy
from sympy import solveset
from sympy.abc import x,y
from sympy.solvers.inequalities import solve_rational_inequalities
# print(solveset((10000 / x) - 1 < 0, x, sympy.Reals))

#range of optimality 
# 2x+y<=1000
# 3x+4y<=2400
print(solveset(-2 < (-x / 5),x, sympy.Reals))
print(solveset((-x / 5)<(-3/4),x, sympy.Reals))

# print(solveset(-2 < (-x / 5) < (-3/4),x, sympy.Reals))
print(solveset(-2 < (-8 / y),y, sympy.Reals))
print(solveset((-8 / y)<(-3/4),y, sympy.Reals))

#range of feasibility