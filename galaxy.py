# galaxy industries production  porblem
# by anika tabassum
from ortools.linear_solver import pywraplp
# [END import]


def main():
    # [START solver]
    # Create the linear solver with the GLOP backend.
    solver = pywraplp.Solver.CreateSolver('GLOP')
    # [END solver]

    # [START variables]
    infinity = solver.infinity()
    # Create the variables x and y.
    x = solver.NumVar(0.0, infinity, 'x')
    # x.SetInteger(True)
    # if x.Integer():
    #     print("true")
    y = solver.NumVar(0.0, infinity, 'y')

    print('Number of variables =', solver.NumVariables())
    # [END variables]

    # [START constraints]
    # 2x+y<=1000
    solver.Add(2 * x + 1 * y <= 1000)
    #3x+4y<=2400
    solver.Add(3 * x + 4 * y <= 2400)

    #x+y<=700
    solver.Add(x + y <= 700)

    #x-y<=350
    solver.Add(x - y <= 350)

    # x >=0
    solver.Add(x >= 0.0)

    # y >=0
    solver.Add(y >= 0.0)

    print('Number of constraints =', solver.NumConstraints())
    # [END constraints]

    # [START objective]
    # Maximize 8x+5y
    solver.Maximize(8 * x + 5 * y)
    # [END objective]

    # [START solve]
    status = solver.Solve()

    # [END solve]

    # [START print_solution]
    if status == pywraplp.Solver.OPTIMAL:
        print('Solution:')
        print('Objective value =', solver.Objective().Value())
        print('x =', x.solution_value())
        print('y =', y.solution_value())
        print("offset ", solver.Objective().BestBound())
    else:
        print('The problem does not have an optimal solution.')
    # [END print_solution]
    print("cons",solver.constraints()[2].ub())
    # x=round(x.solution_value())
    constraint_compute=solver.ComputeConstraintActivities()

    compute_slack=[]
    for i in range(len(solver.constraints())):
        # print("poss ", solver.constraints()[i].ub()-round(constraint_compute[i]))
        if solver.constraints()[i].ub()==infinity:
            print("inf")
        else:
            compute_slack.append( {"constraint":solver.constraints()[i],"slack": solver.constraints()[i].ub()-round(constraint_compute[i])})
    # [START advanced]
    print('\nAdvanced usage:')
    print('Problem solved in %f milliseconds' % solver.wall_time())
    print('Problem solved in %d iterations' % solver.iterations())
    print('Problem solved in %d branch-and-bound nodes' % solver.nodes())
    # [END advanced]

    ############# slack/surplus  --------------------------------

    for val in compute_slack:
        # print(val['constraint'])
        if val['slack'] ==0.0:
            print("This constraint is a binding constraint")
            print("the constraint ", val['constraint'].GetCoefficient(x),"* x +",val['constraint'].GetCoefficient(y) ,"* y has a direct impact on the optimal solution")
        else:
            print("This constraint is a non-binding constraint", val['constraint'])
            print("the constraint ", val['constraint'].GetCoefficient(x),"* x +",val['constraint'].GetCoefficient(y) ,"* y has no direct impact on the optimal solution")
            if  val['slack'] >0.0:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()-val['slack'])
            else:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()+val['slack'])

    print("the reduced cost of x is", x.reduced_cost(), y.basis_status())

def checkSign(val):
    if val <0:
        return True
    else:
        return False

if __name__ == '__main__':
    main()
# [END program]





