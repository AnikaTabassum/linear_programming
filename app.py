from flask import Flask
from flask import render_template, request
# importing Flask and other modules 
app = Flask(__name__)

#generic linear programming
from ortools.linear_solver import pywraplp
Obj_Func = ""
sign = 3
ans_map = {}
num_of_var = 0
splitted_objective_function =""

def getConstrExpr(constraint):
    if "<=" in constraint:
        splitted_constraint = constraint.split("<=")

    elif ">=" in constraint:
        splitted_constraint = constraint.split(">=")

    lhs = splitted_constraint[0]
    rhs = splitted_constraint[1]

    splitted_lhs = []
    if "+" in lhs:
        splitted_lhs = lhs.split("+")

    elif "-" in lhs:
        splitted_lhs = lhs.split("-")
    
    else:
        splitted_lhs.append(lhs)

    #print("splitted lhs:",splitted_lhs)
    
    num_of_var = len(splitted_lhs)
    
    new_constr = ""
    for nv in range(0,num_of_var):
        coeff_index = splitted_lhs[nv].split('x')
        coeff = coeff_index[0]
        index = coeff_index[1]
        if coeff=="":
            coeff = "1"
        new_constr+=str(coeff)+" * x["+str(index)+"]"
        if nv!=num_of_var-1:
            if "+" in lhs:
                new_constr+=" + "
            elif "-" in lhs:
                new_constr+=" - "
          
    if "<=" in constraint:
        new_constr+=" <= "+str(rhs)

    elif ">=" in constraint:
        new_constr+=" >= "+str(rhs)
    return new_constr
  
    
def getExpr(splitted_objective_function,num_of_var):
    obj_func = ""
    for nv in range(0,num_of_var):
        coeff_index = splitted_objective_function[nv].split('x')
        coeff = coeff_index[0]
        index = coeff_index[1]
        if coeff=="":
            coeff = "1"
        obj_func+=str(coeff)+" * x["+str(index)+"]"
        if nv!=num_of_var-1:
            #if sign==1:
            #    obj_func+=" + "
            #elif sign==0:
            #    obj_func+=" - "
            obj_func+=" + "
    
    return obj_func

def main(ObjectiveFunction, ConstraintList):
    # [START solver]
    # Create the linear solver with the GLOP backend.
    solver = pywraplp.Solver.CreateSolver('GLOP')
    # [END solver]

    # [START variables]
    infinity = solver.infinity()
    
    # Define the decision variables
    #command = "Maximize 8x1+5x2"
    #command = input("Enter command\n")
    command = ObjectiveFunction
    splitted_command = command.split()

    ans_map['Objective_Function'] = command

    todo = splitted_command[0]

    if "+" in splitted_command[1]:
        sign = 1
        splitted_objective_function = splitted_command[1].split("+")

    elif "-" in splitted_command[1]:
        sign = 0
        splitted_objective_function = splitted_command[1].split("-")

    num_of_var = len(splitted_objective_function)
    
    x = {i: solver.NumVar(0.0, infinity, f'x{i}') for i in range(1, num_of_var+1)}

    print('Number of variables =', solver.NumVariables())
    ans_map['NumOfVar'] = solver.NumVariables()
    # [END variables]

    # [START constraints]
    # 2x+y<=1000
    #constr_list = ["2x1+x2<=1000","3x1+4x2<=2400","x1+x2<=700","x1-x2<=350","x1>=0","x2>=0"]
    
    #constr_input = input("Enter constraints\n")
    constr_input = ConstraintList
    constr_list = constr_input.split(",")
    for constr in constr_list:
        constr = getConstrExpr(constr)
        constr = eval(constr)
        solver.Add(constr)

    ans_map['Constraints'] = constr_input

    # x >=0
    #solver.Add(x[1] >= 0.0)

    # y >=0
    #solver.Add(x[2] >= 0.0)

    print('Number of constraints =', solver.NumConstraints())
    ans_map['NumOfConstraints'] = solver.NumConstraints()

    # [END constraints]
    # [START objective]
    # Maximize 8x+5y
    objFunc = getExpr(splitted_objective_function,num_of_var)
    #converting string into exp
    if todo=="Maximize":
        #solver.Maximize(8 * x[1] + 5 * x[2])
        objFunc = eval(objFunc)
        solver.Maximize(objFunc)
    # [END objective]

    # [START solve]
    status = solver.Solve()

    # [END solve]
    solution = {}
    # [START print_solution]
    if status == pywraplp.Solver.OPTIMAL:
        solution['exists']= "yes"
        print('Solution:')
        print('Objective value =', solver.Objective().Value())
        solution['ObjectiveValue'] = solver.Objective().Value()
        
        #generic kora baki
        print('x1 =', x[1].solution_value())
        #x1_sol_val = 'x1 ='+ str(x[1].solution_value())
        solution['x1_sol_val'] = x[1].solution_value()

        print('x2 =', x[2].solution_value())
        solution['x2_sol_val'] = x[2].solution_value()

        print("offset ", solver.Objective().BestBound())
        
    else:
        print('The problem does not have an optimal solution.')
        solution['exists']="no"

    ans_map['solution'] = solution

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
            print("the constraint ", val['constraint'].GetCoefficient(x[1]),"* x1 +",val['constraint'].GetCoefficient(x[2]) ,"* x2 has a direct impact on the optimal solution")
            print("shadow price is ", val['constraint'].dual_value())
        else:
            print("This constraint is a non-binding constraint", val['constraint'])
            print("dual ", val['constraint'].dual_value())
            print("the constraint ", val['constraint'].GetCoefficient(x[1]),"* x1 +",val['constraint'].GetCoefficient(x[2]) ,"* x2 has no direct impact on the optimal solution")
            if  val['slack'] >0.0:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()-val['slack'])
            else:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()+val['slack'])

    print("the reduced cost of x1 is", x[1].reduced_cost(), x[2].basis_status())
    #print("the reduced cost of y is", y.reduced_cost(), x.basis_status())

    return ans_map

def checkSign(val):
    if val <0:
        return True
    else:
        return False

#if __name__ == '__main__':
#    main()
# [END program]

#Maximize 8x1+5x2
#2x1+x2<=1000,3x1+4x2<=2400,x1+x2<=700,x1-x2<=350,x1>=0,x2>=0

def demo_main(input):
    return "Maximized "+str(input)

@app.route("/hello/")
@app.route("/hello/<name>")
def ui(name = None):
    return render_template(
        "ui.html",
        name=name
    )
@app.route("/api/data")
def get_data():
    return app.send_static_file("data.json")

@app.route("/solve")
def solve(name = None):
    return render_template(
        "solve.html",
        result="solve kori"
    )

ans=None
@app.route('/', methods =["GET", "POST"]) 
def gfg(ans=""): 
    if request.method == "POST": 
       # getting input with name = fname in HTML form 
       objfunc = request.form.get("objfunc") 
       # getting input with name = lname in HTML form  
       constr = request.form.get("constr") 
       #ans = demo_main(objfunc+constr) 
       ans = main(objfunc, constr)
       #return ans
    return render_template(
        "form.html",
        result=ans
    )  