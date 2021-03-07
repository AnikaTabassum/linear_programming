from flask import Flask
from flask import render_template, request
# importing Flask and other modules 
app = Flask(__name__)

#generic linear programming
import json
import re
import numpy as np
from ortools.linear_solver import pywraplp

Obj_Func = ""
sign = 3
ans_map = {}
num_of_var = 0
splitted_objective_function =""
range_constraint_list = []

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
    
    range_constr = ""    
    new_constr = ""
    
    for nv in range(0,num_of_var):
        coeff_index = splitted_lhs[nv].split('x')
        coeff = coeff_index[0]
        index = coeff_index[1]
        
        if coeff=="":
            coeff = "1"
            
        range_constr+=str(coeff)+"x"+str(index)
        new_constr+=str(coeff)+" * x["+str(index)+"]"
        
        if nv!=num_of_var-1:
            if "+" in lhs:
                new_constr+=" + "
                range_constr+="+"
            elif "-" in lhs:
                new_constr+=" - "
                range_constr+="-"
          
    if "<=" in constraint:
        new_constr+=" <= "+str(rhs)

    elif ">=" in constraint:
        new_constr+=" >= "+str(rhs)
    
    range_constr+="-"+str(rhs)
    return new_constr,range_constr
  
    
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

def getRangeOfOptimality(equation1,equation2,optimalEquation):
    slope1 = [float(i) for i in re.split('[xy]', equation1)]
    print((slope1[0]),slope1[1])
    slope2 = [float(i) for i in re.split('[xy]', equation2)]
    print((slope2[0]),slope2[1])
    optimalEquation+='-0'
    optimalSlope = [float(i) for i in re.split('[xy]', optimalEquation)]
    print((optimalSlope[0]),optimalSlope[1])
    slope1 = (-slope1[0]/slope1[1])
    slope2 = (-slope2[0] / slope2[1])
    optimalSlopefinal = (-optimalSlope[0]/optimalSlope[1])
    #print(slope1,slope2,optimalSlope)
    maximumForCx,minimumForCx = 0,0
    maximumForCy, minimumForCy =0,0

    if(slope1>slope2):
        temp = slope1
        slope1 = slope2
        slope2 = temp

    if(optimalSlopefinal<0):
        maximumForCx = (-slope1)*optimalSlope[1]
        minimumForCx = (-slope2)*optimalSlope[1]
        maximumForCy = optimalSlope[0]/(-slope2)
        minimumForCy = optimalSlope[0]/(-slope1)
        print('range variable x [',minimumForCx,maximumForCx,']')
        print('range variable y [', minimumForCy, maximumForCy, ']')
    else:
        minimumForCx = (slope1) * optimalSlope[1]
        maximumForCx = (slope2) * optimalSlope[1]
        minimumForCy = optimalSlope[0] / (slope2)
        maximumForCy = optimalSlope[0] / (slope1)
        print('range variable x [', minimumForCx, maximumForCx, ']')
        print('range variable y [', minimumForCy, maximumForCy, ']')

    RangeOfOptimality = {
        'minValX': minimumForCx,
        'maxValX': maximumForCx,
        'minValY': minimumForCy,
        'maxValY': maximumForCy,
    }
    return RangeOfOptimality

def isSatisfyAllEquation(equationList,intersectPoint):
    tag = False
    for equation in equationList:
        #print(equation)
        point = [float(i) for i in re.split('[xy]', equation[0])]
        point[2] = - point[2]
        #print(point)
        lhs = point[0]*intersectPoint[0]+point[1]*intersectPoint[1]
        #print(lhs)
        if(equation[1]==1):
            if lhs >= point[2]:
                tag = True
            else:
                tag = False
                break
        else:
            if lhs <= point[2]:
                tag = True
            else:
                tag = False
                break
        #print(tag)


    return tag

def getRangeOfFeasibility(equationList,equation1,equation2):

    # print(equationList[0][0])
    # # greater than constrains  == 0
    # # less than constrains  == 1
    # print(equation1)

    finfIndexList=[]
    for i in range(0,len(equationList)):
        if(equationList[i][0]==equation1 or equationList[i][0]==equation2 ):
            finfIndexList.append(i)
    #print(finfIndexList)

    del equationList[finfIndexList[0]]
    del equationList[finfIndexList[1]-1]

    #print(equationList)

    point1 = [float(i) for i in re.split('[xy]', equation1)]
    point1[2] = - point1[2]
    #print(point1)
    point2 = [float(i) for i in re.split('[xy]', equation2)]
    point2[2] = - point2[2]
    #print(point2)
    cof = np.array([[point1[0], point1[1]], [point2[0], point2[1]]])

    optimalEquation1Increase,optimalEquation1Decrease,optimalEquation2Increase,optimalEquation2Decrease=0,0,0,0;

    for i in range(1,1000):
        constrains = np.array([point1[2]+i, point2[2]])
        intersectPoint = np.linalg.solve(cof, constrains)
        tag = isSatisfyAllEquation(equationList,intersectPoint)
        #print(tag)
        if tag==False:
            print(i)
            optimalEquation1Increase =i-1
            break

    for i in range(1,1000):
        constrains = np.array([point1[2], point2[2]+i])
        intersectPoint = np.linalg.solve(cof, constrains)
        tag = isSatisfyAllEquation(equationList,intersectPoint)
        #print(tag)
        if tag==False:
            print(i)
            optimalEquation2Increase = i-1
            break

    for i in range(1,1000):
        constrains = np.array([point1[2]-i, point2[2]])
        intersectPoint = np.linalg.solve(cof, constrains)
        tag = isSatisfyAllEquation(equationList,intersectPoint)
        #print(tag)
        if tag==False:
            print(i)
            optimalEquation1Decrease=i-1
            break

    for i in range(1,1000):
        constrains = np.array([point1[2], point2[2]-i])
        intersectPoint = np.linalg.solve(cof, constrains)
        tag = isSatisfyAllEquation(equationList,intersectPoint)
        #print(tag)
        if tag==False:
            print(i)
            optimalEquation2Decrease = i-1
            break

    print(intersectPoint)
    RangeOfFeasibility={
        'optimalEquation1Increase': optimalEquation1Increase,
        'optimalEquation1Decrease' : optimalEquation1Decrease,
        'optimalEquation2Increase': optimalEquation2Increase,
        'optimalEquation2Decrease': optimalEquation2Decrease
    }
    return RangeOfFeasibility

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
        constr,rangeconstr = getConstrExpr(constr)
        constr = eval(constr)
        solver.Add(constr)
        range_constraint_list.append(rangeconstr)

    ans_map['Constraints'] = constr_input
    print("RANGE CONSTR LIST: ",range_constraint_list)

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
        
        #generic 
        for var in range(1, num_of_var+1):
            print('x[',var,'] = ', x[var].solution_value())
            key = 'x'+str(var)+'_sol_val'
            solution[key] = x[var].solution_value()

        print("offset ", solver.Objective().BestBound())
        
    else:
        print('The problem does not have an optimal solution.')
        solution['exists']="no"

    ans_map['solution'] = solution

    # [END print_solution]
    print("cons",solver.constraints()[2].ub())
    # x=round(x.solution_value())
    constraint_compute=solver.ComputeConstraintActivities()

    #Slack: RHS – LHS value in a  ≤ constraint
    #The amount of resource that is not used
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
    ans_map['solving_time']=solver.wall_time()
    print('Problem solved in %d iterations' % solver.iterations())
    ans_map['iterations']=solver.iterations()
    print('Problem solved in %d branch-and-bound nodes' % solver.nodes())
    ans_map['branch_bound_nodes']=solver.nodes()
    # [END advanced]

    ############# slack/surplus  --------------------------------

    #print("compute slack: ",compute_slack)
    track_index = 1
    
    constraint_details={}
    binding_constraint_list = []
    non_binding_constraint_list = []
       
    build_constraint_list = []
    for val in compute_slack:
        print("*********G: ")
        build_constraint = ""
        for var in range(1, num_of_var+1):
            coefficient = int(val['constraint'].GetCoefficient(x[var]))
            if coefficient!=1:
                build_constraint += str(coefficient)+"x"+str(var)
            else:
                build_constraint += "x"+str(var)
                
            if var!=num_of_var:
                build_constraint+=" + "
        
        print(build_constraint)
        build_constraint_list.append(build_constraint)
            
        
        constraint_map={}
        if val['slack'] ==0.0:
            print("This constraint ", str(build_constraint)," is a binding constraint & has a direct impact on the optimal solution")
            constraint_map['binding'] = 1
            #binding_constraint_list.append(build_constraint)
            #get_constraint = val['constraint'].GetCoefficient(x[1]),"* x1
            #shadow price
            print("shadow price is ", val['constraint'].dual_value())
            constraint_map['shadow_price'] = val['constraint'].dual_value()
        else:
            print("This constraint ",str(build_constraint)," is a non-binding constraint & has no direct impact on the optimal solution")
            constraint_map['binding'] = 0
            #dual
            print("dual ", val['constraint'].dual_value())
            constraint_map['dual']=val['constraint'].dual_value()
        
            if  val['slack'] >0.0:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()-val['slack'])
                #ans_map[]
                constraint_map['total_limit']=val['constraint'].ub()
                constraint_map['used']=val['constraint'].ub()-val['slack']
            else:
                print("The total limit is not used yet. It had total limit of", val['constraint'].ub(), "but used only",  val['constraint'].ub()+val['slack'])
        
        constraint_details[build_constraint]=constraint_map
        
    #print("the reduced cost of x1 is", x[1].reduced_cost(), x[2].basis_status())
    #print("the reduced cost of y is", y.reduced_cost(), x.basis_status())
    ans_map['constraint_details']=constraint_details
    
    #range_constraint_list
    arg_optimality = []
    for key in ans_map['constraint_details']:
        if constraint_details[key]['binding']==1:
            for rcl in range(len(build_constraint_list)):
                if build_constraint_list[rcl]==key:
                    arg_optimality.append(range_constraint_list[rcl])
    
    arg_optimality.append(splitted_command[1])
                    
    print("arg optimality: ",arg_optimality)
            
    #print(getRangeOfOptimality(arg_optimality))
  
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
       ans=json.dumps(ans, indent=1)
       #return ans
    return render_template(
        "form.html",
        result=ans
    )  