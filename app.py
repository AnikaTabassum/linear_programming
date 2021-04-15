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
global ans_map
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

def getRangeOfOptimality(equationList,optimalEquation):
    print('optimalEquation',optimalEquation)
    print('binding List',equationList)
    optimalEquation = optimalEquation.split(' ')
    optimalEquation =optimalEquation[1]
    RangeOfOptimality=[]
    optimalEquation =optimalEquation.replace('x1','x')
    optimalEquation =optimalEquation.replace('x2','y')
    for i in range(len(equationList)):
        #print(realEquationList[i][0],realEquationList[i][0].rfind('x1'))
        if(equationList[i][0].rfind('x2')==-1):
            equationList[i][0]+='+0x2'

        if (equationList[i][0].rfind('x1') == -1):
            equationList[i][0] += '+0x1'
        
        if(equationList[i][1].rfind('x2')==-1):
            equationList[i][1]+='+0x2'

        if (equationList[i][1].rfind('x1') == -1):
            equationList[i][1] += '+0x1'

    for i in range(len(equationList)):
        if len(re.split('[-+]', equationList[i][0]))<3:
            equationList[i][0]+='-0'

        elemnt  = re.split('[-+]', equationList[i][0])

        for ii in range(0,len(elemnt)):
            print(elemnt[ii])
            if(elemnt[ii]=='x1'):
                equationList[i][0] = equationList[i][0].replace('x1', '1x1')
            if (elemnt[ii] == 'x2'):
                equationList[i][0] = equationList[i][0].replace('x2','1x2')
                #print("hey",equationList[i][0])


        equationList[i][0] = equationList[i][0].replace('x2','y')
        equationList[i][0] = equationList[i][0].replace('x1', 'x')
        
        if len(re.split('[-+]', equationList[i][1]))<3:
            equationList[i][1]+='-0'

        elemnt  = re.split('[-+]', equationList[i][1])

        for ii in range(0,len(elemnt)):
            #print(elemnt[ii])
            if(elemnt[ii]=='x1'):
                equationList[i][1] = equationList[i][1].replace('x1', '1x1')
            if (elemnt[ii] == 'x2'):
                equationList[i][1] = equationList[i][1].replace('x2','1x2')
                #print("hey",equationList[i][1])


        equationList[i][1] = equationList[i][1].replace('x2','y')
        equationList[i][1] = equationList[i][1].replace('x1', 'x')
        
    optimalEquation += '-0'
    
    for i in range(len(equationList)):
        tempEquationList = equationList.copy()
        equation1 = tempEquationList[i][0].replace('x2','y')
        equation2 = tempEquationList[i][1].replace('x2','y')
        print(equation1,equation2)
        slope1 = [float(i) for i in re.split('[xy]', equation1)]
        print((slope1[0]),slope1[1])
        slope2 = [float(i) for i in re.split('[xy]', equation2)]
        print((slope2[0]),slope2[1])

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

        RangeOfOptimality.append({
            "minValX": minimumForCx,
            "maxValX": maximumForCx,
            "minValY": minimumForCy,
            "maxValY": maximumForCy,
        })

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

def getRangeOfFeasibility(realEquationList,bindingEquationList):
    
    # print(equationList[0][0])
    # # greater than constrains  == 1
    # # less than constrains  == 0
    # print(equation1)
    RangeOfFeasibility=[]
    for i in range(len(realEquationList)):
        print(realEquationList[i][0],realEquationList[i][0].rfind('x1'))
        if(realEquationList[i][0].rfind('x2')==-1):
            #realEquationList[i][0]='0x2+'+realEquationList[i][0]
            realEquationList[i][0]+='+0x2'
            

        if (realEquationList[i][0].rfind('x1') == -1):
            #realEquationList[i][0] = '0x1+'+realEquationList[i][0]
            realEquationList[i][0]='0x1+'+realEquationList[i][0]
            

    for i in range(len(realEquationList)):
        if len(re.split('[-+]', realEquationList[i][0]))<3:
            realEquationList[i][0]+='-0'

        elemnt  = re.split('[-+]', realEquationList[i][0])

        for ii in range(0,len(elemnt)):
            #print(elemnt[ii])
            if(elemnt[ii]=='x1'):
                realEquationList[i][0] = realEquationList[i][0].replace('x1', '1x1')
            if (elemnt[ii] == 'x2'):
                realEquationList[i][0] = realEquationList[i][0].replace('x2','1x2')
                #print(realEquationList[i][0])


        realEquationList[i][0] = realEquationList[i][0].replace('x2','y')
        realEquationList[i][0] = realEquationList[i][0].replace('x1', 'x')




    for i in range(len(bindingEquationList)):
        #print(realEquationList[i][0],realEquationList[i][0].rfind('x1'))
        if(bindingEquationList[i][0].rfind('x2')==-1):
            #bindingEquationList[i][0]='0x2+'+bindingEquationList[i][0]
            bindingEquationList[i][0]+='+0x2'

        if (bindingEquationList[i][0].rfind('x1') == -1):
            #bindingEquationList[i][0] = '0x1+'+bindingEquationList[i][0]
            bindingEquationList[i][0]='0x1+'+bindingEquationList[i][0]
            
        ########################################### begin
        if(bindingEquationList[i][1].rfind('x2')==-1):
            #bindingEquationList[i][1]='0x2+'+ bindingEquationList[i][1]
            bindingEquationList[i][1]+='+0x2'

        if (bindingEquationList[i][1].rfind('x1') == -1):
            #bindingEquationList[i][1] = '0x1+'+ bindingEquationList[i][1] 
            bindingEquationList[i][1]='0x1+'+bindingEquationList[i][1]
        ################################## finish
        
        
        
    for i in range(len(bindingEquationList)):
        if len(re.split('[-+]', bindingEquationList[i][0]))<3:
            bindingEquationList[i][0]+='-0'

        elemnt  = re.split('[-+]', bindingEquationList[i][0])

        for ii in range(0,len(elemnt)):
            #print(elemnt[ii])
            if(elemnt[ii]=='x1'):
                bindingEquationList[i][0] = bindingEquationList[i][0].replace('x1', '1x1')
            if (elemnt[ii] == 'x2'):
                bindingEquationList[i][0] = bindingEquationList[i][0].replace('x2','1x2')
                #print(realEquationList[i][0])


        bindingEquationList[i][0] = bindingEquationList[i][0].replace('x2','y')
        bindingEquationList[i][0] = bindingEquationList[i][0].replace('x1', 'x')

        ##################### begin
        if len(re.split('[-+]', bindingEquationList[i][1]))<3:
            bindingEquationList[i][1]+='-0'

        elemnt  = re.split('[-+]', bindingEquationList[i][1])

        for ii in range(0,len(elemnt)):
            #print(elemnt[ii])
            if(elemnt[ii]=='x1'):
                bindingEquationList[i][1] = bindingEquationList[i][1].replace('x1', '1x1')
            if (elemnt[ii] == 'x2'):
                bindingEquationList[i][1] = bindingEquationList[i][1].replace('x2','1x2')
                #print(realEquationList[i][0])


        bindingEquationList[i][1] = bindingEquationList[i][1].replace('x2','y')
        bindingEquationList[i][1] = bindingEquationList[i][1].replace('x1', 'x')
        ########################################## finish
        
    #print(realEquationList,len(bindingEquationList))
    for j in range(len(bindingEquationList)):
        equationList = realEquationList.copy()
        tempbindingEquationList = bindingEquationList.copy()
        equation1 = tempbindingEquationList[j][0].replace('x2','y')
        equation1 = equation1.replace('x1','x')
        equation2 = tempbindingEquationList[j][1].replace('x2','y')
        equation2 =  equation2.replace('x1','x')
        #print(equationList,j)
        print(equation1,equation2)
        finfIndexList=[]
        for ii in range(0,len(equationList)):
            if equationList[ii][0]==equation1 or equationList[ii][0]==equation2:
                finfIndexList.append(ii)
        print(finfIndexList)
        print(equationList)
        del equationList[finfIndexList[0]]
        del equationList[finfIndexList[1]-1]

        #print(equationList)

        point1 = [float(i) for i in re.split('[xy]', equation1)]
        point1[2] = - point1[2]
        print(point1)
        point2 = [float(i) for i in re.split('[xy]', equation2)]
        point2[2] = - point2[2]
        print(point2)
        cof = np.array([[point1[0], point1[1]], [point2[0], point2[1]]])

        optimalEquation1Increase,optimalEquation1Decrease,optimalEquation2Increase,optimalEquation2Decrease=0,0,0,0;
        intersectPoint=None
        for i in range(1,1000):
            constrains = np.array([point1[2]+i, point2[2]])
            intersectPoint = np.linalg.solve(cof, constrains)
            tag = isSatisfyAllEquation(equationList,intersectPoint)
            #print(tag)
            if tag==False:
                #print(i)
                optimalEquation1Increase =i-1
                break

        for i in range(1,1000):
            constrains = np.array([point1[2], point2[2]+i])
            intersectPoint = np.linalg.solve(cof, constrains)
            tag = isSatisfyAllEquation(equationList,intersectPoint)
            #print(tag)
            if tag==False:
                #print(i)
                optimalEquation2Increase = i-1
                break

        for i in range(1,1000):
            constrains = np.array([point1[2]-i, point2[2]])
            intersectPoint = np.linalg.solve(cof, constrains)
            tag = isSatisfyAllEquation(equationList,intersectPoint)
            #print(tag)
            if tag==False:
                #print(i)
                optimalEquation1Decrease=i
                break

        for i in range(1,1000):
            constrains = np.array([point1[2], point2[2]-i])
            intersectPoint = np.linalg.solve(cof, constrains)
            tag = isSatisfyAllEquation(equationList,intersectPoint)
            #print(tag)
            if tag==False:
                #print(i)
                optimalEquation2Decrease = i
                break

        RangeOfFeasibility.append({
            'optimalEquation1Increase': optimalEquation1Increase,
            'optimalEquation1Decrease' : optimalEquation1Decrease,
            'optimalEquation2Increase': optimalEquation2Increase,
            'optimalEquation2Decrease': optimalEquation2Decrease
        })
    return RangeOfFeasibility



def main(ObjectiveFunction, ConstraintList):
    # [START solver]
    # Create the linear solver with the GLOP backend.
    solver = pywraplp.Solver.CreateSolver('GLOP')
    # [END solver]
    actual_binding_containts=[]

    # [START variables]
    infinity = solver.infinity()
    
    # Define the decision variables
    #command = "Maximize 8x1+5x2"
    #command = input("Enter command\n")
    command = ObjectiveFunction
    splitted_command = command.split()

    ans_map = {}
    ans_map['Solver_Result'] = "Solver Result"
    ans_map['Objective_Function'] = command
    ans_map["obj_func_exp"] = " Need to find the optimal solution for the objective function: "

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
    ans_map['NumOfVar_exp'] = "There are total " + str(solver.NumVariables()) + " decision variables"
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

    new_constr_list = ConstraintList.replace(",",", ")

    ans_map['Constraints'] = new_constr_list
    ans_map['cdh'] = "Constraint Details"
    
    print("RANGE CONSTR LIST: ",range_constraint_list)

    # x >=0
    #solver.Add(x[1] >= 0.0)

    # y >=0
    #solver.Add(x[2] >= 0.0)

    print('Number of constraints =', solver.NumConstraints())
    ans_map['NumOfConstraints'] = solver.NumConstraints()
    ans_map['Constraints_exp'] = "There are total "+str(solver.NumConstraints())+" constraints."

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
        ans_map['solution_exists']= "yes"
        print('Solution:')
        print('Objective value =', solver.Objective().Value())
        ans_map['ObjectiveValue'] = solver.Objective().Value()
        ans_map['ObjectiveValue_exp'] = "The optimal solution for this function is: "

        
        #generic 
        optimal_value_list = []
        for var in range(1, num_of_var+1):
            print('x[',var,'] = ', x[var].solution_value())
            key = 'x'+str(var)+'_sol_val'
            key_exp = str(key)+"_exp"
            ans_map[key] = x[var].solution_value()
            ans_map[key_exp] = "The value of this decision variable in the optimal point."
            optimal_value_list.append(x[var].solution_value())

        print("offset ", solver.Objective().BestBound())
        ans_map["opt_val_list"] = optimal_value_list
        ans_map['opt_val_exp'] = "The corresponding optimal values for each of the variables are: "
        
    else:
        print('The problem does not have an optimal solution.')
        ans_map['solution_exists']="no"

    #ans_map['solution'] = solution

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

    constraint_details_str = ""

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
            constraint_details_str = constraint_details_str + "\n\n\n"
            constraint_exp = str(build_constraint)+" is a binding constraint & has a direct impact on the optimal solution."
            constraint_details_str = constraint_details_str + constraint_exp 
            print(constraint_exp)

            constraint_map['binding'] = 1
            actual_binding_containts.append(build_constraint)
            constraint_map['constraint_exp'] = constraint_exp
            #binding_constraint_list.append(build_constraint)
            #get_constraint = val['constraint'].GetCoefficient(x[1]),"* x1
            shadow_price_exp = " Shadow price for this constraint is "+ str(val['constraint'].dual_value())
            print(shadow_price_exp)

            constraint_details_str = constraint_details_str + shadow_price_exp

            constraint_map['shadow_price'] = val['constraint'].dual_value()
            constraint_map['shadow_price_exp'] = shadow_price_exp
        else:
            
            constraint_details_str = constraint_details_str + ". "

            constraint_exp = str(build_constraint)+" is a non-binding constraint & has no direct impact on the optimal solution. "
            print(constraint_exp)
            constraint_details_str = constraint_details_str + constraint_exp

            constraint_map['binding'] = 0
            dual_exp = "dual "+str(val['constraint'].dual_value())
            print(dual_exp)
            constraint_map['dual']=val['constraint'].dual_value()
            constraint_map['dual_exp']=dual_exp
        
            if  val['slack'] >0.0:
                slack_exp = " The total limit is not used yet. It had total limit of "+ str(val['constraint'].ub()) +" but used only " + str(val['constraint'].ub()-val['slack'])
                print(slack_exp)
                constraint_details_str = constraint_details_str + slack_exp

                #ans_map[]
                constraint_map['total_limit']=val['constraint'].ub()
                constraint_map['used']=val['constraint'].ub()-val['slack']
                constraint_map['slack_exp'] = slack_exp
            else:
                slack_exp = " The total limit is not used yet. It had total limit of "+ str(val['constraint'].ub())+ " but used only "+str(val['constraint'].ub()+val['slack'])
                print(slack_exp)
                constraint_details_str = constraint_details_str + slack_exp
                constraint_map['slack_exp'] = slack_exp
        
        constraint_details[build_constraint]=constraint_map
        
    #print("the reduced cost of x1 is", x[1].reduced_cost(), x[2].basis_status())
    #print("the reduced cost of y is", y.reduced_cost(), x.basis_status())
    ans_map['constraint_details']=constraint_details
    ans_map['constraint_details_str'] = constraint_details_str
    
    arg_optimality = []
    for key in ans_map['constraint_details']:
        if constraint_details[key]['binding']==1:
            for rcl in range(len(build_constraint_list)):
                if build_constraint_list[rcl]==key:
                    arg_optimality.append(range_constraint_list[rcl])
    
    arg_optimality.append(splitted_command[1])
                    
    print("arg optimality: ",arg_optimality)
    print("arg optimality: ",arg_optimality)
    if(solver.NumVariables()==2):
            
        equationList=[]
        print(type(ConstraintList))
        ConstraintList = ConstraintList.split(',')
        ConstraintList[0] = ConstraintList[0].replace(' ','')
        
        for index in range(len(ConstraintList)):
            bindTag=False
            temp = ''
            if ConstraintList[index].find('<=')!=-1: 
                temp=ConstraintList[index].replace('<=','-')
                bindTag=False
            elif ConstraintList[index].find('>=0')!=-1: 
                temp = ConstraintList[index].replace('>=0','')
                bindTag=True
            elif ConstraintList[index].find('>=')!=-1: 
                temp = ConstraintList[index].replace('>=','-')
                bindTag=True
            
            
            for secondIndex in range(len(actual_binding_containts)):
                actual_binding_containts[secondIndex] = actual_binding_containts[secondIndex].replace(' ','')
                print(ConstraintList[index],actual_binding_containts[secondIndex])
                if ConstraintList[index].find(actual_binding_containts[secondIndex])!=-1:
                    actual_binding_containts[secondIndex]=temp
                    break
            
            if bindTag : equationList.append([temp,1])
            else:  equationList.append([temp,0])
    
        print("Range Of Optimality")
        
        print("Range Of Optimality")
        optimalityRange = getRangeOfOptimality([actual_binding_containts.copy()],command)
        print(optimalityRange)
        print()

        ans_map['RangeOfOptimality_minValX1']=optimalityRange[0]['minValX']
        ans_map['RangeOfOptimality_maxValX1']=optimalityRange[0]['maxValX']
        ans_map['RangeOfOptimality_minValX2']=optimalityRange[0]['minValY']
        ans_map['RangeOfOptimality_maxValX2']=optimalityRange[0]['maxValY']

        ans_map["roo_string_1"] = "Range of optimality for X1 is "+str(optimalityRange[0]['minValX'])+" to "+str(optimalityRange[0]['maxValX'])
        ans_map["roo_string_2"] = "Range of optimality for X2 is "+str(optimalityRange[0]['minValY'])+" to "+str(optimalityRange[0]['maxValY'])
        ans_map["RangeOfOptimality_exp"] = "The coeffiecient of the decision variables can be changed between this range keeping the optimal value unchanged."


        print("Range Of Feasibility")
        #equationList=[["2x1+x2-1000",0],["3x1+4x2-2400",0],["x1+x2-700",0],["x1-x2-350",0],["x1",1],["x2",1]]
        print(actual_binding_containts,equationList)
        feasibilityRange = getRangeOfFeasibility(equationList.copy(),[actual_binding_containts.copy()])
        print(feasibilityRange)
        
        ans_map['RangeOfFeasibility_optimalEquation1Increase'] = feasibilityRange[0]['optimalEquation1Increase']
        ans_map['RangeOfFeasibility_optimalEquation1Decrease'] = feasibilityRange[0]['optimalEquation1Decrease']
        ans_map['RangeOfFeasibility_optimalEquation2Increase'] = feasibilityRange[0]['optimalEquation2Increase']
        ans_map['RangeOfFeasibility_optimalEquation2Decrease'] = feasibilityRange[0]['optimalEquation2Decrease']

        ans_map['rof_string_1'] = "The binding constraint 1 can be increased "+str(feasibilityRange[0]['optimalEquation1Increase'])+" units and decreased "+str(feasibilityRange[0]['optimalEquation1Decrease'])+" units"
        ans_map['rof_string_2'] = "The binding constraint 2 can be increased "+str(feasibilityRange[0]['optimalEquation2Increase'])+" units and decreased "+str(feasibilityRange[0]['optimalEquation2Decrease'])+" units"
        ans_map['RangeOfFeasibility_exp'] = "Range of feasibility : Determines the change range of the RHS of a constraint."
    

    
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
       #ans=json.dumps(ans, indent=1)
       #return ans
    return render_template(
        "form.html",
        result=ans
    ) 