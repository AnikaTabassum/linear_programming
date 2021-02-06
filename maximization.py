import pulp as p
# Set Up a LP Maximization Problem:
Lp_prob = p.LpProblem('Activity-Analysis_1', p.LpMaximize) # Here we named the Problem "Acitity-Analysis_1".
  
# Set Up Problem Variables: 
c = p.LpVariable("c", lowBound = 0) # "c" for chair
t = p.LpVariable("t", lowBound = 0) # "t" for table
d = p.LpVariable("d", lowBound = 0) # "d" for desk
b = p.LpVariable("b", lowBound = 0) # "b" for bookcase

# Create Objective Function:
Lp_prob += 45 * c + 80 * t + 110 * d + 55 * b    
  
# Create Constraints: 
Lp_prob += 5 * c + 20 * t + 15 * d + 22 * b <= 20000
Lp_prob += 10 * c + 15 * t + 25 * d + 20 * b <= 4000
Lp_prob += 3 * c + 8 * t + 15 * d + 10 * b <= 2000
Lp_prob += 4 * c + 20 * d <= 3000
Lp_prob += 20 * b <= 500

# Show the problem:
print(Lp_prob) # note that it's shown in alphabetical order