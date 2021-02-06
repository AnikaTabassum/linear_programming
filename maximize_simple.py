import pulp as p

# Generate a New LP Maximization Problem:
Lp_prob2 = p.LpProblem('Activity-Analysis_2', p.LpMaximize)
  
# Generate Problem Variables (>= 0): 
c = p.LpVariable("c", lowBound = 0)
t = p.LpVariable("t", lowBound = 0)
# Create Objective Function:
Lp_prob2 += 45 * c + 80 * t #+ 110 * d + 55 * b    
  
# Set Up the Constraints: 
Lp_prob2 += 5 * c + 20 * t <= 400
Lp_prob2 += 10 * c + 15 * t <= 450
# Show the problem:
print(Lp_prob2) # note that it's shown in alphabetical order

# Solve the Problem:
status = Lp_prob2.solve()
print(p.LpStatus[status])   # Display Solution Status

# Printing the final solution 
print(p.value(c), p.value(t), p.value(Lp_prob2.objective))
# Result: 24.0 14.0 2200.0