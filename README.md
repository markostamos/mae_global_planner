# mae-global-planner
Ros planner package for the efficient visit of N targets using K agents. 

# Algorithm
Kmeans clustering is used to split the targets between the agents.

Each of the K Agents solves the Traveling Salesman problem using a genetic algorithm with the following operators 
  - Tournament Selection.
  - Mutation.
  - 2-opt Heuristic.
  - Elitisism.
  - Social disaster.

  

# Results

<p float="left">
  
  <img src="https://user-images.githubusercontent.com/42519053/191318340-c194b1b0-6564-4121-a59c-55ae21bf1eec.png" alt="Plan" width="400" height="350"/>


 <img src="https://user-images.githubusercontent.com/42519053/191318398-3633ad8a-1b0e-4bca-b2b3-cc2a16b1061a.png" alt="Plan" width="400"/>

</p>

