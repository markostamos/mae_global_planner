# mae-global-planner
Ros planner package for the efficient visit of N targets using both distributed and global optimization methods.


# Distributed optimization
Each of the k Agent solves the Subtour Traveling Salesman problem of N/k targets using a genetic algorithm with the following operators 
  - Tournament Selection.
  - Mutation.
  - 2-opt Heuristic.
  - Elitisism.
  - Social disaster.
  
 
  <img src="https://user-images.githubusercontent.com/42519053/188947744-f1400478-d960-4393-a887-d503250e20e3.png" alt="Local Plans" width="500"/>
 
 # Global optimization 
 After an initial plan has been created it is optimized by another evolution algorithm that consists of
  - Crossover 
  - 2-opt Heuristic.
  - Extended 2-opt Heuristic.
  
 <img src="https://user-images.githubusercontent.com/42519053/188947793-367690ed-0e84-4106-a8d7-8bc496b855ac.png" alt="Global Plan" width="500"/>


 # Reference
 Giardini, G. and Kalmar-Nagy, T., 2007. Genetic Algorithm for Multi-Agent Space Exploration.
 
 Available at: <https://www.researchgate.net/publication/268570527_Genetic_Algorithm_for_Multi-Agent_Space_Exploration>
