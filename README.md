# RRTx

[RRTx](http://ottelab.com/html_stuff/pdf_files/Otte.Frazzoli.IJRR15.pdf) is an asymptotically optimal sampling-based replanning algorithm for dynamic environments where the obstacles unpredictably appears, disappears, or moves [1]. 

## About Project

To demonstrate the algorithm, in this project, I consider a static environment in which the robot does not know the location of obstacles (but knows the location of itself and the goal). The robot has a 360-degree field of view up to a specific range, and if an obstacle is within that range, the robot can identify it. In this project, the robot starts at the bottom left corner and navigates to the goal, which is at the top right corner. Initially, the robot samples the configuration space for a 
certain period of time (set by the user), and then after sufficient sampling, it navigates to the goal making changes
to the path (i.e., repairing the tree) as necessary when it sees an obstacle. I've placed the obstacles in such a way that the robot repairs the path multiple times before reaching the goal. **Check out the video
[here](https://drive.google.com/open?id=1g_cOHdx0kDleEXW9ISh2A4etzxu8Uugt)**.


**_Note_**: This algorithm was one of the first complex code that I've written, and I wrote it in 2017 for my final project in
Path Planning course for my Master's degree. I've changed my coding style a lot, and this code is long due for an update in this regard. I've made readable to the best of my ability, and I'll rewrite this algorithm when I get some time until then you can keep this implementation as a reference for path planning algorithms. 

## References
[1] RRTX: Asymptotically Optimal Single-Query Sampling-Based Motion Planning with Quick Replanning. Michael Otte and Emilio Frazzoli. The International Journal of Robotics Research. Volume 29, Issue 7. 2016. p. 797-822. 

