# intersection_management

## 4-Way Intersection Scenario: Simulation Videos
Each of the following videos comes from a set of 1000 simulated trials of the 4-way intersection crossing scenario between 4 communicating vehicles. The videos are shown in order of increasing sophistication. For information on the use of this repository, scroll to the bottom.

### Exponential CBF QP Control
In the video below, a relative-degree two exponential CBF with no predictive power is used to perform centralized QP based control. The controller prevents collisions between vehicles, however, due to the lack of forward prediction the vehicles become deadlocked at the center of the intersection. 

https://user-images.githubusercontent.com/67293038/160519184-f2041312-2d1e-4252-82ef-77b108c758c1.mp4

### Future-Focused CBF QP Control
In the video below, a future focused CBF (ff-CBF) is used to perform centralized, quadratic program (QP) based control. Due to the ff-CBF construction, the QP becomes infeasible during virtual barrier violation despite wide physical margin between vehicles, and thus the simulation terminates.

https://user-images.githubusercontent.com/67293038/160519172-8eba91d1-20bf-4f4e-81ff-b823d39b502e.mp4

### Relaxed-Virtual CBF QP Control
In the video below, a relaxed version of the future focused control barrier function QP based controller is used to solve the intersection crossing problem with high empirical success.

https://user-images.githubusercontent.com/67293038/160519126-8371cca7-fa74-42d0-a05b-4b33eeb01599.mp4

The above videos may be viewed on YouTube here:
https://youtu.be/dgMUDRlxLOg <br>
https://youtu.be/sVao3ALdDQg <br>
https://youtu.be/pIRh8zLnUHo

