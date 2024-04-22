| Deliverable | Due Date              |
|---------------|----------------------------------------------------------------------------|
| Race Day | Saturday, May 11th 11AM - 2PM EST |
| Code Pushed to Github  | Saturday, May 11th 1PM EST |
| Briefing (15 min presentation + 5 min Q&A) OR Report ([github pages](https://github.mit.edu/rss/website2021)) | Monday, May 13th at 1:00PM EST (NB: deadline is for briefing slides, briefings are from 3-5pm) |
| [Team Member Assessment](https://forms.gle/z4t7jNufTrGH2JX58)  | Monday, May 13th at 11:59PM EST |

# Final Challenge 2024


## Introduction

Congratulations on completing the six labs of RSS! 

This semester, you've learned how to implement real-time robotics software on a widely-used software framework (ROS). You know how to read sensor data (LIDAR, Camera, Odometry) and convert it into a useful representation of the world (homography, localization). You've written algorithms which make plans over the world's state (parking, line following, path planning) and couple with controllers (PD control, pure pursuit) to accomplish tasks. 

Now, your team will synthesize all that you've learned to design a competitive entry for the *2024 RSS Final Challenge*! 

<img src="media/Lakitu_750px.png" width="300"/>

### 3, 2, 1, GO!

You have been perfecting your racecar for the last 3 months. Now it's time to test your racing skills! You will be tested on two tracks: Mario Circuit and Luigi’s Mansion. These two circuits will test you on your speed, handling, and planning. 
- On Mario Circuit, you will try to beat other racers by going as fast as you can without falling off the track
- In Luigi’s Mansion, you will have to navigate a dynamic  environment and collect and deliver special objects

Luckily, through RSS you have gained all the skills you need to make your racecar win the Grand Prix!

## Grading

| Deliverable  Grade | Weighting             |
|---------------|----------------------------------------------------------------------------|
| Part A: Final Race (out of 100)  | 35% |
| Part B: City Driving  | 25% |
| Briefing OR Report Grade (out of 10) | 40% |

### Part A: Final Race (Mario Circuit)
Part A is worth 35% of your Final Challenge technical grade. Your grade will be calculated based on the time your car takes to drive around the track (`best_race_split`, in seconds) as follows:

  `Part A grade = min(100 + (50 - best_race_split), 110)  - penalties`

Where `penalties` is calculated as follows:

  `penalties = 15 * num_collisions + 5 * num_lane_line_breaches + 5 * num_long_breaches`
  
And `num_lane_line_breaches` is the number of times the car drives outside of either lane line, and `num_long_breaches` is the number of times the car has driven outside of its lane and stayed outside of the lane for greater than 3 seconds.

As you can see from this grading scheme, it is possible to receive bonus points for a very fast and precise solution. The **maximum speed of your car should be capped at 4 m/s**; you should be able to get full points (with bonus!) with a good controller. You should, above all, prioritize avoiding collisions, and if your car leaves its lane, it should quickly recover. More information about race day can be found below in this handout.

### Part B: City Driving (Luigi’s Mansion)

Part B is worth 25% of your Final Challenge technical grade. You get 3 attempts and your grade is based on your best attempt out of 3. Your grade will be calculated based on  completion of the course and the number of penalties you incur as follows. 

`Part B grade = location_score - penalties`

Location Scoring:

You will receive 40 points for reaching the first location, 30 points for the second location, and 30 points for the third. At each location, you must stop long enough for a TA to place a shell on your robot (5 seconds) or else you will not receive credit for reaching the location. If you return to the starting location after acquiring each of the shells, then you will receive a bonus 20 points.

`location_score = 40*L1 + 30*L2 + 30*L3 + 20*(L1^L2^L3^Start)`

Formula for Penalities: 

`penalties =  min(5 * traffic_infractions, 30) + 10 * manual_assist + min(seconds_over_baseline, 30)`

`traffic_infractions` is the number of times the car passes a stop sign without coming to a full stop, stops at a non-stop sign, ignores a traffic light, cross over the central lane line, or hits a pedestrian. The maximum penalty you can receive for traffic infractions is 30 points. The `manual_assist` is the number of maneuvers (counted individually for turning a corner, stopping at a stop sign, resetting a car, etc.) that required manual teleop intervention. `seconds_over_baseline` will be calculated as the difference between a TA provided baseline to reach all 3 of the locations and your solution, with a maximum penalty of 30 points. The formula for calculating score and penalty values may change for fairness (Penalties may be decreased in severity for a clearly functioning solution, for example).


## Briefing or Report

**Your team will choose between completing a final briefing or report (you do not need to complete both).**

### Briefing Evaluation (see [technical briefing rubric](https://docs.google.com/document/d/1NmqQP7n1omI9bIshF1Y-MP70gfDkgEeoMjpWv8hjfsY/edit?usp=sharing) for grading details)
When grading the Technical approach and Experimental evaluation portions of your briefing, we will be looking specifically for **illustrative videos of your car following the track lane and as well as executing city driving maneuvers.** Specifically, we would like videos highlighting:
- Visualization of lane / marker tracking and stable drive control within a lane
- Stopping at stop signs and making turns through the course
- Recovery of your car if it is outside of its assigned track lane

### Report Evaluation (see [technical report rubric](https://docs.google.com/document/d/1B6l7vKJFN3CPPcMn8cKKArHUU_Bq_YUZ5KxKoP6qMk0/edit?usp=sharing) for grading details)
When grading the Technical approach and Experimental evaluation portions of your report, we will be looking specifically for the following items:

- **Numerical evidence that your algorithms work in the form of charts/data**
  - Numerical evaluation of the success of your lane tracking + following
    - Make sure to mention your method for finding the lane and tuning the controller to closely track it.
  - Numerical evidence evaluating the success of your city driving algorithm
    - Make sure to mention your method for recognizing stop signs and traffic lights, along with executing u-turns and conditional logic


## Part A: Final Race

### Environment and Task

The Final Race will take place on the entire Johnson track loop. This is a standard-size 200m track. Cars may be assigned to follow any of the track's six lanes and will be informed of their lane assignment the morning of the race. Lanes are numbered from left to right as shown in the image below.


<!-- <img src="media/final_race.PNG" width="300" /> -->
<img src="media/start_area.jpg" width="400"/>

Your car's task is to complete the 200-meter loop around the track as fast as possible, while staying in your assigned lane. Any kind of collision (with another car or with something in Johnson) will be penalized heavily. You should have some kind of safety controller running on your car, but be careful that this doesn't stop your car if there is another car driving next to it on the track!

### Race Day
On race day, multiple teams will set up on the track at the same time. A TA will give the start signal, at which point the race begins! You must have a member of your team closely follow your car along the track with the controller ready to take manual control at any moment (yes, a great opportunity to exercise). Your car's split will be recorded at the finish line, and TAs will also be stationed at various points along the track recording lane breaches, if they occur (but hopefully no collisions). Each team will have the opportunity to race **three** times, and we will take the best score.

### Tips

Here are some things you may consider in developing your approach:

- How can you reliably segment the lane lines?
- How can you obtain information about the lane lines in the world frame?
- How can you detect if the car has drifted into a neighboring lane?

Please note that Hough Transforms will very likely be useful; helpful resources are [here](https://towardsdatascience.com/lines-detection-with-hough-transform-84020b3b1549) and here(https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html).

## Part B: City Driving

### Environment and Task

The City Driving challenge will take place in "Luigi's Mansion" (stata basement). He is a big fan of car-centric infrastructure, so you will find stop signs, traffic lights, lanes, and pedestrian crossings in his basement.

Your goal, after finishing the race successfully, is to drive through the course to 3 TA selected locations to pick up shells**TM** while following the rules of the road. Below is a representative map of Luigi's mansion and 3 goal locations (the stars). The exact configuration of the pickup locations and stop signs is kept a secret until Race Day, however the traffic lights or pedestrian crossings will not move.

Luigi, in his infinite wisdom, has already created a ~ machine learning ~ based stop sign detector for you (located in /city_driving)! It not only tells you if there's a stop sign, but where in your image the stop sign is (nifty!). If you don't use it, Luigi will be mad that their hard work went to waste, but you are free to modify the code for the detector and add higher level logic to take advantage of it.

Here are the details of the challenge:

* You will be given 3 locations on the stata basement map (TA's will click 3 points in rviz briefly before you head off)
  * You must stay on the right side road in whichever direction you are headed
    * The road will have a centerline
* You must "park" (stop for 5 seconds) next to the designated point in order to receive a shell
* If you return to the start with all of the shells, you'll receive bonus stars

Things to note: 
* You will also encounter stop signs at random places throughout the course, where your robot must come to a FULL STOP
* Any 3 points on the map along the orange line can be chosen, so you should test your ability to navigate between any two points.

<img src="media/Final Challenge City Driving Map.png" width="400"/>

### Tips

The City Driving Challenge is meant to be open-ended. You should make use of whatever techniques you believe will best solve the problem.

Here are some things you may consider in developing your approach:

- How do you implement the high-level logic that puts together localization, path-planning, stop sign detection, and collision avoidance?
  - Perhaps a state machine would be helpful--how would you connect the different tasks?
- How should the speed of your car be adjusted as you detect a stop sign, decide it is close enough, and turn corners?
- The stop sign detector is good, but not perfect. How might you account for this for robust city navigation?

As always, your safety controller should be turned on for this portion of the Final Challenge as well.


### Race Day!


## General Notes

### Structuring your code

The final challenge is the most open-ended assignment in RSS, and comes with minimal starter code. We suggest referring to previous labs for good practices in structuring your solution:
- Start by defining what nodes you will create, what they will subscribe to, and what they will publish. From this plan, write skeleton files, then split up the work of populating them with working code.
- Make useful launch and parameter files. Refer to previous labs for syntax.

### Leveraging previous labs

You are encouraged to build your solution on code written in previous labs! If you are using your old homography solution, it's a good idea to verify its accuracy. 

## FAQ

### Part A: Final Race

*Do we need to design a safety controller for this challenge?* 
* You should run some kind of safety controller during the challenge, but don't need to spend a lot of time adapting it to the race setting. The easiest way to keep the race collision-free will be for each team to design a robust lane-following solution and remain in-lane. Note: some teams showed solutions in Lab 3 that considered a fixed angle range in front of a car only when deciding when to stop the car. **You should make sure that cars racing alongside yours will not wrongly trigger your safety controller, especially when turning bends in the track!** Consider testing with objects in adjacent lanes.

*Will we be penalized if another car comes into our lane and we crash?*
* No. If you stay in your lane, you will not be considered at fault for any collision. We will give every team the opportunity to record three interference-free lap times on race day.

*Doesn't the car in the outside lane have to travel farther?*
* We will stagger the starting locations so every car travels the same distance. You should be prepared to race in any lane.

### Part B: City Driving Challenge

*How far should the car stop before the stop lights and stop signs?*
* The front of your car must stop between .5-1 meters in front of the stop lights or stop signs to receive credit for the stop.

*How long does the car need to stop for in front of stop lights and stop signs?*
* There is no time requirement, but your car must come to a **full stop** (no California stops!).
