# ME495 Embedded Systems Homework 1
Author: James Oubre
1. Use `ros2 launch turtle_control waypoints_launch.xml ` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "mixer: [{x: 1.7, y: 1.7, theta: 0.0}, {x: 2.3, y: 9.5, theta: 0.0}, {x: 7.3, y: 2.5, theta: 0.0}, {x: 4.3, y: 2.5, theta: 0.0}, {x: 8.3, y: 1.4, theta: 0.0}, {x: 4.3, y: 5.2, theta: 0.0}]"` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty ` starts and stops the turtle
4. Here is a video of the turtle in action

   [turtle_vid.webm](https://user-images.githubusercontent.com/46512429/193491118-21042707-626d-49c2-bbda-dcab09c84b9e.webm)
   
Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi , Rintaroh Shima, Nicolas Morales