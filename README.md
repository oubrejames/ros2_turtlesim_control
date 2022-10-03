# ME495 Embedded Systems Homework 1
Author: James Oubre
1. Use `ros2 launch turtle_control waypoints_launch.xml ` to run the code
2. The `ros2 service call /load turtle_interfaces/srv/Waypoints "mixer: [{x: 1.7, y: 1.7, theta: 0.0}, {x: 2.3, y: 9.5, theta: 0.0}, {x: 7.3, y: 2.5, theta: 0.0}, {x: 4.3, y: 2.5, theta: 0.0}, {x: 8.3, y: 1.4, theta: 0.0}, {x: 4.3, y: 5.2, theta: 0.0}]"` service loads waypoints for the turtle to follow
3. The `ros2 service call /toggle std_srvs/srv/Empty ` starts and stops the turtle
4. Here is a video of the turtle in action
   `https://raw.githubusercontent.com/ME495-EmbeddedSystems/homework1-oubrejames/main/turtle_vid.webm?token=GHSAT0AAAAAABYS5UVZO2YGA2OK7ZQSOO7YYZ2I7JA`
Worked With: Liz Metzger, Dilan Wijesinghe, Meg Sindelar, Marno Nel, Katie Hughes, Ava Zahedi , Rintaroh Shima, Nicolas Morales