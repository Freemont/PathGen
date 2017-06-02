# PathGeneration
3rd degree spline curve generator, useable for generating autonomous robot paths for FTC or FRC,
though it can feasibly used for any autonomous path generation with a little
know how.
# Using the Generator
There are two main classes, a GUI and a non GUI. The non GUI can be found
in PathGeneration.java. Useage is overly simple.
## Waypoints
Waypoints can be added to the control Array List with a few parameters-X position, Y position, and angle. *_Units do not matter as long as your
units are consistent across every measurement_*. The angle measurement reflects the heading the robot should
have at the given waypoint. Sample control polygon"
```java
        ArrayList<Waypoint> ControlPolygon = new ArrayList();
        ControlPolygon.add(new Waypoint(0,67,0));
        ControlPolygon.add(new Waypoint(45,60,-Math.PI/4));
        ControlPolygon.add(new Waypoint(60,40,-Math.PI/2));
        ControlPolygon.add(new Waypoint(60,10,-Math.PI / 2));
 ```
        
## Generating a Trajectory from a set of Waypoints
Simply create a new trajectory and then run the Generate() function.
Each trajectory can be written to  a text file for robot use with toTextFile();
```java
Trajectory t1 = new Trajectory(wheelbasewidth,WaypointsArrayList,MaxVel,MaxAccel,"Filename",timedelta);
        t1.Generate();
        t1.toTextFile();
```
#### Control Variables
WheelbaseWidth - the width of your robot drivetrain (again, units dont matter- consistency does)  
WaypointsArrayList - ArrayList containing the needed Waypoints  .
MaxVel - The maximum velocity the**CENTER** of the robot will move at - leave yourself some margin as 
the left and right wheels can independtly move faster than the set MaxVel.
MaxAccel - The maximum velocity the**CENTER** of the robot will accelerate at to acheive the MaxVel.  
Filename - The name of the text file to be written to. (include .txt)  
Time Delta - How often readouts will occur in the written path. 0.1 = readouts every 0.1 seconds.

# GUI
The GUI facilitates viewing the designed path, but forewarning:  
**IF THE PATH YOU NEED MUST BE ACCURATE TO WITHIN AN INCH, DO NOT USE THE GUI**.
At the time of writing this, the field image doesn't appear in the correct location so there is a small error in where the waypoint is made versus where was clicked.
(Less than 2 or 3 inches to the best of my knowledge)
## Using the GUI
Again, very straightforward. Click somewhere on the field to point a location, and specify an angle in the box to the right. Click 'Add Waypoint' 
to add the Waypoint to the control polygon. The trajectory will appear once two Waypoints have been added.
To write to a text file, type the target name*without the .txt extension* and click To Text File! To change the tuneables
(width,maxvel,maxaccel,timedelta), look no further than Line 29 of GUI.java


