# Mobile-Manipulation
This work was the capstone/final project in the "Modern Robotics: Mechanics, Planning, and Control Specialization" offered on Coursera by Kevin Lynch.

The main aim was to develop a controller for an omnidirectional drive platform with a manipulator attached on top. Robot's main task was to pick and place a cube, whose start and end coordinates, as well as the desired orientation could be adjusted form the "input.csv" excel sheet.  

Link to the final simulation output on youtube https://youtu.be/6A4LB7m7yrs

•	This Code consists of Main files (developed by myself) and supplementary files taken from the MR course library. 

•	To run the code, all coding files should  be in one directory. Now, run the 'RunMe' matlab code.

•	Code output files:  1. error_plot.csv  2. Output.csv  3. TrajectoryGenerator.csv

•	The file produced by the code to be used in V-rep simulation is called 'Output'.  

•	V-rep simulation environment is not added to the repository for copyright reasons.

•	The excel file named 'Input' is used as a control sheet for the input variables, such as initial/final cube configuration, max speed limit for joints and wheel, or the feedback 
gain factors. write the new value of the variable then run the 'RunMe' to see the new results.

•	Limitations on Joint and wheel speeds was implemented in this project.




•One of the required outputs is a control error plot depicted below.

![Error Plot - Best](https://user-images.githubusercontent.com/72637753/111034390-0d2efa80-841e-11eb-82fb-2a392707b7f7.jpg)
