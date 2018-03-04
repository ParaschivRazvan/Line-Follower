# Line-Follower
My very first line follower

This is an infrared sensors based line follower robot which uses 6 analogic IR sensors to guide itself along a black/white line. The robot  has been built by using a 3D printed model, an Arduino Genuino/Uno board, a breadboard, 2 DC motors, a ball caster and of course the analogic IR sensors. In terms of electronical components I have used and L293D driver, a L7805D voltage stabilizer, 2 push-buttons, one switch, 2 capacitors and 4 AA batteries. The robot is pretty easy to use, since it has 2 buttons on the back: one black button and one red button. The black button starts the callibration process and the red button starts the race mode of the robot(basically gets it going). In terms of coding, besides the basic line follower implementation : motor speed, calibration and the weighted average of the scaled sensors; I have tried to implement a PID controller. At its current state the line follower uses a PD controller.

Link to BOM: https://docs.google.com/spreadsheets/d/1dTVm6qOk78ZboE6h7FOp6h80o8KzOUQlbJ8yJAf9IiY/edit#gid=0
