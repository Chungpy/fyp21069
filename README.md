# fyp21069
Pick and Place Game with Mobile App using Robotic Arm


Installation


Install OpenCV Manager on the smartphone

Download the source code from https://github.com/Chungpy/fyp21069

Download OpenCV 3.4.14

Open Android Studio and import the OpenCV library as module

Set the OpenCV library in the dependency list to utilize the functions provided by OpenCV

Then connect the smartphone to the computer and run the app


The Arduino code is not prepared in this project. It is previous work by others. The Arduino code can be found at https://github.com/hkucs-makerlab/robotArm  


1	Controls

The buttons functions are as follows:

“Open” button: the app will establish the socket connection with the Arduino board.

“Close” button: the app will close the socket connection with the Arduino board.

“Steppers” switch button: the app will turn on or off the stepping motors.

“Fan” switch button: the app will turn on or off the fan.

“Gripper Open” / “Gripper Close” buttons: the app will turn on or off the vacuum to draw out the air inside the tube to pick or place objects.
The number picker under “Gripper Open” / “Gripper Close” buttons: the app will set the value of the vacuum.

Positions buttons (“Home” / “Transition Flipper” / “Transition Chessboard”): the app will control the robotic arm to move to the original position when powered on, the transition position above the flipper and the transition position above the chessboard respectively.

Movements buttons (“X+” / “+” / “X-” / “-” / “Y+” / “+” / “Y-” / “-” / “Z+” / “+” / “Z-” / “-”): the app will control the robotic arm to move in the corresponding direction with short or long distance. For X-axis, “X+” moves the robotic arm to its right side for a long distance, “X-” moves the robotic arm to its left side for a long distance. “+” moves the robotic arm to its right side for a short distance, “-” moves the robotic arm to its left side for a short distance. For Y-axis, “+” means forward and “-” means backward. For Z-axis, “+” means upward and “-” means downward.

Calibration buttons:

“Calibration i” button: the app will record the current position of the robotic arm and link it to the grid index i. For example, “Calibration 0” means recording the top right grid according to the view of the robotic arm. From i=0 to i=7, they are the grid from top right grid to top left grid. From i=7 to i=14, they are the grid from top left grid to bottom left grid. In other words, from i=0 to i=14, the positions recorded forms a rotated “L” shape containing the upper side grids and left side grids.

“Flipper In” / “Flipper Out” buttons: the app will record the positions of the entrance and exit of the flipper which are the positions that the robotic arm place down the not yet flipped chess and pick up the flipped chess.

“New Chess” button: the app will record the position that the robotic arm picks up a new chess to place on the chessboard in the robotic arm’s turn.

“Automatic” switch button: the app will turn on the automatic mode if switched on and control the robotic arm automatically.

“AI First” switch button: the app will switch the first turn player to the robotic arm allowing the robotic arm to make a move first if switched on. This switch button only works before switching on the “Automatic” switch button.

 
2	Instructions / Steps

1.	Start the app and turn on the robotic arm
2.	Turn on Bluetooth and press “Open” button to connect to the Arduino board of the robotic arm
3.	Hold the robotic arm with the front arm in horizontal position and the back arm in vertical position as shown in figure 15
4.	Press the “Steppers” switch button to turn on the stepping motors of the robotic arm. Then the robotic arm should hold still at this position
5.	Set up the smartphone stand, chessboard and chess
6.	Check if the chesses are detected and are all the corners detected and no light reflection is affecting the detection
7.	Press the movement buttons to move the end point of the robotic arm to the top right grid from the view of the robotic arm
8.	Press the “Calibration i” button to record the grid position
9.	Repeat step 7 and 8 until grids from top right to top left and from top left to bottom left are recorded (the order must follow from top right to top left to bottom left with total of 15 positions to record)
10.	Move the end point of the robotic arm to the entrance of the flipper
11.	Press “Flipper In” button to record the position
12.	Move the end point of the robotic arm to the exit of the flipper
13.	Press “Flipper Out” button to record the position
14.	Move the end point of the robotic arm to the position of picking a new chess
15.	Press “New Chess” button to record the position
16.	Press “Transition Flipper” to check if the robotic arm moves above the exit of the flipper
17.	Press “Transition Chessboard” to check if the robotic arm moves to the center of the chessboard
18. Press “Transition Flipper” to move the robotic arm to the standby position
19.	Turn on the “AI First” switch button if the player wants to play second
20.	Turn on the “Automatic” switch button to start the automatic mode
21.	The player can now play with the robotic arm
22.	Every time the robotic arm places a new chess, the player needs to refill the chess at the “new chess” position where the player recorded it
