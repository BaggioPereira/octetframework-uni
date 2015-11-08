Tools and Middleware Project

Mentions: Michael Craddock - He helped me with the physics setup, without his help I wouldnt have been able to do collision checks easily.

This document will detail the my project, showing the code used and how it works.
My project is a platformer where you can change your prespective to get around obstables to reach the goal.
The first step in this project was to read in a text file which contains the data for the level.
This was done using loadtxt():
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/loadtxt.JPG "Load Text File")

This reads in the entire text file and stores it into memory where it will be used later to create the level.
After the text file has be read in, we will build the level.
To build the level, I have used multiple functions so it creates the level correctly.
First I use levelCreate():
![alt text]()

This gets a specific portion of the text file before it begins to create it.
Next I use createLevelArea():
![alt text]()

This uses a switch statement, where it iterates through the section of text file and if a certain character is recognized it will issue certain commands to do.
It will also set the location for the block so it is placed correctly in the world.
For example _ = a floor block, ¦ = an invisible wall block, P = the player block, B = a bouncing block, F = a spinning hinge block.
Each of these characters would create a block to do a specific action whilst other characters would just be ignored and carry on iterating.
Any character not defined in the switch statement, it will still change the location so all other blocks will be placed correctly.
The switch is also where add_rigid_body() is called and assigns what type of object it is once add_part() creates the mesh and attaches a rigid body to it.

I also created a function that clears and resets the scene and also defines all the materials, mesh sizes, loads any shaders and sounds.
And also it prints out the controls to the console so the player knows the controls.
![alt text]()
This piece of code was intented to be used when having multiple levels.
It resets the world to a blank state and then redifines all the materials, mesh sizes, any shaders and sounds.

I have used Xinput to add Xbox controller compatibility in my project.
This is done using XINPUT_STATE getState(), controllerUpdate() and buttonPress().
getState() checks if the controller is connected or disconnected. This then switches the controls to and from the keyboard.
controllerUpdate() is the fucntion that calls getState() and stores the current connection state of the controller.
buttonPress() get the state of the button, if it is pressed or not.
I have also used a function called buttonDelay() which slows down the input fron the controller slightly but is still quite fast when updating.
I will need to look into this and modify this function so the button update is not triggered when a button is pressed.

All the commands for the controls are handled in controls().
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/controls.JPG "Controls")

In this piece of code it uses the state of the controller then only use part of the code depending on the outcome of the state.
It also checks what perspective the player is currently in and modifies the controls to work to correctly.

Finally in draw_world(), the physics of the game is active and also camera updates and collision checks.
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/collision%20check.JPG "Collision Checks")
This piece of code does a collision check between the player with the obstacles and plays a sound when they collide.

![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/physics.JPG "Physics")
This piece of code does the physics update.

![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/camera%20update.JPG "Camera")
This piece of code updates the camera position, it is a modification from helper_fps_controller.
