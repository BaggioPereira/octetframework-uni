Tools and Middleware Project

Video Link:
https://youtu.be/798jppN8knw

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
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/level%20create.JPG "Level Create")

This gets a specific portion of the text file before it begins to create it. It also checks to see if the level data has a 3 lane layout or a single lane layout.
Next I use createLevelArea():
![alt text]()

This uses a switch statement, where it iterates through the section of text file and if a certain character is recognized it will issue certain commands to do.
It will also set the location for the block so it is placed correctly in the world.
For example _ = a floor block, ¦ = an invisible wall block, P = the player block, B = a bouncing block, F = a spinning hinge block.
There is a singular special block created for the origin point of the level which is later used for the constraints.
Each of these characters would create a block to do a specific action whilst other characters would just be ignored and carry on iterating.
Any character not defined in the switch statement, it will still change the location so all other blocks will be placed correctly.
The switch is also where add_rigid_body() is called and assigns what type of object it is once add_part() creates the mesh and attaches a rigid body to it.
add_rigid_body() also sets restictions to the objects for example, the player object can only move along the x and y axis and only rotate in the z axis by default.
These restrictions will change when switching views.

I also created a function that clears and resets the scene and also defines all the materials, mesh sizes, loads any shaders and sounds.
And also it prints out the controls to the console so the player knows the controls.
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/new%20scene.JPG "New Scene")
This piece of code was intented to be used when having multiple levels.
It resets the world to a blank state and then redifines all the materials, mesh sizes, any shaders and sounds.
I have included a shader to create invisible walls by taking an image and a mask and modifying the shader code and images so the colours are ignored and gives a transparency effect.
I needed this due to the camera placement.

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
When in first person perspective, there are added controls so the player can move forwards, backwards, left and right.
In third person perspective(2D mode,traditional playerformer view), the player can only move left and right.
When switching from first to third, the z axis is reset so the player is centered again and when switching back to first, the player will need to move left and right again.
I created it this way so the player will need to use both perspective and cannot cheat by setting it to one perspective and only using it. 
Also in first person perspective, the user can turn around and view the back and move in that direction.
Although the game can be played mostly in first person mode.

Finally in draw_world(), the physics of the game is active and also camera updates and collision checks.
![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/collision%20check.JPG "Collision Checks")
This piece of code does a collision check between the player with the obstacles and plays a sound when they collide. The checks are done by using the enums that were assigned to the objects when created.
This gives a audible indicator to the player when the player object is rather close to an object and needs to move away.    

![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/physics.JPG "Physics")
This piece of code does the physics update.

![alt text](https://github.com/BaggioPereira/octetframework-uni/blob/master/octet/src/examples/tandm_game/Images/camera%20update.JPG "Camera")
This piece of code updates the camera position, it is a modification from helper_fps_controller. It gets the players position and stores it in the 'w' column of a 4x4 matrix which is then used to translate the camera.

Additions I would like:
I would like to change the code slightly so the text file would include more data such as block size and shape, a legend for the level data that the user can define and multiplayer race style setup.
