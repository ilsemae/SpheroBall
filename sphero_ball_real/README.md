Before running the program, each of the *n* robots must be connected via Bluetooth and calibrated.

To make the necessary services available, after running `roscore`, run

    rosrun sphero_node_real robot_connector.py

and then open a new terminal window. Now for each robot, run

    rosservice call connect_sphero <i>

where *i* is an integer from 1 to *n*. If the service returns false, try connecting again. If the service returns true but the robot's LED is green, you will need to disconnect and try again.

To disconnect a robot from Bluetooth, run

    rosservice call disconnect_sphero <i>

where *i* is the same integer used to connect the robot.

Once all the robots are connected, make sure they are all publishing odometry information by running

    rostopic echo sphero<i>/odom

for each robot *i*. If the echo produces no results, you will need to disconnect and reconnect that robot.

Now all of the robots need to have their headings set. To do this, first run

    rosservice call calibrate_spheros 1

which will turn off the main LED on each robot and turn on the small heading LED in its place. Physically turn each robot so that its heading LED is facing in the positive x-direction. Once all the robots are calibrated, run

    rosservice call calibrate_spheros 0

to get the robots out of calibration mode and back into dancing mode. Then place the leader robots across from the follower robots, as in our simulation, organized by number, left to right. Lastly, run

    roslaunch sphero_ball_real ball.launch n:=<n>

with the appropriate number of robots substituted in after `n:=`. The dance begins at this point. Yay!

