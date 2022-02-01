<h1> RatSLAM extension using LiDAR data </h1>

The following files were added to the original library:
<pre>
|--graphics
|   |--ground_truth_scene.h
|--ratslam
|   |--ground_truth.cpp
|   |--ground_truth.h
|   |--local_view_match_laser.cpp
|   |--local_view_match_laser.h
|--main_gt.cpp
|--main_lv_laser.cpp</pre>

<h2> ground_truth_scene.h </h2>
Contains the basic methods and fields for the plot of the ground truth. Uses the irrlicht library.

<h2> ground_truth.h </h2>
Declares the methods and fields to allocate the ground truth poses of the robot to be plotted. Most functions are also defined.

<h2> ground_truth.cpp </h2>
Defines a couple functions that were declared in ground_truth.h.

<h2> local_view_match_laser.cpp </h2>
The core of our work. Here were implemented the functions for downsampling and matching of the LiDAR data, i.e., the generation of the view templates and the following comparison with the existing pool of view templates, to perform loop closure when a match is detected.

<h2> local_view_match_laser.h </h2> 
Declaration of methods and fields for local_view_match_laser.cpp.

<h2> main_gt.cpp </h2>
Subscribes to the odometry topic. Add the pose and updates the plot in the callback.

<h2> main_lv_laser.cpp </h2>
Subscribes to the LiDAR data and publishes the view templates. In the callback the data is read and forwarded to the method responsible for the creation and comparison of view templates, whose definition is located in local_view_match.cpp.

<h1> Running the project </h1>

Follow the next steps:
<ol>
  <li> Clone the repo of our code into the catkin workspace, specifically in the src folder</li>
  <li> Install the <a href=https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/>turtlebot3 simulation package</a></li>
  <li> Copy the ratslam_maze folder into .gazebo/models folder </li>
  <li> Run our RatSLAM extension launch file: 
  <pre>roslaunch ratslam_ros myrat.launch </pre></li>
  <li> Launch the simulation on Gazebo with the maze we built: 
  <pre>roslaunch my_simulation my_world.launch</pre></li>
  <li> Either operate the robot using the package installed with the turtlebot3:
  <pre>export TURTLEBOT3_MODEL=waffle
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch</pre>
  Or run a bag file (we can provide one on request per email):
  <pre>rosbag play name_of_the_bag_file.bag</pre>
  </li>
  <li><em> Have fun! </em></li>
</ol>

<br><br>
<h2> *Update 23/08/21: Data fusion implemented. </h2>
Run <pre> roslaunch ratslam_ros ratfusion.launch</pre> (instead of myrat.launch).

The publishing frequecy of the laser data is 5 Hz and it is 12 Hz for the camera, so the logic of the Local view cells is executed by the callback of the image topic iff a new message of the laser topic has already been received and stored as a visual template. 

The bag file used for the tests can be dowloaded <a href=https://we.tl/t-7mCpVcFPyf> here </a>. If the link does not work, please let us know.
