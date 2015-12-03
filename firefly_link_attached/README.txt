Just copy these files to /catkin_ws/src/rotors_simulator/rotors_description/urdf. 
If not renamed, however, they will replace the original ones.
(But if you comment out lines 81-88 from the firefly.xacro file then the link will be removed from the quad).
Note that the joint added (that connect the quad base to the link) is a revolute joint with respect to x-axis.
To make it a ball joint (3 DoFs) I guess we can add 3 joints wrt the 3 axis and create really short links in between them.
(I don't think there's a 3 DoF joint in gazebo.)
More http://wiki.ros.org/urdf/XML/joint and http://wiki.ros.org/urdf/XML/link
