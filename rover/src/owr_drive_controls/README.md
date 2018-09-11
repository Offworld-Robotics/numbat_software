# Owr_Drive_Controls

This package provides core driving functionality for the NUMBAT and BLUEtongue Mar's Rovers.

## Nodes

- `owr_drive_board` - module for communicating with the original PIC based PCB for BLUEtongue, as well as its replacement arduino. 
Currently provides functionality for the controlling the NUMBAT rover's arm. 

- `cmd_vel_2_joints` - converts the `linear` component of a ROS `Twist` message into movements of each of the individual joints in the drive system.
Currently provides two modes of steering for NUMBAT.

- `cmd_vel_conversion` - converts a ROS `Twist` message providing a linear velocity on the X axis and a rotation around the Z axis into the format used by `owr_joint_controls`.