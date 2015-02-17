# Userspace tools

These tools are provided to ease human interaction
with the OpenFlow core for debugging purposes.
Be aware that they are not thoroughly tested and read
them carefully before execution.

####clear_cam.py
The clear_cam.py script takes the device node of a NetFPGA card as parameter and deletes
all contents from the CAM.

 ---
####tuple_calculator.py
The tuple_calculator.py script opens an interactive shell which provides functions to cre-
ate tuple bitstrings and hash these according to the same polynomial as the lookup engine. It
also is able to generate shell commands to write tuples to the CAM of a connected NetFPGA
card.

 ---
####cam_switch.py
The cam_switch.py script takes as arguments a list of the NetFPGA’s virtual interfaces. It
generates CAM entries for packets it receives which are broadcasted if it does not know the
destination of a packet. The MAC addresses which are used to forward packets to the correct
port are hardcoded in the script.