# COMP0127_Robotic_Systems_Engineering-Courseworks
/* Software License Agreement (MIT License)
 *
 *  Copyright (c) 2022, Joseph Rowell
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


Dependencies:
sudo apt install ros-melodic-controller-manager ros-melodic-joint-state-controller ros-melodic-effort-controllers ros-melodic-gazebo-ros-control ros-melodic-joint-trajectory-controller ros-melodic-velocity-controllers ros-melodic-ros-controllers ros-melodic-ros-control

Note:
In case you use the ROS distribution rather than melodic, e.g. noetic, you can change `melodic` in the command to match your distribution accordingly. Please also note that your ROS distribution has to be compatible with your Ubuntu version.
`sudo apt install ros-<distro>-controller-manager ros-<distro>-joint-state-controller ros-<distro>-effort-controllers ros-<distro>-gazebo-ros-control ros-<distro>-joint-trajectory-controller ros-<distro>-velocity-controllers ros-<distro>-ros-controllers`


See question paper and report for questions and answers.  
Ensure to:  
``` catkin_make```  
for cw1q4:  
see cw1/cw1q4/src/cw1q4_node.py   

for cw1q5:
``` roslaunch cw1q5 cw1q5b.launch  ```   
``` roslaunch cw1q5 cw1q5d.launch ```  
for cw2q4; see cw2/cw2q4/src/cw2q4/youbotKineStudent.py
for cw2q6
``` roslaunch cw2q6 cw2q6py.launch ```
