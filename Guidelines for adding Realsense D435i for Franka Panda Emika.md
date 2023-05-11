### Guidelines for adding Realsense D435i for Franka Panda Emika

#### Dependencies 

- Pre-built binary packages

​	`ros-noetic-panda-moveit-config`, `ros-noetic-franka-gazebo`, `ros-noetic-franka-description`, `ros-noetic-realsense2-description`

- Package from Github

 	Clone the package from `git@github.com:ZhouYixuanRobtic/realsense_gazebo_plugin.git`. 

​	 **Notice: ** Compile this package inside a ROS workspace, and re-source the workspace.

#### Modifications

-  Franka Description

  Use commands below to locate the file

  `roscd franka_description && sudo gedit robots/panda/panda.urdf.xacro`.

  Before:

  ```xml
  <?xml version='1.0' encoding='utf-8'?>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">
  
    <xacro:include filename="$(find franka_description)/robots/common/franka_robot.xacro"/>
  
    <xacro:arg name="arm_id" default="panda" />
  
    <xacro:franka_robot arm_id="$(arg arm_id)"
                        joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}">
    </xacro:franka_robot>
  </robot>
  ```

  After:

  ```xml
  <?xml version='1.0' encoding='utf-8'?>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">
  
    <xacro:include filename="$(find franka_description)/robots/common/franka_robot.xacro"/>
  
    <xacro:arg name="arm_id" default="panda" />
  
    <xacro:franka_robot arm_id="$(arg arm_id)"
                        joint_limits="${xacro.load_yaml('$(find franka_description)/robots/panda/joint_limits.yaml')}">
    </xacro:franka_robot>
    <!-- Add mount Board -->
    <xacro:include filename="$(find realsense_gazebo_plugin)/urdf/mount.xacro"/>
    <xacro:mount connected_to="$(arg arm_id)_hand" xyz="0. 0 0" rpy="0 0 0"/>
  	  
    <!-- Add realsense -->
    <xacro:include filename="$(find realsense_gazebo_plugin)/urdf/_d435i.urdf.xacro"/>
  	  
    <xacro:sensor_d435i parent="mount_board" name="D435i_camera" topics_ns="D435i_camera">
  	<origin xyz="0.063 0. 0.042" rpy="0.0 -${pi/2.} 0.0"/>
    </xacro:sensor_d435i>
  </robot>
  ```

- Panda Moveit Config

  Use commands below to locate the file

  `roscd panda_moveit_config && sudo gedit config/panda.srdf.xacro`.

  Before:

  ```xml
  <?xml version="1.0" encoding="UTF-8"?>
  <!--This does not replace URDF, and is not an extension of URDF.
      This is a format for representing semantic information about the robot structure.
      A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
  -->
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">
      <xacro:arg name="arm_id" default="panda" />
  
      <xacro:include filename="$(find panda_moveit_config)/config/arm.xacro" />
      <!-- panda_arm group: eef frame aligned to robot's flanche -->
      <xacro:arm name="$(arg arm_id)_arm" tip_link="$(arg arm_id)_link8"/>
  
      <!--Add the hand if people request it-->
      <xacro:arg name="hand" default="false" />
      <xacro:if value="$(arg hand)">
          <!-- manipulator group: eef frame aligned to hand -->
          <xacro:arm name="$(arg arm_id)_manipulator" tip_link="$(arg arm_id)_hand_tcp" />
          <xacro:include filename="$(find panda_moveit_config)/config/hand.xacro" />
          <xacro:hand />
  
          <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
          <end_effector name="$(arg arm_id)_hand_tcp" parent_link="$(arg arm_id)_hand_tcp" group="$(arg arm_id)_hand" parent_group="$(arg arm_id)_manipulator" />
          <!-- old end-effector -->
          <end_effector name="$(arg arm_id)_hand" parent_link="$(arg arm_id)_link8" group="$(arg arm_id)_hand" parent_group="$(arg arm_id)_arm" />
          
      </xacro:if>
  
  </robot>
  ```

  After:

  ```xml
  <?xml version="1.0" encoding="UTF-8"?>
  <!--This does not replace URDF, and is not an extension of URDF.
      This is a format for representing semantic information about the robot structure.
      A URDF file must exist for this robot as well, where the joints and the links that are referenced are defined
  -->
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="panda">
      <xacro:arg name="arm_id" default="panda" />
  
      <xacro:include filename="$(find panda_moveit_config)/config/arm.xacro" />
      <!-- panda_arm group: eef frame aligned to robot's flanche -->
      <xacro:arm name="$(arg arm_id)_arm" tip_link="$(arg arm_id)_link8"/>
  
      <!--Add the hand if people request it-->
      <xacro:arg name="hand" default="false" />
      <xacro:if value="$(arg hand)">
          <!-- manipulator group: eef frame aligned to hand -->
          <xacro:arm name="$(arg arm_id)_manipulator" tip_link="$(arg arm_id)_hand_tcp" />
          <xacro:include filename="$(find panda_moveit_config)/config/hand.xacro" />
          <xacro:hand />
  
          <!--END EFFECTOR: Purpose: Represent information about an end effector.-->
          <end_effector name="$(arg arm_id)_hand_tcp" parent_link="$(arg arm_id)_hand_tcp" group="$(arg arm_id)_hand" parent_group="$(arg arm_id)_manipulator" />
          <!-- old end-effector -->
          <end_effector name="$(arg arm_id)_hand" parent_link="$(arg arm_id)_link8" group="$(arg arm_id)_hand" parent_group="$(arg arm_id)_arm" />
          
          <disable_collisions link1="D435i_camera_link" link2="mount_board" reason="Adjacent"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_hand" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_hand_sc" reason="Default"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_leftfinger" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link3" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link4" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link4_sc" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link6" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link6_sc" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link7" reason="Never"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_link7_sc" reason="Always"/>
          <disable_collisions link1="D435i_camera_link" link2="panda_rightfinger" reason="Never"/>
          
          <disable_collisions link1="mount_board" link2="panda_hand" reason="Adjacent"/>
          <disable_collisions link1="mount_board" link2="panda_hand_sc" reason="Default"/>
          <disable_collisions link1="mount_board" link2="panda_leftfinger" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link3" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link4" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link4_sc" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link6" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link6_sc" reason="Never"/>
          <disable_collisions link1="mount_board" link2="panda_link7" reason="Default"/>
          <disable_collisions link1="mount_board" link2="panda_link7_sc" reason="Default"/>
          <disable_collisions link1="mount_board" link2="panda_rightfinger" reason="Never"/>
      </xacro:if>
  
  </robot>
  ```

#### Usage

- Moveit 

  `roslaunch panda_moveit_config demo.launch`

- Gazebo

  `roslaunch franka_gazebo panda.launch`



Enjoy!.