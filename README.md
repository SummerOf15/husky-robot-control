# husky-robot-control
some control experiments on husky robot

## ROS errors

system environment: ROS melodic and ubuntu 18.04

### Gazebo error 255 

- Q:[gazebo-2] process has died [pid 21136, exit code 255, cmd /opt/ros/melodic/…
- killall gzserver killall gzclient

### python 2.7 torch 1.4

- Q: no module named bullitins

  ```bash
  pip2 install future
  ```

  

### catkin_make new package

- ln -s source_directory_package ./src/
- add `CMakeLists.txt` and `package.xml`

### cpp 

- `catkin_make` every time you modified the cpp

### launch file \<param\>

It should be inside of `<node> </node>`

```xml
<node name="husky_highlevel_controller" pkg="husky_highlevel_controller" type="husky_highlevel_controller_node" output="screen">
	<param name="topic" type="str" value="/scan" />
	<param name="queensize" type="int" value="1"/>
</node>
```

