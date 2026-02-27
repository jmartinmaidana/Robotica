# Each multi-dof joint can specify a transform (up to 6 DOF)
geometry_msgs/Transform transform

# There can be a velocity specified for the origin of the joint 
geometry_msgs/Twist velocity

# There can be an acceleration specified for the origin of the joint 
geometry_msgs/Twist acceleration

builtin_interfaces/Duration time_from_start
