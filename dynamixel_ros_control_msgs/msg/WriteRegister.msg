uint8 DOUBLE_VALUE=0 # Write a double value
uint8 BOOL_VALUE=1   # Write a bool value
uint8 INT_VALUE=2    # Write a signed integer value

# Set either joint_name or id. If both is set, joint_name is used
string joint_name # Name of target joint 
uint8 id # ID of target dynamixel
string register_name # Register name to be written (see control table)

int32 value_type # 0 double value, 1 bool value, 2 signed integer value
float64 dvalue # Double value
bool bvalue   # Bool value
int32 ivalue  # Signed integer value
