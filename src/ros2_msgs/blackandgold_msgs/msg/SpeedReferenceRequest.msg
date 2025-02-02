std_msgs/Header header
float32 desired_speed           # the reference speed
float32 desired_acceleration    # desired minimum acceleration to reach that speed
string origin                   # the origin of this request (used as a unique identifier)
string message                  # the message to display as the reason we are limmiting speed (this can be the same as origin)