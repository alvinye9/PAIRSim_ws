autoware_auto_perception_msgs/TrackedObject object   # the tracked object for this detection
geometry_msgs/Vector3 pathorientation           # points in the direction of the section of path closest to this detection
float32 distdownpath                            # longitudinal distance down the path
float32 latdisttopath                           # lateral distance to the path
float32 velocitydownpath                        # the component of the velocity traveling down the path