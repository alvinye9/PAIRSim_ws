# DEPRECATED use TrackedObjectWithPath
autoware_auto_perception_msgs/BoundingBox box   # the bounding box for this detection
geometry_msgs/Vector3 pathorientation           # points in the direction of the section of path closest to this detection
float32 distdownpath                            # longitudinal distance down the path
float32 latdisttopath                           # lateral distance to the path
