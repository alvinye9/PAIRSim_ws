# Based on costmap_converter/ObstacleMsg
# Special types:
# Polygon with 1 vertex: Point obstacle (you might also specify a non-zero value for radius)
# Polygon with 2 vertices: Line obstacle
# Polygon with more than 2 vertices: First and last points are assumed to be connected

string name

int64 id

# Obstacle centroid location (x,y,z)
float32[] center

# Specify the radius for circular/point obstacles
float64 radius

# Obstacle footprint (polygon descriptions)
geometry_msgs/Polygon polygon

# Individual orientation (centroid)
geometry_msgs/Quaternion orientation

# Individual velocities (centroid)
geometry_msgs/TwistWithCovariance velocities
