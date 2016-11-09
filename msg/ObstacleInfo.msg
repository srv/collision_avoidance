Header header             # message header

bool detection            # True if obstacle detection


uint32 points             # Number of 3D points of the obstacle (stereo vision only)
uint32 min_points         # Parameter: minimum number of points to be considered obstacle (stereo vision only)

float64 height            # Obstacle height
float64 width             # Obstacle width
float64 size              # Size of the obstacle (m2: width x height)
float64 min_size          # Parameter: minimum size to be considered obstacle

float64 distance          # Minimum distance to the obstacle
