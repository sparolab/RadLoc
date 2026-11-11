# The RadLoc descriptor for one radar scan.

std_msgs/Header header

# Mean return per range band, unweighted. Rotation invariant, because the bands
# are averaged over azimuth. Range weighting belongs to the search, not to the
# place, so it is applied when the coarse key is built - see radloc/retrieval.hpp.
float32[] bands
