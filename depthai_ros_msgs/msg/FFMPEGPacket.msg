std_msgs/Header header
int32 width       # original image width
int32 height      # original image height
string encoding	  # encoding used
uint64 pts        # packet pts
uint8  flags      # packet flags
bool is_bigendian # true if machine stores in big endian format
uint8[] data      # ffmpeg compressed payload
