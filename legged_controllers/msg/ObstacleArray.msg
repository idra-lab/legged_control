# Every obstacle currently seen. Each message replaces the previous one: an empty array clears them.
std_msgs/Header header  # frame_id must be odom (or empty)
Obstacle[] obstacles
