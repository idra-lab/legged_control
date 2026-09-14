# One obstacle seen by perception. The Opti-Pessi controller keeps it out of a disk whose radius and
# assumed maximum speed are fixed per type (opti_pessi_interface/config/task.info: obstacleTypes).

uint8 HUMAN=0
uint8 CAR=1

uint8 type
geometry_msgs/Point position  # centre, in the frame of the enclosing ObstacleArray; z is ignored
