
##########################
Vehicle Configuration
##########################

We list the vehicle config here. Observation Space will be adjusted by these config automatically.
Find more information and in our source code and test scripts!

- :code:`lidar(tuple)`: (laser num, distance, other vehicle info num)
- :code:`rgb_cam(tuple)`: (camera resolution width(int), camera resolution height(int), we use (84, 84) as the default value like what Nature DQN did in Atari.
- :code:`depth_cam(tuple)`: (camera resolution width(int), camera resolution height(int), view_ground(True)). The last option will switch to another type of depth camera,
  and the lane lines will also be captured in it.
- :code:`mini_map(tuple)`: (camera resolution width(int), camera resolution height(int), camera height). The bird-view image can be captured by this camera,
  but we are working on a 2D-top-down-rendering with pygame, which can achieve this soon.
- :code:`show_navi_mark(bool)`: A spinning navigation mark will be shown in the scene
- :code:`increment_steering(bool)`: For keyboard control using. When set to True, the steering angle is determined by the key pressing time.
- :code:`wheel_friction(float)`: Friction coefficient
