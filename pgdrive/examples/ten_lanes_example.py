"""

This script demonstrates how to build a world with always 10 lanes in each side.
The "map" is set to "SSSSS", which means there will be 5 blocks of straight road.
You can expand the length by adding more blocks such as "SSSSSS" (6 straight road blocks).
You can also use other blocks with 10 lanes, such as "CCC" (3 curve blocks).
Mixed different blocks is also possible, e.g. using "CSCS" (4 blocks with 2 straight and 2 curve blocks).

Changing the "traffic_density" will increase or decrease the number of traffic vehicles.

The "environment_num" controls the number of different scenes. Since we already specify the block sequences,
different scene will only have different traffic, road curvature, etc.

Remember to press H to see help message if you wish to operate the vehicle with keyboard when setting
"manual_control = True".

Note: This script require rendering, please following the installation instruction to setup a proper
environment that allows popping up an window.
"""

from pgdrive import PGDriveEnv

if __name__ == "__main__":
    env = PGDriveEnv(
        dict(
            # ===== Environmental settings =====
            traffic_density=0.2,
            environment_num=10,
            map_config=dict(

                # We default to use "block_num" generation method instead of this "block_sequence" method.
                # In "block_sequence" method, we can determine a sequence of blocks (with their unique names such as
                # "S" or "C" in the "config" setting.)
                # In the default "block_num" method, we only need to determine the number of blocks in each scenes,
                # and the environment will automatically sample those quantity of blocks from a block type distribution.
                type="block_sequence",

                # We now decide to use 5 straight blocks in each scene.
                config="SSSSS",

                # Set the number of lanes in each side to 10.
                lane_num=10
            ),

            # ===== Visualization settings =====
            # Set to False to run in headless machines.
            use_render=True,

            # If you with to watch visualization (use_render=True) in a extremely old computer with limited memory,
            # set this to False.
            fast=True,

            # Disable this when you use RL agent to control the vehicle.
            manual_control=True,

            # We will use map range from seed [start_seed, start_seed + environment_num).
            # So change this if you wish to repeat experiments in different sets of scenes.
            start_seed=0
        )
    )
    env.reset()
    for i in range(1, 100000):
        o, r, d, info = env.step([0, 0])
        env.render()
        if d and info["arrive_dest"]:
            env.reset()
    env.close()
