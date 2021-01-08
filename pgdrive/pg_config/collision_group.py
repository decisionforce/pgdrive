class CollisionGroup:
    Terrain = 2
    Ego_Vehicle = 1
    Lane_line = 3
    Traffic_vehicle = 4
    Lane_surface = 5

    @classmethod
    def set_collision_rule(cls, pg_physics_world):
        # terrain collision
        pg_physics_world.setGroupCollisionFlag(cls.Terrain, cls.Terrain, False)
        pg_physics_world.setGroupCollisionFlag(cls.Terrain, cls.Lane_line, False)
        pg_physics_world.setGroupCollisionFlag(cls.Terrain, cls.Lane_surface, False)
        pg_physics_world.setGroupCollisionFlag(cls.Terrain, cls.Ego_Vehicle, True)
        # change it after we design a new traffic system !
        pg_physics_world.setGroupCollisionFlag(cls.Terrain, cls.Traffic_vehicle, False)

        # block collision
        pg_physics_world.setGroupCollisionFlag(cls.Lane_line, cls.Lane_line, False)
        pg_physics_world.setGroupCollisionFlag(cls.Lane_line, cls.Lane_surface, False)
        pg_physics_world.setGroupCollisionFlag(cls.Lane_line, cls.Ego_Vehicle, True)
        # change it after we design a new traffic system !
        pg_physics_world.setGroupCollisionFlag(cls.Lane_line, cls.Traffic_vehicle, False)

        # traffic vehicles collision
        pg_physics_world.setGroupCollisionFlag(cls.Traffic_vehicle, cls.Traffic_vehicle, False)
        pg_physics_world.setGroupCollisionFlag(cls.Traffic_vehicle, cls.Lane_surface, False)
        pg_physics_world.setGroupCollisionFlag(cls.Traffic_vehicle, cls.Ego_Vehicle, True)

        # ego vehicle collision
        pg_physics_world.setGroupCollisionFlag(cls.Ego_Vehicle, cls.Ego_Vehicle, False)
        pg_physics_world.setGroupCollisionFlag(cls.Ego_Vehicle, cls.Lane_surface, False)

        # lane surface
        pg_physics_world.setGroupCollisionFlag(cls.Lane_surface, cls.Lane_surface, False)
