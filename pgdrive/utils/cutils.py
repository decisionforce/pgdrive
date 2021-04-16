import logging
import math


def _import_panda_position():
    from pgdrive.utils.coordinates_shift import panda_position
    return panda_position


def _get_fake_cutils():
    panda_position = _import_panda_position()

    class FakeCutils:
        @classmethod
        def norm(cls, x, y):
            return math.sqrt(x ** 2 + y ** 2)

        @classmethod
        def clip(cls, a, low, high):
            return min(max(a, low), high)

        @classmethod
        def cutils_perceive(cls, ):
            pass

        @classmethod
        def cutils_perceive(
                cls,
                cloud_points,
                detector_mask,
                mask,
                lidar_range,
                perceive_distance,
                heading_theta,
                velocity_position_x,
                velocity_position_y,
                num_lasers,
                height,
                pg_physics_world,
                extra_filter_node,
                require_colors,
                ANGLE_FACTOR,
                MARK_COLOR
        ):
            """
            Call me to update the perception info
            """
            assert detector_mask is not "WRONG"
            # coordinates problem here! take care
            extra_filter_node = extra_filter_node or set()
            pg_start_position = panda_position((velocity_position_x, velocity_position_y), height)

            # init
            cloud_points.fill(1.0)
            detected_objects = []
            colors = []

            # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
            for laser_index in range(num_lasers):
                # # coordinates problem here! take care

                if (detector_mask is not None) and (not detector_mask[laser_index]):
                    # update vis
                    if self.cloud_points_vis is not None:
                        laser_end = self._get_laser_end(laser_index, heading_theta, vehicle_position)
                        self._add_cloud_point_vis(laser_index, laser_end)
                    continue

                laser_end = self._get_laser_end(laser_index, heading_theta, vehicle_position)
                result = pg_physics_world.rayTestClosest(pg_start_position, laser_end, mask)
                node = result.getNode()
                if node in extra_filter_node:
                    # Fall back to all tests.
                    results = pg_physics_world.rayTestAll(pg_start_position, laser_end, mask)
                    hits = results.getHits()
                    hits = sorted(hits, key=lambda ret: ret.getHitFraction())
                    for result in hits:
                        if result.getNode() in extra_filter_node:
                            continue
                        self.detected_objects.append(result)
                        self.cloud_points[laser_index] = result.getHitFraction()
                        break
                else:
                    hits = result.hasHit()
                    self.cloud_points[laser_index] = result.getHitFraction()
                    if node:
                        self.detected_objects.append(result)

                # update vis
                if self.cloud_points_vis is not None:
                    self._add_cloud_point_vis(laser_index, result.getHitPos() if hits else laser_end)
            return self.cloud_points


def import_cutils():
    try:
        from pgdrive import cutils
    except:
        msg = (
            "It seems you don't install our cython utilities yet! Please reinstall PGDrive via: "
            "<pip install -e .> or <pip install pgdrive>!"
        )
        print(msg)
        logging.warning(msg)
        cutils = None
    return cutils
