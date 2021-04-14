cimport numpy as cnp
import numpy as np

ctypedef cnp.float64_t np_float64_t
ctypedef cnp.int64_t np_int64_t
import cython

def cutils_panda_position(np_float64_t position_x, np_float64_t position_y, np_float64_t z=0.0):
    return position_x, -position_y, z

# Remove this check to further accelerate. But this might cause fatal error! So I just commented them out here.
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
def cutils_perceive(
        cnp.ndarray[np_float64_t, ndim=1] cloud_points,
        mask,
        cnp.ndarray[np_float64_t, ndim=1] lidar_range,
        np_float64_t perceive_distance,
        np_float64_t heading_theta,
        np_float64_t vehicle_position_x,
        np_float64_t vehicle_position_y,
        int num_lasers,
        np_float64_t height,
        pg_physics_world,
        set extra_filter_node,
        cloud_points_vis,
        ANGLE_FACTOR,
        MARK_COLOR
):
    # init
    cloud_points.fill(1.0)
    cdef list detected_objects = []
    cdef tuple pg_start_position = cutils_panda_position(vehicle_position_x, vehicle_position_y, height)

    # lidar calculation use pg coordinates
    cdef cnp.ndarray[np_float64_t, ndim=1] laser_heading = lidar_range + heading_theta
    cdef cnp.ndarray[np_float64_t, ndim=1] point_x = perceive_distance * np.cos(laser_heading) + vehicle_position_x
    cdef cnp.ndarray[np_float64_t, ndim=1] point_y = perceive_distance * np.sin(laser_heading) + vehicle_position_y

    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    for laser_index in range(num_lasers):
        # # coordinates problem here! take care
        laser_end = cutils_panda_position(point_x[laser_index], point_y[laser_index], height)
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
                detected_objects.append(result)
                cloud_points[laser_index] = result.getHitFraction()
                break
        else:
            cloud_points[laser_index] = result.getHitFraction()
            if node:
                detected_objects.append(result)
                hits = result.hasHit()

        # update vis
        if cloud_points_vis is not None:
            cloud_points_vis[laser_index].setPos(result.getHitPos() if hits else laser_end)
            f = laser_index / num_lasers if ANGLE_FACTOR else 1
            cloud_points_vis[laser_index].setColor(
                f * MARK_COLOR[0], f * MARK_COLOR[1], f * MARK_COLOR[2]
            )

    return cloud_points, detected_objects, cloud_points_vis

# cdef extern from "math.h":
#     double sqrt(double x)
#     double sin(double x)
#     double cos(double x)
#     double acos(double x)
#     double fabs(double x)
#     double atan2(double y, double x)
#     double asin(double x)
#     double sqrt(double x)
#     double tan(double x)
#     int floor(double x)
#     int ceil(double x)
#     double fmod(double x, double y)
#
# ctypedef cnp.float64_t np_float64_t
# ctypedef cnp.int64_t np_int64_t
#
# cdef struct Point:
#     np_float64_t x
#     np_float64_t y
#
# cdef struct CarDist:
#     np_int64_t idx
#     np_float64_t dist2
#
# ctypedef Point Vector
#
# cdef struct Rect:
#     Point p0
#     Point p1
#     Point p2
#     Point p3
#
# ctypedef bint bool
#
# cdef np_float64_t pi = np.pi
#
# # @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
# def py_intersectLines(np_float64_t theta,
#                       np_float64_t x0,
#                       np_float64_t y0,
#                       np_float64_t x1,
#                       np_float64_t y1,
#                       np_float64_t x2,
#                       np_float64_t y2,
#                       np_float64_t maximum):
#     cdef double dx1 = cos(theta * pi / 180)
#     cdef double dy1 = sin(theta * pi / 180)
#     cdef double dx2 = x2 - x1
#     cdef double dy2 = y2 - y1
#     cdef double DET = -dx1 * dy2 + dy1 * dx2
#     if fabs(DET) < 1e-9:
#         return maximum
#     cdef double r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
#     cdef double s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
#     if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
#         return r
#     return maximum
#
# # @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
# cdef np_float64_t intersectLines(np_float64_t theta,
#                                  np_float64_t x0,
#                                  np_float64_t y0,
#                                  np_float64_t x1,
#                                  np_float64_t y1,
#                                  np_float64_t x2,
#                                  np_float64_t y2,
#                                  np_float64_t maximum):
#     cdef double dx1 = cos(theta * pi / 180)
#     cdef double dy1 = sin(theta * pi / 180)
#     cdef double dx2 = x2 - x1
#     cdef double dy2 = y2 - y1
#     cdef double DET = -dx1 * dy2 + dy1 * dx2
#     if fabs(DET) < 1e-9:
#         return maximum
#     cdef double r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
#     cdef double s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
#     if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
#         return r
#     return maximum

# import cython
# cdef extern from "math.h":
#     double sqrt(double x)
#     double sin(double x)
#     double cos(double x)
#     double acos(double x)
#     double fabs(double x)
#     double atan2(double y, double x)
#     double asin(double x)
#     double sqrt(double x)
#     double tan(double x)
#     int floor(double x)
#     int ceil(double x)
#     double fmod(double x, double y)
#
# ctypedef cnp.float64_t np_float64_t
# ctypedef cnp.int64_t np_int64_t
#
# cdef struct Point:
#     np_float64_t x
#     np_float64_t y
#
# cdef struct CarDist:
#     np_int64_t idx
#     np_float64_t dist2
#
# ctypedef Point Vector
#
# cdef struct Rect:
#     Point p0
#     Point p1
#     Point p2
#     Point p3
#
# ctypedef bint bool
#
# cdef np_float64_t pi = np.pi
#
# # @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
# def py_intersectLines(np_float64_t theta,
#                       np_float64_t x0,
#                       np_float64_t y0,
#                       np_float64_t x1,
#                       np_float64_t y1,
#                       np_float64_t x2,
#                       np_float64_t y2,
#                       np_float64_t maximum):
#     cdef double dx1 = cos(theta * pi / 180)
#     cdef double dy1 = sin(theta * pi / 180)
#     cdef double dx2 = x2 - x1
#     cdef double dy2 = y2 - y1
#     cdef double DET = -dx1 * dy2 + dy1 * dx2
#     if fabs(DET) < 1e-9:
#         return maximum
#     cdef double r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
#     cdef double s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
#     if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
#         return r
#     return maximum
#
# # @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
# cdef np_float64_t intersectLines(np_float64_t theta,
#                                  np_float64_t x0,
#                                  np_float64_t y0,
#                                  np_float64_t x1,
#                                  np_float64_t y1,
#                                  np_float64_t x2,
#                                  np_float64_t y2,
#                                  np_float64_t maximum):
#     cdef double dx1 = cos(theta * pi / 180)
#     cdef double dy1 = sin(theta * pi / 180)
#     cdef double dx2 = x2 - x1
#     cdef double dy2 = y2 - y1
#     cdef double DET = -dx1 * dy2 + dy1 * dx2
#     if fabs(DET) < 1e-9:
#         return maximum
#     cdef double r = 1.0 / DET * (-dy2 * (x1 - x0) + dx2 * (y1 - y0))
#     cdef double s = 1.0 / DET * (-dy1 * (x1 - x0) + dx1 * (y1 - y0))
#     if 0 - 1e-5 < s < 1 + 1e-5 and r >= 0:
#         return r
#     return maximum


# # @cython.boundscheck(False)
# @cython.wraparound(False)
# @cython.cdivision(True)
# @cython.nonecheck(False)
# def laser_environment(cnp.ndarray[cnp.uint32_t, ndim=2] screen,
#                       cnp.ndarray[Rect, ndim=1, mode='c'] input_rects,
#                       cnp.ndarray[np_float64_t, ndim=1] orients,
#                       cnp.ndarray[np_float64_t, ndim=3] gate_boundary,
#                       double laser_max_length,
#                       int laser_num):
#     cdef int car_count = input_rects.shape[0]
#     cdef int gate_count = gate_boundary.shape[0]
#     cdef int screen_width = screen.shape[1]
#     cdef int screen_height = screen.shape[0]
#     cdef int i, angle_idx, x, y, mid_x, mid_y, gate_idx
#     cdef double angle_delta = 360 / laser_num
#     cdef double endpx, endpy, min_dist, dist, theta, laser_length, laser_search_step
#
#     cdef Rect*rects = <Rect*> (input_rects.data)
#     cdef Point*points
#     cdef Point center
#
#     cdef cnp.ndarray[np_float64_t, ndim=2] lasers = np.empty([car_count, laser_num], dtype=np.float64)
#     lasers.fill(laser_max_length)
#
#     for i in range(0, car_count):
#         points = <Point*> (&rects[i])
#         center.x = (points[0].x + points[1].x + points[2].x + points[3].x) / 4
#         center.y = (points[0].y + points[1].y + points[2].y + points[3].y) / 4
#         for angle_idx in range(0, laser_num):
#             theta = angle_idx * angle_delta + orients[i]
#             min_dist = laser_max_length
#             for gate_idx in range(0, gate_count):
#                 dist = intersectLines(theta, center.x, center.y,
#                                       gate_boundary[gate_idx, 0, 0], gate_boundary[gate_idx, 0, 1],
#                                       gate_boundary[gate_idx, 1, 0], gate_boundary[gate_idx, 1, 1],
#                                       laser_max_length)
#                 min_dist = dist if dist < min_dist else min_dist
#
#             if min_dist < laser_max_length:
#                 # We assert that if a ray touch the gate boundary, it is impossible to touch the environment boundary
#                 lasers[i, angle_idx] = min_dist
#                 continue
#
#             laser_length = laser_max_length
#             laser_search_step = laser_max_length
#
#             while laser_search_step >= 0.05:
#                 # When the search step less than one pixel, quit.
#                 # the x-cord in car system is the y-cord in screen system
#                 # and the y-cord in car system is the reverse-x-cord in screen system.
#                 # the below x, y is in screen system.
#
#                 endpx = cos(theta * pi / 180) * laser_length
#                 endpy = sin(theta * pi / 180) * laser_length
#
#                 y = <int> (center.x + endpx)
#                 x = screen_height - 1 - <int> (center.y + endpy)
#
#                 mid_y = <int> (center.x + endpx / 2)
#                 mid_x = screen_height - 1 - <int> (center.y + endpy / 2)
#
#                 if 0 <= x < screen_height and 0 <= y < screen_width and screen[x, y] == 0 and screen[
#                     mid_x, mid_y] == 0:  #
#                     laser_length = min(laser_length + laser_search_step, laser_max_length)
#                 else:
#                     laser_length = laser_length - laser_search_step
#
#                 laser_search_step = laser_search_step / 2
#             lasers[i, angle_idx] = laser_length
#
#     return lasers
