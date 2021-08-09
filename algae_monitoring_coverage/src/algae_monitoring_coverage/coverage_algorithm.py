import math

from algae_monitoring_msgs.msg import Coordinate2D


def coverage_algorithm(start, area, camera):

    path = []

    N = math.ceil(area.width    / camera.width)
    M = math.ceil(area.height   / camera.height)

    for j in range(M):
        for i in range(N):

            if j % 2 != 0:
                i = N - 1 - i
            
            x = start.x + camera.width  * (i + 0.5)
            y = start.y + camera.height * (j + 0.5)

            path.append(Coordinate2D(x, y))

    return path
