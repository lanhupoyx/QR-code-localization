import random
import math

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"Point({self.x}, {self.y})"

def distance(p1, p2):
    return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

def circle_from_two_points(p1, p2):
    center_x = (p1.x + p2.x) / 2
    center_y = (p1.y + p2.y) / 2
    radius = distance(p1, p2) / 2
    return (Point(center_x, center_y), radius)

def circle_from_three_points(p1, p2, p3):
    ax, ay = p1.x, p1.y
    bx, by = p2.x, p2.y
    cx, cy = p3.x, p3.y

    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))

    if d == 0:
        raise ValueError("The three points are collinear and cannot form a circle.")

    center_x = ((ax * ax + ay * ay) * (by - cy) +
                (bx * bx + by * by) * (cy - ay) +
                (cx * cx + cy * cy) * (ay - by)) / d

    center_y = ((ax * ax + ay * ay) * (cx - bx) +
                (bx * bx + by * by) * (ax - cx) +
                (cx * cx + cy * cy) * (bx - ax)) / d

    center = Point(center_x, center_y)
    radius = distance(center, p1)
    return (center, radius)

def is_inside_circle(point, center, radius):
    return distance(point, center) <= radius

def welzl(points, boundary_points, n):
    if n == 0 or len(boundary_points) == 3:
        if len(boundary_points) == 0:
            return (None, 0)
        elif len(boundary_points) == 1:
            return (boundary_points[0], 0)
        elif len(boundary_points) == 2:
            return circle_from_two_points(boundary_points[0], boundary_points[1])
        elif len(boundary_points) == 3:
            return circle_from_three_points(boundary_points[0], boundary_points[1], boundary_points[2])

    random_index = random.randint(0, n - 1)
    p = points[random_index]
    points[n - 1], points[random_index] = points[random_index], points[n - 1]

    (center, radius) = welzl(points, boundary_points, n - 1)

    if center is not None and is_inside_circle(p, center, radius):
        return (center, radius)

    boundary_points.append(p)
    result = welzl(points, boundary_points, n - 1)
    boundary_points.pop()

    return result

def minimum_enclosing_circle(points):
    random.shuffle(points)
    return welzl(points, [], len(points))
    
# 示例
if __name__ == "__main__":
    points = [Point(0, 0), Point(2, 0), Point(0, 1), Point(1, 1)]
    center, radius = minimum_enclosing_circle(points)
    print(f"Center: {center}")
    print(f"Radius: {radius}")