import random
from shapely.geometry import Polygon, Point

polygon_corners = [
        (47.39819759514877, 8.544560432537802), 
        (47.39755729591149, 8.544054897237322),
        (47.39735973644937, 8.544987792276352),
        (47.39778660507262, 8.545542838766053)]

#smaller polygon for debugging purposes
smaller_polygon_corners = [
        (47.397917429980396, 8.545563816610999), 
        (47.39775458334322, 8.545406845199729),
        (47.39767489225227, 8.545744674976158),
        (47.39784466877918, 8.545804392360882)]

# randomizer function using numpy
def polygon_random_points (lst, num_points):
    poly = Polygon(lst)

    min_x, min_y, max_x, max_y = poly.bounds
    
    points = []
    while len(points) < num_points:
        random_point = Point([random.uniform(min_x, max_x), random.uniform(min_y, max_y)])
        if (random_point.within(poly)):
            points.append(random_point)
            # helpers.debug_log("Generated X : " + str(random_point.x))
            # helpers.debug_log("Generated Y : " + str(random_point.y))
    return points