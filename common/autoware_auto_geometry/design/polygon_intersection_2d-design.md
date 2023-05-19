# 2D Convex Polygon Intersection

Two convex polygon's intersection can be visualized on the image below as the blue area:

<!-- <img src="convex_polygon_intersection.png"> -->

## Purpose / Use cases

Computing the intersection between two polygons can be useful in many applications of scene
understanding. It can be used to estimate collision detection, shape alignment, shape
association and in any application that deals with the objects around the perceiving agent.

## Design

[\(Livermore, Calif, 1977\)](https://doi.org/10.2172/7309916) mention the following
observations about convex polygon intersection:

- Intersection of two convex polygons is a convex polygon
- A vertex from a polygon that is contained in the other polygon is a vertex of the intersection
  shape. (Vertices A, C, D in the shape above)
- An edge from a polygon that is contained in the other polygon is an edge in the intersection
  shape. (edge C-D in the shape above)
- Edge intersections between two polygons are vertices in the intersection shape. (Vertices B,
  E in the shape above.)

### Inner-workings / Algorithms

With the observation mentioned above, the current algorithm operates in the following way:

- Compute and find the vertices from each polygon that is contained in the other polygon
  (Vertices A, C, D)
- Compute and find the intersection points between each polygon (Vertices B, E)
- Compute the convex hull shaped by these vertices by ordering them CCW.

### Inputs / Outputs / API

Inputs:

- Two iterables that contain vertices of the convex polygons ordered in the CCW direction.

Outputs:

- A list of vertices of the intersection shape ordered in the CCW direction.

## Future Work

- #1230: Applying efficient algorithms.

## Related issues

- #983: Integrate vision detections in object tracker
