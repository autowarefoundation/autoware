import shapely.geometry
import shapely.affinity

class Rect:
    def __init__(self, cx, cy, l, w, angle):
        self.cx = cx
        self.cy = cy
        self.l = l
        self.w = w
        self.angle = angle

    def get_contour(self):
        l = self.l
        w = self.w
        #shapely.geometry.geo.box(minx, miny, maxx, maxy, ccw=True)
        c = shapely.geometry.box(-l/2.0, -w/2.0, l/2.0, w/2.0)
        rc = shapely.affinity.rotate(c, self.angle)
        return shapely.affinity.translate(rc, self.cx, self.cy)

    def intersection(self, other):
        return self.get_contour().intersection(other.get_contour())

    def union(self, other):
        return self.get_contour().union(other.get_contour())

    def intersection_over_union(self, other):
        intersection_area = self.intersection(other).area
        union_area        = self.union(other).area
        return intersection_area/union_area
