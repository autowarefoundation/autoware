def get_overlap_of_aabb(box1, box2, return_volumes=False):
    x11, y11, z11, x12, y12, z12 = box1
    dim_x1 = x12 - x11
    dim_y1 = y12 - y11
    dim_z1 = z12 - z11

    x21, y21, z21, x22, y22, z22 = box2
    dim_x2 = x22 - x21
    dim_y2 = y22 - y21
    dim_z2 = z22 - z21

    if ((x11 <= x22 and x12 >= x21) and
            (y11 <= y22 and y12 >= y21) and
            (z11 <= z22 and z12 >= z21)):
        # has intersect
        dim_x3 = min(x12, x22) - max(x11, x21)
        dim_y3 = min(y12, y22) - max(y11, y21)
        dim_z3 = min(z12, z22) - max(z11, z21)
    else:
        dim_x3 = dim_y3 = dim_z3 = 0

    intersect = dim_x3 * dim_y3 * dim_z3
    union = (dim_x1 * dim_y1 * dim_z1) + (dim_x2 * dim_y2 * dim_z2) - intersect
    iu = 1. * intersect / union
    if return_volumes:
        return iu, intersect, union
    else:
        return iu
