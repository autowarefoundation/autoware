import numpy as np


def rects_msg_to_ndarray(rects_msg):
    rects = np.zeros((len(rects_msg.rects), 4), dtype=np.float32)
    for i, r in enumerate(rects_msg.rects):
        xmin = r.x
        ymin = r.y
        xmax = r.x + r.width
        ymax = r.y + r.height
        rects[i] = [xmin, ymin, xmax, ymax]
    return rects


def bounding_box_msg_to_aabb(bbox_msg):
    center_x = bbox_msg.pose.position.x
    center_y = bbox_msg.pose.position.y
    center_z = bbox_msg.pose.position.z
    dim_x = bbox_msg.dimensions.x
    dim_y = bbox_msg.dimensions.y
    dim_z = bbox_msg.dimensions.z
    x1 = center_x - dim_x / 2.
    x2 = center_x + dim_x / 2.
    y1 = center_y - dim_y / 2.
    y2 = center_y + dim_y / 2.
    z1 = center_z - dim_z / 2.
    z2 = center_z + dim_z / 2.
    return x1, y1, z1, x2, y2, z2
