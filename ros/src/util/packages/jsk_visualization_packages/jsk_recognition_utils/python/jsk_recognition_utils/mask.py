#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np


def bounding_rect_of_mask(img, mask):
    where = np.argwhere(mask)
    (y_start, x_start), (y_stop, x_stop) = where.min(0), where.max(0) + 1
    return img[y_start:y_stop, x_start:x_stop]
