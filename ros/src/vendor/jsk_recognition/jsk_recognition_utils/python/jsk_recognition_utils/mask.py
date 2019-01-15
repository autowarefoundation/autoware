#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from skimage.morphology import binary_closing


def bounding_rect_of_mask(img, mask):
    where = np.argwhere(mask)
    (y_start, x_start), (y_stop, x_stop) = where.min(0), where.max(0) + 1
    return img[y_start:y_stop, x_start:x_stop]


def descent_closing(mask, init_selem, n_times):
    S = init_selem.shape
    for i in xrange(n_times):
        selem = np.ones((S[0] * (n_times - i), S[1] * (n_times - i)))
        mask = binary_closing(mask, selem=selem)
    return mask
