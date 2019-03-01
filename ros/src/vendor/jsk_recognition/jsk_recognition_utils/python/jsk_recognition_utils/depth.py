#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
from skimage.segmentation import slic
from skimage.feature import peak_local_max
from skimage.morphology import binary_closing

from jsk_recognition_utils.mask import descent_closing


def split_fore_background(depth_img, footprint=None):
    if footprint is None:
        footprint = np.ones((3, 3))
    segments = slic(depth_img)

    local_maxi = peak_local_max(
        depth_img, labels=segments, footprint=footprint, indices=False)

    fg_mask = descent_closing(local_maxi, init_selem=np.ones((3, 3)), n_times=6)
    bg_mask = ~fg_mask
    return fg_mask, bg_mask
