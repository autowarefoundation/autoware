#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import division
import math

import cv2
import numpy as np
import PIL


def centerize(src, dst_shape, margin_color=None):
    """Centerize image for specified image size

    @param src: image to centerize
    @param dst_shape: image shape (height, width) or (height, width, channel)
    """
    if src.shape[:2] == dst_shape[:2]:
        return src
    centerized = np.zeros(dst_shape, dtype=src.dtype)
    if margin_color is not None:
        centerized[:, :] = margin_color
    pad_vertical, pad_horizontal = 0, 0
    h, w = src.shape[:2]
    dst_h, dst_w = dst_shape[:2]
    if h < dst_h:
        pad_vertical = (dst_h - h) // 2
    if w < dst_w:
        pad_horizontal = (dst_w - w) // 2
    centerized[pad_vertical:pad_vertical+h,
               pad_horizontal:pad_horizontal+w] = src
    return centerized

def _tile_images(imgs, tile_shape, concatenated_image, margin_color=None):
    """Concatenate images whose sizes are same.

    @param imgs: image list which should be concatenated
    @param tile_shape: shape for which images should be concatenated
    @param concatenated_image: returned image. if it is None, new image will be created.
    """
    x_num, y_num = tile_shape
    one_width = imgs[0].shape[1]
    one_height = imgs[0].shape[0]
    if concatenated_image is None:
        concatenated_image = np.zeros((one_height * y_num, one_width * x_num, 3),
                                      dtype=np.uint8)
        if margin_color is not None:
            concatenated_image[:, :] = margin_color
    for y in range(y_num):
        for x in range(x_num):
            i = x + y * x_num
            if i >= len(imgs):
                pass
            else:
                concatenated_image[y*one_height:(y+1)*one_height,x*one_width:(x+1)*one_width,] = imgs[i]
    return concatenated_image


def get_tile_image(imgs, tile_shape=None, result_img=None, margin_color=None):
    """Concatenate images whose sizes are different.

    @param imgs: image list which should be concatenated
    @param tile_shape: shape for which images should be concatenated
    @param result_img: numpy array to put result image
    """
    def get_tile_shape(img_num):
        x_num = 0
        y_num = int(math.sqrt(img_num))
        while x_num * y_num < img_num:
            x_num += 1
        return x_num, y_num

    if tile_shape is None:
        tile_shape = get_tile_shape(len(imgs))

    # get max tile size to which each image should be resized
    max_height, max_width = np.inf, np.inf
    for img in imgs:
        max_height = min([max_height, img.shape[0]])
        max_width = min([max_width, img.shape[1]])

    # resize and concatenate images
    for i, img in enumerate(imgs):
        h, w = img.shape[:2]
        h_scale, w_scale = max_height / h, max_width / w
        scale = min([h_scale, w_scale])
        h, w = int(scale * h), int(scale * w)
        img = cv2.resize(img, (w, h))
        img = centerize(img, (max_height, max_width, 3),
                        margin_color=margin_color)
        imgs[i] = img
    return _tile_images(imgs, tile_shape, result_img,
                        margin_color=margin_color)


def colorize_cluster_indices(image, cluster_indices, alpha=0.3, image_alpha=1):
    from skimage.color import rgb2gray
    from skimage.color import gray2rgb
    from skimage.util import img_as_float
    from skimage.color.colorlabel import DEFAULT_COLORS
    from skimage.color.colorlabel import color_dict
    image = img_as_float(rgb2gray(image))
    image = gray2rgb(image) * image_alpha + (1 - image_alpha)
    height, width = image.shape[:2]

    n_colors = len(DEFAULT_COLORS)
    indices_to_color = np.zeros((height * width, 3))
    for i, indices in enumerate(cluster_indices):
        color = color_dict[DEFAULT_COLORS[i % n_colors]]
        indices_to_color[indices] = color
    indices_to_color = indices_to_color.reshape((height, width, 3))
    result = indices_to_color * alpha + image * (1 - alpha)
    result = (result * 255).astype(np.uint8)
    return result
