#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import cv2
import matplotlib.pyplot as plt
import numpy as np
import PIL


def get_tile_image(imgs, tile_shape=None):

    def get_tile_shape(img_num):
        x_num = 0
        y_num = int(math.sqrt(img_num))
        while x_num * y_num < img_num:
            x_num += 1
        return x_num, y_num

    if tile_shape is None:
        tile_shape = get_tile_shape(len(imgs))

    img_rgb_list = []
    for img in imgs:
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img_rgb_list.append(img_rgb)

    x_num, y_num = tile_shape
    for i, img_rgb in enumerate(img_rgb_list):
        plt.subplot(y_num, x_num, i+1)
        plt.axis('off')
        plt.imshow(img_rgb)

    canvas = plt.get_current_fig_manager().canvas
    canvas.draw()
    pil_img = PIL.Image.fromstring('RGB',
        canvas.get_width_height(), canvas.tostring_rgb())
    out_rgb = np.array(pil_img)
    out_bgr = cv2.cvtColor(out_rgb, cv2.COLOR_RGB2BGR)
    plt.close()
    return out_bgr
