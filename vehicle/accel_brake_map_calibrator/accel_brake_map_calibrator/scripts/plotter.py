#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import matplotlib.pyplot as plt
import numpy as np


class Plotter(object):
    def __init__(self, total_height, total_width, plot=True):
        self.total_height = total_height
        self.total_width = total_width
        self.plot_flag = True
        self.fig = plt.figure(dpi=100, figsize=(24, 18))
        plt.subplots_adjust(left=0.04, right=0.98, bottom=0.05, top=0.95, wspace=0.1, hspace=0.4)

    def subplot(self, plot_num):
        subplot_str = str(self.total_height) + str(self.total_width) + str(plot_num)
        fig = plt.subplot(subplot_str)
        return fig

    def subplot_more(self, plot_num):
        width = plot_num % self.total_width
        height = int((plot_num - width) / self.total_width)
        fig = plt.subplot2grid((self.total_height, self.total_width), (height, width))
        return fig

    def plot_1d(self, data, color, label):
        plt.plot(data, color, label=label)

    def plot(self, x_data, y_data, color, label=None, linestyle="solid"):
        plt.plot(x_data, y_data, color, label=label, linestyle=linestyle, zorder=-1)

    def scatter(self, x, y, color, label=None, colorbar=False):
        plt.scatter(x, y, c=color, label=label, s=3)

    def scatter_color(self, x, y, color, cmap=None, label=None):
        sc = plt.scatter(x, y, c=color, label=label, cmap=cmap, s=3)
        plt.colorbar(sc)

    def plot_text(self, x, y, val, num_data_type="float", color="black"):
        if num_data_type == "float":
            plt.text(
                x,
                y,
                "{0:.2f}".format(float(val)),
                horizontalalignment="center",
                verticalalignment="center",
                color=color,
                clip_on=True,
                fontsize=8,
            )
        elif num_data_type == "int":
            plt.text(
                x,
                y,
                "{0:d}".format(int(val)),
                horizontalalignment="center",
                verticalalignment="center",
                color=color,
                clip_on=True,
                fontsize=8,
            )
        elif num_data_type == "str":
            plt.text(
                x,
                y,
                val,
                horizontalalignment="center",
                verticalalignment="center",
                color=color,
                clip_on=True,
                fontsize=8,
            )

    def imshow(self, data, left, right, ws, bottom, top, hs, num_data_type="float"):
        wm = (right - left) / data.shape[1]
        hm = (top - bottom) / data.shape[0]
        hwm = wm / 2.0
        hhm = hm / 2.0
        hws = ws / 2.0
        hhs = hs / 2.0
        width_range = np.linspace(bottom - hhs + hhm, top + hhs - hhm, data.shape[0])
        height_range = np.linspace(left - hws + hwm, right + hws - hwm, data.shape[1])
        plt.imshow(
            data,
            extent=(left - hws, right + hws, bottom - hhs, top + hhs),
            aspect="auto",
            origin="lower",
        )
        ys, xs = np.meshgrid(width_range, height_range, indexing="ij")
        for (x, y, val) in zip(xs.flatten(), ys.flatten(), data.flatten()):
            self.plot_text(x, y, val, num_data_type)
        plt.colorbar()

    def set_lim(self, fig, xlim, ylim):
        fig.set_xlim(xlim)
        fig.set_ylim(ylim)

    def add_label(self, title, xlabel, ylabel):
        plt.title(title)
        plt.xlabel(xlabel)
        plt.ylabel(ylabel)
        plt.legend(loc="upper left", fontsize=6, labelspacing=0)

    def show(self):
        if self.plot_flag:
            plt.show()

    def save(self, file_name):
        self.fig.savefig(file_name)
