#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2022 Tier IV, Inc.
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

from dataclasses import dataclass
import json
import os
from typing import Dict
from typing import List

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

# cspell: ignore ndim, ndata, linewidth


@dataclass
class Node:
    idx: int
    idx_parent: int
    pose: np.ndarray
    traj: np.ndarray

    @classmethod
    def from_dict(cls, d: Dict) -> "Node":
        if "traj_piece" in d:
            traj = np.array(d["traj_piece"])
            assert traj.ndim == 2
            assert traj.shape[1] == 3  # x, y, yaw
        else:
            traj = None
        return cls(d["idx"], d["parent_idx"], np.array(d["pose"]), traj)


@dataclass
class Tree:
    node_list: List[Node]
    goal_node: Node
    radius: float

    @classmethod
    def from_dict(cls, d: Dict) -> "Tree":
        node_list = [Node.from_dict(ndata) for ndata in d["nodes"]]
        goal_node = Node.from_dict(d["node_goal"])
        radius = d["radius"]
        return cls(node_list, goal_node, radius)

    def back_trace(self, node):
        nodes = [node]
        while not node.idx_parent == -1:
            node = self.node_list[node.idx_parent]
            nodes.append(node)
        return nodes

    def show_path(self, node, ax, color="black", width=1.0, highlight_node=False):
        q_seq = np.array(node.traj)
        ax.plot(q_seq[:, 0], q_seq[:, 1], color=color, linewidth=width)
        q0 = q_seq[0]
        if highlight_node:
            ax.scatter([q0[0]], [q0[1]], color="green", s=50)

    def show_world(self, ax):
        ax.plot([0, 1.0], [0.0, 0], c="k")
        ax.plot([1.0, 1.0], [0.0, 1.0], c="k")
        ax.plot([1.0, 0.0], [1.0, 1.0], c="k")
        ax.plot([0.0, 0.0], [1.0, 0.0], c="k")
        ax.add_patch(patches.Circle(xy=[0.5, 0.5], radius=0.3, ec="k", fc=(0, 0, 0, 0)))

    def visualize(self):
        fig, ax = plt.subplots()
        pts = np.array([n.pose for n in self.node_list])
        self.show_world(ax)
        ax.scatter(pts[:, 0], pts[:, 1], s=5)
        for node in self.node_list:
            if not node.idx_parent == -1:
                self.show_path(node, ax, width=0.1)

        solution_found = self.goal_node.idx_parent != 1
        if not solution_found:
            return

        for node in self.back_trace(self.goal_node):
            if not node.idx_parent == -1:
                self.show_path(node, ax, color="red", highlight_node=True)


if __name__ == "__main__":
    with open("/tmp/rrt_result.txt", "r") as f:
        dict_data = json.load(f)
    tree = Tree.from_dict(dict_data)
    tree.visualize()
    file_name = os.path.join("/tmp", "plot-rrtstar_core.png")
    plt.savefig(file_name)
    print("saved to {}".format(file_name))
