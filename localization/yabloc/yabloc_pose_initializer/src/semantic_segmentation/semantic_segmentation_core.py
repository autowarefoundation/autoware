#!/usr/bin/env python3

# Copyright 2023 TIER IV, Inc.
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

import copy

import cv2
import numpy as np


class SemanticSegmentationCore:
    def __init__(self, model_path):
        self.net_ = cv2.dnn.readNet(model_path)
        self.net_.setPreferableBackend(cv2.dnn.DNN_BACKEND_OPENCV)
        self.net_.setPreferableTarget(cv2.dnn.DNN_TARGET_CPU)

    def makeBlob(self, image: np.ndarray) -> np.ndarray:
        scale = 1.0
        size = (896, 512)
        mean = (0, 0, 0)
        swap = True
        crop = False
        depth = cv2.CV_32F
        return cv2.dnn.blobFromImage(image, scale, size, mean, swap, crop, depth)

    def inference(self, image: np.ndarray, score_threshold=0.5) -> np.ndarray:
        blob = self.makeBlob(image)
        self.net_.setInput(blob)
        output_layers = self.net_.getUnconnectedOutLayersNames()
        mask = self.net_.forward(output_layers)[0]
        mask = np.squeeze(mask).transpose(1, 2, 0)

        mask = cv2.resize(
            mask, dsize=(image.shape[1], image.shape[0]), interpolation=cv2.INTER_LINEAR
        )

        return self.__normalize(mask, score_threshold)

    def __normalize(self, mask, score_threshold=0.5) -> np.ndarray:
        masks = cv2.split(mask)[1:]
        bin_masks = []
        for mask in masks:
            bin_mask = np.where(mask > score_threshold, 0, 1).astype("uint8")
            bin_masks.append(255 - 255 * bin_mask)
        return cv2.merge(bin_masks)

    def drawOverlay(self, image, segmentation) -> np.ndarray:
        overlay_image = copy.deepcopy(image)
        return cv2.addWeighted(overlay_image, 0.5, segmentation, 0.5, 1.0)


def main():
    print("This script is not intended to be run alone.")


if __name__ == "__main__":
    main()
