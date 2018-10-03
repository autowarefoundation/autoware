Setup
=====

Install DNN model on COCO dataset
---------------------------------

DNN-based nodes require their pretrained models.
The nodes in Autoware are:

* `YOLO <https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/detection/vision_detector/packages/vision_darknet_detect/README.md>`_ 
* `SSD <https://github.com/CPFL/Autoware/blob/master/ros/src/computing/perception/detection/vision_detector/packages/vision_ssd_detect/README.md>`_ 

Please follow README.md in each package to install the models if you need these nodes.

.. warning::

    Autoware itself is licensed under BSD 3-Clause "New" or "Revised" License.
    However, pre-trained models of DNN that the users of Autoware use are expected to
    be licensed under another license. For example, KITTI is licensed under CC BY-NC-SA,
    which do now allow commercial uses. **Please follow each license of the models you use.**
