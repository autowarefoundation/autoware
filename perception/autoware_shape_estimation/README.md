# autoware_shape_estimation

## Purpose

This node calculates a refined object shape (bounding box, cylinder, convex hull) in which a pointcloud cluster fits according to a label.

## Inner-workings / Algorithms

### Fitting algorithms

- bounding box

  - L-shape fitting: See reference below for details
  - ML based shape fitting: See ML Based Shape Fitting Implementation section below for details

- cylinder

  `cv::minEnclosingCircle`

- convex hull

  `cv::convexHull`

## Inputs / Outputs

### Input

| Name    | Type                                                     | Description                           |
| ------- | -------------------------------------------------------- | ------------------------------------- |
| `input` | `tier4_perception_msgs::msg::DetectedObjectsWithFeature` | detected objects with labeled cluster |

### Output

| Name             | Type                                             | Description                         |
| ---------------- | ------------------------------------------------ | ----------------------------------- |
| `output/objects` | `autoware_perception_msgs::msg::DetectedObjects` | detected objects with refined shape |

## Parameters

{{ json_to_markdown("perception/autoware_shape_estimation/schema/shape_estimation.schema.json") }}

## ML Based Shape Implementation

The model takes a point cloud and object label(provided by camera detections/Apollo instance segmentation) as an input and outputs the 3D bounding box of the object.

ML based shape estimation algorithm uses a PointNet model as a backbone to estimate the 3D bounding box of the object. The model is trained on the NuScenes dataset with vehicle labels (Car, Truck, Bus, Trailer).

The implemented model is concatenated with STN (Spatial Transformer Network) to learn the transformation of the input point cloud to the canonical space and PointNet to predict the 3D bounding box of the object.
Bounding box estimation part of _Frustum PointNets for 3D Object Detection from RGB-D Data_ paper used as a reference.

The model predicts the following outputs for each object:

- x,y,z coordinates of the object center
- object heading angle classification result(Uses 12 bins for angle classification - 30 degrees each)
- object heading angle residuals
- object size classification result
- object size residuals

### Training ML Based Shape Estimation Model

To train the model, you need ground truth 3D bounding box annotations. When using the mmdetection3d repository for training a 3D object detection algorithm, these ground truth annotations are
saved and utilized for data augmentation. These annotations are used as an essential dataset for training the shape estimation model effectively.

### Preparing the Dataset

#### Install MMDetection3D prerequisites

**Step 1.** Download and install Miniconda from the [official website](https://mmpretrain.readthedocs.io/en/latest/get_started.html).

**Step 2.** Create a conda virtual environment and activate it

```bash
conda create --name train-shape-estimation python=3.8 -y
conda activate train-shape-estimation
```

**Step 3.** Install PyTorch

```bash
conda install pytorch torchvision -c pytorch
```

#### Install mmdetection3d

**Step 1.** Install MMEngine, MMCV, and MMDetection using MIM

```bash
pip install -U openmim
mim install mmengine
mim install 'mmcv>=2.0.0rc4'
mim install 'mmdet>=3.0.0rc5, <3.3.0'
```

**Step 2.** Install Autoware's MMDetection3D fork

```bash
git clone https://github.com/autowarefoundation/mmdetection3d.git
cd mmdetection3d
pip install -v -e .
```

#### Preparing NuScenes dataset for training

**Step 1.** Download the NuScenes dataset from the [official website](https://www.nuscenes.org/download) and extract the dataset to a folder of your choice.

**Note:** The NuScenes dataset is large and requires significant disk space. Ensure you have enough storage available before proceeding.

**Step 2.** Create a symbolic link to the dataset folder

```bash
ln -s /path/to/nuscenes/dataset/ /path/to/mmdetection3d/data/nuscenes/
```

**Step 3.** Prepare the NuScenes data by running:

```bash
cd mmdetection3d
python tools/create_data.py nuscenes --root-path ./data/nuscenes --out-dir ./data/nuscenes --extra-tag nuscenes --only-gt-database True
```

#### Clone Bounding Box Estimator model

```bash
git clone https://github.com/autowarefoundation/bbox_estimator.git
```

#### Split the dataset into training and validation sets

```bash

cd bbox_estimator
python3 utils/split_dbinfos.py --dataset_path /path/to/mmdetection3d/data/nuscenes --classes 'car' 'truck' 'bus' 'trailer'  --train_ratio 0.8
```

### Training and Deploying the model

#### Training the model

```bash
# Detailed training options can be found in the training script
# For more details, run `python3 train.py --help`
python3 train.py --dataset_path /path/to/mmdetection3d/data/nuscenes
```

#### Deploying the model

```bash
# Convert the trained model to ONNX format
python3 onnx_converter.py --weight_path /path/to/best_checkpoint.pth --output_path /path/to/output.onnx
```

Give the output path of the ONNX model to the `model_path` parameter in the `shape_estimation` node launch file.

## Assumptions / Known limits

TBD

## References/External links

L-shape fitting implementation of the paper:

```bibtex
@conference{Zhang-2017-26536,
author = {Xiao Zhang and Wenda Xu and Chiyu Dong and John M. Dolan},
title = {Efficient L-Shape Fitting for Vehicle Detection Using Laser Scanners},
booktitle = {2017 IEEE Intelligent Vehicles Symposium},
year = {2017},
month = {June},
keywords = {autonomous driving, laser scanner, perception, segmentation},
}
```

Frustum PointNets for 3D Object Detection from RGB-D Data:

````bibtex
@inproceedings{qi2018frustum,
title={Frustum pointnets for 3d object detection from rgb-d data},
author={Qi, Charles R and Liu, Wei and Wu, Chenxia and Su, Hao and Guibas, Leonidas J},
booktitle={Proceedings of the IEEE conference on computer vision and pattern recognition},
pages={918--927},
year={2018}
}```
````
