# Autoware artifacts

The Autoware perception stack uses models for inference. These models are automatically downloaded if using `ansible`, but they can also be downloaded manually.

## ONNX model files

### Download instructions

The ONNX model files are stored in a common location, hosted by Web.Auto.

Any tool that can download files from the web (e.g. `wget` or `curl`) is the only requirement for downloading these files.

#### yabloc_pose_initializer

| file             | checksum                                                                |
| ---------------- | ----------------------------------------------------------------------- |
| resources.tar.gz | sha256:1f660e15f95074bade32b1f80dbf618e9cee1f0b9f76d3f4671cb9be7f56eb3a |

```console
$ mkdir -p ~/autoware_data/yabloc_pose_initializer/
$ wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz
```

#### image_projection_based_fusion

| file                                      | checksum                             |
| ----------------------------------------- | ------------------------------------ |
| pts_voxel_encoder_pointpainting.onnx      | md5:25c70f76a45a64944ccd19f604c99410 |
| pts_backbone_neck_head_pointpainting.onnx | md5:2c7108245240fbd7bf0104dd68225868 |

```console
$ mkdir -p ~/autoware_data/image_projection_based_fusion/
$ wget -P ~/autoware_data/image_projection_based_fusion/ \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx
```

#### lidar_apollo_instance_segmentation

| file                   | checksum                             |
| ---------------------- | ------------------------------------ |
| vlp-16.onnx checksum:  | md5:63a5a1bb73f7dbb64cf70d04eca45fb4 |
| hdl-64.onnx checksum:  | md5:009745e33b1df44b68296431cc384cd2 |
| vls-128.onnx checksum: | md5:b2d709f56f73ae2518c9bf4d0214468f |

```console
$ mkdir -p ~/autoware_data/lidar_apollo_instance_segmentation/
$ wget -P ~/autoware_data/lidar_apollo_instance_segmentation/ \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx
```

#### lidar_centerpoint

| file                                         | checksum                                                                |
| -------------------------------------------- | ----------------------------------------------------------------------- |
| pts_voxel_encoder_centerpoint.onnx           | sha256:dc1a876580d86ee7a341d543f8ade2ede7f43bd032dc5b44155b1f0175405764 |
| pts_backbone_neck_head_centerpoint.onnx      | sha256:3fe7e128955646740c41a25be0c8f141d5a94594fe79d7405fe2a859e391542e |
| pts_voxel_encoder_centerpoint_tiny.onnx      | sha256:2c53465715c1fd2e9dc5727ef3fca74f4cdf0538f74286b0946e219d0ca5693b |
| pts_backbone_neck_head_centerpoint_tiny.onnx | md5:e4658325b70222f7c3637fe00e586b82                                    |

```console
$ mkdir -p ~/autoware_data/lidar_centerpoint/
$ wget -P ~/autoware_data/lidar_centerpoint/ \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx
```

#### tensorrt_yolo

| file             | checksum                                                                |
| ---------------- | ----------------------------------------------------------------------- |
| yolov3.onnx      | sha256:61e922f76918dd3d8e0abdc5fb7406f390609e08bd8ab9e5d3b97afb00f30f8c |
| yolov4.onnx      | sha256:7c7343156c1bd4b397fd1e44b27334691a6219db3ce2e29a03b72af65ddb8f39 |
| yolov4-tiny.onnx | sha256:0e877c716fbf8a2b431ee3e57f6c7411a6741319b52c32c6dafc53c7e1b17027 |
| yolov5s.onnx     | sha256:be335ff7746957debf1a6903a61fa3f568b780b4afe4958edf2d4bc98e9e0825 |
| yolov5m.onnx     | sha256:ee6f67f7c00a34cc4cef2fdd9db30dd714df1a4fb2d7e9fc1731cfe85b673133 |
| yolov5l.onnx     | sha256:a627e5f70180a8746482b572194090466db62c8d1256602c1cd20374dd960e34 |
| yolov5x.onnx     | sha256:d7cb4cd7078f87bda22a37828d72867accecedf9f74d0d87b5cc1f6f1180a019 |
| coco.names       | sha256:634a1132eb33f8091d60f2c346ababe8b905ae08387037aed883953b7329af84 |

```console
$ mkdir -p ~/autoware_data/tensorrt_yolo/
$ wget -P ~/autoware_data/tensorrt_yolo/ \
       https://awf.ml.dev.web.auto/perception/models/c \
       https://awf.ml.dev.web.auto/perception/models/c \
       https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx \
       https://awf.ml.dev.web.auto/perception/models/coco.names
```

#### tensorrt_yolox

| file                                        | checksum                                                                |
| ------------------------------------------- | ----------------------------------------------------------------------- |
| yolox-tiny.onnx                             | sha256:471a665f4243e654dff62578394e508db22ee29fe65d9e389dfc3b0f2dee1255 |
| yolox-sPlus-opt.onnx                        | md5:bf3b0155351f90fcdca2626acbfd3bcf                                    |
| yolox-sPlus-opt.EntropyV2-calibration.table | md5:c6e6f1999d5724a017516a956096701f                                    |
| label.txt                                   | sha256:3540a365bfd6d8afb1b5d8df4ec47f82cb984760d3270c9b41dbbb3422d09a0c |

```console
$ mkdir -p ~/autoware_data/tensorrt_yolox/
$ wget -P ~/autoware_data/tensorrt_yolox/ \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt
```

#### traffic_light_classifier

| file                                                  | checksum                                                                |
| ----------------------------------------------------- | ----------------------------------------------------------------------- |
| traffic_light_classifier_mobilenetv2_batch_1.onnx     | md5:caa51f2080aa2df943e4f884c41898eb                                    |
| traffic_light_classifier_mobilenetv2_batch_4.onnx     | md5:c2beaf60210f471debfe72b86d076ca0                                    |
| traffic_light_classifier_mobilenetv2_batch_6.onnx     | md5:28b408710bcb24f4cdd4d746301d4e78                                    |
| traffic_light_classifier_efficientNet_b1_batch_1.onnx | md5:82baba4fcf1abe0c040cd55005e34510                                    |
| traffic_light_classifier_efficientNet_b1_batch_4.onnx | md5:21b549c2fe4fbb20d32cc019e6d70cd7                                    |
| traffic_light_classifier_efficientNet_b1_batch_6.onnx | md5:378526d9aa9fc6705cf399f7b35b3053                                    |
| lamp_labels.txt                                       | sha256:1a5a49eeec5593963eab8d70f48b8a01bfb07e753e9688eb1510ad26e803579d |

```console
$ mkdir -p ~/autoware_data/traffic_light_classifier/
$ wget -P ~/autoware_data/traffic_light_classifier/ \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels.txt
```

#### traffic_light_fine_detector

| file                     | checksum                             |
| ------------------------ | ------------------------------------ |
| tlr_yolox_s_batch_1.onnx | md5:2b72d085022b8ee6aacff06bd722cfda |
| tlr_yolox_s_batch_4.onnx | md5:4044daa86e7776ce241e94d98a09cc0e |
| tlr_yolox_s_batch_6.onnx | md5:47255a11bde479320d703f1f45db1242 |
| tlr_labels.txt           | md5:e9f45efb02f2a9aa8ac27b3d5c164905 |

```console
$ mkdir -p ~/autoware_data/traffic_light_fine_detector/
$ wget -P ~/autoware_data/traffic_light_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_labels.txt
```

#### traffic_light_ssd_fine_detector

| file                  | checksum                                                                |
| --------------------- | ----------------------------------------------------------------------- |
| mb2-ssd-lite-tlr.onnx | sha256:e29e6ee68751a270fb285fd037713939ca7f61a897b4c3a7ab22b0d6a9a21ddf |
| voc_labels_tl.txt     | sha256:a41e6e3324e32c30b3b2fe38908eaf3471e2bfdaeb9e14ca0c1c3bc0275119c6 |

```console
$ mkdir -p ~/autoware_data/traffic_light_ssd_fine_detector/
$ wget -P ~/autoware_data/traffic_light_ssd_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt
```
