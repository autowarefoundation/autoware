# Machine learning models

The Autoware perception stack uses models for inference. These models are automatically downloaded if using `ansible`, but they can also be downloaded manually.

## ONNX model files

### Download instructions

The ONNX model files are stored in a common location, hosted by Web.Auto

Any tool that can download files from the web (e.g. `wget` or `curl`) is the only requirement for downloading these files:

```console
# yabloc_pose_initializer

$ mkdir -p ~/autoware_data/yabloc_pose_initializer/
$ wget -P ~/autoware_data/yabloc_pose_initializer/ \
       https://s3.ap-northeast-2.wasabisys.com/pinto-model-zoo/136_road-segmentation-adas-0001/resources.tar.gz


# image_projection_based_fusion

$ mkdir -p ~/autoware_data/image_projection_based_fusion/
$ wget -P ~/autoware_data/image_projection_based_fusion/ \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_voxel_encoder_pointpainting.onnx \
       https://awf.ml.dev.web.auto/perception/models/pointpainting/v4/pts_backbone_neck_head_pointpainting.onnx


# lidar_apollo_instance_segmentation

$ mkdir -p ~/autoware_data/lidar_apollo_instance_segmentation/
$ wget -P ~/autoware_data/lidar_apollo_instance_segmentation/ \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vlp-16.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/hdl-64.onnx \
       https://awf.ml.dev.web.auto/perception/models/lidar_apollo_instance_segmentation/vls-128.onnx


# lidar_centerpoint

$ mkdir -p ~/autoware_data/lidar_centerpoint/
$ wget -P ~/autoware_data/lidar_centerpoint/ \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx


# tensorrt_yolo

$ mkdir -p ~/autoware_data/tensorrt_yolo/
$ wget -P ~/autoware_data/tensorrt_yolo/ \
       https://awf.ml.dev.web.auto/perception/models/yolov3.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx \
       https://awf.ml.dev.web.auto/perception/models/coco.names


# tensorrt_yolox

$ mkdir -p ~/autoware_data/tensorrt_yolox/
$ wget -P ~/autoware_data/tensorrt_yolox/ \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt


# traffic_light_classifier

$ mkdir -p ~/autoware_data/traffic_light_classifier/
$ wget -P ~/autoware_data/traffic_light_classifier/ \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier/v2/lamp_labels.txt


# traffic_light_fine_detector

$ mkdir -p ~/autoware_data/traffic_light_fine_detector/
$ wget -P ~/autoware_data/traffic_light_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_yolox_s_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/tlr_yolox_s/v2/tlr_labels.txt


# traffic_light_ssd_fine_detector

$ mkdir -p ~/autoware_data/traffic_light_ssd_fine_detector/
$ wget -P ~/autoware_data/traffic_light_ssd_fine_detector/ \
       https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt
```
