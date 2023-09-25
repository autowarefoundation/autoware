# Autoware artifacts

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
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.onnx \
       https://awf.ml.dev.web.auto/perception/models/object_detection_yolox_s/v1/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table \
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

After downloading you can check intergity of the files with ```sha256sum```.

You need to put ```SHA256SUMS``` file listed below in to autoware_data directory:

```console
7fe62fcebe0e0f62a000d06aa94d779feb444d933671a4a3189fe01be8c19a00  ./image_projection_based_fusion/pts_backbone_neck_head_pointpainting.onnx
3ca452ea5ca9467bf782955f75704ba8466841e275e8b8acd991b9911d53249e  ./image_projection_based_fusion/pts_voxel_encoder_pointpainting.onnx
86348d8c4bced750f54288b01cc471c0d4f1ec9c693466169ef19413731e6ecc  ./lidar_apollo_instance_segmentation/hdl-64.onnx
eec521ebad7553d0ea2c90472a293aecb7499ab592632f0e100481c8196eb421  ./lidar_apollo_instance_segmentation/vlp-16.onnx
95ef950bb694bd6de91b7e47f5d191d557e92a7f5e2a6bdf655a8b5eed4075cc  ./lidar_apollo_instance_segmentation/vls-128.onnx
3fe7e128955646740c41a25be0c8f141d5a94594fe79d7405fe2a859e391542e  ./lidar_centerpoint/pts_backbone_neck_head_centerpoint.onnx
9bb0b634f3664bd098ce7d6a3d8a9fb7cc8d9b8252b27f302c71e43316bab551  ./lidar_centerpoint/pts_backbone_neck_head_centerpoint_tiny.onnx
dc1a876580d86ee7a341d543f8ade2ede7f43bd032dc5b44155b1f0175405764  ./lidar_centerpoint/pts_voxel_encoder_centerpoint.onnx
2c53465715c1fd2e9dc5727ef3fca74f4cdf0538f74286b0946e219d0ca5693b  ./lidar_centerpoint/pts_voxel_encoder_centerpoint_tiny.onnx
634a1132eb33f8091d60f2c346ababe8b905ae08387037aed883953b7329af84  ./tensorrt_yolo/coco.names
61e922f76918dd3d8e0abdc5fb7406f390609e08bd8ab9e5d3b97afb00f30f8c  ./tensorrt_yolo/yolov3.onnx
0e877c716fbf8a2b431ee3e57f6c7411a6741319b52c32c6dafc53c7e1b17027  ./tensorrt_yolo/yolov4-tiny.onnx
7c7343156c1bd4b397fd1e44b27334691a6219db3ce2e29a03b72af65ddb8f39  ./tensorrt_yolo/yolov4.onnx
a627e5f70180a8746482b572194090466db62c8d1256602c1cd20374dd960e34  ./tensorrt_yolo/yolov5l.onnx
ee6f67f7c00a34cc4cef2fdd9db30dd714df1a4fb2d7e9fc1731cfe85b673133  ./tensorrt_yolo/yolov5m.onnx
be335ff7746957debf1a6903a61fa3f568b780b4afe4958edf2d4bc98e9e0825  ./tensorrt_yolo/yolov5s.onnx
d7cb4cd7078f87bda22a37828d72867accecedf9f74d0d87b5cc1f6f1180a019  ./tensorrt_yolo/yolov5x.onnx
3540a365bfd6d8afb1b5d8df4ec47f82cb984760d3270c9b41dbbb3422d09a0c  ./tensorrt_yolox/label.txt
cc378d327db5616b0b3a4d077bf37100c25a50ecd22d2b542f54098da100f34c  ./tensorrt_yolox/yolox-sPlus-T4-960x960-pseudo-finetune.EntropyV2-calibration.table
f5054e8a890c3be86dc1b4b89a5a36fb2279d4f6110b0159e793be062641bf65  ./tensorrt_yolox/yolox-sPlus-T4-960x960-pseudo-finetune.onnx
b9e9d7da33342262ccaea4469b4d02b8abb32b6d7bf737f9e0883fece1b8f580  ./tensorrt_yolox/yolox-sPlus-opt.EntropyV2-calibration.table
36b0832177b01e6b278e00c7369f1de71e616c36261cbae50f0753d41289da01  ./tensorrt_yolox/yolox-sPlus-opt.onnx
471a665f4243e654dff62578394e508db22ee29fe65d9e389dfc3b0f2dee1255  ./tensorrt_yolox/yolox-tiny.onnx
1a5a49eeec5593963eab8d70f48b8a01bfb07e753e9688eb1510ad26e803579d  ./traffic_light_classifier/lamp_labels.txt
55ebb0d117a5e8943f8d1c6769f1d856b533079d4d871d8e923255cc992ad48a  ./traffic_light_classifier/traffic_light_classifier_efficientNet_b1_batch_1.onnx
684e29843e3128eadb774018730644b3ab9b0a06dc4cdaeed579c2f3fa5d5265  ./traffic_light_classifier/traffic_light_classifier_efficientNet_b1_batch_4.onnx
44d94540fa8b89dfb39cd9a8523cf010ddfb10ea2f1f9b53bf3618ce7f4912ad  ./traffic_light_classifier/traffic_light_classifier_efficientNet_b1_batch_6.onnx
455b71b3b20d3a96aa0e49f32714ba50421f668a2f9b9907c30b1346ac8a3703  ./traffic_light_classifier/traffic_light_classifier_mobilenetv2_batch_1.onnx
41bb79a23a4ac57956adb8e9cb3904420db1b0cd032e97b670cc4f8b174ae3fe  ./traffic_light_classifier/traffic_light_classifier_mobilenetv2_batch_4.onnx
e4792eed6a46fdbd02be2f3a4f1ce91f36fa77698493caf3102e445178c0f058  ./traffic_light_classifier/traffic_light_classifier_mobilenetv2_batch_6.onnx
a41e6e3324e32c30b3b2fe38908eaf3471e2bfdaeb9e14ca0c1c3bc0275119c6  ./traffic_light_fine_detector/tlr_labels.txt
922839fcf22bd32ae5065146fcec193e9d6360ca03bd4c83faea835045daf8eb  ./traffic_light_fine_detector/tlr_yolox_s_batch_1.onnx
b3c6e00acc6ff547d165469684ffb620a9a6330e9d591d445f50c4cf5cb4e292  ./traffic_light_fine_detector/tlr_yolox_s_batch_4.onnx
2824d4c5b7ab5f6bfd41e43e82747107c53e1c727b1cf1dd6746bc49e6749128  ./traffic_light_fine_detector/tlr_yolox_s_batch_6.onnx
e29e6ee68751a270fb285fd037713939ca7f61a897b4c3a7ab22b0d6a9a21ddf  ./traffic_light_ssd_fine_detector/mb2-ssd-lite-tlr.onnx
a41e6e3324e32c30b3b2fe38908eaf3471e2bfdaeb9e14ca0c1c3bc0275119c6  ./traffic_light_ssd_fine_detector/voc_labels_tl.txt
1f660e15f95074bade32b1f80dbf618e9cee1f0b9f76d3f4671cb9be7f56eb3a  ./yabloc_pose_initializer/resources.tar.gz
```

And run the command:

```console
sha256sum -c SHA256SUMS 2>&1
```
