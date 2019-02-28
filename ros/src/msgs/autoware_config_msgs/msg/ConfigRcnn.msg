Header  header
float32 score_threshold #minimum score required to keep the detection [0.0, 1.0] (default 0.6)
float32 group_threshold #minimum overlap percentage area required to supress detections(NMS threshold) [0.0, 1.0] (default 0.3)
float32 slices_overlap  #overlap percentage between image slices [0.0, 1.0] (default 0.7)
float32 b_mean
float32 g_mean
float32 r_mean
uint8   image_slices    #number of times to slice the image and search (1, 100], larger value might improve detection but reduce performance (default 16)
uint8   use_gpu
uint8   gpu_device_id