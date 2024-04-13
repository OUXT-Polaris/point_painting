# point_painting
The PointPainting means to fuse the semantic segmentation results based  
on RGB images and add class scores to the raw LiDAR pointcloud.  
Prepared with reference to [the original paper](https://arxiv.org/pdf/1911.10150.pdf)

## subscribe
- `pointcloud` : LiDAR data  (/wamv/sensors/lidars/lidar_wamv_sensor/points)
- `segmentation_result` : Segmentation class and score results(/SegmentationInfo)

Receiving topics can be changed in `config/pointpainting_config.yaml`

## publish
- `point_painting` : point_painting result(/point_painting)


![Screencast from 2024年04月14日 07時09分19秒](https://github.com/OUXT-Polaris/point_painting/assets/82552894/cf63aa6a-de7a-458e-9d65-a2a85c77d294)
