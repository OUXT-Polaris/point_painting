# point_painting
The PointPainting means to fuse the semantic segmentation results based on RGB images and add class scores to the raw LiDAR pointcloud .

## subscribe
- `pointcloud` : LiDAR data  (/wamv/sensors/lidars/lidar_wamv_sensor/points)
- `segmentation_result` : Segmentation class and score results(/SegmentationInfo)

Receiving topics can be changed in `config/pointpainting_config.yaml`

## publish
- `point_painting` : point_painting result(/point_painting)
