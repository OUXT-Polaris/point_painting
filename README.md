# point_painting
The PointPainting means to fuse the semantic segmentation results based  
on RGB images and add class scores to the raw LiDAR pointcloud.  
Prepared with reference to [the original paper](https://arxiv.org/pdf/1911.10150.pdf)

## subscribe
- `pointcloud` : LiDAR data 
- `segmentation_result` : Segmentation class and score results(/SegmentationInfo)

Receiving topics can be changed in `config/pointpainting_config.yaml`

## publish
- `point_painting` : point_painting result(/point_painting)



https://github.com/OUXT-Polaris/point_painting/assets/82552894/d23173d4-4e3f-45e6-a366-ca3db836a0b7

