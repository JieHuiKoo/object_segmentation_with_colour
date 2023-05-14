# object_segmentation_with_colour

About: This package takes in an RGB image, processes it, and outputs the information of the objects within the image

To run:
Execute the following command
```
rosrun object_segmentation_with_colour segmented_colour.py
```
The node will publish 3 topics:
1) armCamera/segmentedBlobs_AnnotatedImage - An RGB image annotated with bounding boxes surrounding objects within the image
2) armCamera/segmentedBlobs_RawImage - The raw RGB image for debugging purposes
3) armCamera/segmentedBlobs_BoundingBoxPoints - The coordinates of the bounding boxes surrounding objects within the image
   <br />
   <br />


![Screenshot from 2022-10-24 00-31-46](https://user-images.githubusercontent.com/31171083/197404846-715f3acf-c0db-4246-b9a4-b8f15bf21aae.png)
