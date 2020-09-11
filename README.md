### Image Labeling Utils

This package is developed and used by MRS group to create labeled dataset in simulation using ground truth position of the object from gazebo-like simulators.
The 3d position of the object is projected into camera frame using the camera info topic in ROS format.

## Requirements:
- sdf file of the objects that satisfy the format for parsing it in scripts/artefact_gt_publisher.py
- odometry for the vehicle
- transformation from artefact frame to the frame of the vehicle odometry
- camera info - camera calibration and information from ROS.

## Output:
- The package outputs an image on /UAV_NAME/image_labeling_utils/artefact_boundings
![config/backpack_label.png](config/backpack_label.png)
- The boudning box is managed using the ROS rqt_reconfigure plugin, as shown in the picture, it also provides an offset to the artefact position, to surpress the odometry drift of your vehicle
![config/rqt_pic.png](config/rqt_pic.png)
- Using the rqt button "labeling_on" you can turn on image saving. The images are saved to dataset/images/<name_of_the_object>.time/*.png. The labels are saved to  dataset/labels/<name_of_the_object>.time/*.json.
- The label format is presented in two types: [labelme](https://github.com/wkentaro/labelme) and csv. In each folder you will get a json for each image, and a csv file for the whole directory (containing every frame).
- Json format is like following:
   ```json
   {
   "flags" : {},
   "imageData" : null,
   "imageHeight" : 480,
   "imagePath" : "../../images/backpack_5_92.980000/111.340000_0.png",
   "imageWidth" : 640,
   "shapes" : [
      {
         "flags" : {},
         "label" : "backpack",
         "points" : [
            [ 105.52516802928605, 364.03153186411134 ],
            [ 128.28468286033288, 343.0733904960552 ]
         ],
         "shape_type" : "rectangle"
      }
   ],
   "version" : "0.0.1"
  }
  ```

- the csv format is as follows:
  ```
    ../../images/backpack_3_214.968000/230.120000_0.png,x1,y1,x2,y2,obj_name
  ```
