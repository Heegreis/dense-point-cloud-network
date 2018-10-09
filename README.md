# dense-point-cloud-network
This project uses the kitti dataset, but the original point cloud data is in bin format.
In order to facilitate the use of Point Cloud Library (PCL), we convert it to pcd format file.

## Convert .bin to .pcd format file
Convert the point cloud bin format file of the kitti data set to the pcd format file.
### Usage
Create a `build` folder under the `bin2pcd` folder.
Under the `build` folder compiler the program and run it:
```shell
project_root/bin2pcd/build$ cmake ..
project_root/bin2pcd/build$ make
project_root/bin2pcd/build$ ./bin2pcd
```
It will creat converted files under the default path.

## extract object
Extract the point cloud of object in th every frame of a sequence from kitti raw data ,for which kitti dataset provided the labels and save as pcd format.
### Usage
Create a `build` folder under the `extract_object` folder.
Under the `build` folder compiler the program and run it:
```shell
project_root/extract_object/build$ cmake ..
project_root/extract_object/build$ make
project_root/extract_object/build$ ./bin2pcd
```
It will creat extracted files under the default path.

**Note:** The feature doesn't set objects to the coordinate origin.
