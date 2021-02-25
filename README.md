# asscpp
Adaptive Search Space Coverage Path Planning

# Installation
In your [catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace):

`git clone --recurse-submodules https://github.com/psilberberg/asscpp.git`

## Dependencies
In addition to `ros-noetic-desktop-full`, please install:
[rviz_visual_tools](https://github.com/PickNikRobotics/rviz_visual_tools ), 
[octomap](http://wiki.ros.org/octomap), 
[octomap_mapping](http://wiki.ros.org/octomap_mapping ), 
[octomap_rviz_plugins](http://wiki.ros.org/octomap_rviz_plugins), 
[CGAL](https://www.cgal.org/download/linux.html), 

```
sudo apt-get install ros-noetic-rviz-visual-tools ros-noetic-octomap ros-noetic-octomap-mapping ros-noetic-octomap-rviz-plugins libcgal-dev
```


## Citing
Please cite the following papers when using the work for your research:

[ASSCPP](https://ieeexplore.ieee.org/abstract/document/8593719)
```bibtex
@INPROCEEDINGS{almadhoun2018asscpp,
  author={R. {Almadhoun} and T. {Taha} and D. {Gan} and J. {Dias} and Y. {Zweiri} and L. {Seneviratne}},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={Coverage Path Planning with Adaptive Viewpoint Sampling to Construct 3D Models of Complex Structures for the Purpose of Inspection}, 
  year={2018},
  volume={},
  number={},
  pages={7047-7054}
}
```


[SSCPP](https://link.springer.com/chapter/10.1007/978-3-030-27541-9_21)
```bibtex
@InProceedings{almadhoun2019sscpp,
  author="Almadhoun, Randa and Taha, Tarek and Dias, Jorge and Seneviratne, Lakmal and Zweiri, Yahya",
  title="Coverage Path Planning for Complex Structures Inspection Using Unmanned Aerial Vehicle (UAV)",
  booktitle="Intelligent Robotics and Applications",
  year="2019",
  publisher="Springer International Publishing",
  address="Cham",
  pages="243--266",
  isbn="978-3-030-27541-9"
}
```

