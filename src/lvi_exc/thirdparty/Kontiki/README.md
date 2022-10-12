Kontiki - the continuous time toolkit
====================================

A modified versions of [Kontiki](https://github.com/hovren/kontiki) used in [LI_Calib](https://github.com/APRIL-ZJU/lidar_IMU_calib). 

Major changes:

- Remove the interface of Python

- Remove the dependence of Sophus

  Sophus is only required at [se3_spline_trajectory](https://github.com/hovren/kontiki/blob/master/cpplib/include/kontiki/trajectories/uniform_se3_spline_trajectory.h). Since we model the 6DoF  trajectory with a split representation, we delete that file for removing the dependence of Sophus. If you are interested in exploring the difference between different representation of trajectories, you cloud add it back to the project.

- Add LiDAR sensor

- Add LiDAR measurements

----

Kontiki is a toolkit for continuous-time structure from motion.
In short, it can estimate a trajectory (and 3D structure) from a set of measurements.

The documentation is available at https://hovren.github.io/kontiki/.



If you find Kontiki useful, please cite that fantastic work [using the following Bibtex entry](https://hovren.github.io/kontiki/index.html#citation):

```
@misc{kontiki,
  author = {Hannes Ovr\'en},
  title = "Kontiki - the continuous time toolkit",
  howpublished = "\url{https://github.com/hovren/kontiki}",
  year = {2018}
}
```

