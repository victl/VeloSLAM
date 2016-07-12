VeloSLAM

This is the early development of a library that partially based on the
Kitware/VeloView project.

The VeloView is basically an application that is focused on point cloud
acquisition and visualization. It is also deeply coupled with the vtk's 'pipeline'
workflow, which does not fit my target very well. My aim is to build a library
that:
- Does not couple with vtk.
- Suited for online processing of point clouds.
- Incorporate state-out-the-art SLAM algorithms avialable nowadays.
- Enable the possibilities of developing new algorithms and applications of
  point clouds (using Velodyne LiDARs).

Here are some features I would like to (or already) implemented:
- [x] Online point cloud acquisition.
- [x] A vehicle motion model that adjust point clouds according to vehicle
  movements.
- [x] Time syncing with INS/GPS devices
- [x] An unified memory model that automatically deals the swapping of data
  from/to external hard disk. This lower level api is invisible to upper layer
  users. They just query for some data, if the data isn't in memory, it'll be
  read from disk. (partially complete)
- [ ] LiDAR calibration (This is necessary for old Velodyne LiDARs)
- [ ] Implement various SLAM algorithms.

THIS LIBRARY IS CURRENTLY IN-COMPLETE AND ERROR PRONE RIGHT NOW.
I'm working hard on it to make it more usable. If you are interested, have any
suggestion, or even would like to join the development, you can contact me via:
gvictl@gmail.com
or
victl@163.com
