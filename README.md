# Adaptative Sampling of 3D point cloud

*This subproject is in progress, and not yet working.*

We aim to preprocess 3D points clouds such as Semantic-8 as an alternative to voxelisation or random sampling.
We keep less points where there are planes, and more where there is complex details.

## Dependancies

We use the librairy PCL.

## Install and usage

```
git clone https://github.com/mathieuorhan/curvature-sampling.git
cd curvature-sampling
mkdir build
cd build
cmake ..
make
./sampling path/to/data/file
```