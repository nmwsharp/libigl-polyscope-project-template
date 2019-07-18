# libIGL & Polyscope Example Project

Demonstrates basic functionality and project setup for using Polyscope with libIGL.

IGL code mimicks examples from the IGL tutorials.


## To download and build

```
git clone --recurse-submodules https://github.com/nmwsharp/libigl-polyscope-project-template.git
cd libigl-polyscope-project-template
mkdir build
cd build
cmake ..
make -j4

./bin/libIGL-polyscope-example ../bunnyhead.obj

./bin/libIGL-polyscope-example ../spot.obj
```

## Usage

After running the demo appliation as above, try clicking around the command UI in the upper right corner.

Notice that as quantities are added to Polyscope, they appear in the selection window on the left.

Try clicking a vertex to see the quantities associated with that vertex. Clicking may be easier if you first show edge by dragging the "edge width" slider in the left panel.

Check out `src/main.cpp` to see how easy it is! There are only 12 lines of Polyscope code in the whole demo to generate this visualization.
