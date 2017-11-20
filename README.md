# Kalman Filter Compressive Tracking 
Kalman Filter Compressive Tracking (KFCT) is a modified version of the original Compressive Tracking (CT) presented by Zhang <i>et al.</I> in <a href="http://www4.comp.polyu.edu.hk/~cslzhang/CT/eccv_ct_camera.pdf" target="_blank">Real-time Compressive Tracking</a>. 
In such a version, a Kalman Filter has been implemented for supporting the original CT, where the rectangles, containing the possible candidate, are sampled around the prediction of the CT.

<p align="center">
<a href="https://www.youtube.com/watch?v=_CEy4R6JTZk"  target="_blank"><img src="https://img.youtube.com/vi/_CEy4R6JTZk/0.jpg"/></a>
</p>
<br>

## Requirements
* OpenCV

## How to build

KCFT works under Linux environments. I recommend a so-called out of source build which can be achieved by the following command sequence:

* mkdir build
* cd build
* cmake ../
* make -j<number-of-cores+1>

## How to use

Go to the main KCFT directory and launch one of the scripts:

```bash
sh scripts/deer_kalman.sh
```

Or if you want to use your own set of images, go into the bin folder and launch the script as:
```bash
./ct -kalman x y w h dt path/to/the/image/folder
```
for the Kaman version.

Or:
```bash
./ct -origin x y w h path/to/the/image/folder
```

for the original version.

Where:
* x,y,w,h: are the coordinates of the first rectangle
* dt: is the parameter of the Kalman Filter.