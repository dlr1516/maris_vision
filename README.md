#  MARIS Vision - ROS Packages of MARIS Project Vision System 
#### Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
#### With contributions by Fabio Oleari.


OVERVIEW
-------------------------------------------------

The software distributed with MARIS Vision is a collection of ROS packages 
implementing the software of vision system used in MARIS 
(Marine Autonomous Robotics for InterventionS). 
MARIS aims at the development of autonomous underwater vehicles capable 
also of interacting with and in the underwater environment by object manipulation. 
The Research Unit of the University of Parma has developed the stereo vision 
system for detection, pose estimation and grasp planning for object manipulation. 


If you use this library, please cite one of the following papers: 

```    
    @article{lodirizzini2017caee,
      author={Lodi Rizzini, D. and Kallasi, F. and Aleotti, J. and Oleari, F. and Caselli, S.},
      title={{Integration of a Stereo Vision System into an Autonomous Underwater Vehicle for Pipe Manipulation Tasks}}
      journal={Computers and Electrical Engineering (CAEE)},
      volume={58},
      pages={560--571},
      month={feb},
      year={2017},
      issn = {0045-7906},
      doi = {10.1016/j.compeleceng.2016.08.023},
      note = {DOI 10.1016/j.compeleceng.2016.08.023, EID 2-s2.0-84994797689},
    }
    
    @article{lodirizzini2015ijars,
      author = {Lodi Rizzini, D. and Kallasi, F. and Oleari, F. and Caselli, S.},
      title = {{Investigation of Vision-based Underwater Object Detection with Multiple Datasets}},
      journal = {International Journal of Advanced Robotic Systems (IJARS)},
      volume = {12},
      number = {77},
      pages = {1--13},
      month = {may},
      year = {2015},
      publisher = {InTech},
      doi = {10.5772/60526},
      note = {DOI 10.5772/60526},
    }
```    

or the most relevant associated publications by visiting: 
http://rimlab.ce.unipr.it/FALKOLib.html


DEPENDENCIES
-------------------------------------------------

The software requires the Robot Operating System (ROS), http://www.ros.org/, and has been tested with ROS Indigo and Jade. 
Beside ROS tools and libraries, it requires

- Boost 
- Eigen 3.0 
- OpenCV >= 2.4
- EDLines (included in the project as external library)

The library also requires the third party library EDLines
developed by C. Akinlar, which has been included in folder external.

