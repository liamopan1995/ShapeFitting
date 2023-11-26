#TODO: guideline for compiling this work. ( which package to install and so on.)

## Main update

The CloudSegmentation.cpp file has been updated,  so that it can further cluster points that have been segmented as one stem, into  smaller clusters, each of of the smaller clusters is likely to form a circle.

## GpsImuIntegral.cpp

- This node at present runs seperately in a node  , later this function has to be integrated into our main node in CloudSegmentation.cpp
- This node does the job of converting  the vihecle's GPS coordnates (longtitude and latitude coordinates )into UTM coordinates 

## CircleFitting 
- Circle fitting algorithms implementation and the helper function are in folder : Circle_Fitting_CPP
- Visualization and Validations of algorithms of 2D/3D circle fittings are in colab file : DiskFitting.ipynd
- You can access the Colab notebook [here](https://colab.research.google.com/drive/1mpwGbVmmLUSy8GPRjVKwKurZ131K-x20?usp=sharing).





