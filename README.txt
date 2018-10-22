First Iteration:
A first installation guide has been made with all the instructions of how to install the libraries and drivers for the lidar. 
First code has been written, which depends on a perpendicular wall. It measures the distance to the wall (range at the middle point) and compares the rest of the ranges with that range. If it’s bigger than a pre-defined threshold, it tags the point as an object (a 1), otherwise it’s a hole (0). It would calculate and print out all the points id’s (1 or 0) after each other until a next scan is initialized. There is a function that recognizes clusters, which are collections of equal point id’s next to each other. A cluster can thus be seen as an object. 

Second Iteration:
We went through the installation process and added some extra steps and modified others, so the process would be less time consuming. 
We had some ideas to have a more flexible lidar. The code was modified so that the lidar does a first scan, saves all the range values and compares the next scans to the first values. If they surpass a certain threshold (It’s set on 20cm for now) it’s seen as an object, otherwise as a hole. This is still not a perfect way of doing it, and it also introduces a lot more noise, so we added some code to take out some of the noise, based on a tolerance. Basically, the tolerance is the minimum number of equal id’s there need to be, so you can’t have clusters smaller than the tolerance. We changed some of the code structure, for example: we now store all the id’s in a vector (called ids) and print every element in the vector instead of just printing them without saving. 

Recommendation for next iteration:
Maybe think about a better way of detecting objects, independent of the wall. If the object is already in the first scan, it will give some error results when it moves. 
Instead of printing the vector (ids) in the terminal, use it to remap a part of it to the amount of pixels on the Microsoft webcam, dependent on the viewing angle and resolution. Then print the object clusters on the live feed. 
Writing other functions than 
Making a real time map of the lidar. The data is already saved into vectors, but needs to be plotted in real time. Now we are able to do it one time (if REAL-TIME-VISUAL == false) and use the c++ wrapper for the Python MatPlotLib library.  -> Do research on different plotting libraries. 



