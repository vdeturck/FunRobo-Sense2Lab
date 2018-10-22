/*
 * Collects data from a Lidar scan, groups data points and identifies them as walls or objects,
 * then displays data in a nice visual
 *
 * Last edited: 10/22/18
 * Author: Amy Phung
 * Editors: Viktor Deturck, Cali Wierzbanowski
 *
 * Requires:
 *  flexiport (dependency of hokuyoaist)
 *  hokuyoaist (library for getting data from lidar)
 *  matplotlib (plotting library)
 *
 * Notes:
 * *************************************PLEASE READ HISTORY OF LIDAR IN DRIVE FIRST*************************************
 *
 *  Since matplotlib library is usually a python library and we're using a C++ wrapper for it,
 *  real-time plotting doesn't work. If REAL_TIME_VISUAL is set to false, a single frame of the
 *  color coded plot will be displayed. If this is set to true, the data will be printed in the
 *  command line real-time but with no formatting or color coding.
 *  The real-time data we are getting we want to superimpose on top of the video image in a different color to show
 *  whether an object is there or not.
 *
 *
 */

#include <iostream>
#include <flexiport/flexiport.h>
#include <hokuyoaist/hokuyoaist.h>
#include <hokuyoaist/hokuyo_errors.h>
#include "matplotlibcpp.h"

bool REAL_TIME_VISUAL = true; // set to true for no-color, real-time display

//TODO: Make code not reliant on perpendicular wall (Check)
//TODO: Handle the case when a person/object is in the middle of a scan (Check)
//TODO: Have an object threshold size to avoid picking up noise as objects (Check)
//TODO: Make pretty real-time visualization
//TODO: Use webcam input to determine color of objects
//TODO: Fill in code for pointCluster member functions
//TODO: Determine units of distance measurement
//TODO: Remove unneccessary hardcode (Partly removed)

//Current progress:
// Structure of saving attributes to detected features works.
// Can currently detect start and end point, and whether cluster is a hole or object.

//CODE START************************************************************//
// Use matplotlib C++ wrapper for python
namespace plt = matplotlibcpp;

//pointCluster CLASS****************************************************//
class pointCluster // For each cluster of points, a pointCluster object is
                   // created to save attributes of the cluster
{
public: // Public data members can be accessed anywhere in code
    // Data Members
    int type_id; // 1 for object, 0 for wall
    int start_pt; // Index of start point
    int end_pt; // Index of end point

    /*
    // Member Functions
    int computeWidth()
    {
        // Fill in code here to determine object/hole width in meters given start pt and end pt
        // Keep in mind that a constant interval of index does not necessarily mean a constant width in meters
    }

    std::vector<int> computeCenter()
    {
        // Compute the "center of mass" of the cluster by averaging x and y values
    }

    float computeAngle()
    {
        // Compute the angle of the "center of mass" of the cluster from the lidar - assume straight is 0,
        // left is positive, and right is negative (right hand rule conventions)
    }

    float computeDistance()
    {
        // Compute the distance to the "center of mass" from the lidar
    }
     */
    void print() // Print data for verification
    {
        std::cout << "Type ID:" << std::endl;
        std::cout << type_id << std::endl;
        std::cout << "Start Point:" << std::endl;
        std::cout << start_pt << std::endl;
        std::cout << "End Point:" << std::endl;
        std::cout << end_pt << std::endl;
    }
};

//GLOBAL DATASETS*******************************************************//
// Prepare data.
std::vector<double> x(682), y(682); // 682 is the number of points the lidar scans. This was a workaround
                                    // since I couldn't figure out how to give the callback function in for_each
                                    // parameters, so I had to make this a global variable - eventally, it'd be a good
                                    // idea to make this not global

//FUNCTION PROTOTYPES***************************************************//
int checkPoint(int index, std::vector<double> rangedata, std::vector<double> firstRanges, int tolerance, std::vector<int> ids, int threshold);
std::vector<pointCluster> clusterPoints(int max_pts, int threshold, std::vector<double> rangedata, std::vector<double> angledata, std::vector<double> firstRanges,int tolerance, std::vector<int> ids);
void plotCluster(pointCluster cluster);


//MAIN FUNCTION********************************************************//
int main(int argc, char **argv)
{
    // Variables
    std::string port_options("type=serial,device=/dev/ttyACM0,timeout=1");
    int multiecho_mode(0);
    unsigned int speed(0), cluster_count(1);
    int num_pts = 682; // In lidar documentation - "Scanable steps: 682"
    int midpoint = num_pts/2; // Midpoint is in middle of scan
    int threshold = 200; // Hardcoded difference between midpoint and object to warrant detection
    const float res = 0.00613592; // Resolution in radians/step - from lidar documentation
    float angle;
    int range;
    hokuyoaist::ScanData data; // For saving raw lidar data
    std::vector<pointCluster> clusters; // For saving vector containing all pointCluster objects
    std::vector<double> rangedata(num_pts), angledata(num_pts); // For saving the range and angle data in an easy-to-manipulate vector
    std::vector<int> ids(num_pts);   // stores id's of the data points (0 or 1)
    std::vector<double> firstRanges; // Vector with all ranges of the first scan
    int scanCount = 0;
    int tolerance = 5; // The tolerance used to distinguish noise from objects
    int changeCounter;


// Variables no longer needed (Because Lidar isn't dependent anymore on a perpendicular wall)
//    int wall_start = 200; // Hardcoded wall points, 341 is midpoint
//    int wall_end = 500;
//    int wall_distance;
//

#if defined(WIN32) //define a port
    port_options = "type=serial,device=COM4,timeout=1";
#endif // defined(WIN32)

    try
    {
        hokuyoaist::Sensor laser; // Create new laser scanner object

        // Open the laser
        laser.open(port_options);

        // Calibrate the laser time stamp
        std::cout << "Calibrating laser time\n";
        laser.calibrate_time();
        std::cout << "Calculated offset: " << laser.time_offset() << "ns\n";
        std::cout << "Calculated drift rate: " << laser.drift_rate() << '\n';
        std::cout << "Calculated skew alpha: " << laser.skew_alpha() << '\n';

        // Turn the laser on
        laser.set_power(true);

        // I'm not sure what this block of code does but it seems important *******************************
        // Set the motor speed
        try
        {
            laser.set_motor_speed(speed);
        }
        catch(hokuyoaist::MotorSpeedError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        catch(hokuyoaist::ResponseError &e)
        {
            std::cerr << "Failed to set motor speed: " << e.what() << '\n';
        }
        // Set multiecho mode
        switch(multiecho_mode)
        {
            case 1:
                laser.set_multiecho_mode(hokuyoaist::ME_FRONT);
                break;
            case 2:
                laser.set_multiecho_mode(hokuyoaist::ME_MIDDLE);
                break;
            case 3:
                laser.set_multiecho_mode(hokuyoaist::ME_REAR);
                break;
            case 4:
                laser.set_multiecho_mode(hokuyoaist::ME_AVERAGE);
                break;
            case 0:
            default:
                laser.set_multiecho_mode(hokuyoaist::ME_OFF);
                break;
        }

        //**********************************************************************************************

        // Get some laser info
        std::cout << "Laser sensor information:\n";
        hokuyoaist::SensorInfo info;    // SensorInfo object from hokuyoaist
        laser.get_sensor_info(info);    // method of laser
        std::cout << info.as_string();

        while(true)
        {
            laser.get_ranges(data, -1, -1, cluster_count);
            for (int i=tolerance; i<682.; i++)
            {
                // Create laserdata dataset
                range = data[i];
                rangedata.at(i) = range;

                // Create angledata dataset
                angle = (i-midpoint)*res; // Angle, measured from the middle of the LIDAR
                angledata.at(i) = angle;

                // Save cartesian coordinates to plotting vectors
                x.at(i) = range*sin(angle);
                y.at(i) = range*cos(angle);
                if (scanCount > 0) { // Don't check the points of the first scan (nothing to compare to)
                    ids.at(i) = checkPoint(i,rangedata,firstRanges,tolerance, ids, threshold); // Will put a 1 or 0 in a vector on place i
                }
            }
            if (scanCount == 0) // First scan
            {
                firstRanges = rangedata;    // Saves all the points of the first scan to compare them when things change
            }

            else
                {
                    clusters = clusterPoints(num_pts, threshold, rangedata, angledata, firstRanges, tolerance,ids); // Find clusters in dataset
                    std::reverse(ids.begin(),ids.end()); // reverses the vector

                    // Filters out the noise with respect to the tolerance. #Todo: Make a function out of this to make it prettier (add it to the checkpoint function)
                    for (int j = 1; j<ids.size()-tolerance;j++)
                    {
                        if (ids.at(j) != ids.at(j-1))
                        {
                            changeCounter = 0; // counts how many changes (1->0 or reversed) are happening over a length equal to the tolerance, if there's more than 1, the noise will be filtered
                            for (int k =0; k < tolerance+1; k++  )
                            {
                                if (ids.at(j) != ids.at(j+k)) // If two adjacent points are not equal
                                {
                                    changeCounter ++;
                                }
                            }
                            if (changeCounter > 1) // If there were multiple changes in the tolerance range
                            {
                                ids.at(j) = ids.at(j-1); // Change the value to that of the adjacent point.
                            }
                        }
                        std::cout<<ids.at(j);   // Prints out every element of the vector
                    }
                    std::cout << std::endl;
            }
            scanCount ++;

            if (REAL_TIME_VISUAL == false) // Code will only run once to gather data if REAL_TIME_VISUAL is set to false
            {
                break;
            }
        }

        // Plot the data
        // Set the size of output image = 1200x780 pixels
        plt::figure_size(1200, 780);
        // Plot line from given x and y data. Color is selected automatically.
        plt::plot(x, y, "b*");
        for_each(clusters.begin(), clusters.end(), plotCluster);
        // Set x-axis to interval [0,1000000]
        plt::xlim(0, 1000*1000);
        // Enable legend.
        plt::legend();
        plt::axis("equal");
        // Save the image (file format is determined by the extension)
        plt::show();

        // Close the laser
        laser.close();
    }
    catch(hokuyoaist::BaseError &e)
    {
        std::cerr << "Caught exception: " << e.what() << '\n';
        return 1;
    }
    return 0;
}

//FUNCTIONS*************************************************************//
int checkPoint(int index, std::vector<double> rangedata, std::vector<double> firstRanges, int tolerance, std::vector<int> ids, int threshold)
{
    int id = 0; // ID system: flag 0 for wall, 1 for object
    int changeCounter = 0;
    int avg = 0;
    int sum = 0;

    if (abs(rangedata.at(index)-firstRanges.at(index)) > threshold)       // If it detects points that are more than 20 cm difference with the first scanned flag, flag as object
    {
        id = 1;
    }
    // Idea of making an average of the previous values
//    for (int i = 0; i < tolerance; i++)     // Goes back through the id's of until index - tolerance to cancel out noise (fast changes in id)
//    {
//        sum += ids.at(index-i-1);           // calculates sum to take avg
//        if (ids.at(index) != ids.at(index-i))  // if element changes more than one time in the resolution, average of the elements is taken
//        {
//            changeCounter ++;
//        }
//        if (changeCounter > 1)
//        {   avg = sum/i;                    // average
//            id = static_cast<int>(avg);     // converts float into string
//        }
//    }
    return id;
}

std::vector<pointCluster> clusterPoints(int max_pts, int threshold, std::vector<double> rangedata, std::vector<double> angledata, std::vector<double> firstRanges,int tolerance, std::vector<int> ids)
{ // Datastructure: vector containing pointClusters
    std::vector<pointCluster> all_clusters(100); // arbitrarily set max number of things to detect to 100
    std::vector<int> new_cluster_points(2); //make an array to save start and end points
    pointCluster new_cluster; // make new pointCluster object

    for (int i=1; i<max_pts; i++) // Checks if the adjacent points have the same id - object points will have an id of 1, wall points will have an id of 0
    {
        int id_1;
        int id_2;
        new_cluster_points.at(0) = i; // Start point of cluster is the first index
        bool same_id = true;
        while (same_id == true) // While adjacent points have the same id
        {
            i++; // increment i
            if(i >= max_pts-1) // if i goes past last data point, break out of loop
            {
                new_cluster_points.at(1) = (i-1);
                break;
            }
            id_1 = ids.at(i);
            id_2 = ids.at(i-1); // check id of adjacent point
//            std::cout << id_1; // Print of a point
            if (id_1 != id_2) // if the two points aren't both walls or objects
            {
                new_cluster_points.at(1) = (i); // log the last point as the current index
                same_id = false; // break out of while loop
            }
        }

        new_cluster.start_pt = new_cluster_points.at(0); // set start point of cluster equal to first saved point
        new_cluster.end_pt = new_cluster_points.at(1); // set last point of cluster equal to second saved point

        if (id_2 == 1)
        { new_cluster.type_id = 1; } // assign id to cluster
        else
        { new_cluster.type_id = 0; } // assign id to cluster

        all_clusters.push_back(new_cluster); // Add new cluster to all_clusters vector
    }
    return all_clusters; // return vector containing all the found clusters (print them)
}

void plotCluster(pointCluster cluster)
{
    std::vector<int> cluster_x(800); // arbritrarliy set max number of points in cluster
    std::vector<int> cluster_y(800);

    for (int i=cluster.start_pt; i<=cluster.end_pt; i++)
    {
        cluster_x.push_back(::x.at(i));//using global datatset
        cluster_y.push_back(::y.at(i));
    }
    if (cluster.type_id == 0) // Plot wall as red
    {
        plt::plot(cluster_x, cluster_y, "r*");
    }
    else // Plot objects as green
    {
        plt::plot(cluster_x, cluster_y, "g*");
    }
}