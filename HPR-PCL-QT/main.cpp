#include "pclviewer.h"
#include <QApplication>
#include <QMainWindow>
#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include<pcl/io/ply_io.h>

using namespace std;
using namespace pcl;
using namespace pcl::console;

/**
 * Main method.
 *
 * Reads the point cloud in PCD format.
 *
 * ./pcl_visualizer -pcdFile ../../data/data-master/tutorials/ism_test_michael.pcd
 *
 * @param argc
 * @param argv
 * @return
 */
int main (int argc, char** argv)
{
    std::string pcdFile;
    parse_argument (argc, argv, "-pcdFile", pcdFile);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdFile, *cloud) == -1) //* load the file
    {
        // try PLY
        pcl::PLYReader Reader;
        Reader.read(pcdFile, *cloud);
    }

    QApplication a (argc, argv);
    PCLViewer w;

    // set the point cloud
    w.setPointCloud(cloud);

    // set-up
    w.setUp();

    // show the window
    w.show ();

    return a.exec ();
}
