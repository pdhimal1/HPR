#pragma once

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include <pcl/surface/convex_hull.h>

using namespace std;

typedef pcl::PointCloud <pcl::PointXYZ> PointCloudT;

namespace Ui {
    class PCLViewer;
}

class PCLViewer : public QMainWindow {
    Q_OBJECT

public:
    explicit PCLViewer(QWidget *parent = 0);

    ~PCLViewer();

public
    Q_SLOTS:

    void computeVisibility();

    void HPR();

    void radiusChanged(double value);

    void setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void setUp();

protected:
    pcl::visualization::PCLVisualizer::Ptr viewer;
    pcl::visualization::PCLVisualizer::Ptr visibilityViewer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;


    double radius = 100;

private:
    Ui::PCLViewer *ui;

};
