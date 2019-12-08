#include "pclviewer.h"
#include "ui_pclviewer.h"


PCLViewer::PCLViewer(QWidget *parent) :
        QMainWindow(parent),
        ui(new Ui::PCLViewer) {
    ui->setupUi(this);
    this->setWindowTitle("The Point Cloud viewer with Hidden Point Removal");
}


void
PCLViewer::setUp() {
    // Set up the viewers
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    viewer->addText("Original Cloud", 70, 10, "v1 text");

    visibilityViewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    visibilityViewer->addText("Visibility Cloud", 70, 10, "v2 text");

    // Set up the Original Cloud viewer window
    ui->OriginalViewer->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->OriginalViewer->GetInteractor(), ui->OriginalViewer->GetRenderWindow());
    ui->OriginalViewer->update();

    // Set up the visibility cloud viewer window
    ui->visibilityViewer->SetRenderWindow(visibilityViewer->getRenderWindow());
    visibilityViewer->setupInteractor(ui->visibilityViewer->GetInteractor(), ui->visibilityViewer->GetRenderWindow());
    ui->visibilityViewer->update();

    // Connect "compute" button and the function
    connect(ui->computeButton, SIGNAL(clicked()), this, SLOT(computeVisibility()));

    // connect the "radius
    connect(ui->Radius, SIGNAL(valueChanged(double)), this, SLOT(radiusChanged(double)));

    // add point clouds
    viewer->addPointCloud(cloud, "cloud");
    viewer->resetCamera();

    // initially, this will get the original point cloud.
    visibilityViewer->addPointCloud(cloud, "cloud");
    visibilityViewer->resetCamera();

    ui->OriginalViewer->update();
    ui->visibilityViewer->update();
}

void
PCLViewer::setPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {
    cloud = inputCloud;
}


void
PCLViewer::computeVisibility() {
    HPR();
}


/**
 * This is the implementation of the Hidden Point Removal operator described
 * in Katz et. al. 'Direct Visibility of Point Sets', 2007.
 *
 * =====================================================================================================================
 * The Radius:
 * The importance of radius as described in the paper.
 *
 * Suppose that P is a ρ-sample of S, i.e., if we surround each sample point pi ∈ P by
 * an open ball of radius ρ, the surface S will be completely contained within the union of these balls.
 *
 *
 * In the matlab code (provided by the author):
 * The code is very simple and short and works for D-dimensional point clouds. The only parameter that the operator gets
 * is used to compute radius R. This parameter can be either set by the user, or R can be computed automatically.
 *
 * todo - compute the radius here?
 */
void
PCLViewer::HPR() {
    /**
     * Given a set of points P = {pi |1 ≤ i ≤ n} ⊂ ℜD,
     * which is considered a sampling of a continuous surface S, and a viewpoint (camera position) C,
     * our goal is to determine ∀pi ∈ P whether pi is visible from C.
     */
    // cam is the C, cloud_hull holds all the points visible from C
    std::vector <pcl::visualization::Camera> cam;
    viewer->getCameras(cam);

    pcl::PointXYZ cameraPoint = pcl::PointXYZ(cam[0].pos[0], cam[0].pos[1], cam[0].pos[2]);

    // pcl::PointXYZ cameraPoint = cloud->points[0];
    Eigen::Vector3d camera_location(cameraPoint.x, cameraPoint.y, cameraPoint.z);

    std::vector <Eigen::Vector3d> spherical_projection;

    pcl::PointCloud<pcl::PointXYZ>::Ptr newCloud(new pcl::PointCloud <pcl::PointXYZ>);

    /**
     * Step 1:
     * Perform spherical projection
     *
     * Given P and C, we associate with P a coordinate system, in which the viewpoint C is placed at the origin.
     * We seek a function that maps a point pi ∈ P along the ray from C to pi and is monotonically decreasing
     * in ||pi ||. (||·|| is a norm.)
     */
    for (size_t pidx = 0; pidx < cloud->points.size(); ++pidx) {

        pcl::PointXYZ currentPoint = cloud->points[pidx];
        Eigen::Vector3d currentVector(currentPoint.x, currentPoint.y, currentPoint.z);

        Eigen::Vector3d projected_point = currentVector - camera_location;

        double norm = projected_point.norm();

        if (norm == 0)
            norm = 0.0001;

        /**
         * pi = f(pi) = pi +2(R−||pi||) * ( pi / ||pi||)
         *
         * here:
         *  - ||pi|| is the norm
         *  - pi is the projected_point
         *  - we have the radius (described above)
         *
         */
        spherical_projection.push_back(
                projected_point + 2 * (radius - norm) * projected_point / norm);
    }

    size_t origin_pidx = spherical_projection.size();
    // add the last point, may be this is not necessary?
    spherical_projection.push_back(Eigen::Vector3d(0, 0, 0));


    /**
     * This is just adding the points after performing spherical inversion to the newCloud
     */
    for (std::size_t i = 0; i < spherical_projection.size(); ++i) {
        Eigen::Vector3d currentVector = spherical_projection.at(i);
        pcl::PointXYZ currentPoint(currentVector.x(), currentVector.y(), currentVector.z());
        newCloud->push_back(currentPoint);
    }

    // Compute the convex hull, all the points that are in the convex hull are visible from C
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud <pcl::PointXYZ>);
    pcl::ConvexHull <pcl::PointXYZ> chull;
    chull.setInputCloud(newCloud);
    chull.reconstruct(*cloud_hull);

    cout << "radius: " << radius << std::endl;
    cout << "Original Cloud size " << cloud->points.size() << std::endl;
    cout << "New Cloud's size " << cloud_hull->points.size() << std::endl;

    // reset the window with the new cloud
    visibilityViewer->updatePointCloud(cloud_hull, "cloud");
    visibilityViewer->resetCamera();
    ui->visibilityViewer->update();
}


void
PCLViewer::radiusChanged(double value) {
    radius = value;
}


PCLViewer::~PCLViewer() {
    delete ui;
}


