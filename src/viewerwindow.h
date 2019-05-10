#ifndef VIEWERWINDOW_H
#define VIEWERWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QMovie>


#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <vector>
#include <string>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

#include "utilcloud.h"
#include "cloudqtdata.h"

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace std;
namespace Ui {
class viewerWindow;
}



class WindowViewer : public QMainWindow
{
    Q_OBJECT

public:
    explicit WindowViewer(QWidget *parent = 0);
    ~WindowViewer();

public Q_SLOTS:

    void clickLoadPCLFile();

    void clickGetCamera();

    void clickResetCamera();

    void clickColorCloudTable(int row, int col);

    void clickCloudTable(int row, int col);

    void rightClickCloudTable(const QPoint& pos);

    void clickSubmenuDensify();

    void clickSubmenuRemove();

    void changeSpinPointSize(int value);


protected:

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;


    vector <CloudQtData> clouds_qt;

    int cloud_selected;


private:
    Ui::viewerWindow *ui;

    void colorCloud( PointCloudT::Ptr cloud, Qt::GlobalColor q_color);
};

#endif // VIEWERWINDOW_H
