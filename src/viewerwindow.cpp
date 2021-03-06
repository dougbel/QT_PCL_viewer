#include "viewerwindow.h"
#include "ui_viewerwindow.h"

WindowViewer::WindowViewer(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::viewerWindow) {

    ui->setupUi(this);


    // Set up the QVTK window
    viewer.reset(new pcl::visualization::PCLVisualizer("viewer", false));
    ui->qvtkWidget->SetRenderWindow(viewer->getRenderWindow());
    viewer->setupInteractor(ui->qvtkWidget->GetInteractor(), ui->qvtkWidget->GetRenderWindow());
    viewer->resetCamera();
    viewer->initCameraParameters ();
    viewer->setBackgroundColor(.7,.7,.7);
    clickResetCamera();
    
    //viewer->addSphere(pcl::PointXYZ(0.951572, 0.556679, 1.20407), 0.2, 0.5, 0.5, 0.0,"circle");
    //viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "circle");
    
    //viewer->addSphere(pcl::PointXYZ(0.951572, 0.556679, 1.20407), 0.01, 0.5, 0.0, 0.0,"center");
    //viewer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME, "center");
    
    ui->qvtkWidget->update();

    ui->table_cloud_color->insertRow( 0 );
    ui->table_cloud_color->setItem(0, 0, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 1, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 2, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 3, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 4, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 5, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 6, new QTableWidgetItem("") );
    ui->table_cloud_color->setItem(0, 7, new QTableWidgetItem("") );

    ui->table_cloud_color->item(0, 0)->setBackgroundColor(Qt::white);
    ui->table_cloud_color->item(0, 1)->setBackgroundColor(Qt::gray);
    ui->table_cloud_color->item(0, 2)->setBackgroundColor(Qt::black);
    ui->table_cloud_color->item(0, 3)->setBackgroundColor(Qt::red);
    ui->table_cloud_color->item(0, 4)->setBackgroundColor(Qt::green);
    ui->table_cloud_color->item(0, 5)->setBackgroundColor(Qt::yellow);
    ui->table_cloud_color->item(0, 6)->setBackgroundColor(Qt::cyan);
    ui->table_cloud_color->item(0, 7)->setBackgroundColor(Qt::blue);
    //ui->table_cloud_color->setEnabled(false);
    ui->table_cloud_color->hide();


    connect(ui->chkSeeCoordenate, SIGNAL(clicked(bool)), this, SLOT(clickChkSeeCoordenate(bool)));

    connect(ui->table_cloud, SIGNAL(cellClicked(int,int)), this, SLOT(clickCloudTable(int,int)));

    ui->table_cloud->setContextMenuPolicy(Qt::CustomContextMenu);

    connect(ui->table_cloud, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(rightClickCloudTable(const QPoint&)));

    setAcceptDrops(true);

    connect(ui->table_cloud_color, SIGNAL(cellClicked(int,int)), this, SLOT(clickColorCloudTable(int,int)));

    connect(ui->spinPointSize, SIGNAL(valueChanged (int)), this, SLOT(changeSpinPointSize(int)));

    //Button
    // load a pc file
    connect(ui->btn_loadFile, SIGNAL(clicked()), this, SLOT(clickLoadPCLFile()));
    //get camera position
    connect(ui->btn_getCamera, SIGNAL(clicked()), this, SLOT(clickGetCamera()));
    //get camera position
    connect(ui->btn_resetCamera, SIGNAL(clicked()), this, SLOT(clickResetCamera()));

}


void WindowViewer::clickChkSeeCoordenate(bool checked){

    if(checked)
        viewer->addCoordinateSystem (.5);

    else
        viewer->removeCoordinateSystem();

    ui->qvtkWidget->update();
}

void WindowViewer::dropEvent(QDropEvent *e)
{
    foreach (const QUrl &url, e->mimeData()->urls()) {
        QString filepath = url.toLocalFile();        

        boost::filesystem::path p(filepath.toStdString());

        QString filename = QString::fromStdString( p.filename().string() );

        if( isValidFile(filepath) ){
            QMessageBox::information(this, "Loading", "Loading dropped file:\n"+filename,QMessageBox::Ok);
            ctrlLoadFile( filepath );
        }
        else{
            QMessageBox::critical(this, "Loading", "No valid file type:\n"+filename,QMessageBox::Ok);
        }
    }

}

bool WindowViewer::isValidFile(QString filepath){
    string file_extension  = boost::filesystem::extension(filepath.toStdString());

    if(file_extension == ".ply" || file_extension == ".pcd" )
        return true;

    return false;
}

void WindowViewer::dragEnterEvent(QDragEnterEvent *e)
{
    if (e->mimeData()->hasUrls()) {
        e->acceptProposedAction();
    }
}

void WindowViewer::colorCloud( PointCloudT::Ptr cloud, Qt::GlobalColor q_color){

    if(q_color == Qt::black){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 0;
            cloud->points[i].g = 0;
            cloud->points[i].b = 0;
       }
    }
    else if(q_color == Qt::gray){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 128;
            cloud->points[i].g = 128;
            cloud->points[i].b = 128;
       }
    }
    else if(q_color == Qt::white){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 255;
            cloud->points[i].g = 255;
            cloud->points[i].b = 255;
       }
    }
    else if(q_color == Qt::red){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 255;
            cloud->points[i].g = 0;
            cloud->points[i].b = 0;
       }
    }
    else if(q_color == Qt::green){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 0;
            cloud->points[i].g = 255;
            cloud->points[i].b = 0;
       }
    }
    else if(q_color == Qt::yellow){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 255;
            cloud->points[i].g = 255;
            cloud->points[i].b = 0;
       }
    }
    else if(q_color == Qt::cyan){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 0;
            cloud->points[i].g = 255;
            cloud->points[i].b = 255;
       }
    }
    else if(q_color == Qt::blue){
        for (size_t i = 0; i < cloud->size(); i++) {
            cloud->points[i].r = 0;
            cloud->points[i].g = 0;
            cloud->points[i].b = 255;
       }
    }
}

void WindowViewer::clickLoadPCLFile() {

    QString filepath = QFileDialog::getOpenFileName(this,
                                                tr("Open Point Cloud File"), "",
                                                tr("PCL file (*.pcd *.ply);;All Files (*)"));

    if (filepath.isEmpty ())
        return;
    
    ctrlLoadFile( filepath );

}

void WindowViewer::ctrlLoadFile(QString filepath){
    boost::filesystem::path pathObj(filepath.toStdString());
    string cloud_name = pathObj.filename().string();

    for(int i =0; i<clouds_qt.size(); i++){
      if( clouds_qt[i].file_name.compare(cloud_name) == 0 ) {
        QMessageBox::critical(this, "Failed", "Point cloud already in list",QMessageBox::Ok);
        return;
      }
    }


    clouds_qt.push_back(CloudQtData(filepath));    

    //table update
    int n_column =  ui->table_cloud->rowCount();
    ui->table_cloud->insertRow( n_column );
    ui->table_cloud->setItem(n_column, 0, new QTableWidgetItem("") );
    ui->table_cloud->setItem(n_column, 1, new QTableWidgetItem( clouds_qt.back().file_name_qt) );
    ui->table_cloud->item(n_column,0)->setBackgroundColor(Qt::black);
    ui->table_cloud->item(n_column,0)->setFlags( ui->table_cloud->item(n_column,0)->flags() & ~Qt::ItemIsSelectable );

    viewer->addPointCloud(clouds_qt.back().cloud, clouds_qt.back().file_name);

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, ui->spinPointSize->value(), clouds_qt.back().file_name);

    if (clouds_qt.back().type_cloud == "ply"){
        viewer->addPolygonMesh(clouds_qt.back().cloud_mesh,clouds_qt.back().file_name+clouds_qt.back().type_cloud,0);
    }

    ui->qvtkWidget->update();
}




void WindowViewer::clickGetCamera() {

    std::vector<pcl::visualization::Camera> cam;

    //Save the position of the camera
    viewer->getCameras(cam);

    std::stringstream message;
    //Print recorded points on the screen:
    message << " - pos: (" << cam[0].pos[0] << ", "    << cam[0].pos[1] << ", "    << cam[0].pos[2] << ")" << endl
	    //  Focal point or lookAt.
            << " - viewing at: ("   << cam[0].focal[0] << ", "  << cam[0].focal[1] << ", "  << cam[0].focal[2] << ")"   << endl
            // Up vector of the camera. (it is a bad naming parameter in pcl)
            << " - up: ("    << cam[0].view[0] << ", "   << cam[0].view[1] << ", "   << cam[0].view[2] << ")"    << endl;


     QMessageBox::information(this, "Camera  pose", QString::fromStdString(message.str()), QMessageBox::Ok);
}

void WindowViewer::clickResetCamera() {

    //position,foward and up directions of the camera
    
    viewer->setCameraPosition(-2.0331, 0.350903, 1.28742, 0.646312, 0.253512, 0.495582, 0.284091, 0.022551, 0.958532);
}

void WindowViewer::changeSpinPointSize (int value)
{

  for(int i =0; i<clouds_qt.size(); i++){
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, clouds_qt[i].file_name);
  }

  ui->qvtkWidget->update ();

}

void WindowViewer::clickColorCloudTable(int row, int col){

    Qt::GlobalColor q_color;

    switch (col) {
        case 0: q_color = Qt::white; break;
        case 1: q_color = Qt::gray; break;
        case 2: q_color = Qt::black; break;
        case 3: q_color = Qt::red; break;
        case 4: q_color = Qt::green; break;
        case 5: q_color = Qt::yellow; break;
        case 6: q_color =Qt::cyan; break;
        case 7: q_color = Qt::blue; break;
        default: q_color = Qt::black; break;
    }

    clouds_qt[cloud_selected].cloud_color_qt = q_color ;

    this->colorCloud( clouds_qt[cloud_selected].cloud, q_color );

    viewer->updatePointCloud (  clouds_qt[cloud_selected].cloud, clouds_qt[cloud_selected].file_name );

    ui->table_cloud->item( cloud_selected, 0 )->setBackgroundColor( q_color );
    ui->qvtkWidget->update();


}

void WindowViewer::clickCloudTable(int row, int col){

    cloud_selected = row;
    ui->label_cloud->setText( clouds_qt[cloud_selected].file_name_qt );
    ui->table_cloud_color->show();

}

void WindowViewer::rightClickCloudTable(const QPoint& pos) // this is a slot
{

    QMenu myMenu;
    int row;

    row = ui->table_cloud->currentRow();


    QAction* act_hide_mesh = new QAction("Hide", this);
    QAction* act_show_mesh = new QAction("Show", this);
    QAction* act_densify_pc = new QAction("Densify", this);
    QAction* act_hide_pc = new QAction("Hide", this);
    QAction* act_show_pc = new QAction("Show", this);
    QAction* act_delete = new QAction("Delete", this);

    if (clouds_qt[ row ].type_cloud == "ply")
    {
        QMenu* menu_mesh = myMenu.addMenu(tr("&Mesh"));

        menu_mesh->addAction(act_hide_mesh);
        menu_mesh->addAction(act_show_mesh);
    }

    QMenu* menu_pc = myMenu.addMenu(tr("&PC"));

    if (clouds_qt[ row ].type_cloud == "ply")
        menu_pc->addAction(act_densify_pc);

    menu_pc->addAction(act_hide_pc);
    menu_pc->addAction(act_show_pc);


    myMenu.addAction(act_delete);


    QPoint globalPos = ui->table_cloud->mapToGlobal(pos);
    QAction* selectedItem = myMenu.exec(globalPos);



    if(selectedItem == act_densify_pc)
    {
        clickSubmenuDensify( row );
    }
    else if(selectedItem == act_hide_pc)
    {
        clickHidePC( row );
    }
    else if(selectedItem == act_show_pc)
    {
        clickShowPC( row );
    }
    else if(selectedItem == act_hide_mesh)
    {
        clickHideMesh( row );
    }
    else if(selectedItem == act_show_mesh)
    {
        clickShowMesh( row );
    }
    else if(selectedItem == act_delete)
    {
        clickSubmenuRemove( row );
    }

}

void WindowViewer::clickShowMesh(int num_cloud){

    viewer->addPolygonMesh(clouds_qt[ num_cloud ].cloud_mesh,clouds_qt[ num_cloud ].file_name+clouds_qt[ num_cloud ].type_cloud,0);

    ui->qvtkWidget->update();
}


void WindowViewer::clickHideMesh(int num_cloud){

    viewer->removePolygonMesh(clouds_qt[ num_cloud ].file_name+clouds_qt[ num_cloud ].type_cloud);

    ui->qvtkWidget->update();
}


void WindowViewer::clickShowPC(int num_cloud){


    viewer->addPointCloud(clouds_qt[ num_cloud ].cloud, clouds_qt[ num_cloud ].file_name);

    ui->qvtkWidget->update();
}


void WindowViewer::clickHidePC(int num_cloud){

    viewer->removePointCloud(clouds_qt[ num_cloud ].file_name);

    ui->qvtkWidget->update();
}

void WindowViewer::clickSubmenuDensify(int num_cloud){
    
    Densifier sampler;
    
    clouds_qt[ num_cloud ].cloud = sampler.densecloud(clouds_qt[ num_cloud ].file_path, 100000);


    this->colorCloud( clouds_qt[num_cloud].cloud, clouds_qt[num_cloud].cloud_color_qt );
    
    viewer->updatePointCloud(clouds_qt[ num_cloud ].cloud, clouds_qt[ num_cloud ].file_name);

    ui->qvtkWidget->update();
}


void WindowViewer::clickSubmenuRemove(int num_cloud){

    QMessageBox::StandardButton reply;

    std::stringstream message;

    message << " Delete \"" << clouds_qt[ num_cloud ].file_name <<"\" ?";

    reply = QMessageBox::question(this, "Confirm delete action", QString::fromStdString(message.str()), QMessageBox::No|QMessageBox::Yes, QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
        //Erase point cloud
        viewer->removePointCloud(clouds_qt[ num_cloud ].file_name);

        if (clouds_qt.back().type_cloud == "ply"){
            viewer->removePolygonMesh(clouds_qt[ num_cloud ].file_name+clouds_qt.back().type_cloud);
        }

        ui->qvtkWidget->update();

        clouds_qt.erase( clouds_qt.begin() + num_cloud );

        ui->table_cloud->removeRow( num_cloud );
        ui->table_cloud_color->hide();
        ui->label_cloud->setText( "" );
    }

}



WindowViewer::~WindowViewer() { delete ui; }
