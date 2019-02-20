#include "cloudqtdata.h"

CloudQtData::CloudQtData( QString filepath_qt  ):CloudData(filepath_qt.toStdString())
{
    this->file_path_qt = filepath_qt;
    this->file_name_qt = QString::fromStdString( file_name );

}

