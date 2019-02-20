#ifndef CLOUDQTDATA_H
#define CLOUDQTDATA_H


#include <QString>

#include "clouddata.h"

class CloudQtData : public CloudData
{
public:


    QString file_name_qt;
    QString file_path_qt;
    Qt::GlobalColor cloud_color_qt;

    CloudQtData( QString filepath );
};

#endif // CLOUDQTDATA_H
