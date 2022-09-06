#include "MVSObject.h"
#include <QDir>
#include <QString>

MVSObject::MVSObject()
{
}

std::string MVSObject::type_name()
{
    return typeid(this).name();
}

bool MVSObject::isDirExist(std::string path)
{
    QDir dir;
    return dir.exists(QString::fromStdString(path));
}

bool MVSObject::mkdir(std::string path)
{
    QDir dir;
    if (dir.exists(QString::fromStdString(path)))
    {
        return true;
    }
    else
    {
        return dir.mkdir(QString::fromStdString(path));
    }
}
