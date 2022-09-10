#include "MVSObject.h"
#include <filesystem>


MVSObject::MVSObject()
{
}

std::string MVSObject::type_name()
{
    return typeid(this).name();
}


using namespace std::filesystem;
bool MVSObject::isDirExist(std::string path)
{
    std::filesystem::path p1 = path;
    return exists(p1);
}

bool MVSObject::mkdir(std::string path)
{
    if (this->isDirExist(path))
        return true;
    else
    {
        return create_directories(path);
    }
}
