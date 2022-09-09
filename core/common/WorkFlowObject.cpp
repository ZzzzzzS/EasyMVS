#include "WorkFlowObject.h"

#include <QMetaType>

WorkFlowObject::WorkFlowObject()
{
    //TODO:  在这里添加注册信号槽类型
    qRegisterMetaType<DataQueue>("DataQueue");
    qRegisterMetaType<std::string>("stdstring");

}

bool WorkFlowObject::isInit()
{
    return this->m_isInit;
}
