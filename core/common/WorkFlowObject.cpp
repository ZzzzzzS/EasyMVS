#include "WorkFlowObject.h"

WorkFlowObject::WorkFlowObject()
{
    //TODO:  在这里添加注册信号槽类型
}

std::string WorkFlowObject::getFlowName()
{
    std::string name = std::string("Workflow ");
    name += std::string(typeid(this).name());

    return name;
}

bool WorkFlowObject::isInit()
{
    return this->m_isInit;
}
