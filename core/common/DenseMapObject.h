#pragma once

#include "MVSConfig.h"
#include "DataFlowObject.h"

class DenseMapObject : public DataFlowObject
{
public:
    using Ptr = std::shared_ptr<DenseMapObject>;
public:
    DenseMapObject(/* args */);
    virtual ~DenseMapObject();

    
};


