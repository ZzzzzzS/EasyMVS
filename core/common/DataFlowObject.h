#pragma once
#include "MVSObject.h"

/**
 * @brief this is the base class for all data storage types.
 * 
 */
class DataFlowObject : public MVSObject
{
public:
/**
 * @brief Construct a new Data Flow Object object
 * 
 */
    DataFlowObject();

    /**
     * @brief Destroy the Data Flow Object object
     * 
     */
    virtual ~DataFlowObject() {}

public:
/**
 * @brief the shared pointer type of DataFlowObject
 * 
 */
    using Ptr=std::shared_ptr<DataFlowObject>;

    /**
     * @brief create shared pointer of DataFlowObject,
     * do NOT use this method directly, use the Create() method in derived class.
     * 
     * @return DataFlowObject::Ptr 
     */
    //static DataFlowObject::Ptr Create();

private:
    /* data */
};