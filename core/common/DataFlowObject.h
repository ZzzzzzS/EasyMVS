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
private:
    /* data */
};