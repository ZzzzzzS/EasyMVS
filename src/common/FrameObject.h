#include "MVSConfig.h"
#include "DataFlowObject.h"

class FrameObject : public DataFlowObject
{
public:
	using Ptr = std::shared_ptr<FrameObject>;
	//static Ptr Create();
public:
	FrameObject();
	~FrameObject();


private:

};

