#include "PoseReconstructor.h"

PoseReconstructor::PoseReconstructor()
{
}

PoseReconstructor::PoseReconstructor(GlobalMapObject::Ptr GlobalMap)
{
}

PoseReconstructor::~PoseReconstructor()
{
}

std::string PoseReconstructor::type_name()
{
	return std::string("Workflow-Pose-Reconstructor");
}

void PoseReconstructor::Trigger()
{
	emit this->Error("Can NOT triggered without input data");
}

void PoseReconstructor::Trigger(DataQueue data)
{
	DataQueue output;
	while (true)
	{
		auto ptr = std::dynamic_pointer_cast<FrameObject>(output.front());
		if (ptr)
		{
			if (this->Compute(ptr))
			{
				output.push(ptr);
			}
			else
			{
				emit this->Warning(this->type_name() + ": failed to compute pose of this frame");
			}
		}
		else
		{
			emit this->Error(this->type_name() + ": Can NOT cast to FrameObject");
		}
		data.pop();
		
		if (data.empty())
			break;
	}
	if (!output.empty())
		this->Finished(output);
	else
		this->Warning(this->type_name() + ": no output data");
}
