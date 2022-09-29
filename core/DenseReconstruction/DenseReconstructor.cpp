#include "DenseReconstructor.h"

DenseReconstructor::Ptr DenseReconstructor::Create(GlobalMapObject::Ptr GlobalMap)
{
    return DenseReconstructor::Ptr();
}

DenseReconstructor::DenseReconstructor(GlobalMapObject::Ptr GlobalMap)
    :GlobalMap(GlobalMap)
{
}

DenseReconstructor::~DenseReconstructor()
{
}

std::string DenseReconstructor::type_name()
{
    return std::string("Workflow DenseReconstructor");
}

bool DenseReconstructor::clear()
{
    return false;
}

bool DenseReconstructor::save(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::load(JsonNode& fs)
{
    return false;
}

bool DenseReconstructor::Compute(FrameObject::Ptr frame)
{
    return false;
}

bool DenseReconstructor::Compute(FrameObject::Ptr frame1, FrameObject Frame2)
{
    return false;
}

bool DenseReconstructor::Compute(std::vector<FrameObject::Ptr>& frames)
{
    return false;
}

void DenseReconstructor::Trigger()
{
    std::cout << this->type_name() << ": this node can NOT be triggered without input data!" << std::endl;
}

void DenseReconstructor::Trigger(DataQueue data)
{
    if (data.empty())
    {
        std::cout << this->type_name() << ": this node can NOT be triggered without input data!" << std::endl;
        return;
    }
	
    DataQueue out;
    while (true)
    {
        auto frame = std::dynamic_pointer_cast<FrameObject>(data.front());
		data.pop();
        if (frame == nullptr)
        {
			std::cout << this->type_name() << ": input data is not a frame object!" << std::endl;
			break;
        }
        if (this->Compute(frame))
        {
			out.push(frame);
		}
		else
		{
			std::cout << this->type_name() << ": compute failed!" << std::endl;
		}
		if (data.empty())
		{
			break;
        }
    }
	
    if (!out.empty())
    {
        this->Finished(out);
    }
    else
    {
        std::cout << this->type_name() << ": No output data" << std::endl;
    }
}
