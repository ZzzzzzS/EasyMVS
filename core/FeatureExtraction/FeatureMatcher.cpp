#include "FeatureMatcher.h"

FeatureMatcher::Ptr FeatureMatcher::Create(GlobalMapObject::Ptr GlobalMap)
{
    return FeatureMatcher::Ptr();
}

FeatureMatcher::FeatureMatcher(GlobalMapObject::Ptr GlobalMap)
{
}

FeatureMatcher::~FeatureMatcher()
{
}

std::string FeatureMatcher::type_name()
{
    return std::string("Workflow FeatureMatcher");
}

bool FeatureMatcher::clear()
{
    return false;
}

bool FeatureMatcher::Compute(FrameObject::Ptr frame, GlobalMapObject::Ptr GlobalMap)
{
    return false;
}

bool FeatureMatcher::MatchRelatedFrame(FrameObject::Ptr frame, std::list<FrameObject::Ptr>& related, GlobalMapObject::Ptr GlobalMap)
{
    return false;
}

bool FeatureMatcher::MatchKeyPoints(FrameObject::Ptr frame)
{
    return false;
}

bool FeatureMatcher::MatchKeyPoints(FrameObject::Ptr queryFrames, FrameObject::Ptr trainFrames, std::vector<cv::DMatch>& matches)
{
    return false;
}

bool FeatureMatcher::save(JsonNode& fs)
{
    return false;
}

bool FeatureMatcher::load(JsonNode& fs)
{
    return false;
}

void FeatureMatcher::Trigger()
{
    emit this->Error("Can NOT triggered without input data");
}

void FeatureMatcher::Trigger(DataQueue data)
{
    DataQueue outdata;
    while (!data.empty())
    {
        auto tmp = std::dynamic_pointer_cast<FrameObject>(data.front());
        data.pop();
        if (this->Compute(tmp))
        {
            outdata.push(tmp);
        }
    }
    if (!outdata.empty())
    {
        emit this->Finished(outdata);
    }
    else
    {
        emit this->Warning(this->type_name() + ": failed to match input data");
    }
}
