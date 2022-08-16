#include <MVSObject.h>
#include <MVSConfig.h>
#include <QObject>
#include <opencv2/opencv.hpp>



class WorkFlowObject : public QObject, public MVSObject
{
	Q_OBJECT
public:
	WorkFlowObject();
	virtual ~WorkFlowObject() {}
	
public:
	virtual std::string getFlowName() = 0;
	virtual bool clear() = 0;
	virtual bool init(Json& fs) = 0;
	virtual bool isInit() = 0;
	virtual bool saveParameter(Json& fs) = 0;
	
signals:
	void Finished();
	void Error(std::string error);
	void Progress(int progress);
	void Message(std::string message);
	void Warning(std::string warning);
	void Info(std::string info);
	void Debug(std::string debug);
	
public slots:
	virtual void Trigger() = 0;
	

public:
	using Ptr = std::shared_ptr<WorkFlowObject>;
	
protected:
	std::string m_flowName;
	bool m_isInit = false;
};

