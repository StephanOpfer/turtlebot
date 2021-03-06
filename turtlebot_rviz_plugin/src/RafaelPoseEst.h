#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <ros/ros.h>

#include <rviz/default_plugin/tools/pose_tool.h>
#endif

namespace rviz
{
class Arrow;
}

namespace turtlebot_rviz_plugin
{

class RafaelPoseEst: public rviz::PoseTool
{
Q_OBJECT
public:
	RafaelPoseEst();
	~RafaelPoseEst();

	virtual void onInitialize();

protected:
	virtual void onPoseSet(double x, double y, double theta);

private:
	ros::NodeHandle nh;
	ros::Publisher pub;


};

} // namespace
