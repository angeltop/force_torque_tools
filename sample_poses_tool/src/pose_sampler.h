
#ifndef POSE_SAMPLER_H_
#define POSE_SAMPLER_H_

#include <rviz/display.h>
#include "ui_pose_sampler.h"


#include <ros/package.h>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>

#include <rviz/display_context.h>
#include <rviz/visualization_manager.h>
#include <rviz/display_factory.h>

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <QtCore/QThread>
#include <QtCore/QDebug>
#include <QtGui/QPushButton>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseArray.h>
#include <sarafun_msgs/CompleteFeedback.h>
#include <yaml-cpp/yaml.h>
#include <fstream>
namespace sample_poses_tool
{
class PoseSampler : public rviz::Display
{
Q_OBJECT
public:
PoseSampler();
	virtual ~PoseSampler();

	virtual void onInitialize();
	virtual void update(float wall_dt, float ros_dt);
private Q_SLOTS:
	void samplePose();
private:

	QWidget* w_;
	Ui::PoseSampler ui_;
	ros::NodeHandle n_;
    ros::AsyncSpinner spinner_;
	sarafun_msgs::CompleteFeedback lastMsg_;
	ros::Subscriber feedBackSub_;
	std::string arm_;
	std::string fileName_;
	geometry_msgs::PoseArray poses_;
	
	void feedbackCB(const sarafun_msgs::CompleteFeedback::ConstPtr &msg);
	bool savePoses();
	int print_;
	
};

}
#endif