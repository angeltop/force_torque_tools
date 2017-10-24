#include "pose_sampler.h"
#include <pluginlib/class_list_macros.h>


Q_DECLARE_METATYPE(QItemSelection)

namespace sample_poses_tool 
{

	PoseSampler::PoseSampler():spinner_(2),print_(0)
	{
		rviz::Display();
		n_ = ros::NodeHandle("");

		feedBackSub_ = n_.subscribe<sarafun_msgs::CompleteFeedback>("/egmri_feedback_topic", 1, &PoseSampler::feedbackCB, this);
		
		lastMsg_.left_pose.pose.orientation.w=1.0;
		lastMsg_.right_pose.pose.orientation.w=1.0;
		if(n_.hasParam("/sample_pose/arm"))
		{
			n_.getParam("/sample_pose/arm", arm_);
		}
		else
		{
			arm_ = "left_arm";
			ROS_INFO_STREAM("PoseSampler: Missing ROS PARAM /sample_pose/arm using default value: " <<arm_);
		}
		if(n_.hasParam("/sample_pose/file"))
		{
			n_.getParam("/sample_pose/file", fileName_);
		}
		else
		{
			fileName_ = "poses_test";
			ROS_INFO_STREAM("PoseSampler: Missing ROS PARAM /sample_pose/file using default value: " <<fileName_);
		}
		
	}

	PoseSampler::~PoseSampler()
	{
		spinner_.stop();

	}
	void PoseSampler::update(float wall_dt, float ros_dt)
	{
		rviz::Display::update(wall_dt, ros_dt);
	}

	void PoseSampler::onInitialize()
	{
		spinner_.start();
		w_ = new QWidget;
		ui_.setupUi(w_);
		setAssociatedWidget(w_);

		connect(ui_.samplePose, SIGNAL(clicked(bool)), SLOT(samplePose()));
		qRegisterMetaType<QItemSelection>();
	}

	void PoseSampler::samplePose()
	{
		std::stringstream ss;
		if(arm_=="left_arm")
		{
			ss<< "Position: ("<<lastMsg_.left_pose.pose.position.x<<", "<<lastMsg_.left_pose.pose.position.y<<", "<<lastMsg_.left_pose.pose.position.z<<")\n";
			ss<< "Orientation: ("<<lastMsg_.left_pose.pose.orientation.x<<", "<<lastMsg_.left_pose.pose.orientation.y<<", "<<lastMsg_.left_pose.pose.orientation.z<<", "<<lastMsg_.left_pose.pose.orientation.w<<")";
			poses_.poses.push_back(lastMsg_.left_pose.pose);
		}
		else
		{
			ss<< "Position: ("<<lastMsg_.right_pose.pose.position.x<<", "<<lastMsg_.right_pose.pose.position.y<<", "<<lastMsg_.right_pose.pose.position.z<<")\n";
			ss<< "Orientation: ("<<lastMsg_.right_pose.pose.orientation.x<<", "<<lastMsg_.right_pose.pose.orientation.y<<", "<<lastMsg_.right_pose.pose.orientation.z<<", "<<lastMsg_.right_pose.pose.orientation.w<<")";
			poses_.poses.push_back(lastMsg_.right_pose.pose);
		}
		ui_.lastPose->setText(ss.str().c_str());
		savePoses();
		print_ =0;
	}

	void PoseSampler::feedbackCB(const sarafun_msgs::CompleteFeedback::ConstPtr &msg)
	{
		lastMsg_ = *msg;
		std::stringstream ss;
		if(arm_=="left_arm")
		{
			ss<< "Position: ("<<lastMsg_.left_pose.pose.position.x<<", "<<lastMsg_.left_pose.pose.position.y<<", "<<lastMsg_.left_pose.pose.position.z<<")\n";
			ss<< "Orientation: ("<<lastMsg_.left_pose.pose.orientation.x<<", "<<lastMsg_.left_pose.pose.orientation.y<<", "<<lastMsg_.left_pose.pose.orientation.z<<", "<<lastMsg_.left_pose.pose.orientation.w<<")";
		}
		else
		{
			ss<< "Position: ("<<lastMsg_.right_pose.pose.position.x<<", "<<lastMsg_.right_pose.pose.position.y<<", "<<lastMsg_.right_pose.pose.position.z<<")\n";
			ss<< "Orientation: ("<<lastMsg_.right_pose.pose.orientation.x<<", "<<lastMsg_.right_pose.pose.orientation.y<<", "<<lastMsg_.right_pose.pose.orientation.z<<", "<<lastMsg_.right_pose.pose.orientation.w<<")";
		}
		if(print_%50==0)
			ui_.cPose->setText(ss.str().c_str());
		print_++;
	}

	bool PoseSampler::savePoses()
	{
		// discard storing empty messages
		if(poses_.poses.size() == 0)
		{
			return false;
		}
		
		std::string filepath;
		filepath = ros::package::getPath("sample_poses_tool")+"/config/"+fileName_+".yaml";
		std::ofstream file(filepath.c_str());
		if(!file.is_open())
		{
			ROS_WARN_STREAM (filepath<<" does not exist. FAILED TO SAVE POSES");
			return false;
		}
		YAML::Node mainNode;
		// get frame
		for(int i=0; i<poses_.poses.size(); i++)
		{
			std::stringstream ss;
			ss<<"pose";
			ss<<i;
			std::vector<double> p;
			p.push_back(poses_.poses[i].position.x);
			p.push_back(poses_.poses[i].position.y);
			p.push_back(poses_.poses[i].position.z);
			p.push_back(poses_.poses[i].orientation.x);
			p.push_back(poses_.poses[i].orientation.y);
			p.push_back(poses_.poses[i].orientation.z);
			p.push_back(poses_.poses[i].orientation.w);
			mainNode[ss.str().c_str()]= p;
		}
		file<<mainNode;
		file.close();
		return true;
	}

}


PLUGINLIB_EXPORT_CLASS(sample_poses_tool::PoseSampler, rviz::Display)