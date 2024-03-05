#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <find_object_2d/ObjectsStamped.h>
#include <find_object_2d/DetectionInfo.h>
#include <QtCore/QString>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <string>
#include <algorithm>
#include <Eigen/Eigen>
#include <cstdlib>

class ObjectVisualizer
{
private:
	ros::NodeHandle nh_;
	ros::Timer loop_publisher_timer_;

	std::string targetFrameId_;
	std::string objNamePath_;
	std::string objNamePath_previous_;
	std::string objectFrameId_;
	int objID_;
	double objColorRed_;
	double objColorBlue_;
	double objColorGreen_;
	// find_object_2d::ObjectsStamped objStampmsg;
	// find_object_2d::DetectionInfo objInfomsg;

	// class
	std::string objFrame_Prefix_;
	std::string target_FrameId_;

	std::string param_target_FrameId_;
	std::string param_objFrame_Prefix_;

	geometry_msgs::TransformStamped object_tf_;
	tf2_ros::TransformBroadcaster tf_broadcaster_;
	tf2_ros::Buffer tf_buffer_;
	tf2_ros::TransformListener tf_listener_;

	ros::Publisher marker_pub_;
	visualization_msgs::MarkerArray marker_object_array_msg_;
	visualization_msgs::Marker marker_object_msg_;
	visualization_msgs::MarkerArray previous_marker_object_array_msg_;

	message_filters::Subscriber<find_object_2d::ObjectsStamped> obj_stamp_sub;
    message_filters::Subscriber<find_object_2d::DetectionInfo> obj_info_sub;
	typedef message_filters::sync_policies::ApproximateTime<find_object_2d::ObjectsStamped, find_object_2d::DetectionInfo> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync;

	size_t lastSlashPos_;
	std::string fileObjectname_;
	size_t underscorePos_;
	std::string objectName_;

public:
	ObjectVisualizer() : target_FrameId_("base_link"), objFrame_Prefix_("object"), nh_(), obj_stamp_sub(nh_, "/objectsStamped", 10), obj_info_sub(nh_, "/info", 10),
					sync(MySyncPolicy(10), obj_stamp_sub, obj_info_sub), tf_listener_(tf_buffer_)

    {
		marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("object_markers", 10);
		loop_publisher_timer_ = nh_.createTimer(ros::Duration(0.05), &ObjectVisualizer::ObjectPublisherCallback, this);

		ros::NodeHandle pnh_("~");
        pnh_.param("target_frame_id", param_target_FrameId_, target_FrameId_);
        pnh_.param("object_prefix", param_objFrame_Prefix_, objFrame_Prefix_);
		// Sync messages 
		sync.registerCallback(boost::bind(&ObjectVisualizer::ObjectCallback, this, _1, _2));
		objID_ = 1;
		objColorRed_ = 0.0;
		objColorBlue_ = 0.9;
		objColorGreen_ = 0.0;
		objNamePath_previous_ = "none object";
    }

	void ObjectCallback(const find_object_2d::ObjectsStampedConstPtr &_obj_stamp_msg,
                        const find_object_2d::DetectionInfoConstPtr &_obj_info_msg)
	{
		//ROS_INFO("Synchronization successful");
		if (_obj_stamp_msg->objects.data.size())
		{
			objNamePath_ = "none";
			objNamePath_ = FindNameObjectPath(_obj_info_msg->file_paths[0].data);
			if (objNamePath_ == "none")
			{
				objNamePath_ = param_objFrame_Prefix_;
			}
			targetFrameId_ = param_target_FrameId_;
			if (targetFrameId_.empty())
			{
				targetFrameId_ = _obj_stamp_msg->header.frame_id;
			}

			char multiSubId = 'b';
			int previousId = -1;

			for (unsigned int i = 0; i < _obj_stamp_msg->objects.data.size(); i += 12)
			{
				// get data
				int id = (int)_obj_stamp_msg->objects.data[i];

				QString multiSuffix;
				if (id == previousId)
				{
					multiSuffix = QString("_") + multiSubId++;
				}
				else
				{
					multiSubId = 'b';
				}
				previousId = id;

				// "object_1", "object_1_b", "object_1_c", "object_2"
				objectFrameId_ = QString("%1_%2%3").arg(param_objFrame_Prefix_.c_str()).arg(id).arg(multiSuffix).toStdString();

				try
				{
					// Get transformation from "object_#" frame to target frame
					// The timestamp matches the one sent over TF
					//ROS_INFO("targetFrameId_: %s",targetFrameId_.c_str());
					//ROS_INFO("objectFrameId_: %s",objectFrameId_.c_str());
					object_tf_ = tf_buffer_.lookupTransform(targetFrameId_, objectFrameId_, ros::Time(0.0), ros::Duration(1));
				}
				catch (tf2::TransformException &ex)
				{
					ROS_WARN("%s", ex.what());
				}

				// Here "object_tf_" is the position of the object's name from object_info_file_path "id" in target frame.
				// ROS_INFO("%s [x,y,z] [qx,qy,qz,qw] in %s frame: [%f, %f, %f] [%f, %f, %f, %f]",
                //         objNamePath_.c_str(), targetFrameId_.c_str(),
                //         object_tf_.transform.translation.x, object_tf_.transform.translation.y, object_tf_.transform.translation.z,
          		// 		object_tf_.transform.rotation.x, object_tf_.transform.rotation.y, object_tf_.transform.rotation.z, object_tf_.transform.rotation.w);

				if (objNamePath_previous_ == objNamePath_)
				{
					objID_++;
				}
				else
				{
					objID_ = 1;
					objColorRed_ = static_cast<double>(rand()) / RAND_MAX; // Generate a random value between 0.0 and 1.0
					objColorGreen_ = static_cast<double>(rand()) / RAND_MAX;
					objColorBlue_ = static_cast<double>(rand()) / RAND_MAX;
				}

				marker_object_msg_.header = object_tf_.header;
				marker_object_msg_.ns = objNamePath_;
				marker_object_msg_.id = objID_;
				marker_object_msg_.type = marker_object_msg_.SPHERE;
				marker_object_msg_.action = marker_object_msg_.ADD;
				marker_object_msg_.pose.position.x = object_tf_.transform.translation.x;
				marker_object_msg_.pose.position.y = object_tf_.transform.translation.y;
				marker_object_msg_.pose.position.z = object_tf_.transform.translation.z;
				marker_object_msg_.pose.orientation = object_tf_.transform.rotation;
				marker_object_msg_.scale.x = 0.3;
				marker_object_msg_.scale.y = 0.3;
				marker_object_msg_.scale.z = 0.3;
				marker_object_msg_.color.r = objColorRed_; // Red component
				marker_object_msg_.color.g = objColorGreen_; // Green component
				marker_object_msg_.color.b = objColorBlue_; // Blue component
				marker_object_msg_.color.a = 1.0; // Alpha component
				marker_object_msg_.lifetime = ros::Duration();
				marker_object_msg_.frame_locked  = false;

				for (unsigned int i = 0; i < marker_object_array_msg_.markers.size(); ++i)
				{
					if (marker_object_array_msg_.markers[i].ns == objNamePath_)
					{
						marker_object_array_msg_.markers.erase(marker_object_array_msg_.markers.begin() + i); // delete same old object name before add new object
					}
				}
				marker_object_array_msg_.markers.push_back(marker_object_msg_); // add new object
				
				objNamePath_previous_ = objNamePath_;
			}
		}
		objID_ = 1; // reset for next callback image
	}

	void ObjectPublisherCallback(const ros::TimerEvent &event)
    {
        // Publish the marker array
        if (!marker_object_array_msg_.markers.empty())
        {
            // delete old same marker
            for (auto &previous_marker : previous_marker_object_array_msg_.markers)
            {
                for (auto &marker : marker_object_array_msg_.markers)
                {
                    if (previous_marker.ns == marker.ns)
                    {
                        previous_marker.action = previous_marker.DELETE;
                        previous_marker.header.stamp = ros::Time::now();
                        marker_pub_.publish(previous_marker_object_array_msg_);
                    }
                }
            }

			// Set header stamp for each new marker
			for (auto &marker : marker_object_array_msg_.markers)
            {
				marker.header.stamp = ros::Time::now();
            }

            // Publish new marker array
            marker_pub_.publish(marker_object_array_msg_);
        }
        else
        {
            //ROS_WARN("No markers to publish.");
        }
		previous_marker_object_array_msg_ = marker_object_array_msg_;
    }

	std::string FindNameObjectPath(const std::string filePath)
	{
		std::string result = "none";
			// std::string filePath = pathObject;

			// Find the last '/' character in the path
			lastSlashPos_ = filePath.find_last_of('/');

		if (lastSlashPos_ != std::string::npos)
		{
			// Extract the substring after the last '/'
			fileObjectname_ = filePath.substr(lastSlashPos_ + 1);

			// Find the position of the underscore '_' in the filename
			underscorePos_ = fileObjectname_.find('_');

			if (underscorePos_ != std::string::npos)
			{
				// Extract the substring before the underscore '_'
				objectName_ = fileObjectname_.substr(0, underscorePos_);

				// Convert the substring to lowercase (optional)
				std::transform(objectName_.begin(), objectName_.end(), objectName_.begin(), ::tolower);
				//std::cout << "Object name from path: " << objectName_ << std::endl;
				result = objectName_;

			}
			else
			{
				ROS_WARN("Underscore not found in the filename.");
			}
		}
		else
		{
			ROS_ERROR("Path does not contain a '/' character.");
		}
		return result;
	}
};

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "object_markers_detection");
	ObjectVisualizer objectVisualizer;
    ros::spin();
    return 0;
}