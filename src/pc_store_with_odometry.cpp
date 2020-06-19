#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>

class PCStore{
	private:
		/*node handle*/
		ros::NodeHandle _nh;
		ros::NodeHandle _nhPrivate;
		/*subscribe*/
		ros::Subscriber _sub_pc;
		ros::Subscriber _sub_odom;
		/*publish*/
		ros::Publisher _pub_pc;
		/*pc*/
		// std::vector<sensor_msg::PointCloud2> _pc_record;
		sensor_msgs::PointCloud2 _pc_submsg;
		sensor_msgs::PointCloud2 _pc_pubmsg;
		/*_viewer*/
		pcl::visualization::PCLVisualizer _viewer{"pc_store_with_odometry"};
		/*odom*/
		nav_msgs::Odometry _odom_last;
		/*list*/
		std::vector<std::string> _list_fields;
		std::vector<size_t> _list_num_scanpoints;
		/*flag*/
		bool _got_first_pc = false;
		bool _got_new_pc = false;
		bool _got_first_odom = false;
		/*parameter*/
		int _scan_limit;
		bool _downsampling;
		double _leafsize;

	public:
		PCStore();
		void listUpPointType(void);
		void callbackPC(const sensor_msgs::PointCloud2ConstPtr& msg);
		void callbackOdom(const nav_msgs::OdometryConstPtr& msg);
		void checkTypeAndStorePC(nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr> void storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now);
		template<typename CloudPtr, typename PointT> void downsampling(CloudPtr pc, PointT no_use);
		template<typename CloudPtr> void erasePoints(CloudPtr pc);
		void publication(void);
		template<typename CloudPtr, typename PointT> void visualizePC(CloudPtr pc, PointT no_use);
		template<typename CloudPtr, typename PointT> void visualizeNC(CloudPtr nc, PointT no_use);
};

PCStore::PCStore()
	: _nhPrivate("~")
{
	/*parameter*/
	_nhPrivate.param("scan_limit", _scan_limit, -1);
	std::cout << "_scan_limit = " << _scan_limit << std::endl;
	_nhPrivate.param("downsampling", _downsampling, false);
	std::cout << "_downsampling = " << _downsampling << std::endl;
	_nhPrivate.param("leafsize", _leafsize, 0.3);
	std::cout << "_leafsize = " << _leafsize << std::endl;
	/*subscriber*/
	_sub_pc = _nh.subscribe("/cloud", 1, &PCStore::callbackPC, this);
	_sub_odom = _nh.subscribe("/odom", 1, &PCStore::callbackOdom, this);
	/*publisher*/
	_pub_pc = _nh.advertise<sensor_msgs::PointCloud2>("/cloud/stored", 1);
	/*initialize*/
	listUpPointType();
	_viewer.setBackgroundColor(1, 1, 1);
	_viewer.addCoordinateSystem(0.5, "axis");
}

void PCStore::listUpPointType(void)
{
	_list_fields = {
		"x y z",
		"x y z intensity",
		"x y z intensity ring",	//velodyne
		"x y z strength",
		"x y z normal_x normal_y normal_z curvature",
		"x y z intensity normal_x normal_y normal_z curvature"
	};
}

void PCStore::callbackPC(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	if(!_got_first_pc){
		_pc_pubmsg = *msg;
		_got_first_pc = true;
		return;
	}

	_pc_submsg = *msg;
	_got_new_pc = true;
}

void PCStore::callbackOdom(const nav_msgs::OdometryConstPtr& msg)
{
	if(!_got_first_odom){
		_odom_last = *msg;
		_got_first_odom = true;
		return;
	}
	if(!_got_new_pc)	return;

	checkTypeAndStorePC(*msg);
	/*publication*/
	publication();
	/*reset*/
	_odom_last = *msg;
	_got_new_pc = false;
}

void PCStore::checkTypeAndStorePC(nav_msgs::Odometry odom_now)
{
	std::string fields = pcl::getFieldsList(_pc_submsg);

	if(fields == _list_fields[0]){
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[1] || fields == _list_fields[2]){
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::PointCloud<pcl::PointXYZI>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZI>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[3]){
		pcl::PointCloud<pcl::InterestPoint>::Ptr pc_now (new pcl::PointCloud<pcl::InterestPoint>);
		pcl::PointCloud<pcl::InterestPoint>::Ptr pc_store (new pcl::PointCloud<pcl::InterestPoint>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storePC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[4]){
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_now (new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr pc_store (new pcl::PointCloud<pcl::PointNormal>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storeNC(pc_now, pc_store, odom_now);
	}
	else if(fields == _list_fields[5]){
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_now (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::PointCloud<pcl::PointXYZINormal>::Ptr pc_store (new pcl::PointCloud<pcl::PointXYZINormal>);
		pcl::fromROSMsg(_pc_submsg, *pc_now);
		pcl::fromROSMsg(_pc_pubmsg, *pc_store);
		storeNC(pc_now, pc_store, odom_now);
	}
	else{
		std::cout << "This point-type is not supported: fields = " << fields << std::endl;
		exit(1);
	}
}

template<typename CloudPtr>
void PCStore::storePC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(_odom_last.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		_odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		_odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		_odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloud(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	_list_num_scanpoints.push_back(pc_now->points.size());
	/*erase*/
	if(_scan_limit > 0){
		_list_num_scanpoints.push_back(pc_now->points.size());
		erasePoints(pc_store);
	}
	/*downsampling*/
	if(_downsampling)	downsampling(pc_store, pc_store->points[0]);
	/*convert*/
	pcl::toROSMsg(*pc_store, _pc_pubmsg);
	/*visualize*/
	visualizePC(pc_store, pc_store->points[0]);
}

template<typename CloudPtr>
void PCStore::storeNC(CloudPtr pc_now, CloudPtr pc_store, nav_msgs::Odometry odom_now)
{
	/*prepare*/
	tf::Quaternion q_ori_now;
	tf::Quaternion q_ori_last;
	quaternionMsgToTF(odom_now.pose.pose.orientation, q_ori_now);
	quaternionMsgToTF(_odom_last.pose.pose.orientation, q_ori_last);
	tf::Quaternion q_rel_rot = q_ori_last*q_ori_now.inverse();
	q_rel_rot.normalize();	
	Eigen::Quaternionf rotation(q_rel_rot.w(), q_rel_rot.x(), q_rel_rot.y(), q_rel_rot.z());
	tf::Quaternion q_global_move(
		_odom_last.pose.pose.position.x - odom_now.pose.pose.position.x,
		_odom_last.pose.pose.position.y - odom_now.pose.pose.position.y,
		_odom_last.pose.pose.position.z - odom_now.pose.pose.position.z,
		0.0
	);
	tf::Quaternion q_local_move = q_ori_last.inverse()*q_global_move*q_ori_last;
	Eigen::Vector3f offset(q_local_move.x(), q_local_move.y(), q_local_move.z());
	/*transform*/
	pcl::transformPointCloudWithNormals(*pc_store, *pc_store, offset, rotation);
	/*store*/
	*pc_store  += *pc_now;
	pc_store->header.stamp = pc_now->header.stamp;
	/*erase*/
	if(_scan_limit > 0){
		_list_num_scanpoints.push_back(pc_now->points.size());
		erasePoints(pc_store);
	}
	/*downsampling*/
	if(_downsampling)	downsampling(pc_store, pc_store->points[0]);
	/*convert*/
	pcl::toROSMsg(*pc_store, _pc_pubmsg);
	/*visualize*/
	visualizeNC(pc_store, pc_store->points[0]);
}

template<typename CloudPtr, typename PointT>
void PCStore::downsampling(CloudPtr pc, PointT no_use)
{
	typedef pcl::PointCloud<PointT> Cloud;
	CloudPtr tmp(new Cloud);
	pcl::VoxelGrid<PointT> vg;
	vg.setInputCloud(pc);
	vg.setLeafSize(_leafsize, _leafsize, _leafsize);
	vg.filter(*tmp);
	*pc = *tmp;
}

template<typename CloudPtr>
void PCStore::erasePoints(CloudPtr pc)
{
	if(_downsampling){
		int limit_numpoints = _list_num_scanpoints[0]*_scan_limit;
		int erase_numpoints = pc->points.size() - limit_numpoints;
		if(erase_numpoints > 0)	pc->points.erase(pc->points.begin(), pc->points.begin() + erase_numpoints);
		_list_num_scanpoints.erase(_list_num_scanpoints.begin());
		std::cout << "number of stored points: " << pc->points.size() << " / " << limit_numpoints << std::endl;
	}
	else{
		if(_list_num_scanpoints.size() > (size_t)_scan_limit){
			pc->points.erase(pc->points.begin(), pc->points.begin() + _list_num_scanpoints[0]);
			_list_num_scanpoints.erase(_list_num_scanpoints.begin());
		}
		std::cout << "number of stored scans: " << _list_num_scanpoints.size() << " / " << _scan_limit << std::endl;
	}
	pc->width = pc->points.size();
	pc->height = 1;
}

void PCStore::publication(void)
{
	_pub_pc.publish(_pc_pubmsg);
}

template<typename CloudPtr, typename PointT>
void PCStore::visualizePC(CloudPtr pc, PointT no_use)
{
	_viewer.removeAllPointClouds();

	_viewer.addPointCloud<PointT>(pc, "pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "pc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "pc");
	
	_viewer.spinOnce();
}

template<typename CloudPtr, typename PointT>
void PCStore::visualizeNC(CloudPtr nc, PointT no_use)
{
	_viewer.removeAllPointClouds();

	_viewer.addPointCloudNormals<PointT>(nc, 1, 0.5, "nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "nc");
	_viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, "nc");
	
	_viewer.spinOnce();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pc_store_with_odometry");
	
	PCStore pc_store_with_odometry;

	ros::spin();
}
