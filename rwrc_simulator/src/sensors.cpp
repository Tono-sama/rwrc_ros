#include "get_sensors_node.hpp"

// センサ生データからオドメトリ計算用の値を取得
void Get_sensors(){
	// imu
	double n_o_tmp = Pose_quat_to_yaw(raw_topic.imu.orientation);
	double o_o_tmp = Modif_radian(sensor_data.o_o);
	sensor_data.d_o = Modif_radian(n_o_tmp - o_o_tmp);
	sensor_data.o_o = sensor_data.n_o;
	sensor_data.n_o = sensor_data.o_o + sensor_data.d_o;

	// joint_state
	// name: [left_wheel_joint, right_wheel_joint]
	sensor_data.o_l = sensor_data.n_l;
	sensor_data.o_r = sensor_data.n_r;
	// TODO: joint_stateのnameを確認する？
	if(raw_topic.joint_state.position.size()>=2){
		sensor_data.n_l = raw_topic.joint_state.position[0] * wheel_rad_to_m_L;
		sensor_data.n_r = raw_topic.joint_state.position[1] * wheel_rad_to_m_R;
	}
	sensor_data.d_l = Modif_radian(sensor_data.n_l-sensor_data.o_l);
	sensor_data.d_r = Modif_radian(sensor_data.n_r-sensor_data.o_r);
}

// odom計算
void Calc_odom(){
	sensor_data.d_dis = (sensor_data.d_l + sensor_data.d_r)/2.0;
	sensor_data.dis += sensor_data.d_dis;
	sensor_data.o_odom = sensor_data.n_odom;
	sensor_data.n_odom.x = sensor_data.o_odom.x + sensor_data.d_dis*cos(sensor_data.o_odom.o);
	sensor_data.n_odom.y = sensor_data.o_odom.y + sensor_data.d_dis*sin(sensor_data.o_odom.o);
	sensor_data.n_odom.o = sensor_data.o_odom.o + sensor_data.d_o;
	// TODO: odom topic出力
	// odom_msg.header.stamp = ros::WallTime::now();
	odom_msg.header.stamp = ros::Time::now();
	odom_msg.pose.pose = Xyo_to_pose(sensor_data.n_odom);
	odom_pub.publish(odom_msg);
	// TODO: tf出力(点群座標変換で必要なので先に計算)
}

void Trans_pointcloud(){
// TODO: 点群座標変換
}

// callback関数
void LaserScan_callback(const sensor_msgs::LaserScan::ConstPtr &laserscan_msg){
	raw_topic.laser_scan = *laserscan_msg;
}

void Imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg){
	raw_topic.imu = *imu_msg;
}

void JointState_callback(const sensor_msgs::JointState::ConstPtr &jointstate_msg){
	raw_topic.joint_state = *jointstate_msg;
}

void Loop(const ros::TimerEvent&){
	Get_sensors();
	Calc_odom();
	Trans_pointcloud();
}

