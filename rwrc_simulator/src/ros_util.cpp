#include "get_sensors_node.hpp"

double Modif_radian(double rad){
	while(rad>+M_PI) rad-=2.*M_PI;
	while(rad<-M_PI) rad+=2.*M_PI;
	return rad;
}

double Tf_quat_to_yaw(const tf::Quaternion _q){
  double roll=0.0, pitch=0.0, yaw=0.0;
  tf::Matrix3x3 m(_q);
  m.getRPY(roll, pitch, yaw);
  if(std::isnan(yaw))yaw = 0.0;
  return Modif_radian(yaw);
}

double Pose_quat_to_yaw(const geometry_msgs::Quaternion _q){
  tf::Quaternion q(_q.x, _q.y, _q.z, _q.w);
  return Tf_quat_to_yaw(q);
}

geometry_msgs::Transform Pose_to_tf(const geometry_msgs::Pose p){
  geometry_msgs::Transform tf;
  tf.translation.x = p.position.x;
  tf.translation.y = p.position.y;
  tf.translation.z = p.position.z;
  tf.rotation.w = p.orientation.w;
  tf.rotation.x = p.orientation.x;
  tf.rotation.y = p.orientation.y;
  tf.rotation.z = p.orientation.z;
  return tf;
}

geometry_msgs::Pose Tf_to_pose(const geometry_msgs::Transform t){
  geometry_msgs::Pose pose;
  pose.position.x = t.translation.x;
  pose.position.y = t.translation.y;
  pose.position.z = t.translation.z;
  pose.orientation.w = t.rotation.w;
  pose.orientation.x = t.rotation.x;
  pose.orientation.y = t.rotation.y;
  pose.orientation.z = t.rotation.z;
  return pose;
}

_xyo Pose_to_xyo(const geometry_msgs::Pose p){
  _xyo pose;
  pose.x = p.position.x;
  pose.y = p.position.y;
  pose.o = Pose_quat_to_yaw(p.orientation);
  return pose;
}

geometry_msgs::Pose Xyo_to_pose(const _xyo p){
  geometry_msgs::Pose pose;
  pose.position.x = p.x;
  pose.position.y = p.y;
  pose.position.z = 0.0;
  pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, Modif_radian(p.o));
  return pose;
}

// A座標系=>B座標系=>C座標系における座標変換関数(2d)
_xyo Calc_AtoC(const _xyo AtoB, const _xyo BtoC){
  _xyo AtoC;
  AtoC.x = AtoB.x + BtoC.x*cos(AtoB.o) - BtoC.y*sin(AtoB.o);
  AtoC.y = AtoB.y + BtoC.x*sin(AtoB.o) + BtoC.y*cos(AtoB.o);
  AtoC.o = AtoB.o + BtoC.o;
  // AtoC.o = Modify_Radian(AtoC.o);
  return AtoC;
}

_xyo Calc_AtoB(const _xyo AtoC, const _xyo BtoC){
  _xyo AtoB;
  AtoB.o = AtoC.o - BtoC.o;
  AtoB.x = AtoC.x - BtoC.x*cos(AtoB.o) + BtoC.y*sin(AtoB.o);
  AtoB.y = AtoC.y - BtoC.x*sin(AtoB.o) - BtoC.y*cos(AtoB.o);
  // AtoB.o = Modify_Radian(AtoB.o);
  return AtoB;
}

_xyo Calc_BtoC(const _xyo AtoB, const _xyo AtoC){
  _xyo BtoC;
  const double dx = AtoC.x - AtoB.x;
  const double dy = AtoC.y - AtoB.y;
  BtoC.x = +dx*cos(AtoB.o) + dy*sin(AtoB.o);
  BtoC.y = -dx*sin(AtoB.o) + dy*cos(AtoB.o);
  BtoC.o = AtoC.o - AtoB.o;
  // BtoC.o = Modify_Radian(BtoC.o);
  return BtoC;
}