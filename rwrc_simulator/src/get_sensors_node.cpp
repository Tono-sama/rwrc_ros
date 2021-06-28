#include "get_sensors.hpp"

void Init(){

}

int main(int argc, char **argv){

	ros::init(argc, argv, "get_sensors_node");

	ros::NodeHandle n;
	ros::NodeHandle pn("~");

	Init();

	ros::AsyncSpinner spinner(0);  //spinを処理するスレッド数を引数に渡す(0はcpuのコア数)
	spinner.start();

	ros::waitForShutdown();

	return 0;
}