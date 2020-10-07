#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>






int main(int argc, char** argv){
	ros::init(argc, argv, "roboteq_motor_controller_driver");
	
	roboteq::Driver driver;	
	driver.connect();
	driver.roboteq_services();

	driver.run();

        ros::AsyncSpinner spinner(4);

	spinner.start();




	ros::waitForShutdown();
	//spinner.stop();
		
	return 0;


}
