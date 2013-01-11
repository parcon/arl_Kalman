/*
Parker Conroy
ARLab @ University of Utah


This is sample Kalman filter code.

x(t)= A*x(t-1) +B*u(t-1) + w(t-1)
z(t)=Hx(t)+v(t)

x vector is n x 1
u vector is l x 1
z vector is m x 1

A matrix is n x n
B matrix is n x l
H matrix is m x n
Kalman Gain K matrix is m x n  
w is process white noise ~ N(0,Q)
v is measurement white noise ~ N(0,R)
*/

#include <ros/ros.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <ardrone_autonomy/Navdata.h>
#include "kalman.h"

//Matrix part one taken from here PARCON

std_msgs::Float32MultiArray x_msg;
float deg2rad= 0.0174532925;
float rotation_roll =0.0;
float rotation_pitch =0.0;
float rotation_yaw =0.0;
float vel_x=0.0;
float vel_y=0.0;
float vel_z=0.0;
float w_roll=0.0;
float w_pitch=0.0;
float w_yaw=0.0;

float x12_hat;
float y12_hat;
float z12_hat;
float vx1_hat;
float vy1_hat;
float vz1_hat;
float rp1_hat;
float rr1_hat;
float wp1_hat;
float wr1_hat;

int tag_id =0;//CHANGES WHICH TAG TO DISPLAY
float vision_angle[2];
float tag_position[3];
int cam_height=360;
int cam_width=640;
float cam_width_degree=92*deg2rad;
float cam_height_degree=51*deg2rad; //cam_width_degree*(cam_height/cam_width) //NOT MEASURED

uint32_t tags_count;
uint32_t tags_type[10];
uint32_t tags_xc[10];
uint32_t tags_yc[10];
uint32_t tags_width[10];
uint32_t tags_height[10];
float tags_orientation[10];
float tags_distance[10];
double time_stamp;

void state_callback(const ardrone_autonomy::Navdata& msg_in)
{
	//Take in states of robot that we are interested in from the navdata topic
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
//	rotation_yaw=msg_in.rotZ*deg2rad;
	
	vel_x=msg_in.vx*0.001; //  mm/s to m/s
	vel_y=msg_in.vy*0.001; //  mm/s to m/s
	vel_z=msg_in.vz*0.001; //  mm/s to m/s
	
	//tags
	tags_count=msg_in.tags_count;
	time_stamp=msg_in.tm;	

	for (uint32_t i=0; i <tags_count; i++){
		tags_distance[i]=msg_in.tags_distance[i];
		tags_xc[i]=msg_in.tags_xc[i];
		tags_yc[i]=msg_in.tags_yc[i];
		tags_width[i]=msg_in.tags_width[i];
		tags_height[i]=msg_in.tags_height[i];
		tags_orientation[i]=msg_in.tags_orientation[i];
	   }
}

void Imu_callback(const sensor_msgs::Imu& imu_in)
{
	//Take in state of remaining parts of state from the imu topic	

	w_roll=imu_in.angular_velocity.x; //in rads/sec
	w_pitch=imu_in.angular_velocity.y; //in rads/sec
	w_yaw=imu_in.angular_velocity.z; //in rads/sec
}

void get_new_residual_and_H(void){

	//UPDATE Z
	//z = [ux uy d vx1 vy1 vz1 rp1 rr1 wp1 wr1]T
	z(0)=( (((float)(tags_xc[tag_id]-500.0)) /500.0) *cam_width_degree/2.0);
	z(1)=( (((float)(tags_yc[tag_id]-500.0)) /500.0) *cam_height_degree/2.0);
	z(2)=0.1*tags_distance[0]; // cm to meters
	z(3)=vel_x;
	z(4)=vel_y;
	z(5)=vel_z;
	z(6)=rotation_pitch;
	z(7)= rotation_roll;
	z(8)=w_pitch;
	z(9)=w_roll; 

//PARCON TEST VECTOR
z<<1,1,1.4, 0,0,0, 0,0, 0,0;

	x12_hat=x_minus(0);//estamate of position between rotors
	y12_hat=x_minus(1);
	z12_hat=x_minus(2);
	vx1_hat=x_minus(3); //estimate of velocity quad1
	vy1_hat=x_minus(4);
	vz1_hat=x_minus(5);
	//x_minus(6);
	//x_minus(7);
	//x_minus(8);
	rp1_hat=x_minus(9);//estimate of pitch quad1
	rr1_hat=x_minus(10);//estimate of roll quad1
	//x_minus(11);
	//x_minus(12);
	wp1_hat=x_minus(13);//estimate of ang velocity roll quad1
	wr1_hat=x_minus(14);//estimate of ang velocity roll quad1

	float xyz=pow(x12_hat,2)+pow(y12_hat,2)+pow(z12_hat,2);
	float sq_xyz=pow(xyz,0.5);



	std::cout <<"z"<<std::endl;
	std::cout << z << std::endl;

	std::cout <<"	x12_hat"<<std::endl;
	std::cout <<	x12_hat << std::endl;
	
	std::cout <<"	sq_xyz"<<std::endl;
	std::cout <<	sq_xyz << std::endl;

	std::cout <<"	asin(x12_hat/sq_xyz)"<<std::endl;
	std::cout <<	asin(x12_hat/sq_xyz)  << std::endl;

	std::cout <<"y"<<std::endl;
	std::cout << y << std::endl;



//in the form y= new observation minus nonlinear model

	
	y(0)=z(0)-asin(x12_hat/sq_xyz)-rp1_hat; //error in ux
	y(1)=z(1)-asin(y12_hat/sq_xyz)-rr1_hat; //error in uy
	y(2)=z(2)-sq_xyz; //error in distance away

	//linear parts
	y(3)=z(3)- vx1_hat; //error in vx of quad1
	y(4)=z(4)- vy1_hat; //error in vy of quad1
	y(5)=z(5)- vz1_hat; //error in vz of quad1
	y(6)=z(6)- rp1_hat; //error in pitch of quad1
	y(7)=z(7)- rr1_hat; //error in roll of quad1
	y(8)=z(8)- wp1_hat; //error in ang vel pitch of quad1
	y(9)=z(9)- wp1_hat; //error in ang vel roll of quad1


//Build the H matrix 
	float three_halfs_xyz=pow(xyz,1.5);
	float x12_hat2=pow(x12_hat,2.0);
	float y12_hat2=pow(y12_hat,2.0);
	
	
	
	//z = [ux uy d vx1 vy1 vz1 rp1 rr1 wp1 wr1]T
	//H 1st row (ux)
	float dux_dx = - ((x12_hat2/three_halfs_xyz) + 1/sq_xyz)/ pow(1-(x12_hat2/xyz),0.5);
	float dux_dy= (x12_hat*y12_hat)/( three_halfs_xyz* pow( 1-(x12_hat2/xyz),0.5) );
	float dux_dz= (x12_hat*z12_hat)/( three_halfs_xyz* pow( 1-(x12_hat2/xyz),0.5) );
	float dux_drp= -1.0;
	
	H(0,0)=dux_dx; 
	H(0,1)=dux_dy; 
	H(0,2)=dux_dz;
	H(0,9)=dux_drp;
	
	//H 2nd row (uy)
	float duy_dx= (x12_hat*y12_hat)/( three_halfs_xyz* pow( 1-(y12_hat2/xyz),0.5) );
	float duy_dy=- ((y12_hat2/three_halfs_xyz) + 1/sq_xyz)/ pow(1-(y12_hat2/xyz),0.5);
	float duy_dz=(z12_hat*y12_hat)/( three_halfs_xyz* pow( 1-(y12_hat2/xyz),0.5) );
	float duy_drr= -1.0;
	
	H(1,0)=duy_dx; 
	H(1,1)=duy_dy; 
	H(1,2)=duy_dz;
	H(1,10)=duy_drr;
	
	//H 3rd row (d)
	float dd_dx=x12_hat/sq_xyz;
	float dd_dy=y12_hat/sq_xyz;
	float dd_dz=z12_hat/sq_xyz;

	std::cout <<"dd_dx"<<std::endl;
	std::cout << dd_dx << std::endl;

	H(2,0)=dd_dx; 
	H(2,1)=dd_dy; 
	H(2,2)=dd_dz;

	//Rest of the H matrix is just composed of ones and zeros.
	//ones on matching terms aka: dvx1/dvx1, dvy1/dvy1
	//zeros elsewhere
}


int main(int argc, char** argv)
{
	//ROS stuff
	ROS_INFO("Starting Kalman");
	ros::init(argc, argv,"Kalman");
    ros::NodeHandle node;
    ros::Rate loop_rate(100);
	ros::Subscriber nav_sub;
	ros::Subscriber imu_sub;
	ros::Publisher state_pub;
	
	state_pub = node.advertise<std_msgs::Float32MultiArray> ("state_post_KF", 1);
	nav_sub = node.subscribe("/ardrone/navdata", 1, state_callback);
	imu_sub = node.subscribe("/ardrone/imu", 1, Imu_callback);
	

	init_matrix(); //Initilize matrix

	ROS_INFO("Starting Kalman loop \n");
	
	while (ros::ok()){
			
		//Prediction Step
		x_minus=A*x_old+B1*u1_old+B2*u2_old;
		
		std::cout << "A" << std::endl;
		std::cout << A << std::endl;
		
		std::cout <<"x_minus"<<std::endl;
		std::cout << x_minus << std::endl;
		
		P_minus=A*P_old*A.transpose() + Q;
		get_new_residual_and_H();
		
		std::cout <<"H"<<std::endl;
		std::cout << H << std::endl;
		
		//Correction Step
		O=H*P_minus*H.transpose()+R;
		K=P_minus*H.transpose()*O.inverse();
		x=x_minus+K*y;
		P=(I-K*H)*P_minus;

		std::cout <<"x"<<std::endl;
		std::cout << x << std::endl;
	    
	 	x_msg.data.clear(); //clear data
		float move =0.0;
	    for (long int i=0; i<dimention_n; i++)
			{
			move=x[i];
			x_msg.data.push_back(move);//fill msg
			}
		
//		std::cout << x_msg << std::endl;
		state_pub.publish(x_msg); //publish message
		
		x_old=x;
		P_old=P;
		u1_old=u1;
		u2_old=u2;
		
		ros::spinOnce();
		loop_rate.sleep();
		}//while ros ok
		
	
}//main
