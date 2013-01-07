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
//#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Imu.h>
#include <ardrone_autonomy/Navdata.h>

const int dimention_n = 17;
const int dimention_m= 10;
const int dimention_l= 3;


typedef Eigen::Matrix<float,dimention_n,1> State_vector;
typedef Eigen::Matrix<float,dimention_l,1> control_vector;
typedef Eigen::Matrix<float,dimention_m,1> obs_vector;

typedef Eigen::Matrix<float, dimention_n, dimention_n> dynamics_model;
typedef Eigen::Matrix<float, dimention_n, dimention_l> control_model;
typedef Eigen::Matrix<float, dimention_m, dimention_n> state_to_obs;
typedef Eigen::Matrix<float, dimention_n, dimention_m> obs_to_state;
typedef Eigen::Matrix<float, dimention_n, dimention_n> error_cov;
typedef Eigen::Matrix<float, dimention_m, dimention_m> measurement_error_cov;

error_cov I;
obs_to_state K;
state_to_obs H;
control_model B1;
control_model B2;
dynamics_model A;
error_cov Q;
error_cov P;
error_cov P_old;
error_cov P_minus;
measurement_error_cov R;
measurement_error_cov O;

obs_vector n;
obs_vector z;
control_vector u1;
control_vector u1_old;
control_vector u2;
control_vector u2_old;

//Eigen::Vector17f x;
//Eigen::Vector17f x_minus;
//Eigen::Vector17f x_old;
State_vector x_minus;
State_vector x;
State_vector x_old;

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


int tag_id =0;//CHANGES WHICH TAG TO DISPLAY
//int had_message =0;
//int drone_state;
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
	//Take in state of robot that we are interested in
	rotation_roll=msg_in.rotX*deg2rad;
	rotation_pitch=msg_in.rotY*deg2rad;
	rotation_yaw=msg_in.rotZ*deg2rad;
	
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
	//Take in state of remaining parts of state	

	w_roll=imu_in.angular_velocity.x; //in rads/sec
	w_pitch=imu_in.angular_velocity.y; //in rads/sec
	w_yaw=imu_in.angular_velocity.z; //in rads/sec
}

void get_new_observations(void){
tag_position[0]=tags_distance[tag_id]*sin(vision_angle[0]);
tag_position[1]=tags_distance[tag_id]*sin(vision_angle[1]);
tag_position[2]=tag_position[0]/tan(vision_angle[0]);

z<<tag_position[0],tag_position[1],tag_position[2], //p12
	vel_x,vel_y,vel_z, //velocity of robot1
	rotation_roll, rotation_pitch, //rot of robot1
	w_roll,w_roll; //ang vel of robot1
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
	imu_sub = node.subscribe("/ardrone/Imu", 1, Imu_callback);
	
/*
//Mass spring damper system
float k=1;
float m=10;
float d=0;


A<< d/m,-k/m,0,1;
B<<1/m,0;
H<<1,0,0,0; //is this C?
u_old<<0;
x_old<< 1,0;
Q<<.1,0,.1,0;
P_old<<.1,0,.1,0;
R<<1,0,0,1;
*/	
int g =9.8;
int k1= 1.0;
int k2=0.5;
int k3 =0.5;
vision_angle[0]=( (((float)(tags_xc[tag_id]-500.0)) /500.0) *cam_width_degree/2.0);
vision_angle[1]=( (((float)(tags_yc[tag_id]-500.0)) /500.0) *cam_height_degree/2.0);

//A
A<< 0,0,0, 1,0,0, -1,0,0, 0,0, 0,0, 0,0, 0,0, //first line
	0,0,0, 0,1,0, 0,-1,0, 0,0, 0,0, 0,0, 0,0, //2nd line
	0,0,0, 0,0,1, 0,0,-1, 0,0, 0,0, 0,0, 0,0, //3nd line

	0,0,0, 0,0,0,   0,0,0,  0,g, 0,0, 0,0, 0,0, //4 line
	0,0,0, 0,0,0,   0,0,0, -g,0, 0,0, 0,0, 0,0, //5 line
	0,0,0, 0,0,-k1, 0,0,0,  0,0, 0,0, 0,0, 0,0, //6 line

	0,0,0, 0,0,0, 0,0,0,   0,0,  0,g, 0,0, 0,0, //7 line
	0,0,0, 0,0,0, 0,0,0,   0,0, -g,0, 0,0, 0,0, //8 line
	0,0,0, 0,0,0, 0,0,-k1, 0,0,  0,0, 0,0, 0,0, //9 line

	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 1,0, 0,0, //10 line
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,1, 0,0, //11 line

	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 1,0, //12 line
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,1, //13 line

	0,0,0, 0,0,0,   0,0,0,  -k2,0,  0,0, k3,0, 0,0, //14
	0,0,0, 0,0,0,   0,0,0,   0,-k2, 0,0, 0,k3, 0,0, //15

	0,0,0, 0,0,0,   0,0,0,  0,0, -k2,0, 0,0, k3,0, //16
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,-k2, 0,0, 0,k3; //17
std::cout << "A" << std::endl;
std::cout << A << std::endl;


//B1
B1<<0,0,0, //1
	0,0,0, //2
	0,0,0, //3

	0,0,0, //4
	0,0,0, //5
	0,0,k1, //6

	0,0,0, //7
	0,0,0, //8
	0,0,0, //9

	0,0,0, //10
	0,0,0, //11

	0,0,0, //12
	0,0,0, //13

	k2,0,0, //14
	0,k2,0, //15

	0,0,0, //16
	0,0,0; //17
std::cout << std::endl;
std::cout << "B1" << std::endl;
std::cout << B1 << std::endl;

//B2
B2<<0,0,0, //1
	0,0,0, //2
	0,0,0, //3

	0,0,0, //4
	0,0,0, //5
	0,0,0, //6

	0,0,0, //7
	0,0,0, //8
	0,0,k1, //9

	0,0,0, //10
	0,0,0, //11

	0,0,0, //12
	0,0,0, //13

	0,0,0, //14
	0,0,0, //15

	k2,0,0, //16
	0,k2,0; //17
std::cout << std::endl;
std::cout << "B2" << std::endl;
std::cout << B2 << std::endl;

//H
H<< 1,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,0, 
	0,1,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,0,
	0,0,1, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,0,

	0,0,0, 1,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,0, 
	0,0,0, 0,1,0,   0,0,0,  0,0, 0,0, 0,0, 0,0, 
	0,0,0, 0,0,1,   0,0,0,  0,0, 0,0, 0,0, 0,0, 

	0,0,0, 0,0,0,   0,0,0,  1,0, 0,0, 0,0, 0,0,
	0,0,0, 0,0,0,   0,0,0,  0,1, 0,0, 0,0, 0,0,
	
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 1,0, 0,0,
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,1, 0,0;
std::cout << std::endl;
std::cout << "H" << std::endl;
std::cout << H << std::endl;

//I
I<<Eigen::Matrix<float, dimention_n, dimention_n>::Identity();

//Q
Q=I;

//R
R=Eigen::Matrix<float, dimention_m, dimention_m>::Identity();

//P_old
P_old=I;

//u1
u1<< 0,0,0;

//u1_old
u1_old<< 0,0,0;

//u2
u2<< 0,0,0;

//u2_old
u2_old<< 0,0,0;

ROS_INFO("Starting Kalman loop \n");
	
	while (ros::ok()){
			
		//Prediction Step
		x_minus=A*x_old+B1*u1_old+B2*u2_old;
		/*
		std::cout <<"x minus \n";
		std::cout <<x_minus;
		std::cout <<"\n";
		*/
		
		P_minus=A*P_old*A.transpose() + Q;
		get_new_observations();
		ROS_INFO("Correction \n");
		//Correction Step
		O=H*P_minus*H.transpose()+R;
	
		/*
		std::cout <<"O \n";
		std::cout <<O;
		std::cout <<"\n";
		std::cout <<"O.inv \n";
		std::cout <<O.inverse();
		std::cout <<"\n";
		std::cout <<"H \n";
		std::cout <<H;
		std::cout <<"\n";
		std::cout <<"H.trans \n";
		std::cout <<H.transpose();
		std::cout <<"\n";
		std::cout <<"P- * H.trans \n";
		std::cout <<P_minus*H.transpose();
		std::cout <<"\n";
		std::cout <<"P- * H.trans \n";
		std::cout <<P_minus*H.transpose();
		std::cout <<"\n";
		*/
		K=P_minus*H.transpose()*O.inverse();
			
		/*
		std::cout <<"K \n";
		std::cout <<K;
		std::cout <<"\n";
		*/
		x=x_minus+K*(z-H*x_minus);
				
		/*
		std::cout <<"x \n";
		std::cout <<x;
		std::cout <<"\n";
		*/
		P=(I-K*H)*P_minus;
		/*
		std::cout <<"P \n";
		std::cout <<P;
		std::cout <<"\n";
		*/
		//Next step
	    //ROS_INFO("State: %f %f",x(0), x(1));
	    
	    std::cout <<"x"<<std::endl;
		std::cout << x << std::endl;
	    
	 	x_msg.data.clear(); //clear data
		float move =0.0;
	    for (long int i=0; i<dimention_n; i++)
	    {
		move=x[i];
		x_msg.data.push_back(move);	    		//fill msg
		}
		
		std::cout << x_msg << std::endl;
		state_pub.publish(x_msg); //publish message
		
		x_old=x;
		P_old=P;
		u1_old=u1;
		u2_old=u2;
		ros::spinOnce();
		loop_rate.sleep();
		}//while ros ok
		
	
}//main
