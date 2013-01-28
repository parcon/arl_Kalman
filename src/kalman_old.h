//matrix work for kalman


//part one
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
state_to_obs H_trans;
control_model B1;
control_model B2;
dynamics_model A;
dynamics_model A_trans;
error_cov Q;
error_cov P;
error_cov P_old;
error_cov P_minus;
measurement_error_cov R;
measurement_error_cov O;

obs_vector n;
obs_vector z;
obs_vector y;
control_vector u1;
control_vector u1_old;
control_vector u2;
control_vector u2_old;

State_vector x_minus;
State_vector x;
State_vector x_old;
////////////////////////////////

void init_matrix(void){

//part two
	int g=  9.8;
	int k1= .25;
	int k2= 0.5;
	int k3= 0.5;
	

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

	0,0,0, 0,0,0,   0,0,0,  0,0,  0,0, k3,0, 0,0, //14
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,k3, 0,0, //15

	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, k3,0, //16
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,0, 0,0, 0,k3; //17
	

/*
//this is equations where ux and uy are included thus -k2 in w_dot = f(r,w)
	0,0,0, 0,0,0,   0,0,0,  -k2,0,  0,0, k3,0, 0,0, //14
	0,0,0, 0,0,0,   0,0,0,   0,-k2, 0,0, 0,k3, 0,0, //15

	0,0,0, 0,0,0,   0,0,0,  0,0, -k2,0, 0,0, k3,0, //16
	0,0,0, 0,0,0,   0,0,0,  0,0, 0,-k2, 0,0, 0,k3; //17
	
	*/
//std::cout << "A" << std::endl;
//std::cout << A << std::endl;


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
//std::cout << std::endl;
//std::cout << "B1" << std::endl;
//std::cout << B1 << std::endl;

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
//std::cout << std::endl;
//std::cout << "B2" << std::endl;
//std::cout << B2 << std::endl;

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
//std::cout << std::endl;
//std::cout << "H" << std::endl;
//std::cout << H << std::endl;

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

//x_old
x_old <<1,1,1, 0,1,0,   1,0,1,  0,0, 0,0, 0,0, 0,0;
}//end matrix setup
