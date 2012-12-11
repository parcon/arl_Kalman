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
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>

const int dimention_n = 2;
const int dimention_m= 2;
const int dimention_l= 1;


typedef Eigen::Matrix<float,dimention_n,1> State_vector;
typedef Eigen::Matrix<float,dimention_l,1> control_vector;
typedef Eigen::Matrix<float,dimention_m,1> obs_vector;

typedef Eigen::Matrix<float, dimention_n, dimention_n> dynamics_model;
typedef Eigen::Matrix<float, dimention_n, dimention_l> control_model;
typedef Eigen::Matrix<float, dimention_m, dimention_n> obs_to_state;
typedef Eigen::Matrix<float, dimention_n, dimention_n> error_cov;
typedef Eigen::Matrix<float, dimention_m, dimention_m> measurement_error_cov;

error_cov I;
obs_to_state K;
obs_to_state H;
control_model B;
dynamics_model A;
error_cov Q;
error_cov P;
error_cov P_old;
error_cov P_minus;
measurement_error_cov R;
measurement_error_cov O;

obs_vector n;
obs_vector z;
control_vector u;
control_vector u_old;

State_vector x_minus;
State_vector x;
State_vector x_old;
geometry_msgs::Vector3 msg;

//for rand number
float LO= -1;
float HI= 1;
float rand_float=0;

void get_new_measurements()
{
	rand_float = LO + (float)rand()/(1.0/(HI-LO));
	n<< rand_float, 0 ; //n = [rand from -1 to 1, 0]
	z=H*x+n;
	/*
	std::cout <<"n \n";
	std::cout <<n;
	std::cout <<"\n";
	
	std::cout <<"z \n";
	std::cout <<z;
	std::cout <<"\n";
*/
}


int main(int argc, char** argv)
{
//ROS stuff
	ROS_INFO("Starting Kalman");
	ros::init(argc, argv,"Kalman");
    ros::NodeHandle node;
    ros::Rate loop_rate(50);
//	ros::Subscriber sub;
//	ros::Publisher pub;
	ros::Publisher pub = node.advertise<geometry_msgs::Vector3> ("state", 1);
	//
	

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
	
ROS_INFO("Starting Kalman loop \n");
	
	while (ros::ok()){
			
		//Prediction Step
		x_minus=A*x_old+B*u_old;
		/*
		std::cout <<"x minus \n";
		std::cout <<x_minus;
		std::cout <<"\n";
		*/
		P_minus=A*P_old*A.transpose() + Q;
		get_new_measurements();

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
		P=(I.setIdentity()-K*H)*P_minus;
		/*
		std::cout <<"P \n";
		std::cout <<P;
		std::cout <<"\n";
		*/
		//Next step
	    //ROS_INFO("State: %f %f",x(0), x(1));
	    msg.x=x(0);
		msg.y=x(1);
		msg.z=z(0);
		pub.publish(msg);
		
		x_old=x;
		P_old=P;
		}//while ros ok
	
}//main

/*
Explaination of Kalaman filter (http://forums.udacity.com/questions/1010153/what-are-all-those-matrices-for-the-kalman-filter-part-i-x-f-p-h-r-u)
Vector x, the variables
x is the values of all the variables you’re considering in your system. It might, for example, be position and velocity. Or if you were using a Kalman filter to model polling results, it might be Rick Santorum’s support and momentum in Ohio Presidential polls. It might be a sick person’s temperature and how fast the temperature is going up. It’s a vector. Let’s say it has n elements— that will be important for the size of other matrices.

For example, let’s say we’re modeling a sick person’s temperature. She’s lying there in the hospital bed with three temperature sensors stuck to her, and we want to see if her temperature is going up fast, in which case we’ll call a nurse. (For purposes of this example, we’ll assume that in the time period we’re observing, the rate of rise is constant.)

We might initialize x with a temperature of 98.6 and a temperature rising rate of 0. The Kalman filter algorithm will change x—that’s the whole point, to find the correct values for x.

Matrix F, the update matrix
F is the n by n update matrix. Kalman filters model a system over time. After each tick of time, we predict what the values of x are, and then we measure and do some computation. F is used in the update step. Here’s how it works:
For each value in x, we write an equation to update that value, a linear equation in all the variables in x. Then we can just read off the coefficients to make the matrix.

For our example, our x vector would be (x1, x2) where x1 is the temperature in degrees and x2 is the rate of change in hundredths of a degree (doesn’t matter whether Celsius or Fahrenheit, just as long as it’s the same for all) per minute. We check our sensors every minute, and update every minute.

Let’s figure out how to make the matrix F. If our patient's temperature is x1, what is it one minute later? It’s x1, the old temperature, plus the change. Note that because temperature is in degrees but the change is in hundredths of degrees, we have to put in the coefficient of 1/100.

x[1]' = x[1] + x[2]/100
If the rate of change is x2, what is it one minute later? Still x2, because we’ve assumed the rate is constant.

x[2]' = 0x[1] + x[2]
Now we write out all our equations, like this:

x[1]' = x[1] + x[2]/100
x[2]' = 0x[1] + x[2]
In words, the new temperature, x1', is the old temperature plus the rise over the time segment. The new rate of change, x2', is the same as the old rate of change. We can just read off the coefficients of the right hand side, and there’s our matrix F:

1 .01
0 1
Bingo. Note that these equations have to be linear equations in the variables. We might want to take the sine of a variable, or square it, or multiply two variables together. We can’t, though, not in a Kalman filter. These are linear equations.

z, the measurement vector
Now let’s turn to z, the measurement vector. It’s just the outputs from the sensors. Simple. Let’s say we have m sensors. That could be, and probably is, different from n, the number of variables we’re keeping track of. In our example, let’s say we have three temperature probes, all somewhat inaccurate.

H, the extraction matrix
The matrix H tells us what sensor readings we’d get if x were the true state of affairs and our sensors were perfect. It’s the matrix we use to extract the measurement from the data. If we multiply H times a perfectly correct x, we get a perfectly correct z.

So let’s figure out what z1, z2 and z3, the readings from our three temperature sensors, would be if we actually knew the patient’s temperature and rate of temperature rising, and our sensors were perfect. Again, we just write out our equations:

z1 = x1 + 0x2
z2 = x1 + 0x2
z3 = x1 + 0x2
because if we knew the patients real temperature, and our sensors perfectly measured that temperature, z1 would be the same as x1 and so would z2 and z3.
Again, we just read off the coefficients to make H:

1 0
1 0
1 0
Remember F, the update matrix, is n by n. Notice that H, the extraction matrix, is m by n. When we multiply H by x, we get a vector of size m.

P, the covariance matrix of x
P is the covariance matrix of the vector x. x is a vector of dimension n, so P is n by n. Down the diagonal of P, we find the variances of the elements of x: the bigger the number, the bigger our window of uncertainty for that element. On the off diagonals, at P[i][j], we find the covariances of x[i] with x[j]. Covariance matrices must be symmetric matrices, because the covariance of x[i] and x[j] is also the covariance of x[j] and x[i], so P[i][j]==P[j][i]. That is, the whole thing is symmetric down the main diagonal.

P gets updated as we run our Kalman filter and become more certain of the value of the x vector.
For the patient example, we start out pretty uncertain. We’ll give pretty big variances to both x1, the temperature, and x2, the rate of change in temperature. We don’t have any notion that temperature and rise in temperature are correlated, so we’ll make the covariance 0. So the matrix will look like

3 0
0 .1
R, the covariance matrix of the measurement vector z
R is also a covariance matrix, but it’s the variances and covariances of our sensor measurements. Because z has dimension m, R is an m by m matrix. 
The Kalman filter algorithm does not change R, because the process can’t change our belief about the accuracy of our sensors—that’s a property of the sensors themselves. We know the variance of our sensor either by testing it, or by reading the documentation that came with it, or something like that. Note that the covariances here are the covariances of the measurement error. A positive number means that if the first sensor is erroneously low, the second tends to be erroneously low, or if the first reads high, the second tends to read high; it doesn’t mean that if the first sensor reports a high number the second will also report a high number.

In our patient example, the three sensors are measuring exactly the same thing. So of course, their readings will be correlated. But, let’s say that any two sensors don’t tend to be off the same way— the inaccuracy is caused by cheap manufacturing techniques, not by something about the patient. In that case, we’d give them covariances of zero. Our sensors, let’s say, have variances of .2; they’re not very accurate. So our covariance matrix might look like:

    .2 0  0
    0 .2 0
    0 0 .2
But if we knew that the measurements from the three sensors tended to be off in the same direction—maybe all of them read low if a fan is blowing in the room, and high if music is playing—then we’d put positive covariances for them:

.2 .05 .05
.05 .2 .05
.05 .05 .2
u, the move vector
The last input matrix is the vector u. This one is pretty simple; it’s the control input, the move vector. It’s the change to x that we cause, or that we know is happening. Since we add it to x, it has dimension n. When the filter updates, it adds u to the new x.

I can’t think of a good example using our patient and our thermometers, but suppose we were modeling the location and velocity of an object, and in addition to watching it move, we could also give it a shove. In the prediction stage, we’d update the object’s location based on our velocity, but then with u we’d add to its position and velocity because we moved it. u can change for each iteration of the Kalman filter. In our examples in class, it’s always all zeros; we’re not moving anything ourselves, just watching it move.

This is part I of an explanation of all the matrices in the Kalman filter. I'm writing it up for myself to see if I understand what's going on. This is very much a work in progress and needs proofreading. Part II (sketchy right now) covers S, K and y. Part III has some general comments about how the Kalman filter works, what kind of problems it works for, and what kind of problems it doesn't work for.

Added by @jasa:

The State Variable Model
IMHO Sebastian should have introduced the State Variable Model (SVM) which underlies the Kalman Filter. Here goes my contribution to this already nice wiki description of the KF as shown in our unit and homework.

One of the most important models for describing dynamic discrete or discretized systems is the SVM. The SVM consists in a couple of equations: the state equation

x(k) = F x(k-1) + B u(k-1)

where F is the state transition matrix, and the measurement or output equation

y(k) = H x(k)

The names of vectors and matrices follow those used by Sebastian, although they are not the standard in the SVM (which are A, B, C and D matrices). Sebastian has discarded u(k) (and implicitely considered B=Identity) since it puts it at zero. The vector x holds the state variables (2D positions and velocities in our unit).

The Kalman filter applies to the SVM with the addition of one vector to each of the equations, which represent the Gaussian uncertainty in the state equation, w(k), and in the measurement, v(k). Thus:

x(k) = F x(k-1) + u(k-1) + w(k-1)

y(k) = H x(k) + v(k)

Sebastian postulated no uncertainity at system level, so w(k) and its covarianve matrix don't appear in the Kalman filter equations taught in this unit, but v(k) appears, and its covariance matrix is our matrix R.

Kalman deduced, from the SVM, the "best" possible estimator (in the sense of the minimum variance estimator) of the output variable in each step, y(k), and the Kalman filter equations were born. You can find the deduction of the KF equations in many books and papers around the Web.

So, to conclude, our KF equations are based in the SVM which can describe almost all dynamic systems (discrete, in our case). Also, since we are iteratively updating the vectors and matrices inside the measurements loop, using the same data structures, the time iteration index, k, disappears from our implementation.
*/
