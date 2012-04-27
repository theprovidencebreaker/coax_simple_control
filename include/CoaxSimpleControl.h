#ifndef __COAX_SIMPLE_CONTROL__
#define __COAX_SIMPLE_CONTROL__

#define CONTROL_LANDED 0 // State when helicopter has landed successfully                           
#define CONTROL_START 1 // Start / Takeoff                                                          
#define CONTROL_TRIM 2 // Trim Servos                                                               
#define CONTROL_HOVER 3 // Hover                                                                    
#define CONTROL_GOTOPOS 4 // Go to Position                                                         
#define CONTROL_TRAJECTORY 5 // Follow Trajectory                                                   
#define CONTROL_LANDING 6 // Landing maneuver

#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>

#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxRawControl.h>
#include <coax_msgs/CoaxControl.h>
#include <coax_msgs/CoaxReachNavState.h>
#include <coax_msgs/CoaxConfigureComm.h>
#include <coax_msgs/CoaxSetTimeout.h>

#include <coax_simple_control/SetNavMode.h>
#include <coax_simple_control/SetControlMode.h>
#include <coax_simple_control/SetWaypoint.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>


using namespace std;

class CoaxSimpleControl 
{
	// friend class ImageProc;

public:
	CoaxSimpleControl(ros::NodeHandle &);
	~CoaxSimpleControl();

	void loadParams(ros::NodeHandle & n);
	bool reachNavState(int des_state, float timeout);
	bool configureComm(int frequency, int contents);
	bool configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode);
	bool setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms);

	bool setNavMode(coax_simple_control::SetNavMode::Request &req, coax_simple_control::SetNavMode::Response &out);
	bool setControlMode(coax_simple_control::SetControlMode::Request &req, coax_simple_control::SetControlMode::Response &out);
        bool setWaypoint(coax_simple_control::SetWaypoint::Request &req1, coax_simple_control::SetWaypoint::Response &out);

	void coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message);
	void coaxTfCallback(const tf::tfMessage::ConstPtr & message);
	void stabilizationControl(void);
	bool rotorReady(void);
	void controlPublisher(size_t rate);
	
	bool setRawControl(double motor_up,double motor_lo, double servo_ro,double servo_pi);


private:
	ros::ServiceClient reach_nav_state;
	ros::ServiceClient configure_comm;
	ros::ServiceClient configure_control;
	ros::ServiceClient set_timeout;
	
	ros::Subscriber coax_state_sub;
        ros::Subscriber coax_tf_sub;

	ros::Publisher raw_control_pub;
	ros::Publisher simple_control_pub;

	vector<ros::ServiceServer> set_nav_mode;
	vector<ros::ServiceServer> set_control_mode;
        vector<ros::ServiceServer> set_waypoint;

	bool LOW_POWER_DETECTED;

	int CONTROL_MODE;
	bool FIRST_START;
	bool FIRST_LANDING;
	bool FIRST_HOVER;
	bool INIT_DESIRE;

	bool coax_nav_mode;
	bool coax_control_mode;
        bool coax_waypoint;
	int coax_state_age;
	int raw_control_age;
	int init_count;
        int mil_count;
	int rotor_ready_count;
	
	double battery_voltage;
	
	double imu_y; // imu yaw
	double imu_r; // imu roll 
	double imu_p; // imu pitch
	double range_al; // range altitude

	double rc_th; // rc throttle
	double rc_y;  // rc yaw
	double rc_r;  // rc roll
	double rc_p;  // rc pitch
	double rc_trim_th; // rc throttle trim
	double rc_trim_y;  // rc yaw trim
	double rc_trim_r;  // rc roll trim
	double rc_trim_p;  // rc pitch trim
	
	double img_th;  // image throttle
	double img_y; 	// image yaw
	double img_r; 	// image roll
	double img_p; 	// image pitch

	double gyro_ch1;
	double gyro_ch2;
	double gyro_ch3;
	double accel_x;
	double accel_y;
	double accel_z;
        double mag_1;
        double mag_2;
        double mag_3; 

	double motor_up;
	double motor_lo;
	double servo_roll;
	double servo_pitch;
	double roll_trim;
	double pitch_trim;

	double motor_const1;
	double motor_const2;
	double servo1_const;
	double servo2_const;

	double range_base;
        double altitude_trim;

	double motor1_des;
	double motor2_des;
	double servo1_des;
	double servo2_des;

	double r_rc_coef;
	double p_rc_coef;

	double yaw_coef1;
	double yaw_coef2;
        double yaw_offset;

	double thr_coef1;
	double thr_coef2;

	double kp_yaw;
	double kd_yaw;
	double kp_roll;
	double kd_roll;
	double kp_pitch;
	double kd_pitch;
	double kp_altitude;
        double kd_altitude;

        double K1_pitch;
        double K1_roll;

	double yaw_des;
	double yaw_rate_des;
	double roll_des;
	double roll_rate_des;
	double pitch_des;
	double pitch_rate_des;
	double altitude_des;

	double coax_global_x;
        double coax_global_y;
	double coax_global_z;
        double qnd0;
        double qnd1;
        double qnd2;
        double qnd3;
	

        double Gx_des;
        double Gy_des;

        double PI;
        double r2d;
        double d2r;
        
        double ang_err_lim;

        double x_vel_hist_1, x_vel_hist_2, x_vel_hist_3, x_vel_hist_4;
        double y_vel_hist_1, y_vel_hist_2, y_vel_hist_3, y_vel_hist_4;

        double x_gyro_1, x_gyro_2;
        double y_gyro_1, y_gyro_2;
 
        double aux1, aux2, aux3, aux4, aux5, aux6;
        double aux7, aux8, aux9, aux10, aux11, aux12;
        double aux13, aux14, aux15, aux16, aux17;
        int aux_int1, aux_int2, aux_int3, aux_int4, aux_int5, aux_int6;

        double Gz_old1, Gz_old2, Gz_old3;
        double Gx_old[3], Gy_old[3];

        int way_changed;
        double way_old[3];
        double acc_hx[6], acc_hy[6];
        double vx_facc_old, vy_facc_old;


};

#endif
