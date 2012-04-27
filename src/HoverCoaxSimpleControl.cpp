// Source created with guidance/source from Matthias Faessler - UPENN Guest 2011 & Yida Zhang - UPENN Grad Student 2011-2012 

#include <iostream>
#include <cmath>
#include <vector>
#include <queue>
#include <algorithm>
#include <sstream>
#include <ros/ros.h>
#include <coax_msgs/CoaxState.h>
#include <coax_msgs/CoaxConfigureControl.h>

#include <com/sbapi.h>
#include <CoaxSimpleControl.h>

CoaxSimpleControl::CoaxSimpleControl(ros::NodeHandle &node)
:reach_nav_state(node.serviceClient<coax_msgs::CoaxReachNavState>("reach_nav_state"))
,configure_comm(node.serviceClient<coax_msgs::CoaxConfigureComm>("configure_comm"))
,configure_control(node.serviceClient<coax_msgs::CoaxConfigureControl>("configure_control"))
,set_timeout(node.serviceClient<coax_msgs::CoaxSetTimeout>("set_timeout"))

,coax_state_sub(node.subscribe("state",1, &CoaxSimpleControl::coaxStateCallback, this))
,coax_tf_sub(node.subscribe("tf",1, &CoaxSimpleControl::coaxTfCallback, this))

,raw_control_pub(node.advertise<coax_msgs::CoaxRawControl>("rawcontrol",1))
,simple_control_pub(node.advertise<coax_msgs::CoaxControl>("simplecontrol",1))

,LOW_POWER_DETECTED(false)
,CONTROL_MODE(CONTROL_LANDED)
,FIRST_START(false)
,FIRST_LANDING(false)
,FIRST_HOVER(false)
,INIT_DESIRE(false)
,coax_nav_mode(0)
,coax_control_mode(0)
,coax_state_age(0)
,raw_control_age(0)
,init_count(0)
,mil_count(0)
,rotor_ready_count(-1)
,battery_voltage(12.22)
,imu_y(0.0),imu_r(0.0),imu_p(0.0)
,range_al(0.0)
,rc_th(0.0),rc_y(0.0),rc_r(0.0),rc_p(0.0)
,rc_trim_th(0.0),rc_trim_y(0.104),rc_trim_r(0.054),rc_trim_p(0.036)
,img_th(0.0),img_y(0.0),img_r(0.0),img_p(0.0)
,gyro_ch1(0.0),gyro_ch2(0.0),gyro_ch3(0.0)
,accel_x(0.0),accel_y(0.0),accel_z(0.0)
,mag_1(0.0), mag_2(0.0), mag_3(0.0)
,motor_up(0),motor_lo(0)
,servo_roll(0),servo_pitch(0)
,roll_trim(0),pitch_trim(0)
,motor1_des(0.0),motor2_des(0.0),servo1_des(0.0),servo2_des(0.0)
,yaw_des(0.0),yaw_rate_des(0.0)
,roll_des(0.0),roll_rate_des(0.0)
,pitch_des(0.0),pitch_rate_des(0.0)
,altitude_des(1.5), coax_global_x(0.0), coax_global_y(0.0), coax_global_z(0.0)
,qnd0(0.0), qnd1(0.0), qnd2(0.0), qnd3(0.0) 
,Gx_des(0.0), Gy_des(0.0)
,PI(3.14159265), r2d(57.2958), d2r(0.0175)
,ang_err_lim(0.14)
,x_vel_hist_1(0.0), x_vel_hist_2(0.0), x_vel_hist_3(0.0) , x_vel_hist_4(0.0)
,y_vel_hist_1(0.0), y_vel_hist_2(0.0), y_vel_hist_3(0.0) , y_vel_hist_4(0.0)
,x_gyro_1(0.0), x_gyro_2(0.0)
,y_gyro_1(0.0), y_gyro_2(0.0)
,Gz_old1(0.0), Gz_old2(0.0)

{
  set_nav_mode.push_back(node.advertiseService("set_nav_mode", &CoaxSimpleControl::setNavMode, this));
  set_control_mode.push_back(node.advertiseService("set_control_mode", &CoaxSimpleControl::setControlMode, this));
  loadParams(node);
}

CoaxSimpleControl::~CoaxSimpleControl() {}

void CoaxSimpleControl::loadParams(ros::NodeHandle &n) {
  n.getParam("motorconst/const1",motor_const1);
  n.getParam("motorconst/const2",motor_const2);
  n.getParam("rollconst/const",servo1_const);
  n.getParam("pitchconst/const",servo2_const);
  n.getParam("yawcoef/coef1",yaw_coef1);
  n.getParam("yawcoef/coef2",yaw_coef2);
  n.getParam("yawcoef/offset", yaw_offset);
  n.getParam("throttlecoef/coef1",thr_coef1);
  n.getParam("throttlecoef/coef2",thr_coef2);
  n.getParam("rollrccoef/coef",r_rc_coef);
  n.getParam("pitchrccoef/coef",p_rc_coef);
  n.getParam("yawcontrol/proportional",kp_yaw);
  n.getParam("yawcontrol/differential",kd_yaw);
  n.getParam("rollcontrol/proportional",kp_roll);
  n.getParam("rollcontrol/differential",kd_roll);
  n.getParam("pitchcontrol/proportional",kp_pitch);
  n.getParam("pitchcontrol/differential",kd_pitch);
  n.getParam("altitude/base",range_base);
  n.getParam("altitude/trim", altitude_trim);
  n.getParam("altitudecontrol/proportional",kp_altitude);
  n.getParam("altitudecontrol/derivative",kd_altitude);
  n.getParam("errorparams/K1_pitch", K1_pitch);
  n.getParam("errorparams/K1_roll", K1_roll);
  n.getParam("auxparams/aux1", aux1);
  n.getParam("auxparams/aux2", aux2);
  n.getParam("auxparams/aux3", aux3);
  n.getParam("auxparams/aux4", aux4);
  n.getParam("auxparams/aux5", aux5);
  n.getParam("auxparams/aux6", aux6);
  n.getParam("auxparams/aux_int1", aux_int1);
  n.getParam("auxparams/aux_int2", aux_int2);
  n.getParam("auxparams/aux_int3", aux_int3);
  n.getParam("auxparams/aux_int4", aux_int4);
  n.getParam("auxparams/aux_int5", aux_int5);
  n.getParam("auxparams/aux_int6", aux_int6);
  n.getParam("trims/roll", roll_trim);
  n.getParam("trims/pitch", pitch_trim);
  
  
}

//===================
// Service Clients
//===================

bool CoaxSimpleControl::reachNavState(int des_state, float timeout) {
  coax_msgs::CoaxReachNavState srv;
  srv.request.desiredState = des_state;
  srv.request.timeout = timeout;
  reach_nav_state.call(srv);
  return 0;
}

bool CoaxSimpleControl::configureComm(int frequency, int contents) {
  coax_msgs::CoaxConfigureComm srv;
  srv.request.frequency = frequency;
  srv.request.contents = contents;
  configure_comm.call(srv);

  return 0;
}

bool CoaxSimpleControl::configureControl(size_t rollMode, size_t pitchMode, size_t yawMode, size_t altitudeMode) {
  coax_msgs::CoaxConfigureControl srv;
  srv.request.rollMode = rollMode;
  srv.request.pitchMode = pitchMode;
  srv.request.yawMode = yawMode;
  srv.request.altitudeMode = altitudeMode;
  configure_control.call(srv);
  //ROS_INFO("called configure control");
  return 0;
}

bool CoaxSimpleControl::setTimeout(size_t control_timeout_ms, size_t watchdog_timeout_ms) {
  coax_msgs::CoaxSetTimeout srv;
  srv.request.control_timeout_ms = control_timeout_ms;
  srv.request.watchdog_timeout_ms = watchdog_timeout_ms;
  set_timeout.call(srv);

  return 0;
}


//==============
//ServiceServer
//==============
bool CoaxSimpleControl::setNavMode(coax_simple_control::SetNavMode::Request &req, coax_simple_control::SetNavMode::Response &out) {
  out.result = 0;

  switch (req.mode)
    {
    case SB_NAV_STOP:
      break;
    case SB_NAV_IDLE:
      break;
    }
  return 0;
}

bool CoaxSimpleControl::setControlMode(coax_simple_control::SetControlMode::Request &req, coax_simple_control::SetControlMode::Response &out) {
  out.result = 0;

  switch (req.mode)
    {
    case 1:
      if (CONTROL_MODE != CONTROL_LANDED){
	ROS_INFO("Start can only be executed from mode CONTROL_LANDED");
	out.result = -1;
	break;
      }

      if (battery_voltage < 10.5) {
	ROS_INFO("Battery Low!!! (%f V) Start denied",battery_voltage);
	LOW_POWER_DETECTED = true;
	out.result = -1;
	break;
      }

      if (coax_nav_mode != SB_NAV_RAW) {
	if (coax_nav_mode != SB_NAV_STOP) {
	  reachNavState(SB_NAV_STOP, 0.5);
	  ros::Duration(0.5).sleep(); // make sure CoaX is in SB_NAV_STOP mode
	}
	reachNavState(SB_NAV_RAW, 0.5);
      }
      // switch to start procedure
      INIT_DESIRE = false;
      init_count = 0;
      rotor_ready_count = 0;
      CONTROL_MODE = CONTROL_START;
      motor1_des = 0;
      motor2_des = 0;
      servo1_des = 0.0;
      ROS_INFO("servo1_des %f", servo1_des);
      servo2_des = 0;
      break;

    case 9:
      motor1_des = 0;
      motor2_des = 0;
      servo1_des = 0;
      servo2_des = 0;
      //roll_trim = 0;
      //pitch_trim = 0;

      reachNavState(SB_NAV_STOP, 0.5);

      rotor_ready_count = -1;
      CONTROL_MODE = CONTROL_LANDED;
      break;

    default:
      ROS_INFO("Non existent control mode request!");
      out.result = -1;
    }
  return true;
}


//==============
// Subscribers
//==============

void CoaxSimpleControl::coaxStateCallback(const coax_msgs::CoaxState::ConstPtr & message) {
  battery_voltage = 0.8817*message->battery + 1.5299;
  coax_nav_mode = message->mode.navigation;

  if ((battery_voltage < 10.30) && !LOW_POWER_DETECTED){
    ROS_INFO("Battery Low!!! (%fV) Landing initialized",battery_voltage);
    LOW_POWER_DETECTED = true;
  }

  imu_y = message->yaw;
  imu_r = message->roll;
  imu_p = message->pitch;

  range_al = message->zfiltered;

  rc_th = message->rcChannel[0];
  rc_y = message->rcChannel[2];
  rc_r = message->rcChannel[4];
  rc_p = message->rcChannel[6];
  
  mag_1 = message->magneto[0];
  mag_2 = message->magneto[1];
  mag_3 = message->magneto[2];

  // rc_trim_th = message->rcChannel[1];
  // rc_trim_y = message->rcChannel[3];
  // rc_trim_r = message->rcChannel[5];
  // rc_trim_p = message->rcChannel[7];

  gyro_ch1 = message->gyro[0];
  gyro_ch2 = message->gyro[1];
  gyro_ch3 = message->gyro[2];

  accel_x = message->accel[0];
  accel_y = message->accel[1];
  accel_z = message->accel[2];


}


void CoaxSimpleControl::coaxTfCallback(const tf::tfMessage::ConstPtr & message) {

  coax_global_x = message->transforms[0].transform.translation.x;
  coax_global_y = message->transforms[0].transform.translation.y;
  coax_global_z = message->transforms[0].transform.translation.z;

  qnd1 = message->transforms[0].transform.rotation.x;
  qnd2 = message->transforms[0].transform.rotation.y;
  qnd3 = message->transforms[0].transform.rotation.z;
  qnd0 = message->transforms[0].transform.rotation.w;
  
  /* if (mil_count == 1000) {
      
    ROS_INFO("Coax Global X: %f ", coax_global_x);
          
  }; */
  

}


//==============
// Controller
//==============


bool CoaxSimpleControl::setRawControl(double motor1, double motor2, double servo1, double servo2) {
coax_msgs::CoaxRawControl raw_control;


 motor_up = motor1;
 motor_lo = motor2;
 servo_roll = servo1;
 servo_pitch = servo2;

 if (motor1 > 1)
   motor_up = 1;
else if (motor1 < 0)
  motor_up = 0;
 else
   motor_up = motor1;

 if (motor2 > 1)
   motor_lo = 1;
 else if (motor2 < 0)
   motor_lo = 0;
else
  motor_lo = motor2;

 if (servo1 > 1)
   servo_roll = 1;
 else if (servo1 < -1)
   servo_roll = -1;
 else
   servo_roll = servo1;

 if (servo2 > 1)
   servo_pitch = 1;
 else if (servo2 < -1)
   servo_pitch = -1;
 else
   servo_pitch = servo2;


 raw_control.motor1 = motor_up;
 raw_control.motor2 = motor_lo;
 raw_control.servo1 = servo_roll;
 raw_control.servo2 = servo_pitch;
 
 raw_control_pub.publish(raw_control);

 return 1;
}

bool CoaxSimpleControl::rotorReady(void) {
  double tt1 = 0;
  double tt2 = 0;
  if (rotor_ready_count < 0) 
    return false;

  if (rotor_ready_count <= 300) {
    rotor_ready_count++;
    ROS_INFO("rotorReady: %d", rotor_ready_count);
  } 
  if (rotor_ready_count < 150) {
    motor1_des = rotor_ready_count / 150 * motor_const1;
    motor2_des = 0;
    return false;
  }
  else if (rotor_ready_count < 300) {
    motor1_des = motor_const1;
    tt1 = (rotor_ready_count - 150.0);
    tt2 = tt1/150.0;
    motor2_des = (rotor_ready_count - 150.0) / 150.0 * motor_const2;
    ROS_INFO("Spooling up Rotor 2. Params: %f, %f ", tt1, tt2);
    return false;
  }

  /* if (rotor_ready_count == 300) {
    ROS_INFO("rotorReady = 300. Returning true");
  } */

  return true;
}

void CoaxSimpleControl::stabilizationControl(void) {
  // double Dyaw,Dyaw_rate,yaw_control;
  // double Daltitude,altitude_control;
  double pitch_control, roll_control, yaw_control, altitude_control;
  double rot_yaw[3][3];
  double e_roll, e_pitch, e_roll_dot, e_pitch_dot;
  double e_x, e_y, e_z, e_xb, e_yb, e_altitude;
  double e_yaw, e_yaw_dot;
  double psi_vic, psi_vic_d, term1, term2, pi_o;
  double yaw_deg, yaw, yaw_deg2, yaw2;
  double mag_conv1, mag_conv2, mag_conv3;
  double K_xy[2][6], K_hy[2][4];
  double hz_intern, delta_t;
  double Vx_global, Vy_global, V_xb, V_yb;
  double lqr_states[6], lqr_des[6], lqr_error[6];
  double cont_xy[2], u_des, v_des;
  double xgyro_smooth, ygyro_smooth;
  double e_altitude_v;
  double cont_hy[2], h_control[4];
  double mock_motor1, mock_motor2;
  double altitude_v_des;

  u_des = 1;
  v_des = 0;
  
  hz_intern = 100.000;
  delta_t = 1/hz_intern;

  Vx_global = (coax_global_x - x_vel_hist_1)*delta_t;
  Vy_global = (coax_global_y - y_vel_hist_1)*delta_t;

  x_vel_hist_4 = x_vel_hist_3;
  y_vel_hist_4 = y_vel_hist_3;

  x_vel_hist_3 = x_vel_hist_2;
  y_vel_hist_3 = y_vel_hist_2;

  x_vel_hist_2 = x_vel_hist_1;
  y_vel_hist_2 = y_vel_hist_1;

  x_vel_hist_1 = Vx_global;
  y_vel_hist_1 = Vy_global;

  //================================
  // Magnetometer data conversion
  //================================ 

  pi_o = 3.14159265;
  
  mag_conv1 = (mag_1 - 0.375)* 1.0;
  mag_conv2 = (mag_2 - 0.865)* 1.05;
    
  if(mag_conv2 < 0) {
    if(mag_conv1 > 0) {
      yaw_deg = 450 - atan2(mag_conv1, mag_conv2)*(r2d);
      }
    else if(mag_conv1 < 0) {
      yaw_deg = 90 - atan2(mag_conv1, mag_conv2)*(r2d);
      }
  }
  else if(mag_conv2 > 0) {
    yaw_deg = 90 - atan2(mag_conv1, mag_conv2)*(r2d); 
  }
  else if(mag_conv2 == 0) {
    if (mag_conv2 < 0) {
        yaw_deg = 180.0;
    }
    else {
        yaw_deg = 0.0; 
    }
         
  }

  yaw_deg = yaw_deg - 180; 
  if (yaw_deg > 180) {
    yaw_deg = yaw_deg - 180;
  }
  else if (yaw_deg <  -180) {
    yaw_deg = yaw_deg + 180;
  }

  yaw_deg = yaw_deg - yaw_offset; // 2nd term (after 180, is yaw trim)
  
  if (yaw_deg > 180) {
    yaw_deg = yaw_deg - 180;
  }
  else if (yaw_deg <  -180) {
    yaw_deg = yaw_deg + 180;
  }

  yaw = yaw_deg*d2r;
   
  //================================
  // OUTER-LOOP POSITION CONTROLLER
  //================================  

  // e_x = coax_global_x - 0;
  // e_y = coax_global_y - 0;
  e_x = coax_global_x - Gx_des;
  e_y = coax_global_y - Gy_des;
  e_z = coax_global_z - 0;

  term1 = 2*(qnd0*qnd3 + qnd1*qnd2);
  term2 = 1 - 2*(qnd2*qnd2 + qnd3*qnd3);
  // psi_vic = atan(term1/term2);
  psi_vic = atan2(term1, term2);
  

  psi_vic = psi_vic + pi_o/4.00; // + 45 deg
  if (psi_vic > pi_o) {
    psi_vic = psi_vic - pi_o/2.00;
  }
  else if (psi_vic < - pi_o) {
    psi_vic = psi_vic + + pi_o/2.00;
  }

  psi_vic_d = 180.0/pi_o*psi_vic;
	
  // ROTATION MATRIX - assuming imu_y comming in at +/-180
  // sign change

  /* yaw_deg2 = yaw_deg - 90;

  if (yaw_deg2 > 180) {
    yaw_deg2 = yaw_deg2 - 180;
  }
  else if (yaw_deg2 <  -180) {
    yaw_deg2 = yaw_deg2 + 180;
  }

  yaw2 = yaw_deg2*d2r;
  */

  
  rot_yaw[0][0] = cos(-yaw);
  rot_yaw[0][1] = sin(-yaw);
  rot_yaw[0][2] = 0;
  rot_yaw[1][0] = -sin(-yaw);
  rot_yaw[1][1] = cos(-yaw);
  rot_yaw[1][2] = 0;
  rot_yaw[2][0] = 0;
  rot_yaw[2][1] = 0;
  rot_yaw[2][2] = 1;
  

  // COORDINATE FRAME ROTATION
  e_xb = rot_yaw[0][0] * e_x + rot_yaw[0][1] * e_y + rot_yaw[0][2] * e_z;
  e_yb = rot_yaw[1][0] * e_x + rot_yaw[1][1] * e_y + rot_yaw[1][2] * e_z;

  V_xb = rot_yaw[0][0] * Vx_global + rot_yaw[0][1] * Vx_global; 
  V_yb = rot_yaw[1][0] * Vx_global + rot_yaw[1][1] * Vy_global;

  /*
  if (mil_count % 100 == 0) {
    ROS_INFO("V_xb: %f V_yb: %f ", V_xb, V_yb);
  }
  */

  // Update pitch_des and roll_des (everything in radians)
  
  // pitch_des = K1_pitch*e_xb + 0; // trim??
  // roll_des = K1_roll*e_yb + 0; // trim??
  pitch_des = 0;
  roll_des = 0; 
 
  // limit magnitude on error desired calculation
  if(pitch_des > ang_err_lim) {
    pitch_des = ang_err_lim;
  }
  else if(pitch_des < -ang_err_lim) {
    pitch_des = -ang_err_lim;
  }

  if(roll_des > ang_err_lim) {
    roll_des = ang_err_lim;
  }
  else if(roll_des < -ang_err_lim) {
    roll_des = -ang_err_lim;
  }
  
  
  
  //================================
  // INNER-LOOP POSE/ANGLE CONTROLLER
  //================================  

  // pitch and roll control

  e_pitch = imu_p - pitch_des; // pitch error = body y error
  e_roll = imu_r - roll_des; // roll error = body x error
    
  e_roll_dot  = gyro_ch1 - 0; // i.e. e_p_dot. desired = 0
  e_pitch_dot = gyro_ch2 - 0; // i.e. e_q_dot. desired = 0
  
  // roll_control = kp_roll * e_roll + kd_roll * e_roll_dot;
  // pitch_control = kp_pitch * e_pitch + kd_pitch * e_pitch_dot;

  roll_control = kp_roll * e_roll - 0.7*kp_pitch * e_pitch;
  pitch_control = kp_pitch * e_pitch + 0.7*kp_roll * e_roll;

  // yaw control
  
  e_yaw = yaw - yaw_des;
  e_yaw_dot = gyro_ch3 - 0;
  
  yaw_control = kp_yaw * e_yaw + kd_yaw * e_yaw_dot;

  // altitude control
  e_altitude = altitude_des - coax_global_z;
  altitude_v_des = 0.3*e_altitude;
  e_altitude_v = altitude_v_des - (coax_global_z - Gz_old1)*delta_t ;
  Gz_old1 = coax_global_z;
  altitude_control = kp_altitude * e_altitude + kd_altitude * e_altitude_v; // + ki_altitude * eint_altitude
  // altitude_control = altitude_trim;

  /* if (mil_count == 250 || mil_count == 500 || mil_count == 750 || mil_count == 1000 || mil_count == 1250 || mil_count == 1500 || mil_count == 1750 || mil_count == 2000 ) {
    ROS_INFO("Running Feedback Control");
    ROS_INFO("yaw info: desired, current, error, control: %f, %f, %f, %f ", yaw_des, yaw, e_yaw, yaw_control);
       
  } */
  
  /*
  if (mil_count % 100 == 0 ) { 
    ROS_INFO("e_xb: %f, e_yb: %f, pitch_des: %f, roll_des: %f, yaw_deg: %f ", e_xb, e_yb, pitch_des, roll_des, yaw_deg);
    ROS_INFO("alt_cont: %f", altitude_control);
  }
  */

  //================================
  // LQR CONTROL
  //================================  

  // SWASH PLATE

  K_xy[0][0] = -0.6646;
  K_xy[0][1] = -2.134;
  K_xy[0][2] = 6.7930;
  K_xy[0][3] = -2.1100;
  K_xy[0][4] = 1.0620;
  K_xy[0][5] = -0.3222;
  
  K_xy[1][0] = 2.1343;
  K_xy[1][1] = -0.6658;
  K_xy[1][2] = 2.1087;
  K_xy[1][3] = 6.7859;
  K_xy[1][4] = 0.3242;
  K_xy[1][5] = 1.0515;

  xgyro_smooth = gyro_ch2*0.6 + x_gyro_1*0.2 + x_gyro_2*0.2;
  ygyro_smooth = gyro_ch1*0.6 + y_gyro_1*0.2 + y_gyro_2*0.2;


  x_gyro_2 = x_gyro_1;
  y_gyro_2 = y_gyro_1;
  x_gyro_1 = xgyro_smooth;
  y_gyro_1 = ygyro_smooth;


  lqr_states[0] = Vx_global; // V_xb;
  lqr_states[1] = Vy_global; // V_yb;
  lqr_states[2] = imu_r;
  lqr_states[3] = -imu_p;
  lqr_states[4] = gyro_ch1;
  lqr_states[5] = -gyro_ch2;

  u_des = -0.5*e_x;
  v_des = -0.5*e_y;
  
  lqr_des[0] = u_des;
  lqr_des[1] = v_des;
  lqr_des[2] = roll_des;
  lqr_des[3] = pitch_des;
  lqr_des[4] = 0;
  lqr_des[5] = 0;
  
  for(int i=0; i<6; i++) {
    lqr_error[i] = lqr_des[i] - lqr_states[i];    
  } 
  // lqr_error[0] = 0;
  // lqr_error[1] = 0;
  // lqr_error[2] = 0;
  // lqr_error[3] = 0;
  // lqr_error[4] = 0;
  // lqr_error[5] = 0;

  
  cont_xy[0] = K_xy[0][0]*lqr_error[0] + K_xy[0][1]*lqr_error[1] + K_xy[0][2]*lqr_error[2] + K_xy[0][3]*lqr_error[3] + K_xy[0][4]*lqr_error[4] + K_xy[0][5]*lqr_error[5];
  cont_xy[1] = K_xy[1][0]*lqr_error[0] + K_xy[1][1]*lqr_error[1] + K_xy[1][2]*lqr_error[2] + K_xy[1][3]*lqr_error[3] + K_xy[1][4]*lqr_error[4] + K_xy[1][5]*lqr_error[5];
  
  cont_xy[0] = cont_xy[0]*aux1;
  cont_xy[1] = cont_xy[1]*aux1;
  
  if (mil_count % 50 == 0 ) {
    ROS_INFO("control suggested: %f %f", cont_xy[0], cont_xy[1]);
    ROS_INFO("u and v des: %f %f", u_des, v_des);
  }
  
  // ROTORS
  
  K_hy[0][0] = 0.6835;
  K_hy[0][1] = 3.9940;
  K_hy[0][2] = -0.7239;
  K_hy[0][3] = -1.0061;

  K_hy[1][0] = 0.7273;
  K_hy[1][1] = 4.2242;
  K_hy[1][2] = 0.6866;
  K_hy[1][3] = 0.9252;

  h_control[0] = aux2*e_altitude;
  h_control[1] = aux3*e_altitude_v;
  h_control[2] = aux4*e_yaw;
  h_control[3] = aux5*e_yaw_dot;

  cont_hy[0] = K_hy[0][0]*h_control[0] + K_hy[0][1]*h_control[1] + K_hy[0][2]*h_control[2] + K_hy[0][3]*h_control[3];
  cont_hy[1] = K_hy[1][0]*h_control[0] + K_hy[1][1]*h_control[1] + K_hy[1][2]*h_control[2] + K_hy[1][3]*h_control[3];
  
  mock_motor1 = motor_const1 + cont_hy[0];
  mock_motor2 = motor_const2 + cont_hy[1];



  //================================
  // SERVO INPUTS
  //================================

  // desired motor & servo output
  motor1_des = motor_const1 - yaw_control + altitude_control;
  motor2_des = motor_const2 + yaw_control + altitude_control;

  if (mil_count % 50 == 0 ) {
    ROS_INFO("mock motor : %f %f", mock_motor1, mock_motor2);
    ROS_INFO("real motor : %f %f", motor1_des, motor2_des);
  }
   
  if(aux_int2 == 1) {
    motor1_des = mock_motor1;
    motor2_des = mock_motor2;
  }
  else if(aux_int2 == 2) {
    motor1_des = motor_const1 - yaw_control + altitude_control;
    motor2_des = motor_const2 + yaw_control + altitude_control;
  }
   
  // servo1_des = roll_control;
  // servo2_des = pitch_control;
  
  if(aux_int1 == 1) {
    servo1_des = cont_xy[0] + roll_trim;
    servo2_des = cont_xy[1] + pitch_trim;
  }
  else if(aux_int1 == 2) {
    servo1_des = roll_trim;
    servo2_des = pitch_trim;
  } 
   
  

  // motor1_des = motor_const1 - yaw_control + altitude_control;
  // motor2_des = motor_const2 + yaw_control + altitude_control;
  
  // servo1_des = servo1_const + r_rc_coef * (rc_r+rc_trim_r) + roll_control;
  // servo1_des = servo1_des;
  // servo2_des = servo2_const - p_rc_coef * (rc_p+rc_trim_p) + pitch_control;
}

void CoaxSimpleControl::controlPublisher(size_t rate) {
  ros::Rate loop_rate(rate);
  
  double sum_Yaw_desired = 0;
  double sum_Gx_desired = 0; // global "Vicon" x
  double sum_Gy_desired = 0; // global "Vicon" y  
  
  // The continuous while loop. i.e., keeps calling rotorReady
  while(ros::ok()) { 

    if ((init_count<100)) {
      sum_Yaw_desired = 0;
      sum_Gx_desired = sum_Gx_desired + coax_global_x;
      sum_Gy_desired = sum_Gy_desired + coax_global_y;
      init_count ++;
    }
    else if (!INIT_DESIRE) {
      yaw_des = sum_Yaw_desired / 100;
      Gx_des = sum_Gx_desired / 100;
      Gy_des = sum_Gy_desired / 100;
      
      ROS_INFO("Initiated Desired Yaw %f",yaw_des);
      ROS_INFO("Current Battery Voltage %f",battery_voltage);
      sum_Yaw_desired = 0;
      sum_Gx_desired = 0;
      sum_Gy_desired = 0;
      INIT_DESIRE = true;
      ROS_INFO("INIT_DESIRE = true");
    }
    
    if (INIT_DESIRE && rotorReady()) {
      stabilizationControl();
    }
    else {
      if (mil_count == 500 || mil_count == 1000 || mil_count == 1500 || mil_count == 2000 ) {
	// ROS_INFO("NO Stab Control");
        
      }
    }
   

    mil_count++;
    if (mil_count >= 2000) {
      mil_count = 0;
    }
    if (mil_count == 1000) {
      
      ROS_INFO("mil_count: %i ", mil_count);
      ROS_INFO("coax_global_x %f. from tf", coax_global_x);
      ROS_INFO("Publishing Raw Control");
      ROS_INFO("motor 2 commanded: %f ", motor2_des);
      servo1_des = servo1_des*-1;
      
    };

    setRawControl(motor1_des,motor2_des,servo1_des,servo2_des);
    // setRawControl(0,0,servo1_des,0);
    ros::spinOnce();
    loop_rate.sleep();
  }
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "CoaxSimpleControl");
  ros::NodeHandle nh("~");
  
  CoaxSimpleControl control(nh);

  // make sure coax_server has enough time to start
  ros::Duration(1.5).sleep();


  control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_IMU_ALL | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL);
  // control.configureComm(100, SBS_MODES | SBS_BATTERY | SBS_GYRO | SBS_ACCEL | SBS_CHANNELS | SBS_RPY | SBS_ALTITUDE_ALL);
  // control.setTimeout(500, 5000);

  control.configureControl(SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL, SB_CTRL_MANUAL);
  control.setTimeout(500, 5000);

  ROS_INFO("Initially Setup comm and control");

  int frequency = 100;
  control.controlPublisher(frequency);

  ros::spin();
  return 0;
}
