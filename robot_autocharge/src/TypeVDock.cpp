#include "opencv2/opencv.hpp"
//#include "cvtools/cv_plot.h"
#include <ros/ros.h>
#include "../include/robot_autocharge/typevdock.h"
Docking::Docking(void)
{
}

Docking::~Docking(void)
{
}

/// judge stop
bool Docking::Judge_stop()
{
	m_task_status = -1;		// stopped
	return false;
}

/// set V data
void Docking::Set_Vdata(double l1, double ang1, double dis, double outdis)
{
	m_line_rechage = l1;	m_angle_recharge = ang1;	m_lastdis = dis;	m_outdis = outdis;

	tan_min_recharge = tan((m_angle_recharge - MAX_ANGLE_ERROR)*M_PI / 180.0);
	tan_max_recharge = tan((m_angle_recharge + MAX_ANGLE_ERROR)*M_PI / 180.0);
    
	odm_record_flag = false;
	has_typev_flag = false;
	
	ROS_INFO("Docking::Set V Data Over.tan_min_recharge:%.4f, tan_max_recharge:%.4f, odm_record_flag:%d, has_typev_flag:%d",
		tan_min_recharge,tan_min_recharge, odm_record_flag, has_typev_flag);
}

/// handle laser data
void Docking::Set_Laser(PointList laser_data)
{
	boost::mutex::scoped_lock l(mut);

	has_typev_flag = false;
	double include_angle_k;
	m_vdetec.setLaserData(laser_data);
	m_vdetec.split_and_merge();
	m_line = m_vdetec.get_line();
	ROS_INFO("Docking::Set_Laser.laser_data size:%d, line number:%d ",laser_data.size(), m_line.size() );

	for (int i = 0; !has_typev_flag && m_line.size() > 1 &&  i < m_line.size() - 1; ++i)
	{
		double robot2vx, robot2vy;
		if (1.0 != m_line[i].line_k*m_line[i + 1].line_k)
			include_angle_k = (m_line[i + 1].line_k - m_line[i].line_k) / (1.0 - m_line[i].line_k*m_line[i + 1].line_k);
		if (fabs(m_line[i].line_length - m_line_rechage) < MIN_LENGTH_ERROR && 
			fabs(m_line[i + 1].line_length - m_line_rechage) < MIN_LENGTH_ERROR &&
			include_angle_k > tan_min_recharge && include_angle_k < tan_max_recharge )
		{
			/// Todo: get the point
			double a1 = (m_line[i].lbegin_x - m_line[i].lend_x) / (m_line[i].lbegin_y - m_line[i].lend_y);
			double b1 = m_line[i].lbegin_x - a1 * m_line[i].lbegin_y;
			double a2 = (m_line[i+1].lbegin_x - m_line[i+1].lend_x) / (m_line[i+1].lbegin_y - m_line[i+1].lend_y);
			double b2 = m_line[i + 1].lbegin_x - a2 * m_line[i + 1].lbegin_y;
			robot2vy = (b1 - b2) / (a2 - a1);
			robot2vx = a1 * robot2vy + b1;

			//////////////////////////////////////////////////////////////////////////
			if (pre_robot2v.x != 1.5)
			{
				if (fabs(robot2vx - pre_robot2v.x) > 0.07)		robot2vx = pre_robot2v.x;
				if (fabs(robot2vy - pre_robot2v.y) > 0.07)		robot2vy = pre_robot2v.y;
			}

			/// todo some 0 judge
			/// tanα=(k-k1)/(1+kk1)=(k2-k)/(1+kk2)
			double k1 = MAXK;
			double a11 = atan((m_line[i].line_k - k1)/(1 + m_line[i].line_k*k1));
			a11 = M_PI / 2.0 - a11;
		//	double jia = atan(include_angle_k);
		//	if (jia < 0)	jia = M_PI + jia;
			double jia = atan(m_line[i].line_k) - atan(m_line[i + 1].line_k);
			HX = 1.5 * M_PI - jia / 2.0 - a11;

			m_target.y = robot2vx * cos(HX) + robot2vy * sin(HX);
			m_target.x = robot2vx * sin(HX) - robot2vy * cos(HX);

			allx.push_back(m_target.x);
			if (allx.size() > 1)
				m_target.x = fabs(allx[allx.size() - 1]) < fabs(allx[allx.size() - 2]) ? fabs(allx[allx.size() - 2]) : fabs(allx[allx.size() - 1]);
			/// a < b ? b : a

			ally.push_back(m_target.y);
			if (ally.size() > 2)
				m_target.y = (ally[ally.size() - 1] + ally[ally.size() - 2] + ally[ally.size() - 3]) / 3.0;

			allhx.push_back(HX);
			if (allhx.size() > 2)
				HX = (allhx[allhx.size() - 1] + allhx[allhx.size() - 2] + allhx[allhx.size() - 3]) / 3.0;
            
	//		LOG_COUT_INFO(m_vdock_logger, "robot2v: " << robot2vx << "," << robot2vy << "," << HX / M_PI*180.0);
	//		LOG_COUT_INFO(m_vdock_logger, "target: " << m_target.x << "," << m_target.y << " HX: " << HX / M_PI*180.0);

			m_forecast_target.x = 0.0;		m_forecast_target.y = 0.0;		pre_targetx = m_target.x;		pre_targety = m_target.y;		preHX = HX;
			pre_robot2v.x = robot2vx;	pre_robot2v.y = robot2vy;
			has_typev_flag = true;

			if (m_task_status == 110)
			{
				set_task_status(m_reme_status);
	//			LOG_COUT_INFO(m_vdock_logger, "Find tye v_dock again, reset task_status to " << m_reme_status);
			}
		}
		else
		{
			double forecast_t[2];
			forecast_t[0] = pre_targetx - m_forecast_target.x;	forecast_t[1] = pre_targety + m_forecast_target.y;

		//	LOG_COUT_INFO(m_vdock_logger, "forecast: " << pre_targetx << "-" << m_forecast_target.x << " , " << pre_targety << "+" << m_forecast_target.y);

			m_target.x = forecast_t[0];		m_target.y = forecast_t[1];		HX = preHX;
		}
	//	LOG_COUT_INFO(m_vdock_logger, "TTTTT: " << m_target.x << "," << m_target.y << " and pre: " << pre_targetx << "," << pre_targety);
	}

}

/// handle task order
void Docking::Set_Task(int motion_task)
{
	boost::mutex::scoped_lock l(mut);
	odm_record_flag = false;
	has_typev_flag = false;
	last_go_ready_flag = false;
	
	pre_robot2v.x = 1.5;
	pre_robot2v.y = 0.0;
	m_target.x = 1.5;
	m_target.y = 0.0;
	
	m_forecast_target.x = 0.0;	m_forecast_target.y = 0.0;
	pre_targetx = 1.5;	pre_targety = 0.0;	preHX = M_PI / 2.0;
	m_task_flag = motion_task;
	m_task_status = 2;
	allx.clear();
	ally.clear();
	allhx.clear();
	ROS_INFO("Docking::Set_Task.Set Task Over");
}

/// set task status
int Docking::get_status()
{
	return m_task_status;
}

/// handle output speed
geometry_msgs::Twist Docking::get_speed()
{
	  geometry_msgs::Twist cmd_vel;
	
	  cmd_vel.linear.x = m_speedv;
	  cmd_vel.linear.y = 0;
    cmd_vel.linear.z = 0.0;

    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
	  cmd_vel.angular.z = m_speedw;

	return cmd_vel;
}

/// get target point
Point Docking::get_target()
{
	//if (m_task_flag == 0)
	//{
	//	m_target.x = 1.5;
	//	m_target.y = 0.0;
	//}
	//if (fabs(m_target.x) > 2.0)	m_target.x = 2.0;
	//if (fabs(m_target.y) > 1.0) m_target.y = m_target.y / fabs(m_target.y);

	return pre_robot2v;
}

/// set sum_dist of odm
void Docking::Set_Odm(nav_msgs::Odometry odm_data)
{
	m_cur_odom.x = odm_data.twist.twist.linear.x;		
	m_cur_odom.y = odm_data.twist.twist.linear.x;		
	m_cur_odom.theta = odm_data.twist.twist.angular.z;
}

/// record odm
OrientedPoint Docking::Record_odm(bool switch_odm_record)
{
	if (switch_odm_record)
	{
		m_delta_odm = absoluteDifference(m_cur_odom, m_pre_odom);
		m_leave_odm = absoluteSum(m_leave_odm, m_delta_odm);
		m_pre_odom = m_cur_odom;
		return m_leave_odm;
	}
	else
		return OrientedPoint(0,0,0);
}

/// set speed
void Docking::set_speed(double _v, double _w)
{
	m_speedv = _v;	m_speedw = _w;
}
/// set task status
void Docking::set_task_status(int s)
{
	m_task_status = s;
}

void Docking::Run_Navigation()
{
	double _speedv(0.0);
	double _speedw(0.0);

	//LOG_COUT_INFO(m_vdock_logger, "This is Navigation for Roller");

	// chq m_task_status = 0 意味着exit navi loop
	if (m_task_status != 0 )
	{
		//if (!has_typev_flag && m_task_status != 2 && m_task_status != 3)
		//{
		//	if (m_task_status != 110)
		//		m_reme_status = m_task_status;
		//	set_task_status(110);
		//	LOG_COUT_INFO(m_vdock_logger, "Can not find type v_dock set task_status 110, and remember now task_status " << m_reme_status);
		//}
		// chq 进入充电位 任务 
		if (m_task_flag == 1)
		{
			/// todo some filter
			//chq =2 新任务开始状态
			if (m_task_status == 2)// chq when we get a new task, the m_task_status equal to 2
			{
				ROS_INFO("Docking::Run_Navigation.docking target x:%f, y:%f, HX:%f ",m_target.x, m_target.y, HX / M_PI*180.0);
                //chq 如果发现充电桩
				if (has_typev_flag)
				{   
				    //chq 若发生y方向偏差过大现象，失败
					if (fabs(m_target.y) > 0.7)
					{
						m_task_flag = 0;
						ROS_ERROR("Docking::Run_Navigation.has_typev_flag = true.but m_target.y:%f, ERROR TOO LARGE！！！", m_target.y);
						set_task_status(9999);
					}
					//chq 否则，固定线速度，角速度通过实时的偏差Hx与target.y计算
					else
					{
						_speedv = 0.05;
						_speedw = HX - M_PI / 2.0 + m_target.y * 2.0;
						//	_speedw = 1.0 * m_target.y;
						//	_speedv = 0;
						//	_speedw = 0;

                        //chq 如果充电模式下，x方向偏差小于0.8m 进入修正调整角度模式
						if (m_target.x < 0.8)
						{
							odm_record_flag = false;
							set_task_status(35);
							ROS_ERROR("Docking::Run_Navigation.has_typev_flag = true.m_target.x :%f < 0.8 Enter Modified HX Status", m_target.x );
						}
					}
					odm_record_flag = false;
				}
                //chq 如果未发现充电桩
				else
				{
				    //chq 重置里程计计数，重新计算里程计累积量
					if (!odm_record_flag)
					{
						m_pre_odom = m_cur_odom;
						m_leave_odm.x = 0.0;	
						m_leave_odm.y = 0.0; 
						m_leave_odm.theta = 0.0;
						odm_record_flag = true;
					}
					double record_dis = Record_odm(odm_record_flag).mod();
					//chq 如果累计调整超过0.2m 还未找到charge,stop
					if (record_dis > 0.2)	/// if has no typevdock above 20cm  stop it
					{
						_speedv = 0.0;
						_speedw = 0.0;
						set_task_status(999);
						ROS_ERROR("Docking::Run_Navigation.has_typev_flag = false.record_dis %f > 0.2.Long Dis Can Not Find Type V Dock", record_dis);
					}
					//chq 否则，恒速前进，计算危险距离
					else
					{
						m_forecast_target.y = Record_odm(odm_record_flag).x * cos(HX) + Record_odm(odm_record_flag).y * sin(HX);
						m_forecast_target.x = Record_odm(odm_record_flag).x * sin(HX) - Record_odm(odm_record_flag).y * cos(HX);
						_speedv = 0.05;
						_speedw = 0.0;
						ROS_ERROR("Docking::Run_Navigation.record_dis: %f, m_forecast_target.x:%f, m_forecast_target.y:%f.Danger~ ", record_dis, m_forecast_target.x, m_forecast_target.y);
					}
					//chq 如果x方向偏差<0.8，进入修正模式(by blind)
					if (m_target.x < 0.8)
					{
						odm_record_flag = false;
						set_task_status(35);
						ROS_INFO("Docking::Run_Navigation.Enter Modified HX Status BY Blind Moving...");
					}
				}
			}
			//chq x方向偏差已较小，进入角度修正模式
			if (m_task_status == 35) // modified
			{
				ROS_INFO("Docking::Run_Navigation.m_task_status == 35.target x:%f, y:%f, Hx:%f ", m_target.x, m_target.y, HX / M_PI*180.0);
				_speedv = 0.0;
				_speedw = HX - M_PI / 2.0;
				if (fabs(HX/M_PI*180.0-90.0) < 0.5)//若距离较小，且角度偏差已经很小，角速度设为0
				{
					_speedw = 0.0;
					set_task_status(3);
					ROS_INFO("Docking::Run_Navigation.Enter Last_Dist Status");
				}
			}
			//chq x距离偏差已较小且角度调整到位,进入 直行到充电桩 模式
			if (m_task_status == 3)	// last dis
			{
			    //chq 如果y方向发生偏差较大情况，失败
				if (!last_go_ready_flag && fabs(m_target.y) > 0.05)
				{
					m_task_flag = 0;
					ROS_ERROR("Docking::Run_Navigation.m_task_status == 3.LAST Y ERROR:%f > 0.05, IS TOO LARGE",fabs(m_target.y));
					set_task_status(999);
				}
				last_go_ready_flag = true;
                //chq 重置并累计计算里程计距离值
				if (!odm_record_flag)
				{
					m_pre_odom = m_cur_odom;
					m_leave_odm.x = 0.0;	m_leave_odm.y = 0.0; m_leave_odm.theta = 0.0;
					odm_record_flag = true;
				}
				double record_dis = Record_odm(odm_record_flag).mod();
				ROS_INFO("Docking::Run_Navigation.Record odm dist:%f ", record_dis);
				//chq 如果小于设定的限制行驶距离，固定速度前进
				if (record_dis < m_lastdis)
				{
					_speedv = 0.03;
					_speedw = 0.0;
				}
				//chq 否则，停止并置任务为完成状态
				else
				{
					m_task_flag = 0;
					odm_record_flag = false;
					_speedv = 0.0;
					_speedw = 0.0;
					set_task_status(4);	// in finish
					ROS_INFO("Docking::Run_Navigation.Charge In Success!");
				}
			}
		}
		// chq 离开充电位 任务
		if (m_task_flag == 2) 
		{   //chq 重置并累计记录里程计距离值
			if (!odm_record_flag)
			{
				m_pre_odom = m_cur_odom;
				m_leave_odm.x = 0.0;	m_leave_odm.y = 0.0;	m_leave_odm.theta = 0.0;
				odm_record_flag = true;
			}
			//chq 小于限定行驶距离值，恒速倒退
			if (Record_odm(odm_record_flag).mod() < m_outdis)
			{
				_speedv = -0.1;
				_speedw = -0.0;
			}
			//chq 否则，停止，置为退出成功状态
			else
			{
				m_task_flag = 0;
				odm_record_flag = false;
				_speedv = 0.0;
				_speedw = 0.0;
				set_task_status(5);	// out finish
				ROS_INFO("Docking::Run_Navigation.Charge Out Success!");
			}
		}
	}

//	if (Judge_stop())
//	{
//		set_speed(0.0, 0.0);
//		LOG_COUT_INFO(m_vdock_logger, "Laser Stop");
//	}
//	else
//	{
//		set_speed(_speedv, _speedw);
//	}
    //chq 下发速度给控制器
	set_speed(_speedv, _speedw);
	ros::Duration(0.03).sleep();
}

