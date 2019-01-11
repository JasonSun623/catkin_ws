/*******************************************************************************
* Copyright (c) 2018,  Data Storage
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of the copyright holder nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/


#include <std_msgs/String.h>

#include <boost/asio.hpp>
#include <boost/array.hpp>
#include <string>
#include <sensor_msgs/Imu.h>
#include "imu_publisher/JY901.h"
#include <imu_publisher/imu_transport.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

namespace imu_publisher_space
{
class imu_publisher : public Transport
{
 public:
  uint16_t rpms; ///< @brief RPMS derived from the rpm bytes in an LFCD packet
	/**
    * @brief Constructs a new imu_publisher attached to the given serial port
	* @param port The string for the serial port device to attempt to connect to, e.g. "/dev/ttyUSB0"
	* @param baud_rate The baud rate to open the serial port at.
	* @param io Boost ASIO IO Service to use when creating the serial port object
	*/
    imu_publisher(const std::string& port_name, uint32_t baud_rate, boost::asio::io_service& io);

	/**
	* @brief Default destructor
	*/
    ~imu_publisher()
    {
      free(buf);
      free(buf_bak);
    }

    bool init();

    Buffer readBuffer();

    void writeBuffer(Buffer& data);

	/**
    * @brief Blocks until a complete is received or close is called.
    * @param
	*/
    void poll(const ros::TimerEvent& event);

    uint8_t asc2hex(char asccode)
    {
        uint8_t ret;

        if('0'<=asccode && asccode<='9')
            ret = asccode-'0';
        else if('a'<=asccode && asccode<='f')
            ret = asccode-'a'+10;
        else if('A'<=asccode && asccode<='F')
            ret = asccode-'A'+10;
        else
            ret = 0;
        return ret;
    }

	/**
	* @brief Close the driver down and prevent the polling loop from advancing
	*/
    void close() { shutting_down_ = true; }

void enableTimer(bool ifstart){
 _start_timer = ifstart;
}
long long readOneBuffer();

 private:
  std::string port_name; ///< @brief The serial port the driver is attached to
	uint32_t baud_rate_; ///< @brief The baud rate for the serial connection
  int data_rate;//data rec rate
  uint8_t *buf;
  uint8_t *buf_bak;

	bool shutting_down_; ///< @brief Flag for whether the driver is supposed to be shutting down or not
    boost::asio::serial_port serial_; ///< @brief

  ros::NodeHandle priv_nh;
  ros::Publisher imu_pub ;
  std::string _imu_frame,_imu_topic;
	// for async read
	Buffer temp_read_buf_;
	Buffer poll_read_buf_;
  int readbuffersize;//每组数据１１byte,并且设置好数据一般是４类，为保证数据读取完整，这这里设置偏大一点
  int max_one_buffer_size;//最不好的情况一个read_buffer_有１０２４个字节，一般不可能这么大
  double orient_covar,angle_v_covar,acc_covar;//朝向　角速度　加速度协方差

  int max_array_size;
  boost::shared_ptr<sensor_msgs::Imu> imu_msg;

  ros::Timer timer;
  bool _start_timer;

  int count_;

	boost::thread thread_;
	// locks
	boost::mutex port_mutex_;
	boost::mutex write_mutex_;
	boost::mutex read_mutex_;
  CJY901 JY901;

	void mainRun();

	void start_a_read();
	void start_a_write();
	void readHandler(const boost::system::error_code &ec, size_t bytesTransferred);
	void writeHandler(const boost::system::error_code &ec);

};
}
