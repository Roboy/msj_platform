/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: msj platform main
*/

#pragma once

#define IORD(base, reg) (*(((volatile int32_t*)base)+reg))
#define IOWR(base, reg, data) (*(((volatile int32_t*)base)+reg)=data)

#define MSJ_READ_Kp(base, motor) IORD(base, (uint32_t)(0x00<<8|motor&0xff) )
#define MSJ_READ_Ki(base, motor) IORD(base, (uint32_t)(0x01<<8|motor&0xff) )
#define MSJ_READ_Kd(base, motor) IORD(base, (uint32_t)(0x02<<8|motor&0xff) )
#define MSJ_READ_sp(base, motor) IORD(base, (uint32_t)(0x03<<8|motor&0xff) )
#define MSJ_READ_control_mode(base, motor) IORD(base, (uint32_t)(0x04<<8|motor&0xff) )
#define MSJ_READ_outputPosMax(base, motor) IORD(base, (uint32_t)(0x05<<8|motor&0xff) )
#define MSJ_READ_outputNegMax(base, motor) IORD(base, (uint32_t)(0x06<<8|motor&0xff) )
#define MSJ_READ_deadBand(base, motor) IORD(base, (uint32_t)(0x07<<9|motor&0xff) )
#define MSJ_READ_outputDivider(base, motor) IORD(base, (uint32_t)(0x08<<8|motor&0xff) )
#define MSJ_READ_sensor_angle(base, motor) IORD(base, (uint32_t)(0x09<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_absolute(base, motor) IORD(base, (uint32_t)(0x0A<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_offset(base, motor) IORD(base, (uint32_t)(0x0B<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_relative(base, motor) IORD(base, (uint32_t)(0x0C<<8|motor&0xff) )
#define MSJ_READ_sensor_angle_velocity(base, motor) IORD(base, (uint32_t)(0x0D<<8|motor&0xff) )
#define MSJ_READ_sensor_revolution_counter(base, motor) IORD(base, (uint32_t)(0x0E<<8|motor&0xff) )
#define MSJ_READ_dutys(base, motor) IORD(base, (uint32_t)(0x0F<<8|motor&0xff) )
#define MSJ_READ_zero_speed(base, motor) IORD(base, (uint32_t)(0x10<<8|motor&0xff) )

#define MSJ_WRITE_Kp(base, motor, data) IOWR(base, (uint32_t)(0x00<<8|motor&0xff), data )
#define MSJ_WRITE_Ki(base, motor, data) IOWR(base, (uint32_t)(0x01<<8|motor&0xff), data )
#define MSJ_WRITE_Kd(base, motor, data) IOWR(base, (uint32_t)(0x02<<8|motor&0xff), data )
#define MSJ_WRITE_sp(base, motor, data) IOWR(base, (uint32_t)(0x03<<8|motor&0xff), data )
#define MSJ_WRITE_control_mode(base, motor, data) IOWR(base, (uint32_t)(0x04<<8|motor&0xff), data )
#define MSJ_WRITE_reset_control(base, data) IOWR(base, (uint32_t)(0x05<<8|0), data )
#define MSJ_WRITE_outputDivider(base, motor, data) IOWR(base, (uint32_t)(0x06<<8|motor&0xff), data )
#define MSJ_WRITE_outputPosMax(base, motor, data) IOWR(base, (uint32_t)(0x07<<8|motor&0xff), data )
#define MSJ_WRITE_outputNegMax(base, motor, data) IOWR(base, (uint32_t)(0x08<<8|motor&0xff), data )
#define MSJ_WRITE_integralPosMax(base, motor, data) IOWR(base, (uint32_t)(0x09<<8|motor&0xff), data )
#define MSJ_WRITE_integralNegMax(base, motor, data) IOWR(base, (uint32_t)(0x0A<<8|motor&0xff), data )
#define MSJ_WRITE_deadBand(base, motor, data) IOWR(base, (uint32_t)(0x0B<<8|motor&0xff), data )
#define MSJ_WRITE_zero_speed(base, motor, data) IOWR(base, (uint32_t)(0x0C<<8|motor&0xff), data )
#define MSJ_WRITE_pwm_mute(base, data) IOWR(base, (uint32_t)(0x0D<<8|0), data )
#define MSJ_WRITE_sensor_update_freq(base, data) IOWR(base, (uint32_t)(0x0E<<8|0), data )

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <signal.h>
#include <stdlib.h>
#include "hwlib.h"
#include "socal/hps.h"
#include "msj_platform/hps_0.h"
#include <ros/ros.h>
#include <roboy_control_msgs/SetControllerParameters.h>
#include <roboy_middleware_msgs/DarkRoom.h>
#include <roboy_middleware_msgs/DarkRoomOOTX.h>
#include <roboy_middleware_msgs/DarkRoomStatus.h>
#include <roboy_middleware_msgs/MagneticSensor.h>
#include <roboy_middleware_msgs/MotorStatus.h>
#include <roboy_middleware_msgs/MotorCommand.h>
#include <std_msgs/Int32.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <thread>
#include <vector>
#include "msj_platform/tlv493d.hpp"
#include "msj_platform/half.hpp"
#include "msj_platform/CRC32.h"
#include <common_utilities/CommonDefinitions.h>

#define NUMBER_OF_MOTORS 8
#define NUM_SENSORS 13 // darkroom sensors
#define SPINDLE_RADIUS 0.0055
#define msjMeterPerEncoderTick(encoderTicks) (((encoderTicks)/4096.0*2.0*M_PI)*(2.0*M_PI*SPINDLE_RADIUS))

#pragma pack(1) // we need this, otherwise the ootx union will be padded and the checksum test fails

using namespace std;
using namespace chrono;
using half_float::half;

class MSJPlatform{
public:
    MSJPlatform(int32_t *msj_platform_base, int32_t *switch_base, vector<int32_t*> i2c_base, int32_t* darkroom_base, int32_t* darkroom_base_ootx);
    ~MSJPlatform();
    void publishStatus();
    void publishMagneticSensors();
    void PIDController();
    bool setControllerParameters( roboy_control_msgs::SetControllerParameters::Request &req,
                                               roboy_control_msgs::SetControllerParameters::Response &res);
    void MotorCommand(const roboy_middleware_msgs::MotorCommandConstPtr &msg);
    bool EmergencyStop(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
    bool Zero(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
/**
     * Publishes lighthouse sensor values
     */
    void darkRoomPublisher();

    /**
     * Publishes decoded lighthouse ootx data
     */
    void darkRoomOOTXPublisher();
private:
    /**
     * Reverses the bit order of a byte, eg. 0b00001011 -> 0b11010000
     * @param b input byte
     * @return reversed byte
     */
    inline uint8_t reverse(uint8_t b) {
        b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
        b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
        b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
        return b;
    }

    /**
     * Reverses the bit order of each byte in a 4 byte uint32_t
     * @param b input
     * @return reversed uint32_t
     */
    inline uint32_t reverse(uint32_t b) {
        uint32_t a = (uint32_t) ((uint8_t) reverse((uint8_t) (b >> 24 & 0xff)) << 24 |
                                 (uint8_t) reverse((uint8_t) (b >> 16 & 0xff)) << 16 |
                                 (uint8_t) reverse((uint8_t) (b >> 8 & 0xff)) << 8 |
                                 (uint8_t) reverse((uint8_t) (b & 0xff)));
        return a;
    }
    ros::NodeHandlePtr nh;
    ros::Publisher motor_status, magnetic_sensor, darkroom_pub, darkroom_ootx_pub, darkroom_status_pub;
    ros::Subscriber motor_command;
    ros::ServiceServer emergency_stop, zero;
    boost::shared_ptr<ros::AsyncSpinner> spinner;
    int32_t *msj_platform_base, *switch_base, *darkroom_base, *darkroom_ootx_base;
    boost::shared_ptr<std::thread> status_thread, magnetic_thread, pid_control_thread, darkroom_thread, darkroom_ootx_thread;
    vector<int32_t> zero_speed = {333,333,330,328,330,330,330,330};
    vector<boost::shared_ptr<TLV493D>> tlv;
    vector<int32_t*> i2c_base;
    int control_mode[NUMBER_OF_MOTORS] = {2};
    int sp[NUMBER_OF_MOTORS] = {0};
    float Kp = 0.01, Kd = 0.01, Ki = 0.0001, integralMax = 3, integral[NUMBER_OF_MOTORS] = {0};
    union {
        uint8_t data[33];
        struct {
            uint16_t fw_version;        // 0x00
            uint32_t ID;                // 0x02
            uint16_t fcal_0_phase;      // 0x06
            uint16_t fcal_1_phase;      // 0x08
            uint16_t fcal_0_tilt;       // 0x0A
            uint16_t fcal_1_tilt;       // 0x0C
            uint8_t unlock_count;       // 0x0E
            uint8_t hw_version;         // 0x0F
            uint16_t fcal_0_curve;      // 0x10
            uint16_t fcal_1_curve;      // 0x12
            int8_t accel_dir_x;         // 0x14
            int8_t accel_dir_y;         // 0x15
            int8_t accel_dir_z;         // 0x16
            uint16_t fcal_0_gibphase;   // 0x17
            uint16_t fcal_1_gibphase;   // 0x19
            uint16_t fcal_0_gibmag;     // 0x1B
            uint16_t fcal_1_gibmag;     // 0x1D
            uint8_t mode;               // 0x1F
            uint8_t faults;             // 0x20
        } frame;
    } ootx;

    uint32_t ootx_sensor_channel = 0;
    string ethaddr = "top";
};