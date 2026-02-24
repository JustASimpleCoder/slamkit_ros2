/*
 *  SLAMKIT ROS NODE
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2016 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
 /*
 *  
 *  Modified by JustASimpleCoder february 23 2026
 *
 * 
 */

#include "slamkit_ros2/node.hpp"

ImuPub::ImuPub() : Node(IMU_PUB_NODE_NAME),
    imu_pub_timer_{},
    imu_msg_{},
    imu_processed_msg_{},
    mag_msg_{}
{
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(TOPIC_IMU_RAW, 100);
    imu_processed_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(TOPIC_IMU_FILTERED, 100);
    mag_pub_ = this->create_publisher<sensor_msgs::msg::MagneticField>(TOPIC_MAG_RAW, 100);


    this->declare_parameters<std::string>("", {
        {CHANNEL_TYPE_LITERAL, "usb"},
        {FRAME_ID_LITERAL,     "imu"}
    });

    this->declare_parameters<int>("", {
        {VENDOR_ID_LITERAL,    64719},
        {PRODUCT_ID_LITERAL,   61696},
        {INTERFACE_ID_LITERAL, 3},
        {TX_ENDPOINT_LITERAL,  5},
        {RX_ENDPOINT_LITERAL,  5}
    });
}

void ImuPub::imu_publish(const sl_imu_raw_data_t & imu_data, const std::string & frame_id)
{
    if (last_ts_ms == imu_data.timestamp)
    {
        return;
    }

    // according to datasheet,sensitivity scale factor is 16384, +-2g
    double acc_x = imu_data.acc_x / SHIFT_15_BITS * 2 * 9.8;
    double acc_y = imu_data.acc_y / SHIFT_15_BITS * 2 * 9.8;
    double acc_z = imu_data.acc_z / SHIFT_15_BITS * 2 * 9.8;
    
    double gyro_x = imu_data.gyro_x / SHIFT_15_BITS * 2000 / 180 * 3.1415926;
    double gyro_y = imu_data.gyro_y / SHIFT_15_BITS * 2000 / 180 * 3.1415926;
    double gyro_z = imu_data.gyro_z / SHIFT_15_BITS * 2000 / 180 * 3.1415926;

    double mag_x = imu_data.mag_x * 4900 / SHIFT_15_BITS / 1000000;
    double mag_y = imu_data.mag_y * 4900 / SHIFT_15_BITS / 1000000;
    double mag_z = imu_data.mag_z * 4900 / SHIFT_15_BITS / 1000000;
    
    imu_msg_.header.stamp =  this->get_clock()->now();  // ros::Time::now();
    imu_msg_.header.frame_id = frame_id;
    imu_msg_.linear_acceleration.x =  acc_x;
    imu_msg_.linear_acceleration.y =  acc_y;
    imu_msg_.linear_acceleration.z =  acc_z;

    imu_msg_.angular_velocity.x = gyro_x;
    imu_msg_.angular_velocity.y = gyro_y;
    imu_msg_.angular_velocity.z = gyro_z;

    imu_pub_->publish(imu_msg_);
    
    mag_msg_.header.stamp = this->get_clock()->now();  // ros::Time::now();
    mag_msg_.header.frame_id = "magnetic";
    mag_msg_.magnetic_field .x =  mag_x;
    mag_msg_.magnetic_field .y =  mag_y;
    mag_msg_.magnetic_field .z =  mag_z;

    mag_pub_->publish(mag_msg_);

    last_ts_ms = imu_data.timestamp;
}

// // ------------------------- Part Raw IMU -----------------
void ImuPub::imu_processed_publish(const sl_slamkit_read_imu_processed_response_t & PImu_resp)
{
    if (processed_last_ts_ms == PImu_resp.timestamp)
    {
        return;
    }
   
    double acc_x = PImu_resp.acc.x_d4/10000.0;
    double acc_y = PImu_resp.acc.y_d4/10000.0;
    double acc_z = PImu_resp.acc.z_d4/10000.0;

    double gyro_x = (PImu_resp.gyro.wx_d4/10000.0) * DEGREE_TO_RAD;
    double gyro_y = (PImu_resp.gyro.wy_d4/10000.0) * DEGREE_TO_RAD;
    double gyro_z = (PImu_resp.gyro.wz_d4/10000.0) * DEGREE_TO_RAD;

    double gyro_sum_x = (std::int32_t)PImu_resp.gyro.sum_x_d4/10000.0;
    double gyro_sum_y = (std::int32_t)PImu_resp.gyro.sum_y_d4/10000.0;
    double gyro_sum_z = (std::int32_t)PImu_resp.gyro.sum_z_d4/10000.0;

    imu_processed_msg_.header.stamp = this->get_clock()->now();
    imu_processed_msg_.header.frame_id = "imu_processed";

    imu_processed_msg_.vector.x =  0;
    imu_processed_msg_.vector.y =  0;
    imu_processed_msg_.vector.z =  gyro_sum_z * RAD_TO_DEGREE;

    imu_processed_pub_->publish(imu_processed_msg_);
    processed_last_ts_ms = PImu_resp.timestamp;
}


// //***************************************** Main Function ****************************************************8
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);   
    std::shared_ptr<ImuPub> main_node = std::make_shared<ImuPub>();

    // echo slamkit version info 
    int ver_major = SL_SLAMKIT_SDK_VERSION_MAJOR;
    int ver_minor = SL_SLAMKIT_SDK_VERSION_MINOR;
    int ver_patch = SL_SLAMKIT_SDK_VERSION_PATCH;   
    RCLCPP_INFO(
        rclcpp::get_logger("main_node"),
        "slamkit running on ROS package slamkit_ros, SDK Version:%d.%d.%d",ver_major,ver_minor,ver_patch
    );

    sl_result  op_result;

    std::shared_ptr<ISlamkitDriver> slamkit_drv = createSlamkitDriver();

    const std::string frame_id = main_node->get_parameter(FRAME_ID_LITERAL).as_string(); 
    const std::string channel_type = main_node->get_parameter(CHANNEL_TYPE_LITERAL).as_string(); 

    // usb communication
    if (channel_type == "usb")
    {
        // SLAMKIT usb channel connect
        auto _channel = createUSBChannel(
            static_cast<std::uint16_t>(
                main_node->get_parameter(VENDOR_ID_LITERAL).as_int()),
            static_cast<std::uint16_t>(
                main_node->get_parameter(PRODUCT_ID_LITERAL).as_int()),
            static_cast<std::uint16_t>(
                main_node->get_parameter(INTERFACE_ID_LITERAL).as_int()),
            static_cast<std::uint16_t>(
                main_node->get_parameter(TX_ENDPOINT_LITERAL).as_int()),
            static_cast<std::uint16_t>
            (main_node->get_parameter(RX_ENDPOINT_LITERAL).as_int())
        );

        if (SL_IS_FAIL((slamkit_drv)->connect(_channel)))
        {
            
            RCLCPP_ERROR(rclcpp::get_logger("slamkit_node"), "Error, cannot connect to slamkit.");
            return -1;
        }
        RCLCPP_INFO(rclcpp::get_logger("slamkit_node"), "slamkit deviece open  ok");
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("slamkit_node"), 
            "Error, channel not support yet, please use usb channel."
        );
        return -1;
    }

    // define parameters
    sl_imu_raw_data_t imu_data;

    sl_slamkit_read_imu_processed_request_t req;
    sl_slamkit_read_imu_processed_response_t processed_data;

    req.motion_hint_bitmap = SLAMKIT_REQUEST_MOTION_HINT_BITMAP_MOTION_BIT;
    // main loop
    rclcpp::Rate rate(460);  // loop rate
    while (rclcpp::ok())
    {
        // 3. Publish IMU Raw topic
        op_result = slamkit_drv->getImuRawData(imu_data);
        if (SL_IS_FAIL(op_result))
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamkit_node") , "can not get Imu Raw Data.\n");
        }
        main_node->imu_publish(imu_data, frame_id);
        //publish_mag(&mag_pub, imu_data);

        op_result = slamkit_drv->set_motion_hit_and_get_imu_processed(req, processed_data);
        if (SL_IS_FAIL(op_result))
        {
            RCLCPP_ERROR(rclcpp::get_logger("slamkit_node") , "can not get Imu processed Data.\n");
        }
        main_node->imu_processed_publish(processed_data);
        rclcpp::spin_some(main_node); // rclcpp::spinOnce(main_node);  
        rate.sleep();
    }

    slamkit_drv->disconnect();
    return 0;
}

