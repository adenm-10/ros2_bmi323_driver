#include <chrono>
#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

extern "C" {
#include "bmi323.h"
#include "common.h"
}

using namespace std::chrono_literals;

class BMI323Node : public rclcpp::Node
{
public:
    BMI323Node() : Node("bmi323_node")
    {
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
        timer_ = this->create_wall_timer(20ms, std::bind(&BMI323Node::timer_callback, this));
        
        init_sensor();
    }

private:
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    struct bmi3_dev dev_;
    float acc_range_ = 2.0f;  // ±2G
    float gyr_range_ = 2000.0f;  // ±2000 DPS
    uint8_t bit_width_;

    static int8_t set_accel_gyro_config(struct bmi3_dev *dev)
    {
        /* Status of API are returned to this variable. */
        int8_t rslt;

        /* Structure to define accelerometer configuration. */
        struct bmi3_sens_config config[2];

        /* Structure to map interrupt */
        // struct bmi3_map_int map_int = { 0 };
        struct bmi3_map_int map_int = { };

        /* Configure the type of feature. */
        config[0].type = BMI323_ACCEL;
        config[1].type = BMI323_GYRO;

        /* Get default configurations for the type of feature selected. */
        rslt = bmi323_get_sensor_config(config, 2, dev);
        bmi3_error_codes_print_result("bmi323_get_sensor_config", rslt);

        if (rslt == BMI323_OK)
        {
            map_int.acc_drdy_int = BMI3_INT1;
            map_int.gyr_drdy_int = BMI3_INT1;
            map_int.temp_drdy_int = BMI3_INT1;

            /* Map data ready interrupt to interrupt pin. */
            rslt = bmi323_map_interrupt(map_int, dev);
            bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);

            if (rslt == BMI323_OK)
            {
                /* NOTE: The user can change the following configuration parameters according to their requirement. */
                /* Output Data Rate. By default ODR is set as 100Hz for accel. */
                config[0].cfg.acc.odr = BMI3_ACC_ODR_50HZ;

                /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
                config[0].cfg.acc.range = BMI3_ACC_RANGE_2G;

                /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
                config[0].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

                /* Set number of average samples for accel. */
                config[0].cfg.acc.avg_num = BMI3_ACC_AVG64;

                /* Enable the accel mode where averaging of samples
                * will be done based on above set bandwidth and ODR.
                * Note : By default accel is disabled. The accel will get enable by selecting the mode.
                */
                config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

                /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
                config[1].cfg.gyr.odr = BMI3_GYR_ODR_50HZ;

                /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
                config[1].cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

                /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
                *  Value   Name      Description
                *    0   odr_half   BW = gyr_odr/2
                *    1  odr_quarter BW = gyr_odr/4
                */
                config[1].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

                /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
                config[1].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

                /* Value    Name    Description
                *  0b000     avg_1   No averaging; pass sample without filtering
                *  0b001     avg_2   Averaging of 2 samples
                *  0b010     avg_4   Averaging of 4 samples
                *  0b011     avg_8   Averaging of 8 samples
                *  0b100     avg_16  Averaging of 16 samples
                *  0b101     avg_32  Averaging of 32 samples
                *  0b110     avg_64  Averaging of 64 samples
                */
                config[1].cfg.gyr.avg_num = BMI3_GYR_AVG1;

                /* Set the accel and gyro configurations. */
                rslt = bmi323_set_sensor_config(config, 2, dev);
                bmi3_error_codes_print_result("bmi323_set_sensor_config", rslt);
            }
        }

        return rslt;
    }

    void init_sensor()
    {
        int8_t rslt = bmi3_interface_init(&dev_, BMI3_I2C_INTF);
        bmi3_error_codes_print_result("bmi3_interface_init", rslt);

        rslt = bmi323_init(&dev_);
        bmi3_error_codes_print_result("bmi323_init", rslt);
        
        bit_width_ = dev_.resolution;

        if (rslt == BMI323_OK)
        {
            rslt = set_accel_gyro_config(&dev_);
            bmi3_error_codes_print_result("set_accel_gyro_config", rslt);
        }
    }

    float lsb_to_g(int16_t val) {
        float half_scale = std::pow(2.0f, bit_width_) / 2.0f;
        return (val * acc_range_) / half_scale;
    }

    float lsb_to_dps(int16_t val) {
        float half_scale = std::pow(2.0f, bit_width_) / 2.0f;
        return (gyr_range_ / half_scale) * val;
    }

    void timer_callback()
    {
        uint16_t int_status;
        // struct bmi3_sensor_data sensor_data[3] = { 0 };
        struct bmi3_sensor_data sensor_data[3] = { };
        sensor_data[0].type = BMI323_ACCEL;
        sensor_data[1].type = BMI323_GYRO;
        sensor_data[2].type = BMI323_TEMP;

        if (bmi323_get_int1_status(&int_status, &dev_) == BMI323_OK &&
            (int_status & BMI3_INT_STATUS_ACC_DRDY) &&
            (int_status & BMI3_INT_STATUS_GYR_DRDY))
        {
            if (bmi323_get_sensor_data(sensor_data, 3, &dev_) == BMI323_OK)
            {
                auto msg = sensor_msgs::msg::Imu();

                msg.header.stamp = this->get_clock()->now();
                msg.header.frame_id = "imu_link";

                msg.linear_acceleration.x = lsb_to_g(sensor_data[0].sens_data.acc.x) * 9.80665;
                msg.linear_acceleration.y = lsb_to_g(sensor_data[0].sens_data.acc.y) * 9.80665;
                msg.linear_acceleration.z = lsb_to_g(sensor_data[0].sens_data.acc.z) * 9.80665;

                msg.angular_velocity.x = lsb_to_dps(sensor_data[1].sens_data.gyr.x) * M_PI / 180.0;
                msg.angular_velocity.y = lsb_to_dps(sensor_data[1].sens_data.gyr.y) * M_PI / 180.0;
                msg.angular_velocity.z = lsb_to_dps(sensor_data[1].sens_data.gyr.z) * M_PI / 180.0;

                // No orientation estimate
                msg.orientation_covariance[0] = -1;

                imu_pub_->publish(msg);
            }
        }
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BMI323Node>());
    rclcpp::shutdown();
    return 0;
}
