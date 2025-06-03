/**\
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/*********************Example description**************************************/

/* Accelerometers measure the change of linear motion by applying the sensing
 * principle of capacitive detection.
 */

/******************************************************************************/
/*!                 Header Files                                              */

#include <stdio.h>
#include <math.h>
#include "bmi330.h"
#include "common.h"
#include <string.h>

/******************************************************************************/
/*!                Macro definition                                           */

/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)

/******************************************************************************/
/*!           Static Function                                                 */

/*!
 *  @brief This internal API is used to set configurations for accel.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_accel_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define accelerometer configuration. */
    struct bmi3_sens_config config;

    /* Structure to map interrupt */
    struct bmi3_map_int map_int = { 0 };

    /* Configure the type of feature. */
    config.type = BMI330_ACCEL;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi330_get_sensor_config(&config, BMI3_N_SENSE_COUNT_1, dev);
    bmi3_error_codes_print_result("bmi330_get_sensor_config", rslt);

    if (rslt == BMI330_OK)
    {
        map_int.acc_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi330_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("bmi330_map_interrupt", rslt);

        if (rslt == BMI330_OK)
        {
            /* NOTE: The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for accel. */
            config.cfg.acc.odr = BMI3_ACC_ODR_100HZ;

            /* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
            config.cfg.acc.range = BMI3_ACC_RANGE_16G;

            /* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
            config.cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;

            /* Set number of average samples for accel. */
            config.cfg.acc.avg_num = BMI3_ACC_AVG64;

            /* Enable the accel mode where averaging of samples
             * will be done based on above set bandwidth and ODR.
             * Note : By default accel is disabled. The accel will get enable by selecting the mode.
             */
            config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

            /* Set the accel configurations. */
            rslt = bmi330_set_sensor_config(&config, BMI3_N_SENSE_COUNT_1, dev);
            bmi3_error_codes_print_result("bmi330_set_sensor_config", rslt);

            printf("*************************************\n");
            printf("Accel Configurations:\n");
            printf("ODR:\t %s\n", enum_to_string(BMI3_ACC_ODR_100HZ));
            printf("Range:\t %s\n", enum_to_string(BMI3_ACC_RANGE_16G));
            printf("Bandwidth:\t %s\n", enum_to_string(BMI3_ACC_BW_ODR_QUARTER));
            printf("Average samples:\t %s\n", enum_to_string(BMI3_ACC_AVG64));
            printf("Accel Mode:\t %s\n", enum_to_string(BMI3_ACC_MODE_NORMAL));
            printf("Resolution:%u\n", dev->resolution);
            printf("\n");
        }
    }

    return rslt;
}

/*!
 * @brief This internal API converts raw sensor register content (LSB) to corresponding accel raw value, g value and m/s2 value
 */
static void lsb_to_ms2(int16_t accel_lsb,
                       float g_range,
                       uint8_t resolution,
                       struct bmi3_accel_processed_data *accel_data)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)resolution) / 2.0f));

    accel_data->accel_raw = (float)accel_lsb / half_scale;
    accel_data->accel_g = accel_data->accel_raw * g_range;
    accel_data->accel_ms2 = accel_data->accel_g * GRAVITY_EARTH;
}

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Variable that store converted Sensor Data from LSB to  (Raw, G-value and m/s2 )  */
    struct bmi3_accel_processed_data accel_data_x;
    struct bmi3_accel_processed_data accel_data_y;
    struct bmi3_accel_processed_data accel_data_z;

    /* Create an instance of sensor data structure. */
    struct bmi3_sensor_data sensor_data = { 0 };

    /* Initialize the interrupt status of accel. */
    uint16_t int_status = 0;

    /* Variable to define maximum sample count we want to show in this example. */
    uint16_t max_sample_count = 100;
    uint16_t sample_count = 0;

    /* Select accel sensor. */
    sensor_data.type = BMI330_ACCEL;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    /* Initialize bmi330. */
    printf("Uploading configuration file\n");
    rslt = bmi330_init(&dev);
    bmi3_error_codes_print_result("bmi330_init", rslt);

    printf("Configuration file uploaded\n");
    printf("Chip ID :0x%x\n", dev.chip_id);

    if (rslt == BMI330_OK)
    {
        /* Accel configuration settings. */
        rslt = set_accel_config(&dev);
        bmi3_error_codes_print_result("set_accel_config", rslt);

        if (rslt == BMI330_OK)
        {
            printf("Acquisition Iteration Count :%d\n", max_sample_count);
            printf(
                "SENS_TIME_LSB\t ACC_LSB_X\t ACC_LSB_Y\t ACC_LSB_Z\t ACC_RAW_X\t ACC_RAW_Y\t ACC_RAW_Z\t ACC_G_X\t ACC_G_Y\t ACC_G_Z\t ACC_MS2_X\t ACC_MS2_Y\t ACC_MS2_Z\n");

            while (sample_count < max_sample_count)
            {
                /* To get the status of accel data ready interrupt. */
                rslt = bmi330_get_int1_status(&int_status, &dev);
                if (rslt != BMI330_OK)
                {
                    bmi3_error_codes_print_result("bmi330_get_int1_status", rslt);
                    bmi3_coines_deinit();

                    return 1;
                }

                /* To check the accel data ready interrupt status and print the status for 100 samples. */
                if (int_status & BMI3_INT_STATUS_ACC_DRDY)
                {
                    /* Get accelerometer data for x, y and z axis. */
                    rslt = bmi330_get_sensor_data(&sensor_data, BMI3_N_SENSE_COUNT_1, &dev);
                    if (rslt != BMI330_OK)
                    {
                        bmi3_error_codes_print_result("Get sensor data", rslt);
                        bmi3_coines_deinit();

                        return 1;
                    }

                    /* Converting lsb to G force for 16-bit accelerometer at 2G range. */
                    lsb_to_ms2(sensor_data.sens_data.acc.x,
                               BMI3_GET_RANGE_VAL(BMI3_ACC_RANGE_16G),
                               dev.resolution,
                               &accel_data_x);
                    lsb_to_ms2(sensor_data.sens_data.acc.y,
                               BMI3_GET_RANGE_VAL(BMI3_ACC_RANGE_16G),
                               dev.resolution,
                               &accel_data_y);
                    lsb_to_ms2(sensor_data.sens_data.acc.z,
                               BMI3_GET_RANGE_VAL(BMI3_ACC_RANGE_16G),
                               dev.resolution,
                               &accel_data_z);

                    /* Print the Sensor data */
                    printf(
#ifndef PC
                        "%8lu\t %8d\t %8d\t %8d\t %+9.3f\t %+9.3f\t %+9.3f\t %+9.3f\t %9.3f\t %+9.3f\t %+9.3f\t %+9.3f\t %+9.3f\n",
#else
                        "%8u\t %8d\t %8d\t %8d\t %+9.3f\t %+9.3f\t %+9.3f\t %+8.3f\t %7.3f\t %+7.3f\t %+9.3f\t %+9.3f\t %+9.3f\n",
#endif
                        sensor_data.sens_data.acc.sens_time,
                        sensor_data.sens_data.acc.x,
                        sensor_data.sens_data.acc.y,
                        sensor_data.sens_data.acc.z,
                        accel_data_x.accel_raw,
                        accel_data_y.accel_raw,
                        accel_data_z.accel_raw,
                        accel_data_x.accel_g,
                        accel_data_y.accel_g,
                        accel_data_z.accel_g,
                        accel_data_x.accel_ms2,
                        accel_data_y.accel_ms2,
                        accel_data_z.accel_ms2);

                    ++sample_count;
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}
