/**\
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include <math.h>
#include "bmi323.h"
#include "common.h"

/******************************************************************************/
/*!         Macros definition                                       */

#define ACCEL                             UINT8_C(0x00)
#define GYRO                              UINT8_C(0x01)

/* Defines the size of the raw data buffer in the FIFO */
#define BMI323_FIFO_RAW_DATA_BUFFER_SIZE  UINT16_C(2048)

/*Defines the maximum user-defined length of raw data to be read from the FIFO */
#define BMI323_FIFO_RAW_DATA_USER_LENGTH  UINT16_C(2048)

/* Compute the maximum frame count based on the FIFO buffer size and the length of FIFO frame,
 * accounting for all possible FIFO Frame Configurations
 */
#define BMI323_MAX_ACCEL_FRAME_COUNT      UINT16_C(BMI323_FIFO_RAW_DATA_BUFFER_SIZE / BMI3_LENGTH_FIFO_ACC)
#define BMI323_MAX_GYRO_FRAME_COUNT       UINT16_C(BMI323_FIFO_RAW_DATA_BUFFER_SIZE / BMI3_LENGTH_FIFO_GYR)

/*Since temperature data is triggered in sync with the accelerometer ODR,
 * the number of instances for each would be equal to the number of accelerometer data instances.
 */
#define BMI323_MAX_TEMP_DATA_FRAME_COUNT  UINT16_C(BMI323_FIFO_RAW_DATA_BUFFER_SIZE / BMI3_LENGTH_FIFO_ACC)

/******************************************************************************/
/*!         Structure Definition                                              */

/*! Structure to define accelerometer and gyroscope configuration. */
struct bmi3_sens_config config[2];

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for FIFO, accelerometer and gyroscope.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static void set_sensor_config(struct bmi3_dev *dev);

/*! @brief This internal API converts raw sensor values(LSB) to G value
 *
 *  @param[in] val        : Raw sensor value.
 *  @param[in] g_range    : Accel Range selected (4G).
 *  @param[in] bit_width  : Resolution of the sensor.
 *
 *  @return Accel values in Gravity(G)
 *
 */
static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width);

/*!
 *  @brief This function converts lsb to degree per second for 16 bit gyro at
 *  range 125, 250, 500, 1000 or 2000dps.
 *
 *  @param[in] val       : LSB from each axis.
 *  @param[in] dps       : Degree per second.
 *  @param[in] bit_width : Resolution for gyro.
 *
 *  @return Degree per second.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width);

/******************************************************************************/
/*!               Functions                                                   */

/* This function starts the execution of program. */
int main(void)
{
    struct bmi3_dev dev;

    int8_t rslt;

    /* Variable to set the data sample rate of i3c sync
     * 0x0032 is set to 50 samples this value can vary */
    uint16_t sample_rate = 0x0032;

    /* Variable to set the delay time of i3c sync */
    uint8_t delay_time = BMI3_I3C_SYNC_DIVISION_FACTOR_11;

    /* Variable to set the i3c sync ODR */
    uint8_t odr = BMI3_I3C_SYNC_ODR_50HZ;

    /* Variable to enable the filer */
    uint8_t i3c_tc_res = BMI323_ENABLE;

    uint16_t int_status;

    /* Variable to index bytes. */
    uint8_t idx;

    struct bmi3_feature_enable feature = { 0 };

    /* Variable to store temperature */
    float temperature_value;

    float x = 0, y = 0, z = 0;

    /* Number of bytes of FIFO data */
    uint8_t fifo_data[BMI323_FIFO_RAW_DATA_BUFFER_SIZE] = { 0 };

    /* Array to store accelerometer data frames extracted from FIFO */
    struct bmi3_fifo_sens_axes_data fifo_accel_data[BMI323_MAX_ACCEL_FRAME_COUNT];

    /*Array to store gyroscope data frames extracted from FIFO */
    struct bmi3_fifo_sens_axes_data fifo_gyro_data[BMI323_MAX_GYRO_FRAME_COUNT];

    /* Array to store temperature data frames extracted from FIFO */
    struct bmi3_fifo_temperature_data fifo_temp_data[BMI323_MAX_TEMP_DATA_FRAME_COUNT];

    /* Initialize FIFO frame structure */
    struct bmi3_fifo_frame fifoframe = { 0 };

    uint16_t fifo_length = 0;

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_I2C_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize BMI323 */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            rslt = bmi323_configure_enhanced_flexibility(&dev);
            bmi3_error_codes_print_result("bmi323_configure_enhanced_flexibility", rslt);

            if (rslt == BMI323_OK)
            {
                set_sensor_config(&dev);

                /* Enable i3c_sync feature */
                feature.i3c_sync_en = BMI323_ENABLE;

                /* Enable the selected sensors */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("bmi323_select_sensor", rslt);

                /* Set the data sample rate of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tph(sample_rate, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_tph", rslt);

                /* Set the delay time of i3c sync */
                rslt = bmi323_set_i3c_tc_sync_tu(delay_time, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_tu", rslt);

                /* Set i3c sync ODR */
                rslt = bmi323_set_i3c_tc_sync_odr(odr, &dev);
                bmi3_error_codes_print_result("set_i3c_tc_sync_odr", rslt);

                /* Enable the i3c sync filter */
                rslt = bmi323_set_i3c_sync_i3c_tc_res(i3c_tc_res, &dev);
                bmi3_error_codes_print_result("set_i3c_sync_i3c_tc_res", rslt);

                /* After any change to the I3C-Sync configuration parameters,
                 * a config changed CMD (0x0201) must be written
                 * to CMD register to update the internal configuration */
                rslt = bmi323_set_command_register(BMI3_CMD_I3C_TCSYNC_UPDATE, &dev);
                bmi3_error_codes_print_result("bmi323_set_command_register", rslt);

                printf("I3C accel, gyro and temperature data");

                /* Update FIFO structure */
                /* Mapping the buffer to store the FIFO data. */
                fifoframe.data = fifo_data;

                /* Length of FIFO frame. */
                fifoframe.length = BMI323_FIFO_RAW_DATA_USER_LENGTH;

                for (;;)
                {
                    /* Read FIFO data on interrupt. */
                    rslt = bmi323_get_int1_status(&int_status, &dev);
                    bmi3_error_codes_print_result("bmi323_get_int1_status", rslt);

                    /* To check the status of FIFO full interrupt. */
                    if ((rslt == BMI323_OK) && (int_status & BMI3_INT_STATUS_FFULL))
                    {
                        printf("\nFIFO full interrupt occurred");

                        rslt = bmi323_get_fifo_length(&fifoframe.available_fifo_len, &dev);
                        bmi3_error_codes_print_result("bmi323_get_fifo_length", rslt);

                        /* Convert available fifo length from word to byte */
                        fifo_length = (uint16_t)(fifoframe.available_fifo_len * 2);

                        fifoframe.length = fifo_length + dev.dummy_byte;

                        printf("\nFIFO length in words : %d\n", fifoframe.available_fifo_len);
                        printf("\nFIFO data bytes available : %d \n", fifo_length);
                        printf("\nFIFO data bytes requested : %d \n", fifoframe.length);

                        /* Read FIFO data */
                        rslt = bmi323_read_fifo_data(&fifoframe, &dev);
                        bmi3_error_codes_print_result("bmi323_read_fifo_data", rslt);

                        if (rslt == BMI323_OK)
                        {
                            /* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
                            (void)bmi323_extract_accel(fifo_accel_data, &fifoframe, &dev);
                            printf("\nParsed accelerometer data frames: %d\n", fifoframe.avail_fifo_accel_frames);

                            printf("Accel data in LSB units and in Gravity\n");

                            printf(
                                "\nACCEL_DATA_SET, Acc_Raw_X, Acc_Raw_Y, Acc_Raw_Z, Acc_G_X, Acc_G_Y, Acc_G_Z, SensorTime(lsb)\n");

                            /* Print the parsed accelerometer data from the FIFO buffer */
                            for (idx = 0; idx < fifoframe.avail_fifo_accel_frames; idx++)
                            {
                                /* Converting lsb to gravity for 16 bit accelerometer at 2G range.
                                 * */
                                x = lsb_to_g(fifo_accel_data[idx].x, 2.0f, dev.resolution);
                                y = lsb_to_g(fifo_accel_data[idx].y, 2.0f, dev.resolution);
                                z = lsb_to_g(fifo_accel_data[idx].z, 2.0f, dev.resolution);

                                /* Print the data in Gravity. */
                                printf("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d\n",
                                       idx,
                                       fifo_accel_data[idx].x,
                                       fifo_accel_data[idx].y,
                                       fifo_accel_data[idx].z,
                                       x,
                                       y,
                                       z,
                                       fifo_accel_data[idx].sensor_time);
                            }

                            /* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
                            (void)bmi323_extract_gyro(fifo_gyro_data, &fifoframe, &dev);
                            printf("\nParsed gyroscope data frames: %d\n", fifoframe.avail_fifo_gyro_frames);

                            printf("Gyro data in LSB units and degrees per second\n");

                            printf(
                                "\nGYRO_DATA_SET, Gyr_Raw_X, Gyr_Raw_Y, Gyr_Raw_Z, Gyr_dps_X, Gyr_dps_Y, Gyr_dps_Z, SensorTime(lsb)\n");

                            /* Print the parsed gyroscope data from the FIFO buffer */
                            for (idx = 0; idx < fifoframe.avail_fifo_gyro_frames; idx++)
                            {
                                /* Converting lsb to degree per second for 16 bit gyro at 2000dps range. */
                                x = lsb_to_dps(fifo_gyro_data[idx].x, (float)2000, dev.resolution);
                                y = lsb_to_dps(fifo_gyro_data[idx].y, (float)2000, dev.resolution);
                                z = lsb_to_dps(fifo_gyro_data[idx].z, (float)2000, dev.resolution);

                                /* Print the data in dps. */
                                printf("%d, %d, %d, %d, %4.2f, %4.2f, %4.2f, %d\n",
                                       idx,
                                       fifo_gyro_data[idx].x,
                                       fifo_gyro_data[idx].y,
                                       fifo_gyro_data[idx].z,
                                       x,
                                       y,
                                       z,
                                       fifo_gyro_data[idx].sensor_time);
                            }

                            /* Parse the FIFO data to extract temperature data from the FIFO buffer */
                            (void)bmi323_extract_temperature(fifo_temp_data, &fifoframe, &dev);
                            printf("\nParsed temperature data frames: %d\n", fifoframe.avail_fifo_temp_frames);

                            printf(
                                "\nTEMP_DATA_SET, TEMP_DATA_LSB, Temperature data (Degree celcius), SensorTime(lsb)\n");

                            /* Print the parsed temperature data from the FIFO buffer */
                            for (idx = 0; idx < fifoframe.avail_fifo_temp_frames; idx++)
                            {
                                temperature_value =
                                    (float)((((float)((int16_t)fifo_temp_data[idx].temp_data)) / 512.0) + 23.0);
                                printf("%d, %d, %f, %d\n",
                                       idx,
                                       fifo_temp_data[idx].temp_data,
                                       temperature_value,
                                       fifo_temp_data[idx].sensor_time);
                            }
                        }

                        break;
                    }
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for accelerometer, gyroscope and FIFO.
 */
static void set_sensor_config(struct bmi3_dev *dev)
{
    int8_t rslt;

    /* Array to define set FIFO flush */
    uint8_t data[2] = { BMI323_ENABLE, 0 };

    /* Structure to define interrupt with its feature */
    struct bmi3_map_int map_int = { 0 };

    /* Configure type of feature */
    config[ACCEL].type = BMI323_ACCEL;
    config[GYRO].type = BMI323_GYRO;

    /* Get default configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("get_sensor_fifo_config", rslt);

    /* Configure the accel and gyro settings */
    config[ACCEL].cfg.acc.bwp = BMI3_ACC_BW_ODR_QUARTER;
    config[ACCEL].cfg.acc.range = BMI3_ACC_RANGE_2G;
    config[ACCEL].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

    config[GYRO].cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
    config[GYRO].cfg.gyr.range = BMI3_GYR_RANGE_125DPS;
    config[GYRO].cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

    rslt = bmi323_set_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("set_sensor_fifo_config", rslt);

    /* Get configurations for the type of feature selected */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("get_sensor_fifo_config", rslt);

    /* To enable the accelerometer, gyroscope, temperature and sensor time in FIFO conf addr */
    rslt = bmi323_set_fifo_config(BMI3_FIFO_ALL_EN, BMI323_ENABLE, dev);
    bmi3_error_codes_print_result("bmi323_set_fifo_config", rslt);

    /* Set the FIFO flush in FIFO control register to clear the FIFO data */
    rslt = bmi323_set_regs(BMI3_REG_FIFO_CTRL, data, 2, dev);
    bmi3_error_codes_print_result("bmi323_set_regs", rslt);

    /* Map the FIFO full interrupt to INT1 */
    /* Note: User can map the interrupt to INT1 or INT2 */
    map_int.fifo_full_int = BMI3_INT1;

    /* Map the interrupt configuration */
    rslt = bmi323_map_interrupt(map_int, dev);
    bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);
}

/*!
 * @brief This internal API converts raw sensor values(LSB) to G value
 */
static float lsb_to_g(int16_t val, float g_range, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}
