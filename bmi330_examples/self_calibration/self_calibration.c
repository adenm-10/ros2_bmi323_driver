/**\
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi330.h"
#include "common.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for gyro.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_gyro_config(struct bmi3_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Structure to define the self-calibration result and error result */
    struct bmi3_self_calib_rslt sc_rslt;

    /* Variable to choose apply correction */
    uint8_t apply_corr = BMI3_SC_APPLY_CORR_EN;

    /* Variable to define error. */
    int8_t rslt;

    /* Variable to define index. */
    uint8_t idx;

    /* Variable to define limit. */
    uint8_t limit = 3;

    /* Array to define self-calibration modes. */
    uint8_t sc_selection[3] = { BMI3_SC_SENSITIVITY_EN, BMI3_SC_OFFSET_EN, BMI3_SC_SENSITIVITY_EN | BMI3_SC_OFFSET_EN };

    /* Structure instance of gyro dp gain offset */
    struct bmi3_gyr_dp_gain_offset gyr_dp_gain_offset = { 0 };

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI330_OK)
    {
        for (idx = 0; idx < limit; idx++)
        {
            /* Initialize bmi330 */
            printf("\nUploading configuration file\n");
            rslt = bmi330_init(&dev);
            bmi3_error_codes_print_result("bmi330_init", rslt);

            printf("Configuration file uploaded\n");
            printf("Chip ID :0x%x\n", dev.chip_id);

            rslt = set_gyro_config(&dev);

            if (rslt == BMI330_OK)
            {
                if (sc_selection[idx] == BMI3_SC_SENSITIVITY_EN)
                {
                    printf("Self-calibration for sensitivity mode\n");
                }
                else if (sc_selection[idx] == BMI3_SC_OFFSET_EN)
                {
                    printf("\n\nSelf-calibration for offset mode\n");
                }
                else
                {
                    printf("\n\nSelf-calibration for sensitivity and offset mode\n");
                }

                /* Performs self-calibration for either sensitivity, offset or both */
                rslt = bmi330_perform_gyro_sc(sc_selection[idx], apply_corr, &sc_rslt, &dev);
                bmi3_error_codes_print_result("bmi330_perform_gyro_sc", rslt);

                if ((rslt == BMI330_OK) && (sc_rslt.gyro_sc_rslt == BMI330_TRUE))
                {
                    printf("Self calibration is successfully completed \n");
                }

                if ((rslt == BMI330_OK) && (sc_rslt.gyro_sc_rslt == BMI330_FALSE))
                {
                    printf("Self calibration is not successfully completed \n");

                    switch (sc_rslt.sc_error_status)
                    {
                        case BMI3_SC_ST_ABORTED_MASK:
                            printf("SC_ST_ABORTED\n");
                            break;
                        case BMI3_SC_ST_PRECON_ERR_MASK:
                            printf("BMI330_SC_ST_PRECON_ERR\n");
                            break;
                        case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
                            printf("BMI330_MODE_CHANGE_WHILE_SC_ST\n");
                            break;
                        default:
                            break;
                    }
                }

                printf("Result of self-calibration error \n");

                if (((sc_rslt.sc_error_status & BMI3_SET_LOW_NIBBLE) == BMI3_NO_ERROR_MASK))
                {
                    printf("\tNo error\n");
                }
                else
                {
                    printf("\tError: %d\n", sc_rslt.sc_error_status & BMI3_SET_LOW_NIBBLE);
                }

                if (sc_rslt.sc_error_status & BMI3_SC_ST_COMPLETE_MASK)
                {
                    printf("\tSelf-calibration procedure is completed.\n");
                }
                else
                {
                    printf("\tError: Self-calibration procedure is not completed\n");
                }

                printf("\nResult of self-calibration : %d\n", sc_rslt.gyro_sc_rslt);

                rslt = bmi330_get_gyro_dp_off_dgain(&gyr_dp_gain_offset, &dev);
                bmi3_error_codes_print_result("bmi330_get_gyro_off_dgain", rslt);

                printf("Result of gyro dp offset x(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_off_x);
                printf("Result of gyro dp offset y(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_off_y);
                printf("Result of gyro dp offset z(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_off_z);
                printf("Result of gyro dp gain x(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_dgain_x);
                printf("Result of gyro dp gain y(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_dgain_y);
                printf("Result of gyro dp gain z(LSB) : %d\n", gyr_dp_gain_offset.gyr_dp_dgain_z);
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 *  @brief This internal API is used to set configurations for gyro.
 */
static int8_t set_gyro_config(struct bmi3_dev *dev)
{
    struct bmi3_map_int map_int = { 0 };

    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config;

    /* Configure the type of feature. */
    config.type = BMI330_GYRO;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi330_get_sensor_config(&config, BMI3_N_SENSE_COUNT_1, dev);
    bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI330_OK)
    {
        map_int.gyr_drdy_int = BMI3_INT1;

        /* Map data ready interrupt to interrupt pin. */
        rslt = bmi330_map_interrupt(map_int, dev);
        bmi3_error_codes_print_result("Map interrupt", rslt);

        if (rslt == BMI330_OK)
        {
            /* The user can change the following configuration parameters according to their requirement. */
            /* Output Data Rate. By default ODR is set as 100Hz for gyro. */
            config.cfg.gyr.odr = BMI3_GYR_ODR_100HZ;

            /* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
            config.cfg.gyr.range = BMI3_GYR_RANGE_500DPS;

            /*  The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
             *  Value   Name      Description
             *    0   odr_half   BW = gyr_odr/2
             *    1  odr_quarter BW = gyr_odr/4
             */
            config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;

            /* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
            config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;

            /* Value    Name    Description
             *  0b000     avg_1   No averaging; pass sample without filtering
             *  0b001     avg_2   Averaging of 2 samples
             *  0b010     avg_4   Averaging of 4 samples
             *  0b011     avg_8   Averaging of 8 samples
             *  0b100     avg_16  Averaging of 16 samples
             *  0b101     avg_32  Averaging of 32 samples
             *  0b110     avg_64  Averaging of 64 samples
             */
            config.cfg.gyr.avg_num = BMI3_GYR_AVG1;

            /* Set the gyro configurations. */
            rslt = bmi330_set_sensor_config(&config, BMI3_N_SENSE_COUNT_1, dev);
            bmi3_error_codes_print_result("Set sensor config", rslt);

            printf("*************************************\n");
            printf("Gyro configurations\n");
            printf("ODR:\t %s\n", enum_to_string(BMI3_GYR_ODR_100HZ));
            printf("Range:\t %s\n", enum_to_string(BMI3_GYR_RANGE_500DPS));
            printf("Bandwidth:\t %s\n", enum_to_string(BMI3_GYR_BW_ODR_HALF));
            printf("Average samples:\t %s\n", enum_to_string(BMI3_GYR_AVG1));
            printf("Gyro Mode:\t %s\n", enum_to_string(BMI3_GYR_MODE_NORMAL));
            printf("Resolution:%u\n", dev->resolution);
        }
    }

    return rslt;
}
