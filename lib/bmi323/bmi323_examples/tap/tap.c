/**\
 * Copyright (c) 2024 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 **/

/******************************************************************************/
/*!                 Header Files                                              */
#include <stdio.h>
#include "bmi323.h"
#include "common.h"

/******************************************************************************/
/*!         Static Function Declaration                                       */

/*!
 *  @brief This internal API is used to set configurations for tap interrupt.
 *
 *  @param[in] dev       : Structure instance of bmi3_dev.
 *
 *  @return Status of execution.
 */
static int8_t set_feature_config(struct bmi3_dev *dev);

/******************************************************************************/
/*!            Functions                                                      */

/* This function starts the execution of program. */
int main(void)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    uint8_t data[2];

    /* Variable to get tap interrupt status. */
    uint16_t int_status = 0;

    /* Sensor initialization configuration. */
    struct bmi3_dev dev = { 0 };

    /* Feature enable initialization. */
    struct bmi3_feature_enable feature = { 0 };

    /* Loop variable to increment single tap, double tap and triple tap */
    uint8_t s_tap = 0, d_tap = 0, t_tap = 0;

    /* Interrupt mapping structure. */
    struct bmi3_map_int map_int = { 0 };

    /* Function to select interface between SPI and I2C, according to that the device structure gets updated.
     * Interface reference is given as a parameter
     * For I2C : BMI3_I2C_INTF
     * For SPI : BMI3_SPI_INTF
     */
    rslt = bmi3_interface_init(&dev, BMI3_SPI_INTF);
    bmi3_error_codes_print_result("bmi3_interface_init", rslt);

    if (rslt == BMI323_OK)
    {
        /* Initialize bmi323. */
        rslt = bmi323_init(&dev);
        bmi3_error_codes_print_result("bmi323_init", rslt);

        if (rslt == BMI323_OK)
        {
            /* Set feature configurations for tap interrupt. */
            rslt = set_feature_config(&dev);

            if (rslt == BMI323_OK)
            {
                feature.tap_detector_s_tap_en = BMI323_ENABLE;
                feature.tap_detector_d_tap_en = BMI323_ENABLE;
                feature.tap_detector_t_tap_en = BMI323_ENABLE;

                /* Enable the selected sensors. */
                rslt = bmi323_select_sensor(&feature, &dev);
                bmi3_error_codes_print_result("Sensor enable", rslt);

                if (rslt == BMI323_OK)
                {
                    map_int.tap_out = BMI3_INT2;

                    /* Map the feature interrupt for tap interrupt. */
                    rslt = bmi323_map_interrupt(map_int, &dev);
                    bmi3_error_codes_print_result("Map interrupt", rslt);
                    printf("Tap the board either single, double or triple tap\n");

                    /* Loop to get tap interrupt. */
                    do
                    {
                        /* Read the interrupt status from int 2 pin */
                        rslt = bmi323_get_int2_status(&int_status, &dev);
                        bmi3_error_codes_print_result("Get interrupt status", rslt);

                        /* Check the interrupt status of the tap */
                        if (int_status & BMI3_INT_STATUS_TAP)
                        {
                            printf("Tap interrupt is generated\n");
                            rslt = bmi323_get_regs(BMI3_REG_FEATURE_EVENT_EXT, data, 2, &dev);

                            if (data[0] & BMI3_TAP_DET_STATUS_SINGLE)
                            {
                                printf("Single tap asserted\n");

                                s_tap++;
                            }

                            if (data[0] & BMI3_TAP_DET_STATUS_DOUBLE)
                            {
                                printf("Double tap asserted\n");

                                d_tap++;
                            }

                            if (data[0] & BMI3_TAP_DET_STATUS_TRIPLE)
                            {
                                printf("Triple tap asserted\n");

                                t_tap++;
                            }

                            if (s_tap > 0 && d_tap > 0 && t_tap > 0)
                            {
                                break;
                            }
                        }
                    } while (rslt == BMI323_OK);
                }
            }
        }
    }

    bmi3_coines_deinit();

    return rslt;
}

/*!
 * @brief This internal API is used to set configurations for tap interrupt.
 */
static int8_t set_feature_config(struct bmi3_dev *dev)
{
    /* Status of API are returned to this variable. */
    int8_t rslt;

    /* Structure to define the type of sensor and its configurations. */
    struct bmi3_sens_config config[2] = { { 0 } };

    /* Configure the type of feature. */
    config[0].type = BMI323_ACCEL;
    config[1].type = BMI323_TAP;

    /* Get default configurations for the type of feature selected. */
    rslt = bmi323_get_sensor_config(config, 2, dev);
    bmi3_error_codes_print_result("Get sensor config", rslt);

    if (rslt == BMI323_OK)
    {
        /* Enable accel by selecting the mode. */
        config[0].cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;

        /* Set tap configuration settings. */

        /* Accelerometer sensing axis selection for tap detection.
         * Value    Name          Description
         * 0b00    axis_x     Use x-axis for tap detection
         * 0b01    axis_y     Use y-axis for tap detection
         * 0b10    axis_z     Use z-axis for tap detection
         * 0b11   reserved    Use z-axis for tap detection
         */
        config[1].cfg.tap.axis_sel = 1;

        /* Maximum duration between positive and negative peaks to tap */
        config[1].cfg.tap.max_dur_between_peaks = 5;

        /* Maximum duration from first tap within the second and/or third tap is expected to happen */
        config[1].cfg.tap.max_gest_dur = 0x11;

        /* Maximum number of threshold crossing expected around a tap */
        config[1].cfg.tap.max_peaks_for_tap = 5;

        /* Mimimum duration between two consecutive tap impact */
        config[1].cfg.tap.min_quite_dur_between_taps = 7;

        /* Mode for detection of tap gesture
         * Value    Name          Description
         * 0        Sensitive   Sensitive detection mode
         * 1        Normal      Normal detection mode
         * 2        Robust      Robust detection mode
         */
        config[1].cfg.tap.mode = 1;

        /* Minimum quite duration between two gestures */
        config[1].cfg.tap.quite_time_after_gest = 5;

        /* Minimum threshold for peak resulting from the tap */
        config[1].cfg.tap.tap_peak_thres = 0x2C;

        /* Maximum duration for which tap impact is observed */
        config[1].cfg.tap.tap_shock_settling_dur = 5;

        /* Perform gesture confirmation with wait time set by maximum gesture duration
         * Value    Name          Description
         * 0        Disable     Report the gesture when detected
         * 1        Enable      Report the gesture after confirmation
         */
        config[1].cfg.tap.wait_for_timeout = 1;

        /* Set new configurations. */
        rslt = bmi323_set_sensor_config(config, 2, dev);
        bmi3_error_codes_print_result("Set sensor config", rslt);
    }

    return rslt;
}
