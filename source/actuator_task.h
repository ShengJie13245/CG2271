/*
 * Copyright 2025 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef ACTUATOR_TASK_H_
#define ACTUATOR_TASK_H_

/*******************************************************************************
 * API
 ******************************************************************************/

/*!
 * @brief Actuator task function
 * @param pvParameters Task parameters (unused)
 */
void ActuatorTask(void *pvParameters);

/*!
 * @brief Signal mode change to actuator task
 */
void ActuatorTask_SignalModeChange(void);

#endif /* ACTUATOR_TASK_H_ */
