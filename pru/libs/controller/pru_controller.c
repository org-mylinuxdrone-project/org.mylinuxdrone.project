/*
 * pru_controller.c
 *  Created on: 06 gen 2020
 *      Author: Andrea Lambruschini <andrea.lambruschini@gmail.com>
 *
 * Copyright 2019 Andrea Lambruschini
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); 
 * you may not use this file except in compliance with the License. 
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software 
 * distributed under the License is distributed on an "AS IS" BASIS, 
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
 * See the License for the specific language governing permissions and 
 * limitations under the License.
 */

#include <pru_controller.h>

/* TODO:
 * Input: S(RC(i)), Imu(i)
 * Calcolo:
 * - Ini=S(RC(i))-S(RC(i-1))
 * - inv(A)*In(i)=F(i)
 * - inv(A)*Imu(i)=Ftot(i)
 * Output:F(i), Ftot(i)
 *   l'output è inviato direttamente al controller motori.
 */
