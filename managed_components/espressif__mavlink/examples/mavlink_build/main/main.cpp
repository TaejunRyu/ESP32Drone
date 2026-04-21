/*
 * SPDX-FileCopyrightText: 2010-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
*/

#include <iostream>

#include <c_library_v2/test/mavlink.h>

extern "C" void app_main(void);

void app_main(void)
{
    std::cout << "Mavlink build example. Mavlink build date: "<< MAVLINK_BUILD_DATE << std::endl;
}
