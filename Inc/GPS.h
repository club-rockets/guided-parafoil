/*
 ___                  _      ___   _____  ___
|  _`\               ( )    (  _`\(_   _)(  _`\
| (_) )   _      ___ | |/') | (_(_) | |  | (_(_)
| ,  /  /'_`\  /'___)| , <  |  _)_  | |  `\__ \
| |\ \ ( (_) )( (___ | |\`\ | (_( ) | |  ( )_) |
(_) (_)`\___/'`\____)(_) (_)(____/' (_)  `\____)

Copyright (c) 2015, Cl�ment Rochon
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.


GPS.h

Date creation: Apr 23, 2016
Auteur: Cl�ment Rochon (clemini@gmail.com)
Description: DESCRIBE_THE_FILE_WHAT_IT_DOES_AND_HOW_TO_USE_IT

Last Modification :
  - YYY-MM-DD - DESCRIPTION
*/
#ifndef GPS_H_
#define GPS_H_

#include "string.h"
#include "math.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"

/* USER CODE BEGIN Includes */
#include "ubloc_gnss.h"
#include "usart.h"

#define BUFFER_SIZE 250

void GPS_Init();
void GPS_Task();


#endif /* GPS_H_ */
