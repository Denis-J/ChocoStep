/**
 ******************************************************************************
 * @file    string2.h
 * @author  Denis Jullien
 * @version V1.0
 * @date    09/10/2014
 * @brief   Header for string2.c module
 *
 ******************************************************************************
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STRING_H
#define STRING_H

/* Includes ------------------------------------------------------------------*/

#include <stdlib.h>

/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */
char* strtok(char* str, const char* delimiters);
size_t strlen(const char *s);


#endif
