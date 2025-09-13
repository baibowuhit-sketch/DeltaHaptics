/*
 * sr_portable.h
 *
 *  Created on: Mar 2, 2025
 *      Author: 19325
 */

#ifndef SR_PORTABLE_H_
#define SR_PORTABLE_H_

/***************************************************************************************************
**
**                          Copyright (c) SURGERII. All rights reserved
**
****************************************************************************************************
**
**                          Description:    sr_portable.h
**
***************************************************************************************************/

#ifndef HFB679157_95F5_4FA9_A9A2_753CEE7C69F1
#define HFB679157_95F5_4FA9_A9A2_753CEE7C69F1

#include <stdint.h>
#include <limits.h>

typedef uint8_t		           Boolean;
typedef uint8_t		           U8;
typedef int8_t	               S8;
typedef uint16_t	           U16;
typedef int16_t		           S16;
typedef uint32_t	           U32;
typedef int32_t	               S32;
typedef float                  F32;
typedef double                 F64;
typedef uint64_t		       U64;
typedef int64_t		           S64;


#ifndef FALSE
#define FALSE   false
#endif

#ifndef TRUE
#define TRUE    true
#endif

#ifndef NULL
#define NULL    (0)
#endif

#define SR_TRUE (Boolean)(1U)
#define SR_FALSE (Boolean)(0U)

#define  SR_DEF_PACKED __attribute__((packed))

#endif /*HFB679157_95F5_4FA9_A9A2_753CEE7C69F1*/



#endif /* SR_PORTABLE_H_ */
