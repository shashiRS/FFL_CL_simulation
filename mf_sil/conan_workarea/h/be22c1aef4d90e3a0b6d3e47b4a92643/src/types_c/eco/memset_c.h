// COMPANY: Continental Automotive
// All rights reserved

#ifndef _ECO_MEMSET_C_H_
#define _ECO_MEMSET_C_H_

#include "Platform_Types.h"

//PRQA S 4118 ++
/* review date: 2022-10-06, reviewer: uidd8596, reason: following signature of standard function plus the memory is modified after casting */

inline void* ECO_memset(void* dest, const uint32 ch, uint32 count)
//PRQA S 4118 --
{
  uint32 i;
  uint8* bytes = (uint8*)dest;
  if (count > 7U) {
    //PRQA S 3044 ++
    /* review date: 2022-10-06, reviewer: uidd8596, reason: address offset is by intention a number  */
    const uint32 offset = 8ULL - (uint32)((uint64)dest % 8ULL);
    //PRQA S 3044 --
    if (offset < 8U) {
      for (i = 0U; i < offset; ++i)
      {
        *bytes++ = (uint8)ch;
      }
      count -= offset;
    }
    const uint32 ch8 = ch&0xff;   // masked to use only value from LSB
    const uint64 ch16 = (uint64)(ch8 | (ch8 << 8));
    const uint64 ch32 = ch16 | (ch16 << 16);
    const uint64 ch64 = ch32 | (ch32 << 32);

    uint64* tmp = (uint64*)((void*)bytes);
    const uint32 num_words = count / 8U;
    for(i = 0U; i < num_words; ++i)
    {
      *tmp++ = ch64;
    }
    bytes += num_words * 8U;
    count -= num_words * 8U;
  }

  for(i = 0U; i < count; ++i)
  {
    *bytes++ = (uint8)ch;
  }
  return dest;
}

#endif // _ECO_MEMSET_C_H_
