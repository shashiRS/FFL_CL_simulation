// COMPANY: Continental Automotive
// All rights reserved

#ifndef _ECO_MEMSET_H_
#define _ECO_MEMSET_H_

#include "eco/memset_c.h"

namespace eco
{
inline void* memset(void* dest, const uint32 ch, uint32 count)
{
  return ECO_memset(dest, ch, count);
}

}  // namespace eco

#endif // _ECO_MEMSET_H_

