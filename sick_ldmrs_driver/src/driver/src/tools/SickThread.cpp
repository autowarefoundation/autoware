//
// SickThread.cpp
//

#include "SickThread.hpp"

void* wrapper_prerun(void* state)
{
  reinterpret_cast<ThreadWrapperBase*>(state)->thread_entry();
  return 0;
}


