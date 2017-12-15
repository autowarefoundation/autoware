#ifndef CDEBUG_H_
#define CDEBUG_H_

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <time.h>

namespace cpu {
#define timeDiff(start, end) ((end.tv_sec - start.tv_sec) * 1000000 + end.tv_usec - start.tv_usec)
}

#endif
