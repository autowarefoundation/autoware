#ifndef __COMMON_HPP__
#define __COMMON_HPP__

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINT(str)                                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
    std::cout << str << std::endl;                                                                                     \
  } while (false)
#else
#define DEBUG_PRINT(str)                                                                                               \
  do                                                                                                                   \
  {                                                                                                                    \
  } while (false)
#endif

#endif
