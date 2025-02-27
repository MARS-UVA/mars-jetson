#include "platform_compat.h"

void usleep_simulation(unsigned int microseconds)
{
    volatile unsigned long long i;
    for (i = 0; i < (unsigned long long)microseconds * 1000; i++)
    {
        // Do nothing, just burn CPU cycles
    }
}