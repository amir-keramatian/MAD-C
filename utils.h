#ifndef UTILS_H
#define UTILS_H
#include <unistd.h>
#include <getopt.h>
#include <stdint.h>
#include <limits.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <tgmath.h> 
//#include "x86.h"


#include "barrier.h"

/**************************************************************************************/
#define CAS(addr, old_val, new_val) __sync_bool_compare_and_swap(addr, old_val, new_val)
#define FAA(addr, val) __sync_fetch_and_add(addr, val)
#define CASP(addr, old_val, new_val) __sync_val_compare_and_swap(addr, old_val, new_val)                                                                                                                                                                     

/******************************* Timers ***********************************************/
typedef uint64_t ticks;

static inline ticks
 getticks(void)
{
  unsigned hi, lo;
  __asm__ __volatile__ ("rdtsc" : "=a"(lo), "=d"(hi));
  return ( (unsigned long long)lo)|( ((unsigned long long)hi)<<32 );
}


#define sync_rdtsc1(val) \
   do {\
      unsigned int cycles_low, cycles_high;\
      __asm volatile ("CPUID\n\t"\
      "RDTSC\n\t"\
      "mov %%edx, %0\n\t"\
      "mov %%eax, %1\n\t": "=r" (cycles_high), "=r" (cycles_low):: "%rax", "%rbx", "%rcx", "%rdx"); \
      (val) = ((unsigned long) cycles_low) | (((unsigned long) cycles_high) << 32);\
   } while (0)

#define sync_rdtsc2(val) \
   do {\
      unsigned int cycles_low, cycles_high;\
      __asm volatile("RDTSCP\n\t"\
      "mov %%edx, %0\n\t"\
      "mov %%eax, %1\n\t"\
      "CPUID\n\t": "=r" (cycles_high), "=r" (cycles_low):: "%rax", "%rbx", "%rcx", "%rdx");\
      (val) = ((unsigned long) cycles_low) | (((unsigned long) cycles_high) << 32);\
   } while (0)

static uint8_t __attribute__ ((unused)) the_sockets[] = {
    0, 0, 0, 0, 0, 0, 0, 0, 
    1, 1, 1, 1, 1, 1, 1, 1, 
    2, 2, 2, 2, 2, 2, 2, 2, 
    3, 3, 3, 3, 3, 3, 3, 3, 
    4, 4, 4, 4, 4, 4, 4, 4, 
    5, 5, 5, 5, 5, 5, 5, 5, 
    6, 6, 6, 6, 6, 6, 6, 6, 
    7, 7, 7, 7, 7, 7, 7, 7 
  };

static uint16_t __attribute__ ((unused)) the_cores[] = {
  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71
};

static inline
void set_cpu(int cpu)
{
    int n_cpus = sysconf(_SC_NPROCESSORS_ONLN);
    //    cpu %= (NUMBER_OF_SOCKETS * CORES_PER_SOCKET);
    if (cpu < n_cpus)
      {
        int cpu_use = the_cores[cpu];
	//        printf("%d using CPU %d\n", cpu, cpu_use);
        cpu_set_t mask;
        CPU_ZERO(&mask);
        CPU_SET(cpu_use, &mask);
        #    if defined(PLATFORM_NUMA)
        numa_set_preferred(get_cluster(cpu_use));
        #    endif
        //  printf("%d = %d : %d\n", cpu, cpu_use, get_cluster(cpu_use));
        pthread_t thread = pthread_self();
        if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &mask) != 0)
          {
            fprintf(stderr, "Error setting thread affinity\n");
          }
      }
    else{
      fprintf(stderr, "Not possible to pin the thread");
    }
}
#endif
