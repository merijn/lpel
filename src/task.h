#ifndef _TASK_H_
#define _TASK_H_


#include <lpel.h>
#include <pthread.h>

#ifdef WAITING
#include <sys/time.h>
#endif

#include "arch/mctx.h"


#include "arch/atomic.h"

#include "scheduler.h"


/**
 * If a task size <= 0 is specified,
 * use the default size
 */
#define LPEL_TASK_SIZE_DEFAULT  8192  /* 8k */

struct workerctx_t;
struct mon_task_t;




/**
 * TASK CONTROL BLOCK
 */
struct lpel_task_t {
  /** intrinsic pointers for organizing tasks in a list*/
  struct lpel_task_t *prev, *next;
  unsigned int uid;    /** unique identifier */
  enum lpel_taskstate_t state;   /** state */

  struct workerctx_t *worker_context;  /** worker context for this task */

  sched_task_t sched_info;

  /**
   * indicates the SD which points to the stream which has new data
   * and caused this task to be woken up
   */
  struct lpel_stream_desc_t *wakeup_sd;
  atomic_t poll_token;        /** poll token, accessed concurrently */

  /* ACCOUNTING INFORMATION */
  struct mon_task_t *mon;

  /* CODE */
  int size;             /** complete size of the task, incl stack */
  mctx_t mctx;          /** machine context of the task*/
  lpel_taskfunc_t func; /** function of the task */
  void *inarg;          /** input argument  */
  void *outarg;         /** output argument  */

  int current_worker;   /** The current worker id */

  /**
   * Indicates the worker id which the placement scheduler has decided
   * the task should migrate to, if it is the same as current worker,
   * it keeps running on the current worker.
   */
  int new_worker;

#ifdef WAITING
  /**
   * total time the task is waiting in ready state. the implementation is
   * sort of a sliding window
   * TODO further explanation
   */
  struct timeval total_time_ready[2];

  /** total number of times the state switched to ready. the implementation is
   * sort of a sliding window
   * TODO further explanation
   *
   */
  int total_ready_num[2];
  /** the time at which the last time measurement started*/
  struct timeval last_measurement_start;
  /* The state is either the first sliding window or the second */
  int waiting_state;
  /* The total measured time over a certain period */
  struct timeval total_time[2];

  /** Mutex used for reading from and writing to the different variables */
  pthread_mutex_t t_mu;
#endif
};




void LpelTaskDestroy( lpel_task_t *t);


void LpelTaskBlock( lpel_task_t *t );
void LpelTaskBlockStream( lpel_task_t *ct);
void LpelTaskUnblock( lpel_task_t *ct, lpel_task_t *blocked);

#ifdef WAITING
/**
 * The task is not ready anymore, stop the timing and update the statistics
 */
void LpelTaskStopTiming( lpel_task_t *t);

/**
 * Returns an estimate percentage of the task begin ready over the time from
 * creation
 */
double LpelTaskGetPercentageReady( lpel_task_t *t);
#endif

#endif /* _TASK_H_ */
