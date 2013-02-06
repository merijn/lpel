#include <assert.h>
#include <errno.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#ifdef HAVE_PROC_CAPABILITIES
#include <sys/capability.h>
#endif

#include "lpel.h"
#include "lpelcfg.h"

#include "lpel_hwloc.h"

static int pu_count;

#ifdef HAVE_HWLOC
static int pus;
static int cores;
static int sockets;

static hwloc_cpuset_t   *cpu_sets;
static lpel_hw_place_t  *hw_places;

static hwloc_topology_t topology = 0;

#elif defined(HAVE_PTHREAD_SETAFFINITY_NP)
static cpu_set_t cpuset_workers;
static cpu_set_t cpuset_others;
#endif

static int LpelCanSetExclusive(void)
{
#ifdef HAVE_PROC_CAPABILITIES
  cap_t caps;
  cap_flag_value_t cap;
  /* obtain caps of process */
  caps = cap_get_proc();
  if (caps == NULL) return 0;
  cap_get_flag(caps, CAP_SYS_NICE, CAP_EFFECTIVE, &cap);
  return cap == CAP_SET;
#else
  return 0;
#endif
}

#ifdef HAVE_HWLOC
inline static void traverseBF(hwloc_obj_t object)
{
  static int start, index = 0, socket = -1, core = -1, pu = -1;
  static int setSock = 0, setCore = 0, setPu = 0;
  if (index == pu_count) return;

  if (start == -1) {
    for (start = 0; index < pu_count; start++) {
      for (int i = 0; i < object->arity && index < pu_count; i++) {
        socket = -1;
        core = -1;
        pu = -1;
        traverseBF(object->children[i]);
      }
    }

    return;
  }

  switch (object->type) {
    case HWLOC_OBJ_SOCKET:
      socket++;
      core = -1;
      pu = -1;
      break;
    case HWLOC_OBJ_CORE:
      core++;
      pu = -1;
      break;
    case HWLOC_OBJ_PU:
      pu++;
      if (setSock == socket
          && setCore == core
          && setPu == pu) {
        hw_places[index].socket = socket;
        hw_places[index].core = core;
        hw_places[index].pu = pu;
        cpu_sets[index] = hwloc_bitmap_dup(object->cpuset);
        index++;

        setSock = ++setSock % sockets;
        if (setSock == 0) {
          setCore = ++setCore % cores;
          if (setCore == 0) {
            setPu = ++setPu % pus;
          }
        }
      }
      break;
    default:
      break;
  }

  for (int i = 0; i < object->arity && index < pu_count; i++) {
    traverseBF(object->children[i]);
  }
}

inline static void traverseDF(hwloc_obj_t object)
{
  static int index = 0, socket = -1, core = -1, pu = -1;
  if (index == pu_count) return;

  switch (object->type) {
    case HWLOC_OBJ_SOCKET:
      socket++;
      core = -1;
      pu = -1;
      break;
    case HWLOC_OBJ_CORE:
      core++;
      pu = -1;
      break;
    case HWLOC_OBJ_PU:
      pu++;
      hw_places[index].socket = socket;
      hw_places[index].core = core;
      hw_places[index].pu = pu;
      cpu_sets[index] = hwloc_bitmap_dup(object->cpuset);
      index++;
      break;
    default:
      break;
  }

  for (int i = 0; i < object->arity && index < pu_count; i++) {
    traverseDF(object->children[i]);
  }
}

inline static void traverse(hwloc_obj_t object, int gather)
{
  if (gather) traverseDF(object);
  else traverseBF(object);
}

int LpelHwLocToWorker(lpel_hw_place_t place)
{
    for (int i = 0; i < pu_count; i++) {
        if (hw_places[i].socket == place.socket &&
                hw_places[i].core == place.core &&
                hw_places[i].pu == place.pu) return i;
    }

    return -1;
}

lpel_hw_place_t LpelWorkerToHwLoc(int wid) { return hw_places[wid]; }
#endif

void LpelHwLocInit(lpel_config_t *cfg)
{
#ifdef HAVE_HWLOC
  hwloc_topology_init(&topology);
  hwloc_topology_load(topology);

  pu_count = hwloc_get_nbobjs_by_type(topology, HWLOC_OBJ_PU);
  cores = hwloc_get_nbobjs_by_type(topology, HWLOC_OBJ_CORE);
  sockets = hwloc_get_nbobjs_by_type(topology, HWLOC_OBJ_SOCKET);

  pus = pu_count / cores;
  cores = cores / sockets;

  cfg->sockets = sockets;
  cfg->cores = cores;
  cfg->threads = pus;
  cfg->gather = 0;
#elif defined(HAVE_SYSCONF)
  /* query the number of CPUs */
  pu_count = sysconf(_SC_NPROCESSORS_ONLN);
#else
  char *p = getenv("LPEL_NUM_WORKERS");
  if (p) {
      errno = 0;
      pu_count = strtoul(p, 0, 0);
      if (errno) pu_count = 0;
  }
#endif

  cfg->num_workers = pu_count;
  cfg->proc_workers = pu_count;
  cfg->proc_others = 0;
}

int LpelHwLocCheckConfig(lpel_config_t *cfg)
{
  /* input sanity checks */
  if (  cfg->num_workers <= 0
     || cfg->proc_workers <= 0
     || cfg->proc_others < 0
     ) {
    return LPEL_ERR_INVAL;
  }

  /* check if there are enough processors (if we can check) */
  if (cfg->proc_workers + cfg->proc_others > pu_count) {
    return LPEL_ERR_INVAL;
  }

  pu_count = cfg->proc_workers + cfg->proc_others;

#ifdef HAVE_HWLOC
  if (cfg->threads <= pus && cfg->cores <= cores && cfg->sockets <= sockets
      && pu_count > cfg->threads * cfg->cores * cfg->sockets) {
    return LPEL_ERR_INVAL;
  }
#endif

  /* check exclusive flag sanity */
  if (LPEL_ICFG(LPEL_FLAG_EXCLUSIVE)) {
    /* check if we can do a 1-1 mapping */
    if (cfg->proc_others == 0 || cfg->num_workers > cfg->proc_workers) {
      return LPEL_ERR_INVAL;
    }

    /* pinned flag must also be set */
    if (!LPEL_ICFG(LPEL_FLAG_PINNED)) {
      return LPEL_ERR_INVAL;
    }

    /* check permissions to set exclusive (if we can check) */
    if (!LpelCanSetExclusive()) {
      return LPEL_ERR_EXCL;
    }
  }

  return 0;
}

void LpelHwLocStart(lpel_config_t *cfg)
{
#ifdef HAVE_HWLOC
  pus = cfg->threads;
  cores = cfg->cores;
  sockets = cfg->sockets;

  cpu_sets = malloc(pu_count * sizeof(*cpu_sets));
  hw_places = malloc(pu_count * sizeof(*hw_places));

  traverse(hwloc_get_root_obj(topology), cfg->gather);
#elif defined(HAVE_PTHREAD_SETAFFINITY_NP)
  /* create the cpu_set for worker threads */
  CPU_ZERO(&cpuset_workers);
  for (int i = 0; i < cfg->proc_workers; i++) {
    CPU_SET(i, &cpuset_workers);
  }

  /* create the cpu_set for other threads */
  CPU_ZERO(&cpuset_others);
  if (cfg->proc_others == 0) {
    /* distribute on the workers */
    for (int i = 0; i < cfg->proc_workers; i++) {
      CPU_SET(i, &cpuset_others);
    }
  } else {
    /* set to proc_others */
    for (int i = cfg->proc_workers;
        i < cfg->proc_workers + cfg->proc_others;
        i++) {
      CPU_SET(i, &cpuset_others);
    }
  }
#endif
}

int LpelThreadAssign(int core)
{
  int res;
#ifdef HAVE_HWLOC
  if (core < 0) return 0;

  res = hwloc_set_cpubind(topology, cpu_sets[core],
                          HWLOC_CPUBIND_THREAD | HWLOC_CPUBIND_STRICT);

  if (res == -1) return LPEL_ERR_ASSIGN;
#elif defined(HAVE_PTHREAD_SETAFFINITY_NP)
  pthread_t pt = pthread_self();

  if (core == -1) {
    /* assign an others thread to others cpuset */
    res = pthread_setaffinity_np(pt, sizeof(cpu_set_t), &cpuset_others);
    if (res != 0) return LPEL_ERR_ASSIGN;

  } else if (!LPEL_ICFG(LPEL_FLAG_PINNED)) {
    /* assign along all workers */
    res = pthread_setaffinity_np(pt, sizeof(cpu_set_t), &cpuset_workers);
    if (res != 0) return LPEL_ERR_ASSIGN;

  } else { /* LPEL_FLAG_PINNED */
    /* assign to specified core */
    cpu_set_t cpuset;

    CPU_ZERO(&cpuset);
    CPU_SET( core, &cpuset);
    res = pthread_setaffinity_np(pt, sizeof(cpu_set_t), &cpuset);
    if (res != 0) return LPEL_ERR_ASSIGN;

    /* make non-preemptible */
    if (LPEL_ICFG(LPEL_FLAG_EXCLUSIVE)) {
      struct sched_param param;
      int sp = SCHED_FIFO;
      /* highest real-time */
      param.sched_priority = sched_get_priority_max(sp);
      res = pthread_setschedparam(pt, sp, &param);
      if (res != 0) {
        /* we do best effort at this point */
        return LPEL_ERR_EXCL;
      }
    }
  }
#endif
  return 0;
}

void LpelHwLocCleanup(void)
{
#ifdef HAVE_HWLOC
  if (topology) {
    free(cpu_sets);
    free(hw_places);
    hwloc_topology_destroy(topology);
  }
#endif
}
