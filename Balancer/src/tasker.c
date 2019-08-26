/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#include <stdlib.h>
#include "tasker.h"
#include "hal_intf.h"

/* ****************************************************************************************************************** */

#define MAX_TASKS       4   // sum of system's task
#define MAX_YIELD       2

struct sTaskData
{
    fpWakeup    _wakeup;
    fpWorker    _worker;
    void        *_param;
    eTaskPrio   _prio;
    int8_t      _dyn_prio;
    int8_t      _n_yield;
};
typedef struct sTaskData        sTaskData;

struct sTaskerData
{
    sTaskData   _tasks[MAX_TASKS];
    int8_t      _n_task;
};
typedef struct sTaskerData      sTaskerData;

struct sIdlerData
{
    uint32_t    _last_exec_s;
    uint32_t    _last_exec_ms;
    uint32_t    _exec_count;
    int8_t      _exec_rate;
};
typedef struct sIdlerData       sIdlerData;

/* ****************************************************************************************************************** */

static sIdlerData   idler_data;
static sTaskerData  tasker_data =
{
        ._n_task = 0
};

/* ****************************************************************************************************************** */

static bool idler_wakeup(uint32_t now_us, void *param)
{
    (void)now_us;
    (void)param;
    return true;
}

static void idler_worker(uint32_t now_us, void *param)
{
    sIdlerData  *this = (sIdlerData*)param;
    volatile uint32_t   now_ms = now_us / 1000;
    volatile uint32_t   now_s = now_ms / 1000;

    if (now_ms != this->_last_exec_ms)
    {
        this->_last_exec_ms = now_ms;
        this->_exec_count++;
        if (this->_last_exec_s != now_s)
        {
            this->_exec_rate = this->_exec_count * 100 / 1000;
            this->_exec_count = 0;
            this->_last_exec_s = now_s;
        }
    }
}

bool tasker_init(void)
{
    tasker_data._tasks[0]._wakeup = idler_wakeup;
    tasker_data._tasks[0]._worker = idler_worker;
    tasker_data._tasks[0]._param  = &idler_data;
    tasker_data._tasks[0]._prio   = eTaskPrio_Base;
    tasker_data._tasks[0]._dyn_prio = eTaskPrio_Base;
    tasker_data._tasks[0]._n_yield = 0;
    tasker_data._n_task = 1;
    return true;
}

bool tasker_join(fpWakeup wakeup, fpWorker worker, eTaskPrio prio, void *param)
{
    if (tasker_data._n_task < MAX_TASKS)
    {
        if ((wakeup != NULL) && (worker != NULL) && (prio > eTaskPrio_Base))
        {
            tasker_data._tasks[tasker_data._n_task]._wakeup = wakeup;
            tasker_data._tasks[tasker_data._n_task]._worker = worker;
            tasker_data._tasks[tasker_data._n_task]._param  = param;
            tasker_data._tasks[tasker_data._n_task]._prio   = prio;
            tasker_data._tasks[tasker_data._n_task]._dyn_prio = prio;
            tasker_data._tasks[tasker_data._n_task]._n_yield = 0;
            tasker_data._n_task++;
            return true;
        }
    }
    return false;
}

void tasker_exec(void)
{
    sTaskData   *choose = &tasker_data._tasks[0];   // idler task
    sTaskData   *reserve = NULL;
    volatile uint32_t   now_us = hal_get_micros();

    for (int8_t id = 1; id < tasker_data._n_task; id++)
    {
        reserve = &tasker_data._tasks[id];
        if (reserve->_wakeup(now_us, reserve->_param))
        {
            if (reserve->_dyn_prio > choose->_dyn_prio)
            {
                if ((choose->_prio != eTaskPrio_Base) && (++choose->_n_yield > MAX_YIELD))
                {
                    choose->_dyn_prio++;
                    choose->_n_yield = 0;
                }
                choose = reserve;
            }
        }
    }
    choose->_worker(now_us, choose->_param);
    choose->_dyn_prio = choose->_prio;
    choose->_n_yield = 0;
}

int8_t tasker_idling(void)
{
    return idler_data._exec_rate;
}

/* end of file ****************************************************************************************************** */
