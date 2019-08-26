/**
 * *********************************************************************************************************************
 * @mainpage    RollingFish
 * @author      imindude@gmail.com
 * *********************************************************************************************************************
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ****************************************************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

/* ****************************************************************************************************************** */

enum eTaskPrio
{
    eTaskPrio_Base,     // do not use this (internal use only)
    eTaskPrio_Low,
    eTaskPrio_Mid,
    eTaskPrio_High,
    eTaskPrio_N
};
typedef enum eTaskPrio      eTaskPrio;

typedef bool    (*fpWakeup)(uint32_t now_us, void *param);
typedef void    (*fpWorker)(uint32_t now_us, void *param);

/* ****************************************************************************************************************** */

bool    tasker_init(void);
bool    tasker_join(fpWakeup wakeup, fpWorker worker, eTaskPrio prio, void *param);
void    tasker_exec(void);
int8_t  tasker_idling(void);

/* ****************************************************************************************************************** */

#ifdef __cplusplus
}
#endif

/* end of file ****************************************************************************************************** */
