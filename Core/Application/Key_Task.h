#ifndef __KEY_SCAN_H__
#define __KEY_SCAN_H__
#include "main.h"
extern uint16_t key_mode;
void Key_task(void);
void Key_Task(void const *argument);
uint16_t Key_GetMode(void);
#endif
