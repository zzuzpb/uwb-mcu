#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "port.h"
#include "instance.h"

#define MAX_TAG				(4)
#define MAX_ANCHOR			(8)

#define RANGE_PERIOD { 3, 0 } // 3.0ms for each ranging progress.

void SchedulerInit(enum instanceModes _instance_mode, unsigned _my_tag_no);

void RangeProcessingDetected(unsigned tag_no, unsigned anchor_no, unsigned flag);
void MyRangeProcessingRoundFinished(void);

#endif
