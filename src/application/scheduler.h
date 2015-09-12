#ifndef __SCHEDULE_H__
#define __SCHEDULE_H__

#include "port.h"
#include "instance.h"

#define MAX_TAG				(4)
#define MAX_ANCHOR			(8)

#define RANGE_PERIOD { 2, 0 } // 3.0ms for each ranging progress.

void SchedulerInit(unsigned _my_tag_no);

int MyRangeTurnShouldStart(void);
int MyRangeToAnchorShouldStart(unsigned anchor_no);

void RangeProcessingDetected(unsigned tag_no, unsigned anchor_no, unsigned flag, enum instanceModes mode);
void MyRangeProcessingRoundStarted(void);
void MyRangeProcessingRoundFinished(void);

void ReportRangeResult(unsigned tag_no, unsigned anchor_no, unsigned dist);
#endif
