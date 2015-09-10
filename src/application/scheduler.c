#include "compiler.h"
#include "sleep.h"

#include "scheduler.h"

typedef struct {
	unsigned ms;   //
	unsigned tick; // sys-tick unit, 1/72MHz
}hrtime_t;

#define SYSCLK_FREQ_72MHz 72000000
#define SYS_TICK_COUNT (SYSCLK_FREQ_72MHz / CLOCKS_PER_SEC) // that value is important to HR timer.

extern __IO unsigned long time32_incr; // system time in ms from booting.
#pragma GCC optimize ("O3")
static void inline HRTimeGetCurrent(hrtime_t *t)
{
	unsigned ms;
	do {
		t->ms   = time32_incr;
		t->tick = SYS_TICK_COUNT - SysTick->VAL;
		ms      = time32_incr;
	} while (t->ms != ms);
}

#pragma GCC optimize ("O3")
static int inline HRTimeBeyond(const hrtime_t *t)
{
	hrtime_t curr;
	int ret;
	HRTimeGetCurrent(&curr);
	ret = curr.ms > t->ms || (curr.ms == t->ms && curr.tick > t->tick);
	return ret;
}

#pragma GCC optimize ("O3")
static void inline HRTimeNormal(hrtime_t *t)
{
	while (t->tick >= SYS_TICK_COUNT) {
		t->tick -= SYS_TICK_COUNT;
		t->ms   += 1;
	}
}

#pragma GCC optimize ("O3")
static inline void HRTimeAdd(hrtime_t *t, const hrtime_t *deta)
{
	t->ms   += deta->ms;
	t->tick += deta->tick;
	HRTimeNormal(t);
}

#pragma GCC optimize ("O3")
static inline void HRTimeNAdd(hrtime_t *t, unsigned n, const hrtime_t *deta)
{
	hrtime_t dt = { n*deta->ms, n*deta->tick, };
	HRTimeAdd(t, &dt);
}

static enum instanceModes instance_mode;

static hrtime_t range_start_time = { -1, -1}; // the most important value
static hrtime_t const range_period = RANGE_PERIOD;
int MyRangeTurnShouldStart(void)
{
	int ret = HRTimeBeyond(&range_start_time);
	return ret;
}

static unsigned my_tag_no = 0xFFFFFFFFU;
void SchedulerInit(enum instanceModes _instance_mode, unsigned _my_tag_no)
{
	if(SYS_TICK_COUNT != SysTick->LOAD + 1) {
		UartSend("Unmatched SYS CLOCK\r\n");
		while(1);
	}
	instance_mode = _instance_mode;
	my_tag_no = _my_tag_no;
	HRTimeGetCurrent(&range_start_time);
	HRTimeNAdd(&range_start_time, 2*MAX_ANCHOR*MAX_TAG, &range_period);
}

void RangeProcessingDetected(unsigned tag_no, unsigned anchor_no, unsigned flag)
{

	if (instance_mode == ANCHOR || instance_mode == TAG && tag_no == my_tag_no) {
		return;
	}
	//return;
#if 1
	// following collect data arrived time for statistical analysis
	static struct {
		unsigned int code;
		unsigned int tag;
		unsigned int anchor;
		hrtime_t time;
	} code_list[1000];
	static int code_index = 0;

	if (code_index >= sizeof code_list / sizeof (code_list[0])) {
		int i;
		for (i = 0; i < code_index; i++) {
			UartPrint("%02X %1X %1X %08d %08d\r\n", code_list[i].code, code_list[i].tag, code_list[i].anchor, code_list[i].time.ms, code_list[i].time.tick);
		}
		code_index = 0;
	}

	code_list[code_index].code   = flag;
	code_list[code_index].anchor = anchor_no;
	code_list[code_index].tag    = tag_no;

	HRTimeGetCurrent(&code_list[code_index].time);
	code_index ++;
#endif
}
void MyRangeProcessingRoundFinished(void)
{
	HRTimeNAdd(&range_start_time, MAX_ANCHOR * MAX_TAG, &range_period);
}

int __weak usleep(useconds_t useconds)
{
	hrtime_t t, delay;
	HRTimeGetCurrent(&t);
	useconds_t tick = useconds * (SYSCLK_FREQ_72MHz/1000000);
	delay.ms   = tick / SYS_TICK_COUNT;
	delay.tick = tick % SYS_TICK_COUNT;
	HRTimeAdd(&t, &delay);
	while (!HRTimeBeyond(&t));
	return 0;
}
