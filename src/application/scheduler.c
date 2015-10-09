#include "compiler.h"
#include "sleep.h"

#include "scheduler.h"
#include "crc.h"

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

static hrtime_t range_start_time = { -1, -1}; // the most important value
static hrtime_t const range_period = RANGE_PERIOD;
int MyRangeTurnShouldStart(void)
{
	int ret = HRTimeBeyond(&range_start_time);
	return ret;
}

int MyRangeToAnchorShouldStart(unsigned anchor_no)
{
	int ret = 1;
	if (anchor_no) {
		hrtime_t t = range_start_time;
		HRTimeNAdd(&t, anchor_no, &range_period);
		ret = HRTimeBeyond(&t);
	}
	return ret;

}

static unsigned my_tag_no;
void SchedulerInit(unsigned _my_tag_no)
{
	if(SYS_TICK_COUNT != SysTick->LOAD + 1) {
		UartSend("Unmatched SYS CLOCK\r\n");
		while(1);
	}
	my_tag_no = _my_tag_no;
	HRTimeGetCurrent(&range_start_time);
	HRTimeNAdd(&range_start_time, 2*MAX_ANCHOR*MAX_TAG, &range_period);
}

void RangeProcessingDetected(unsigned tag_no, unsigned anchor_no, unsigned flag, enum instanceModes mode)
{
	if (my_tag_no >= MAX_TAG) {
		// a pure listener.
		hrtime_t t;
		HRTimeGetCurrent(&t);
		UartPrint("%02X %1X %1X %08d %08d\r\n", flag, tag_no, anchor_no, t.ms, t.tick);
		return;
	}

	if (mode != LISTENER) {
		return;
	}
	if (flag == RTLS_DEMO_MSG_ANCH_TOFR) {
		hrtime_t t;
		unsigned diff_tag, diff_range;
		HRTimeGetCurrent(&t);
		if (tag_no == my_tag_no || tag_no >= MAX_TAG || anchor_no >= MAX_ANCHOR) {
			// WTF
			return;
		}
		if (my_tag_no > tag_no) {
			diff_tag = my_tag_no - tag_no;
		} else {
			diff_tag = my_tag_no + MAX_TAG - tag_no;
		}
		diff_range = diff_tag * MAX_ANCHOR - ( (anchor_no & 0xFFU) + 1); // (anchor_no & 0xFFU) means member ID
		HRTimeNAdd(&t, diff_range, &range_period);
		range_start_time = t;
		// a range finished, which between tag_no and anchor_no
	}
}

static unsigned char crc;
static unsigned report_no;
void MyRangeProcessingRoundStarted(void)
{
	crc8_init(crc);
}

void ReportRangeResult(unsigned tag_no, unsigned anchor_no, unsigned dist)
{
	unsigned short anc = anchor_no;
	CRC8_16(crc, anc);
	CRC8_32(crc, dist);
	UartPrint("%04X %08X;", anchor_no, dist);
}

void MyRangeProcessingRoundFinished(void)
{
	CRC8_32(crc, report_no);
	UartPrint("%08X %02X\r\n", report_no++, crc);
	HRTimeNAdd(&range_start_time, MAX_ANCHOR* MAX_TAG, &range_period);
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
