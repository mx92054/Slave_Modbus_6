#ifndef __SPD_COMM__
#define __SPD_COMM__

#include "stm32f4xx.h"

#define SPD1_QUEUE_LEN 10

//速度計算值隊列
typedef struct _tag_speed_queue
{
    u16 ptr_head;
    u16 ptr_tail;
    long lSum_ang;
    long lSum_tim;
    short queue_ang[SPD1_QUEUE_LEN];
    short queue_tim[SPD1_QUEUE_LEN];
} SpeedValueQueue;

void SpdQueueInit(SpeedValueQueue *svq);
void SpdQueueIn(SpeedValueQueue *svq, short ang, short tim);
short SpdQueueAvgVal(SpeedValueQueue *svq);

#endif
/*------------------end of file------------------------*/
