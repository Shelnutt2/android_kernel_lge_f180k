#include "tcc353x_common.h"
#include "tcc353x_api.h"
#include "tcpal_os.h"
#include "tcc353x_user_defines.h"

I08U Tcc353xStreamData[TCC353X_STREAM_BUFFER_SIZE + 188];
I32U Tcc353xStreamRp[4] = {0,0,0,0};
I32U Tcc353xStreamWp[4] = {0,0,0,0};
I32U Tcc353xStreamBufferedSize[4] = {0,0,0,0};
I32U Tcc353xStreamFlushFlag[4] = {0,0,0,0};


I32U *pTcc353xStreamSema = NULL;
TcpalSemaphore_t Tcc353xStreamSema;

/* for overflow test */
#define _DBG_CHK_OVERFLOW_CNT_
I32U gOverflowcnt = 0;
I32U gDbgIsrCnt = 0;

void Tcc353xStreamBufferInit(I32S _moduleIndex)
{
	if(pTcc353xStreamSema==NULL)	{
		TcpalCreateSemaphore(&Tcc353xStreamSema,
				     (I08S *) "StreamSemaphore", 1);
		pTcc353xStreamSema = &Tcc353xStreamSema;
	}

	Tcc353xStreamRp[_moduleIndex] = 0;
	Tcc353xStreamWp[_moduleIndex] = 0;
	Tcc353xStreamBufferedSize[_moduleIndex] = 0;
	Tcc353xStreamFlushFlag[_moduleIndex] = 0;
 }

void Tcc353xStreamBufferClose(I32S _moduleIndex)
{
	TcpalDeleteSemaphore(&Tcc353xStreamSema);
	pTcc353xStreamSema = NULL;

	Tcc353xStreamRp[_moduleIndex] = 0;
	Tcc353xStreamWp[_moduleIndex] = 0;
	Tcc353xStreamBufferedSize[_moduleIndex] = 0;
	Tcc353xStreamFlushFlag[_moduleIndex] = 0;
}

void Tcc353xStreamBufferReset(I32S _moduleIndex)
{
	TcpalSemaphoreLock(&Tcc353xStreamSema);
	Tcc353xStreamRp[_moduleIndex] = 0;
	Tcc353xStreamWp[_moduleIndex] = 0;
	Tcc353xStreamBufferedSize[_moduleIndex] = 0;
	Tcc353xStreamFlushFlag[_moduleIndex] = 0;
	TcpalSemaphoreUnLock(&Tcc353xStreamSema);
}

void Tcc353xStreamBufferFlush(I32S _moduleIndex)
{
	TcpalPrintStatus((I08S *) "[TCC353X] StreamBufferFlushing\n");
	TcpalSemaphoreLock(&Tcc353xStreamSema);
	Tcc353xStreamRp[_moduleIndex] = 0;
	Tcc353xStreamWp[_moduleIndex] = 0;
	Tcc353xStreamBufferedSize[_moduleIndex] = 0;
	Tcc353xStreamFlushFlag[_moduleIndex] = 1;
	TcpalSemaphoreUnLock(&Tcc353xStreamSema);
}

I32U Tcc353xGetStreamBuffer(I32S _moduleIndex, I08U * _buff, I32U _size)
{
	I32U tsSize = 0;
	I32U totalSize = 0;
	I32U rp;
	I32U wp;
	I32U nextRp;

	TcpalSemaphoreLock(&Tcc353xStreamSema);
	rp = Tcc353xStreamRp[_moduleIndex];
	wp = Tcc353xStreamWp[_moduleIndex];
	Tcc353xStreamFlushFlag[_moduleIndex] = 0;
	TcpalSemaphoreUnLock(&Tcc353xStreamSema);

	if(rp > wp)
		totalSize = wp + (TCC353X_STREAM_BUFFER_SIZE - rp);
	else if (rp < wp)
		totalSize = wp - rp;
	else 
		totalSize = 0;

	if(_size > totalSize)
		tsSize = totalSize;
	else
		tsSize = _size;
	tsSize = tsSize - (tsSize%188);

	if (tsSize<188) {
		return 0;
	}

	nextRp = ((rp+tsSize)%TCC353X_STREAM_BUFFER_SIZE);
	
	if(rp+tsSize>TCC353X_STREAM_BUFFER_SIZE) {	/* read twice */
		I32U first;
		I32U remain;
		first = TCC353X_STREAM_BUFFER_SIZE - rp;
		remain = nextRp;

		TcpalMemcpy (&_buff[0], &Tcc353xStreamData[rp], first);
		TcpalMemcpy (&_buff[first], &Tcc353xStreamData[0], nextRp);
	} else {
		TcpalMemcpy (&_buff[0], &Tcc353xStreamData[rp], tsSize);
	}

	TcpalSemaphoreLock(&Tcc353xStreamSema);
	if(Tcc353xStreamFlushFlag[_moduleIndex]!=0) {
		/* no update read pointer & no push data */
		Tcc353xStreamFlushFlag[_moduleIndex] = 0;
		tsSize = 0;
	} else {
		Tcc353xStreamRp[_moduleIndex] = nextRp;
		if(Tcc353xStreamBufferedSize[0]>=tsSize)
			Tcc353xStreamBufferedSize[0] -= tsSize;
	}
	TcpalSemaphoreUnLock(&Tcc353xStreamSema);
	return tsSize;
}

I32U Tcc353xInterruptProcess(void)
{
	I32U ret = 0;
	I08U irqStatus = 0;
	I32S moduleIndex = 0;
	I32U totalSize = 0;

	/* Read BB Interrupt Status */
	Tcc353xApiGetIrqStatus(moduleIndex, &irqStatus);

	/* Stream Interrupt */
	if (irqStatus&0x01) {
		TcpalPrintErr((I08S *)
			      "[TCC353X] FIFO overflow[0x%02X] flush!!!\n",
			      irqStatus);
		/* Tcc353x IRQ Clear */
		Tcc353xApiIrqClear(moduleIndex, irqStatus);
		Tcc353xApiInterruptBuffClr(moduleIndex);
		gOverflowcnt ++;
		ret = 0;
	} else {
		/* Tcc353x IRQ Clear */
		Tcc353xApiIrqClear(moduleIndex, irqStatus);
		Tcc353xApiGetFifoStatus(moduleIndex, &totalSize);
		ret = totalSize;
	}

	gDbgIsrCnt++;

	if(gDbgIsrCnt>40) {
		gDbgIsrCnt = 0;
#ifdef _DBG_CHK_OVERFLOW_CNT_
		TcpalPrintStatus((I08S *)
				  "[TCC353X] CurrOverflow Cnt %d\n",
				  gOverflowcnt);
#endif
	}
	
	return ret;
}

void Tcc353xInterruptGetStream(I32U _fifoSize)
{
	I32S moduleIndex = 0;
	I32U totalSize = 0;
	I32U wp;

	totalSize = _fifoSize - (_fifoSize%188);

	if(totalSize>=188) {
		I32U nextwp;

		if(Tcc353xStreamBufferedSize[moduleIndex]+totalSize > TCC353X_STREAM_BUFFER_SIZE)
			Tcc353xStreamBufferFlush (moduleIndex);

		TcpalSemaphoreLock(&Tcc353xStreamSema);
		wp = Tcc353xStreamWp[moduleIndex];
		TcpalSemaphoreUnLock(&Tcc353xStreamSema);

		nextwp = ((wp+totalSize)%TCC353X_STREAM_BUFFER_SIZE);
		
		if(wp+totalSize>TCC353X_STREAM_BUFFER_SIZE) {	/* read twice */
			I32U first;
			first = TCC353X_STREAM_BUFFER_SIZE - wp;
			Tcc353xApiStreamRead(moduleIndex,
					     &Tcc353xStreamData[wp],
					     first);
			Tcc353xApiStreamRead(moduleIndex,
					     &Tcc353xStreamData[0],
					     nextwp);
		} else {
			Tcc353xApiStreamRead(moduleIndex,
					     &Tcc353xStreamData[wp],
					     totalSize);
		}

		TcpalSemaphoreLock(&Tcc353xStreamSema);
		Tcc353xStreamWp[moduleIndex] = nextwp;
		Tcc353xStreamBufferedSize[moduleIndex] += totalSize;
		TcpalSemaphoreUnLock(&Tcc353xStreamSema);

		if(Tcc353xStreamData[wp]!=0x47) {
			TcpalPrintErr((I08S *) "[TCC353X] SyncByte Error! [0x%02x]\n",
				     Tcc353xStreamData[wp]);
			TcpalPrintErr((I08S *) "[TCC353X] Buff Flush for SyncByte matching\n");
			Tcc353xApiInterruptBuffClr(moduleIndex);
		}
	}
}

