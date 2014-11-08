#ifndef SONAR
#define SONAR

#include "stm32f10x.h"

typedef struct 
{
	uint16_t ltime;		//echo fly time(last)
	uint16_t ctime;		//echo fly time(current)
	uint16_t beggin;	//echo beggin time
	uint16_t end;		//echo end time
	uint16_t dis;		//echo fly time convert to distance
	float    vec;		//used in VFH method
	uint8_t  echo;		//flag to record echo state(beggin\end)

	int16_t  theta;		//angle

	uint8_t  fcnt; 		//filter count
}sonar;
extern sonar s[16];

typedef struct				    //定义下位机上传数据帧
{
	int8_t buffer[15];	   		//接收下位机数据
	int8_t cnt;			   	    //数据长度
	int8_t finish;				//帧结束标志
	int8_t updated;
}control;
extern control RxCtl;
extern control TxCtl;

//typedef struct					//定义vff结构体
//{
//	uint16_t dis[13];			//距离数据
//	uint16_t repul[13];			//产生的斥力大小
//
//	uint16_t resultant;			//合力大小
//}VFF;
//
//extern VFF vff;

#endif
