#include "tim.h"
#include "bsp_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
extern QueueHandle_t QueueHandle_Buzzer;
TIM_HandleTypeDef *Buzzer_Tim;
uint32_t Buzzer_Channel;

#define BUZZER_PWM_DUTY_PERCENT 50U

static uint16_t buzzer_scale_pwm(uint16_t pwm);

void Buzzer_Init(TIM_HandleTypeDef *BuzzerTim_, uint32_t Channel){
    Buzzer_Tim = BuzzerTim_;
    Buzzer_Channel = Channel;
    HAL_TIM_PWM_Start(Buzzer_Tim, Channel);
}
void Buzzer(void (*Bzfun)(void), uint32_t ISR){
	if(ISR){
	BaseType_t dummy;
	xQueueSendFromISR(QueueHandle_Buzzer, &Bzfun, &dummy);
	}
	else{
        void (*pfun)(void);
        if(xQueuePeek(QueueHandle_Buzzer, &pfun, 0) == pdTRUE){
            if(pfun != Bzfun){
                goto addthisfun;
            }
        }
        else{
            addthisfun:
            xQueueSend(QueueHandle_Buzzer, &Bzfun, 0);
        }
	}
}
void Bix1(void){
	static int promising_young[]=
    {
		H7, 40, Z0, 200
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void Buzzer_OverShot(void){
	static int promising_young[]=
    {
		H7, 30, Z0, 17, H7, 30, Z0, 17,H7, 30, Z0, 10
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void Buzzer_Warning(void){
	static int promising_young[]=
    {
		H7, 60, Z0, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void CapOffline(void){
	static int promising_young[]=
    {
		H7, 60, Z0, 35, L7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void SetlengthExit(void){
	static int promising_young[]=
    {
		L7, 60, Z0, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void BuzeerDownExit(void){
	static int promising_young[]=
    {
		L2, 60, M2, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void BuzeerDownEntry(void){
	static int promising_young[]=
    {
		H7, 60, M2, 35, L7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void BuzeerInit(void){
	static int promising_young[]=
    {
		H6, 45, Z0, 15, H4, 40, Z0, 15
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void TakeOff(void){
	static int promising_young[]=
    {
		M2, 45, M4, 35, M5, 20, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {	
		
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
void Off(void){
	static int promising_young[]=
    {
		L5, 50, L2, 35, L1, 10, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(promising_young[2*i], 600);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}
/**
* NAME: void promising_young(void)
* FUNCTION : promising_young
*/
void promising_young(void)
{
    static int promising_young[]=
    {
        M5,50,M6,50,H1,50,Z0,10,H2,50, Z0,10,H2,50,Z0,1,H2,50, Z0,1, H1,50, Z0,1,H2,50, Z0,15, H1,50, Z0,15,H3,50,  Z0,15,Z0,50,
        M7,40,Z0,10,M7,40,Z0,10,M7,40,Z0,10,M7,75,Z0,15,M7,50, Z0,25, M6,25, Z0,25, H1,25,Z0,25

    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
}

/**
* NAME: void Solitary_brave(void)
* FUNCTION : Solitary_brave
*/
void Solitary_brave(void)
{
	static int16_t solitary_brave[]=
	{
		M6,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10,	//爱你孤身走暗巷
		H1,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10, 	//爱你不跪的模样
		H1,50,H2,50,H3,50,H2,50,H3,50,H2,50,H3,100,H3,50,H3,50,H2,50,H3,100,H5,100,H3,100,Z0,10 //爱你对峙过绝望不肯哭一场
		
	};
	int length = sizeof(solitary_brave)/sizeof(solitary_brave[0]);
	for(uint8_t i=0;i<(length/2);i++)
	{
		buzzer_on(solitary_brave[i*2],500);
		vTaskDelay(5*solitary_brave[i*2+1]);
	}
}

/**
* NAME: void Wind_Rises(void)
* FUNCTION : Wind_Rises
*/
void Wind_Rises(void)
{
	static int wind_rise[]=
	{	//这一路上走走停停 顺着少年漂流的痕迹
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,25,M3,25,M2,25,M1,25,L6,100,Z0,10,
		//迈出车站的前一刻 竟有些犹豫
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,M2,100,Z0,50,
		//不仅笑着这近乡情怯 仍无法避免
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,L6,100,Z0,10,
		//而长野的天 依旧那么暖 风吹起了从前
		M3,25,M2,25,M1,25,M2,25,M1,100, M3,25,M2,25,M1,25,M2,25,M1,50,M1,50, M5,25,M3,25,M2,25,M1,25,M2,25,M1,100,M1,150,Z0,30,
//		//从前初识这世间 万般留恋 看着天边似在眼前 也甘愿赴汤蹈火去走它一遍
		M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,M5,25,M6,25,M6,25,M5,25,M6,25,M5,25,M6,50,M5,25,M2,25,M2,25,M5,50,M5,50,M3,100,M3,100,Z0,25,
//		//如今走过这世间 万般留恋 翻过岁月不同侧脸 措不及防闯入你的笑颜
		M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,H3,25,H3,25,H3,50,M5,50,M6,50,H3,25,H3,25,H3,25,M5,50,M6,25,M6,100,M6,100,M6,100,Z0,25,
//		//我曾难自拔与世界之大 也沉溺于其中梦话
		H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,25,H3,25,H3,50, H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,25,H3,50,H3,50,
//		//不得真假 不做挣扎 不惧笑话
		H2,50,H1,25,M6,25,M6,25,H1,50, M6,25,H2,25,H1,25,M6,50,M6,25,H1,50,H1,50, H3,100,H3,25,H4,25,H3,50,H3,25,H2,50,H2,50,Z0,25,
//		//我曾将青春翻涌成她 也曾指尖弹出盛夏 心之所动且就随缘去吧
		H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,50, H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,100,H1,100,Z0,10,
//		//逆着光行走任风吹雨打吧
		M6,25,H3,50,H3,50,H2,50,H1,25,M6,25,M6,25,H3,50,H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,50,H1,100,Z0,10,
	};
	int length = sizeof(wind_rise)/sizeof(wind_rise[0]);
	
	for(int i=0;i<(length/2);i++)
	{
		buzzer_on(wind_rise[i*2],260);
		vTaskDelay(wind_rise[i*2+1]*5);
	}
}
static uint16_t buzzer_scale_pwm(uint16_t pwm)
{
    uint32_t pwm_max;

    if (pwm == 0U) {
        return 0U;
    }

    pwm_max = __HAL_TIM_GET_AUTORELOAD(Buzzer_Tim);
    if (pwm_max == 0U) {
        return pwm;
    }

    return (uint16_t)((pwm_max * BUZZER_PWM_DUTY_PERCENT) / 100U);
}

void buzzer_on(uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(Buzzer_Tim, psc);
    __HAL_TIM_SetCompare(Buzzer_Tim, Buzzer_Channel, buzzer_scale_pwm(pwm));

}

void buzzer_off(void)
{
    __HAL_TIM_SetCompare(Buzzer_Tim, Buzzer_Channel, 0);
}
