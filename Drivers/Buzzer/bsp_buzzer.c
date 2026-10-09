#include "tim.h"
#include "bsp_buzzer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "bsp_pwm.h"

#include <math.h>
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

}
void Bix1(void){
	static int promising_young[]=
    {
		H7, 40, Z0, 200
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void Buzzer_OverShot(void){
	static int promising_young[]=
    {
		H7, 30, Z0, 17, H7, 30, Z0, 17,H7, 30, Z0, 10
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void Buzzer_Warning(void){
	static int promising_young[]=
    {
		H7, 60, Z0, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void CapOffline(void){
	static int promising_young[]=
    {
		H7, 60, Z0, 35, L7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void SetlengthExit(void){
	static int promising_young[]=
    {
		L7, 60, Z0, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void BuzeerDownExit(void){
	static int promising_young[]=
    {
		L2, 60, M2, 35, H7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void BuzeerDownEntry(void){
	static int promising_young[]=
    {
		H7, 60, M2, 35, L7, 50, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void BuzeerInit(void){
	static int promising_young[]=
    {
		H6, 45, Z0, 15, H4, 40, Z0, 15
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void TakeOff(void){
	static int promising_young[]=
    {
		M2, 45, M4, 35, M5, 20, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {	
		
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
    }
}
void Off(void){
	static int promising_young[]=
    {
		L5, 50, L2, 35, L1, 10, Z0, 30
    };

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(promising_young[2*i], 600);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
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

    int length = sizeof(promising_young)/sizeof(promising_young[0]);//�������鳤��
    for(int i=0; i<(length/2); i++)//ȡ��������
    {
        buzzer_on(promising_young[2*i], 200);
        vTaskDelay(5*promising_young[2*i+1]);//������ʱ�䶼����5��һ��Ϊ500΢�룬��ֵ"5"�ɵ�����ֻ�ǲ��ŵ������������ѣ��е���������Ƶ����ͺ���
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
		M6,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10,	//��������߰���
		H1,50,M7,50,H1,50,H2,50,M7,50,H1,50,H1,100,Z0,10, 	//���㲻���ģ��
		H1,50,H2,50,H3,50,H2,50,H3,50,H2,50,H3,100,H3,50,H3,50,H2,50,H3,100,H5,100,H3,100,Z0,10 //������Ź��������Ͽ�һ��
		
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
	{	//��һ·������ͣͣ ˳������Ư���ĺۼ�
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,25,M3,25,M2,25,M1,25,L6,100,Z0,10,
		//������վ��ǰһ�� ����Щ��ԥ
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,M2,100,Z0,50,
		//����Ц����������� ���޷�����
		M2,50,M2,50,M1,25,M2,50,M2,50,M1,25,M2,50,M3,50,M5,50,M3,50, M2,50,M2,50,M3,25,M2,50,M1,50,L6,100,Z0,10,
		//����Ұ���� ������ôů �紵���˴�ǰ
		M3,25,M2,25,M1,25,M2,25,M1,100, M3,25,M2,25,M1,25,M2,25,M1,50,M1,50, M5,25,M3,25,M2,25,M1,25,M2,25,M1,100,M1,150,Z0,30,
//		//��ǰ��ʶ������ ������� �������������ǰ Ҳ��Ը��������ȥ����һ��
		M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,M5,25,M6,25,M6,25,M5,25,M6,25,M5,25,M6,50,M5,25,M2,25,M2,25,M5,50,M5,50,M3,100,M3,100,Z0,25,
//		//����߹������� ������� �������²�ͬ���� �벻�����������Ц��
		M1,50,M2,50,M3,50,M1,50,M6,50,M5,25,M6,25,M6,50,M6,50, M1,25,M7,50,M6,25,M7,25,M7,100,Z0,5, M7,50,M6,25,M7,25,M7,50,M3,50,H1,25,H2,25,H1,25,M7,25,M6,50,M5,50, M6,50,H3,25,H3,25,H3,50,M5,50,M6,50,H3,25,H3,25,H3,25,M5,50,M6,25,M6,100,M6,100,M6,100,Z0,25,
//		//�������԰�������֮�� Ҳ�����������λ�
		H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,25,H3,25,H3,50, H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,25,H3,50,H3,50,
//		//������� �������� ����Ц��
		H2,50,H1,25,M6,25,M6,25,H1,50, M6,25,H2,25,H1,25,M6,50,M6,25,H1,50,H1,50, H3,100,H3,25,H4,25,H3,50,H3,25,H2,50,H2,50,Z0,25,
//		//�������ഺ��ӿ���� Ҳ��ָ�ⵯ��ʢ�� ��֮�����Ҿ���Եȥ��
		H1,50,H2,50,H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H2,50, H3,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H6,25,H5,25,H5,50,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H3,50,H3,50, H2,50,H1,25,M6,25,M6,25,H1,50,H1,50,H1,100,H1,100,Z0,10,
//		//���Ź������η紵����
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

void Buzzer_PlayPitch(uint8_t pitch)
{
    if (pitch == 0U) {
        buzzer_off();
        return;
    }

    float frequency = BUZZER_A4_FREQUENCY_HZ * powf(2.0f, ((float)pitch - 69.0f) / 12.0f);
    BSP_PWM_SetFrequency(Buzzer_Tim, BUZZER_TIMER_CLOCK_HZ, (uint32_t)(frequency + 0.5f));
    BSP_PWM_SetDutyPermille(Buzzer_Tim, Buzzer_Channel, 500U);
}

void Buzzer_PlayNotesV2(const MusicNoteV2_t *notes, uint32_t count)
{
    for (uint32_t i = 0U; i < count; ++i) {
        Buzzer_PlayPitch(notes[i].pitch);
        vTaskDelay(pdMS_TO_TICKS(notes[i].duration_ms));
    }
    buzzer_off();
}
