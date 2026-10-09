/*
 *                        _oo0oo_
 *                       o8888888o
 *                       88" . "88
 *                       (| -_- |)
 *                       0\  =  /0
 *                     ___/`---'\___
 *                   .' \\|     |// '.
 *                  / \\|||  :  |||// \
 *                 / _||||| -:- |||||- \
 *                |   | \\\  - /// |   |
 *                | \_|  ''\---/''  |_/ |
 *                \  .-\__  '-'  ___/-. /
 *              ___'. .'  /--.--\  `. .'___
 *           ."" '<  `.___\_<|>_/___.' >' "".
 *          | | :  `- \`.;`\ _ /`;.`/ - ` : | |
 *          \  \ `_.   \_ __\ /__ _/   .-` /  /
 *      =====`-.____`.___ \_____/___.-`___.-'=====
 *                        `=---='
 * 
 * 
 *      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * 
 *            佛祖保佑     永不宕机     永无BUG
 * 
 *        佛曰:  
 *                写字楼里写字间，写字间里程序员；  
 *                程序人员写程序，又拿程序换酒钱。  
 *                酒醒只在网上坐，酒醉还来网下眠；  
 *                酒醉酒醒日复日，网上网下年复年。  
 *                但愿老死电脑间，不愿鞠躬老板前；  
 *                奔驰宝马贵者趣，公交自行程序员。  
 *                别人笑我忒疯癫，我笑自己命太贱；  
 *                不见满街漂亮妹，哪个归得程序员？
 */

#include "bsp_dwt.h"
#include "task_define.h"
TaskHandle_t MonitorTaskHandle;
TaskHandle_t ReceiveTaskHandle;
TaskHandle_t TransmitTaskHandle;

/*外设及freertos相关初始化所需的其他源文件中的全局变量*/
void InitTask(void const * argument)
{
	taskENTER_CRITICAL();
	DWT_Init(480);
	StartupNotice();//奏乐！
	xTaskCreate(MonitorTask,        "MonitorTask_",      128,  NULL,  4,  &MonitorTaskHandle      );
	xTaskCreate(RemoteRecTask,      "RemoteRecTask_",    256,  NULL,  7,  &remoteRecTaskHandle    );
	xTaskCreate(StateMachineTask,   "StateMachineTask_", 2048, NULL,  6,  &stateMachineTaskHandle );	
	xTaskCreate(DecisionTask, 	    "DecisionTask_", 	 512,  NULL,  5,  &decisionTaskHandle     );
	PeripheralRecEnable();   	
	vTaskDelete(NULL);   
	taskEXIT_CRITICAL();   
}


