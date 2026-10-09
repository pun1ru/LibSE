#ifndef _JUDGERECEIVE_H_
#define _JUDGERECEIVE_H_

/** Include Header Files **/
#include "stdint.h"
#define REFEREE_DMA_SIZE (130)//一帧最长128字节（见通信协议），留两个字节防炸
#define DISTANCE_DMA_SIZE (49)
#undef REFEREE_DMA_SIZE
#define REFEREE_DMA_SIZE (256)

/*
Version:2025.07.21
修改日期：2025.07.21
修改人：Xu
注意：
*/

/*--------------CmdID(2-Byte)----------------*/
#define GAME_STATUS             0x0001          //比赛状态数据，1Hz 周期发送
#define GAME_RESULT             0x0002          //比赛结果数据，比赛结束后发送
#define ROBOT_HP                0x0003          //比赛机器人血量数据，1Hz周期发送
#define EVENT_DATA              0x0101          //场地事件数据，事件改变后发送
#define REFEREE_WARNING         0x0104          //裁判警告数据，警告发生后发送 
#define DART_INFO               0x0105          //飞镖发射口倒计时，1Hz 周期发送 
#define ROBOT_STATUS            0x0201          //机器人状态数据，10Hz 周期发送
#define POWER_HEAT_DATA         0x0202          //实时功率热量数据，50Hz 周期发送
#define ROBOT_POS               0x0203          //机器人位置数据，1Hz 发送
#define BUFF                    0x0204          //机器人增益数据，3Hz
#define HURT_DATA               0x0206          //伤害状态数据，伤害发生后发送
#define SHOOT_DATA              0x0207          //实时射击数据，子弹发射后发送
#define PROJECTILE_ALLOWANCE    0x0208          //子弹剩余发送数，英雄、步兵、哨兵、空中发送，10Hz周期发送
#define RFID_STATUS             0x0209          //机器人 RFID 状态，3Hz 周期发送
#define DART_CLIENT_CMD         0x020A          //飞镖机器人客户端指令数据，3Hz周期发送
#define GROUND_ROBOT_POS        0x020B          //地面机器人位置数据，发给哨兵，1Hz周期发送
#define RADAR_MARK_DATA         0x020C          //雷达标定进度数据，1Hz周期发送
#define SENTRY_INFO             0x020D          //哨兵自主决策信息同步，1Hz周期发送
#define RADAR_INFO              0x020E          //雷达自主决策信息同步，1Hz周期发送
#define ROBOT_INTERACTION_DATA  0x0301          //机器人间交互数据，发送方触发发送，上限30Hz
#define CUSTOM_ROBOT_DATA       0x0302          //自定义控制器交互数据接口，通过客户端触发发送，上限30Hz//
#define MAP_COMMAND             0x0303          //客户端小地图交互数据，触发发送
#define REMOTE_CONTROL          0x0304          //键盘、鼠标信息，通过图传串口发送，30Hz
#define MAP_ROBOT_DATA          0x0305          //小地图接收雷达数据，10Hz
#define CUSTOM_CLIENT_DATA      0x0306          //自定义控制器和选手端交互数据，30Hz
#define MAP_DATA                0x0307          //小地图接收哨兵数据，1Hz
#define CUSTOM_INFO             0x0308          //小地图接收机器人数据，3Hz
#define ROBOT_CUSTOM_DATA       0x0309          //自定义控制器接收机器人数据，10Hz
/*--------------CmdID(2-Byte)----------------*/

#undef REMOTE_CONTROL
#define ROBOT_CUSTOM_CLIENT_DATA 0x0310
#define CLIENT_CUSTOM_ROBOT_DATA 0x0311

/*--------------DataSize----------------*/
#define GAME_STATUS_DATA_SIZE           (11)//0x0001
#define GAME_RESULT_DATA_SIZE           (1)//0x0002
#define ROBOT_HP_DATA_SIZE              (16)//0x0003, 2026 V1.2.0 detailed table
#define EVENTDATA_DATA_SIZE             (4) //0x0101
#define REFEREE_WARNING_DATA_SIZE       (3) //0x0104
#define DART_INFO_DATA_SIZE             (3) //0x0105
#define ROBOT_STATUS_DATA_SIZE          (13)//0x0201
#define POWER_HEAT_DATA_SIZE            (14)//0x0202
#define ROBOT_POS_DATA_SIZE             (12)//0x0203
#define BUFF_SIZE                       (8)//0x0204
#define ROBOT_HURT_DATA_SIZE            (1)//0x0206
#define SHOOTDATA_DATA_SIZE             (7)//0x0207
#define PROJECTILE_ALLOWANCE_DATA_SIZE  (8)//0x0208
#define RFID_STATUS_DATA_SIZE           (5)//0x0209
#define DART_CLIENT_CMD_DATA_SIZE       (6)//0x020A
#define GROUND_ROBOT_POS_DATA_SIZE      (40)//0x020B        
#define RADAR_MARK_DATA_SIZE            (2) //0x020C
#define SENTRY_INFO_DATA_SIZE           (6) //0x020D
#define RADAR_INFO_DATA_SIZE            (1) //0x020E
#define ROBOT_INTERACTIONDATA_DATA_SIZE (118)//0x0301
//0X0301子内容长度：
#define INTERACTION_LAYER_DELETE_DATA_SIZE   (2)  //0x0100
#define INTERACTION_FIGURE_DATA_SIZE         (15) //0x0101
#define INTERACTION_FIGURE_2_DATA_SIZE       (30) //0x0102
#define INTERACTION_FIGURE_3_DATA_SIZE       (75) //0x0103
#define INTERACTION_FIGURE_4_DATA_SIZE       (105)//0x0104
#define CILENT_CUSTOM_CHARATER_DATA_SIZE     (45) //0x0110
#define SENTRY_CMD_DATA_SIZE                 (4)  //0x0120
#define RADAR_CMD_DATA_SIZE                  (8)  //0x0121
//结束
#define CUSTOM_ROBOT_DATA_SIZE          (30)//0x0302 长度30以内自定,可改长度
#define MAP_COMMAND_DATA_SIZE           (12)//0x0303
#define MAP_ROBOT_DATA_SIZE             (48)//0x0305
#define CUSTOM_CLIENTDATA_DATA_SIZE     (8)//0x0306
#define MAP_DATA_DATA_SIZE              (105)//0x0307
#define CUSTOM_INFO_DATA_SIZE           (34)//0x0308
#define ROBOT_CUSTOM_DATA_SIZE          (30)//0x0309
#define ROBOT_CUSTOM_CLIENT_DATA_SIZE   (300)//0x0310
#define CLIENT_CUSTOM_ROBOT_DATA_SIZE   (30)//0x0311



#define INTERACTIVEHEADER_DATA_SIZE(n) (n + 9)
#define JUDGE_DATA_LENGTH(n) (n + 9)
/*--------------DataSize----------------*/

/*--------------偏移位置----------------*/
//接收数据
#define JUDGE_SOF_OFFSET (0)
#define JUDGE_DATALENGTH_OFFSET (1)
#define JUDGE_SEQ_OFFSET (3)
#define JUDGE_CRC8_OFFSET (4)
#define JUDGE_CMDID_OFFSET (5)
#define JUDGE_DATA_OFFSET (7)
#define JUDGE_CRC16_OFFSET(n) (n + JUDGE_DATA_OFFSET)
//发送数据
#define TRAMSINIT_LENGTH 128
#define TRAMSINIT_HEAD_OFFSET 0
#define TRAMSINIT_SENDID_OFFSET 2
#define TRAMSINIT_CLIENT_OFFSET 4
/*--------------偏移位置----------------*/

#pragma pack(1)
//1.	比赛机器人状态(0x0001) 1Hz
typedef struct
{
    /*
    0-3 bit：比赛类型
    • 1：RoboMaster 机甲大师赛；
    • 2：RoboMaster 机甲大师单项赛；
    • 3：ICRA RoboMaster 人工智能挑战赛
    • 4：RoboMaster 联盟赛3V3
    • 5：RoboMaster 联盟赛1V1
    */
    uint8_t game_type : 4;

    /*
    4-7 bit：当前比赛阶段
    • 0：未开始比赛；
    • 1：准备阶段；
    • 2：自检阶段；
    • 3：5s倒计时；
    • 4：对战中；
    • 5：比赛结算中
    */
    uint8_t game_progress : 4;

    /*
    当前阶段剩余时间，单位 s
    */
    uint16_t stage_remain_time;

    /*
    机器人接收到该指令的精确Unix时间，当机载端收到有效的NTP服务器授时后生效
    */
   uint64_t SyncTimeStamp;

} ext_game_status_t;

//2.比赛结果数据：0x0002。发送频率：比赛结束后发送
typedef struct
{
    /*0 平局 1 红方胜利 2 蓝方胜利*/
    uint8_t winner;
} ext_game_result_t;

//3. 机器人血量数据：0x0003
typedef struct
{
    //红1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为0
    /* 2026 V1.2.0 uses ally-only HP data: 8 x uint16_t = 16 bytes. */
    uint16_t ally_1_robot_HP;
    //红2 工程机器人血量
    uint16_t ally_2_robot_HP;
    //红3 步兵机器人血量
    uint16_t ally_3_robot_HP;
    //红4 步兵机器人血量
    uint16_t ally_4_robot_HP;
    //保留位
    uint16_t reserved;
    //红7 哨兵机器人血量
    uint16_t ally_7_robot_HP;
    //红方前哨站血量
    uint16_t ally_outpost_HP;
    //红方基地血量
    uint16_t ally_base_HP;
    //蓝1 英雄机器人血量
 #if 0
    uint16_t blue_1_robot_HP;
    //蓝2 工程机器人血量
    uint16_t blue_2_robot_HP;
    //蓝3 步兵机器人血量
    uint16_t blue_3_robot_HP;
    //蓝4 步兵机器人血量
    uint16_t blue_4_robot_HP;
    //保留位
    uint16_t reserved_2;
    //蓝7 哨兵机器人血量
    uint16_t blue_7_robot_HP;
    //蓝方前哨站血量
    uint16_t blue_outpost_HP;
    //蓝方基地血量
    uint16_t blue_base_HP;
#endif
} ext_game_robot_HP_t;

//5.场地事件数据：0x0101
typedef struct
{
      /* 2026 V1.3.0 位域修订：
    0：未占领/未激活
    1：已占领/已激活
    bit 0-2：
     bit 0：己方补给区的占领状态，1 为已占领
     bit 1：保留位
     bit 2：己方补给区的占领状态，1 为已占领（仅 RMUL 适用）
    bit 3-6：己方能量机关状态
     bit 3-4：己方小能量机关的激活状态，0为未激活，1为已激活，2为正在激活
     bit 5-6：己方大能量机关的激活状态，0为未激活，1为已激活，2为正在激活
    bit 7-8：己方中央高地的占领状态，1 为被己方占领，2 为被对方占领
    bit 9-10：己方梯形高地的占领状态，1 为已占领
    bit 11-19：对方飞镖最后一次击中己方前哨站或基地的时间（0-420，开局默认为0）
    bit 20-22：对方飞镖最后一次击中己方前哨站或基地的具体目标，开局默认为0，
     1为击中前哨站，2为击中基地固定目标，3为击中基地随机固定目标，
     4为击中基地随机移动目标，5为击中基地末端移动目标
    bit 23-24：中心增益点的占领状态，0为未被占领，1为被己方占领，
     2为被对方占领，3为被双方占领（仅RMUL适用）
    bit 25-26：己方堡垒增益点的占领状态，0为未被占领，1为被己方占领，
     2为被对方占领，3为被双方占领
    bit 27-28：己方前哨站增益点的占领状态，0为未被占领，1为被己方占领，
     2为被对方占领
    bit 29：己方基地增益点的占领状态，1 为已占领
    bit 30-31：保留位
    */
    uint32_t event_data;
} ext_event_data_t;

//7. 裁判警告信息：cmd_id (0x0104)。发送频率：裁判警告数据，己方判罚/判负时
//触发发送，其余时间以 1Hz 频率发送，发送范围：己方机器人。 
typedef struct 
{   
    /*
    己方最后一次受到判罚的等级：
 1：双方黄牌
 2：黄牌
 3：红牌
 4：判负
    */
    uint8_t level;
    /*
      己方最后一次受到判罚的违规机器人 ID。（如红 1 机器人 ID 为 1，蓝
      1 机器人 ID 为 101）
 判负和双方黄牌时，该值为 0
    */
    uint8_t offending_robot_id; 
	/*
      己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。(开局默认为0)
    */
	 uint8_t count;
} ext_referee_warning_t; 

//8. 飞镖状态信息：cmd_id (0x0105)。发送频率：1Hz 周期发送，发送范围：己方机器人。 
typedef struct 
{   
	//己方飞镖发射剩余时间，单位：秒
    uint8_t dart_remaining_time;   //15s 倒计时 
    // bit 0-2：
    // 最近一次己方飞镖击中的目标，开局默认为0，1 为击中前哨站，2 为击中
    // 基地固定目标，3 为击中基地随机固定目标，4 为击中基地随机移动目标，
    // 5 为击中基地末端移动目标
    // ...
    // bit 6-8：
    // 飞镖此时选定的击打目标，开局默认或未选定/选定前哨站时为0，选中基
    // 地固定目标为1，选中基地随机固定目标为2，选中基地随机移动目标为3，
    // 选中基地末端移动目标为4
    // bit 9-15：保留
	uint16_t dart_info;
} ext_dart_info_t;

//9.比赛机器人状态：0x0201。发送频率：10Hz
typedef struct
{
    //本机器人ID
    uint8_t robot_id;
    //机器人等级
    uint8_t robot_level;
    //机器人当前血量
    uint16_t current_HP;
    //机器人血量上限
    uint16_t maximum_HP;
    //机器人射击热量每秒冷却值
    uint16_t shooter_barrel_cooling_value;
    //机器人射击热量上限
    uint16_t shooter_barrel_heat_limit;
    //机器人底盘功率上限
    uint16_t chassis_power_limit;
    /*
    电源管理模块的输出情况：
     bit 0：gimbal 口输出，0 为无输出，1 为 24V 输出
     bit 1：chassis 口输出，0 为无输出，1 为24V 输出
     bit 2：shooter 口输出，0 为无输出，1 为24V 输出
    */
    uint8_t power_management_gimbal_output : 1;
    uint8_t power_management_chassis_output : 1;
    uint8_t power_management_shooter_output : 1;
} ext_robot_status_t;

//10.实时功率热量数据：0x0202。发送频率：50Hz
typedef struct
{
    //保留位
    uint16_t reserved_1;
    //保留位
    uint16_t reserved_2;
    //保留位
    float reserved_3;
    //缓冲能量（单位：J）
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
#if 0
    //17mm发射机构的射击热量
    uint16_t shooter_17mm_barrel_heat;
    //42mm发射机构的射击热量
    uint16_t shooter_42mm_barrel_heat;
#endif
} ext_power_heat_data_t;

//11.机器人位置：0x0203。发送频率：1Hz
typedef struct
{
    float x;   //位置 x 坐标，单位 m
    float y;   //位置 y 坐标，单位 m
    float angle;   //本机器人测速模块的朝向，单位：度。正北为 0 度
} ext_robot_pos_t;

//12. 机器人增益：0x0204。发送频率：3Hz
typedef struct
{
    //机器人回血增益（百分比，值为10 表示每秒恢复血量上限的10%）
    uint8_t recovery_buff;
    //机器人射击热量冷却倍率（直接值，值为5 表示5 倍冷却）
    uint16_t cooling_buff;
    //机器人防御增益（百分比，值为50 表示50%防御增益）
    uint8_t defence_buff;
    //机器人负防御增益（百分比，值为30 表示-30%防御增益）
    uint8_t vulnerability_buff;
    //机器人攻击增益（百分比，值为50 表示50%攻击增益）
    uint16_t attack_buff;
    /*
    bit 0-6：机器人剩余能量值反馈，以16进制标识机器人剩余能量值比例。
     机器人初始能量视为100%。
     bit 0：在剩余能量≥125%时为1，其余情况为0
     bit 1：在剩余能量≥100%时为1，其余情况为0
     bit 2：在剩余能量≥50%时为1，其余情况为0
     bit 3：在剩余能量≥30%时为1，其余情况为0
     bit 4：在剩余能量≥15%时为1，其余情况为0
     bit 5：在剩余能量≥5%时为1，其余情况为0
     bit 6：在剩余能量≥1%时为1，其余情况为0
    */
    uint8_t remaining_energy;
} ext_buff_t;

//14. 伤害状态：0x0206。发送频率：伤害发生后发送
typedef struct
{
    /*
    bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线
    时，该4 bit 组成的数值为装甲模块或测速模块的ID 编号；当其他原因导致
    扣血时，该数值为0
    */
    uint8_t armor_id : 4;
    /*
    bit 4-7：血量变化类型
     0：装甲模块被弹丸攻击导致扣血
     1：裁判系统重要模块离线导致扣血
     5：装甲模块受到撞击导致扣血
    */
    uint8_t HP_deduction_reason : 4;
} ext_hurt_data_t;

//15. 实时射击信息：0x0207。发送频率：射击后发送
typedef struct
{
    /*子弹类型: 1：17mm弹丸 2：42mm弹丸*/
    uint8_t bullet_type; 
    /*
    发射机构ID：
    1：1号17mm发射机构
    2：2号17mm发射机构
    3：42mm 发射机构
    */
    uint8_t  shooter_number;
    /*子弹射频 单位 Hz*/
    uint8_t  launching_frequency;
    /*子弹射速 单位 m/s*/
    float  initial_speed; 
} ext_shoot_data_t;

//16. 子弹剩余发射数：0x0208。发送频率：10Hz周期发送，所有机器人发送 
typedef struct 
{   
    uint16_t projectile_allowance_17mm;//17mm子弹剩余发射数目
    uint16_t projectile_allowance_42mm;//42mm子弹剩余发射数目
    uint16_t remaining_gold_coin;//剩余金币数量
	uint16_t projectile_allowance_fortress;  //堡垒增益点提供的储备17mm弹丸允许发弹量 该值与机器人是否实际占领堡垒无关
} ext_projectile_allowance_t;

//17. 机器人 RFID 状态：0x0209。发送频率：3Hz，发送范围：单一机器人。 
typedef struct 
{   
    /*
    bit 位值为1/0 的含义：是否已检测到该增益点RFID 卡
     bit 0：己方基地增益点
     bit 1：己方中央高地增益点
     bit 2：对方中央高地增益点
     bit 3：己方梯形高地增益点
     bit 4：对方梯形高地增益点
     bit 5：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡前）
     bit 6：己方地形跨越增益点（飞坡）（靠近己方一侧飞坡后）
     bit 7：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡前）
     bit 8：对方地形跨越增益点（飞坡）（靠近对方一侧飞坡后）
     bit 9：己方地形跨越增益点（中央高地下方）
     bit 10：己方地形跨越增益点（中央高地上方）
     bit 11：对方地形跨越增益点（中央高地下方）
     bit 12：对方地形跨越增益点（中央高地上方）
     bit 13：己方地形跨越增益点（公路下方）
     bit 14：己方地形跨越增益点（公路上方）
     bit 15：对方地形跨越增益点（公路下方）
     bit 16：对方地形跨越增益点（公路上方）
     bit 17：己方堡垒增益点
     bit 18：己方前哨站增益点
     bit 19：己方与兑换区不重叠的补给区/RMUL 补给区
     bit 20：己方与兑换区重叠的补给区
     bit 21：己方大资源岛增益点
     bit 22：对方大资源岛增益点
     bit 23：中心增益点（仅 RMUL 适用）
     bit 24： 对方堡垒增益点
    注：所有RFID卡仅在赛内生效。在赛外，即使检测到对应的RFID 卡，对应值也为0。
   */
    uint32_t rfid_status;
    /*
    rfid_status_2 (2026 V1.3.0 新增隧道位):
     bit 0：对方地形跨越增益点（隧道）（靠近对方公路一侧下方）
     bit 1：对方地形跨越增益点（隧道）（靠近对方公路一侧中间）
     bit 2：对方地形跨越增益点（隧道）（靠近对方公路一侧上方）
     bit 3：对方地形跨越增益点（隧道）（靠近对方梯形高地较低处）
     bit 4：对方地形跨越增益点（隧道）（靠近对方梯形高地较中间）
     bit 5：对方地形跨越增益点（隧道）（靠近对方梯形高地较高处）
     bit 6-7：保留
    */
    uint8_t rfid_status_2;
} ext_rfid_status_t; 

//18. 飞镖机器人客户端指令数据：0x020A。发送频率：3Hz，发送范围：单一机器人
typedef struct 
{ 
    /*
   当前飞镖发射站的状态：
 1：关闭
 2：正在开启或者关闭中
 0：已经开启
    */
    uint8_t dart_launch_opening_status;
	//保留
	uint8_t reserved;
    /*
   切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为0。
    */
    uint16_t target_change_time;
    /*最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为0。*/
     uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

//19. 地面机器人位置数据：0x020B。发送频率：1Hz，发送范围：哨兵机器人
typedef struct 
{
    //己方英雄机器人位置x 轴坐标，单位：m
    float hero_x;
    //己方英雄机器人位置y 轴坐标，单位：m
    float hero_y;
    //己方工程机器人位置x 轴坐标，单位：m
    float engineer_x;
    //己方工程机器人位置y 轴坐标，单位：m
    float engineer_y;
    //己方3 号步兵机器人位置x 轴坐标，单位：m
    float standard_3_x;
    //己方3 号步兵机器人位置y 轴坐标，单位：m
    float standard_3_y;
    //己方4 号步兵机器人位置x 轴坐标，单位：m
    float standard_4_x;
    //己方4 号步兵机器人位置y 轴坐标，单位：m
    float standard_4_y;
    //保留位
    float reserved_1;
    //保留位
    float reserved_2;
} ext_ground_robot_position_t;

//20. 雷达标记进度数据：0x020C。发送频率：1Hz，发送范围：单一机器人
typedef struct 
{
       /*
     bit 0：对方1 号英雄机器人易伤情况  对方机器人：标记进度≥100 发送1，<100 发送0
     bit 1：对方2 号工程机器人易伤情况
     bit 2：对方3 号步兵机器人易伤情况  己方机器人：标记进度≥50 发送1，<50 发送0
     bit 3：对方4 号步兵机器人易伤情况
     bit 4：对方空中机器人特殊标识情况
     bit 5：对方哨兵机器人易伤情况
     bit 6：己方1 号英雄机器人特殊标识情况
     bit 7：己方2 号工程机器人特殊标识情况
     bit 8：己方3 号步兵机器人特殊标识情况
     bit 9：己方4 号步兵机器人特殊标识情况
     bit 10：己方空中机器人特殊标识情况
     bit 11：己方哨兵机器人特殊标识情况
     bit 12-15：保留位
    */
    uint16_t mark_progress;
}ext_radar_mark_data_t;

//21.哨兵自主决策信息同步:0X020D。发送频率：1Hz，发送范围：单一机器人
typedef struct 
{
	/*
	bit 0-10：除远程兑换外，哨兵机器人成功兑换的允许发弹量，开局为 0，在
    哨兵机器人成功兑换一定允许发弹量后，该值将变为哨兵机器人成功兑换的
    允许发弹量值。
    bit 11-14：哨兵机器人成功远程兑换允许发弹量的次数，开局为 0，在哨兵
    机器人成功远程兑换允许发弹量后，该值将变为哨兵机器人成功远程兑换允
    许发弹量的次数。
    bit 15-18：哨兵机器人成功远程兑换血量的次数，开局为 0，在哨兵机器人
    成功远程兑换血量后，该值将变为哨兵机器人成功远程兑换血量的次数。
    bit 19：哨兵机器人当前是否可以确认免费复活，可以确认免费复活时值为
    1，否则为0。
    bit 20：哨兵机器人当前是否可以兑换立即复活，可以兑换立即复活时值为
    1，否则为0。
    bit 21-30：哨兵机器人当前若兑换立即复活需要花费的金币数。
    bit 31：保留。
	*/
    uint32_t sentry_info;
    /*
    bit 0：哨兵当前是否处于脱战状态，处于脱战状态时为1，否则为0。
    bit 1-11：队伍17mm 允许发弹量的剩余可兑换数。
    bit 12-15：保留。
    */
    uint16_t sentry_info_2;
} ext_sentry_info_t;

//22.雷达自主决策信息同步:0X020E。发送频率：1Hz，发送范围：单一机器人
typedef struct
{
	/*
	bit 0-1：雷达是否拥有触发双倍易伤的机会，开局为 0，数值为雷达拥有触发
    双倍易伤的机会，至多为 2
    bit 2：对方是否正在被触发双倍易伤
     0：对方未被触发双倍易伤
     1：对方正在被触发双倍易伤
    bit 3-4：己方加密等级（即对方干扰波难度等级），开局为1，最高为3
    bit 5：当前是否可以修改密钥，1 为可修改
    bit 6-7：保留位*/
    uint8_t radar_info;
}ext_radar_info_t;

/*============================================================================================*/
/*23.机器人交互数据:0X0301。发送频率：发送方触发发
送，频率上限为 10Hz 发送范围：单一机器人*/
typedef struct
{
	//子内容 ID
    uint16_t data_cmd_id;
	//发送者 ID
    uint16_t sender_id;
/*
	接收者 ID
    仅限己方通信需为规则允许的多机通讯接收者
    若接收者为选手端，则仅可发送至发送者对应的选手端 ID 编号详见附录
*/
    uint16_t receiver_id;
    uint8_t user_data[112];
}ext_robot_interaction_data_t;

//以下是子内容的ID:
//子内容 ID：0x0100  选手端删除图层
typedef struct
{
	/*删除操作
     0：空操作
     1：删除图层
     2：删除所有
    */
    uint8_t delete_type;
    //图层数 图层数：0~9
    uint8_t layer;
}ext_interaction_layer_delete_t;

//子内容 ID：0x0101  选手端绘制一个图形
typedef struct
{ 
	//图形名 在图形删除、修改等操作中，作为索引
    uint8_t figure_name[3];
	/*
	图形配置1
    bit 0-2：图形操作
     0：空操作
     1：增加
     2：修改
     3：删除
    bit 3-5：图形类型
     0：直线
     1：矩形
     2：正圆
     3：椭圆
     4：圆弧
     5：浮点数
     6：整型数
     7：字符
    bit 6-9：图层数（0~9）
    bit 10-13：颜色
     0：红/蓝（己方颜色）
     1：黄色
     2：绿色
     3：橙色
     4：紫红色
     5：粉色
     6：青色
     7：黑色
     8：白色
    bit 14-31：根据绘制的图形不同，含义不同，详见"表 2-24 图形细节参数说明"
	*/
    uint32_t operate_tpye:3;
    uint32_t figure_tpye:3;
    uint32_t layer:4;
    uint32_t color:4;
    uint32_t details_a:9;
    uint32_t details_b:9;
    /*
    图形配置2
    bit 0-9：线宽，建议字体大小与线宽比例为 10：1
    bit 10-20：起点/圆心 x 坐标
    bit 21-31：起点/圆心 y 坐标
    */
    uint32_t width:10;
    uint32_t start_x:11;
    uint32_t start_y:11;
    //图形配置3 根据绘制的图形不同，含义不同，详见"表 2-24 图形细节参数说明"   
    uint32_t details_c:10;
    uint32_t details_d:11;
    uint32_t details_e:11;
}ext_interaction_figure_t;
/*表 2-24 图形细节参数说明
类型 details_a  details_b  details_c   details_d      details_e
直线     -          -        -         终点x坐标      终点y坐标
矩形     -          -        -       对角顶点x坐标   对角顶点y坐标
正圆     -          -       半径          -               -
椭圆     -          -        -         x半轴长度       y半轴长度
圆弧   起始角度   终止角度    -         x半轴长度       y半轴长度
浮点数 字体大小   无作用     (      该值除以1000即实际显示值  )
整型数 字体大小     -        (     32位整型数，int32_t        )
字符   字体大小   字符长度   -            -               -


角度值含义为：0°指12点钟方向，顺时针绘制；
屏幕位置：（0,0）为屏幕左下角（1920，1080）为屏幕右上角；
浮点数：整型数均为 32 位，对于浮点数，实际显示的值为输入的值/1000，
如在details_c、details_d、details_e 对应的字节输入 1234，
选手端实际显示的值将为 1.234。
即使发送的数值超过对应数据类型的限制,图形仍有可能显示，
但此时不保证显示的效果
*/

//子内容 ID：0x0102  选手端绘制两个图形
typedef struct
{
	/*图形 1 与 0x0101 的数据段相同
      图形 2 与 0x0101 的数据段相同*/
 ext_interaction_figure_t interaction_figure[2];
}ext_interaction_figure_2_t;

//子内容 ID：0x0103  选手端绘制五个图形
typedef struct
{
	/*
	  图形 1 与 0x0101 的数据段相同
      图形 2 与 0x0101 的数据段相同
      图形 3 与 0x0101 的数据段相同
      图形 4 与 0x0101 的数据段相同
      图形 5 与 0x0101 的数据段相同
	*/
    ext_interaction_figure_t interaction_figure[5];
}ext_interaction_figure_3_t;

//子内容 ID：0x0104  选手端绘制七个图形
typedef struct
{
	/*
	图形 1 与 0x0101 的数据段相同
    图形 2 与 0x0101 的数据段相同
    图形 3 与 0x0101 的数据段相同
    图形 4 与 0x0101 的数据段相同
    图形 5 与 0x0101 的数据段相同
    图形 6 与 0x0101 的数据段相同
    图形 7 与 0x0101 的数据段相同
	*/
    ext_interaction_figure_t interaction_figure[7];
}ext_interaction_figure_4_t;


//子内容 ID：0x0110  选手端绘制字符图形
//相比串口协议有修改
typedef struct
{
//	  uint16_t data_ID;
//    uint16_t sender_ID;
//    uint16_t receiver_ID;
	ext_interaction_figure_t character_figure;
	uint8_t char_data[30];
} ext_interaction_character_t;

//子内容 哨兵自主决策指令：0x0120
typedef struct
{
    /*
    bit 0：哨兵机器人是否确认复活
     0 表示哨兵机器人确认不复活，即使此时哨兵的复活读条已经完成
     1 表示哨兵机器人确认复活，若复活读条完成将立即复活
    bit 1：哨兵机器人是否确认兑换立即复活
     0 表示哨兵机器人确认不兑换立即复活；
     1 表示哨兵机器人确认兑换立即复活，若此时哨兵机器人
	    符合兑换立即复活的规则要求，则会立即消耗金币兑换立即复活
    bit 2-12：
	    哨兵将要兑换的发弹量值，开局为0，修改此值后，哨兵在补血点
	    即可兑换允许发弹量。此值的变化需要单调递增，否则视为不合法。
    示例：此值开局仅能为0，此后哨兵可将其从0修改至X，则消
        耗X金币成功兑换 X 允许发弹量。此后哨兵可将其从X修改至X+Y，以此类推。
    bit 13-16：
	    哨兵远程兑换发弹量的请求次数，开局为0，修改此值即可请求远程兑换发弹量。
        此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
    示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消
        耗金币远程兑换允许发弹量。此后哨兵可将其从1修改至2，以此类推。
    bit 17-20：哨兵远程兑换血量的请求次数，开局为0，修改此
        值即可请求远程兑换血量。此值的变化需要单调递增且每次仅能增加 1，否则视为不合法。
    示例：此值开局仅能为0，此后哨兵可将其从0修改至1，则消
        耗金币远程兑换血量。此后哨兵可将其从1修改至 2，以此类推。
    在哨兵发送该子命令时，服务器将按照从相对低位到相对高位
        的原则依次处理这些指令，直至全部成功或不能处理为止。
    示例：若队伍金币数为 0，此时哨兵战亡，"是否确认复活"的
        值为 1，"是否确认兑换立即复活"的值为 1，"确认兑换的允
        许发弹量值"为 100。（假定之前哨兵未兑换过允许发弹量）由
        于此时队伍金币数不足以使哨兵兑换立即复活，则服务器将会
        忽视后续指令，等待哨兵发送的下一组指令。
    bit 21-31：保留
    */
   uint32_t sentry_cmd;
}ext_sentry_cmd_t;

//子内容 雷达自主决策指令：0x0121
typedef struct
{
	/*
	雷达是否确认触发双倍易伤
	开局为 0，修改此值即可请求触发双倍易伤，若此时雷达拥有
    触发双倍易伤的机会，则可触发。此值的变化需要单调递增且
    每次仅能增加 1，否则视为不合法。
	
    示例：此值开局仅能为0，此后雷达可将其从0修改至1，
    若雷达拥有触发双倍易伤的机会，则触发双倍易伤。此后雷达可将
    其从1修改至2，以此类推。若雷达请求双倍易伤时，
    双倍易伤正在生效，则第二次双倍易伤将在第一次双倍易伤结束后生效。
	*/
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password_1;
    uint8_t password_2;
    uint8_t password_3;
    uint8_t password_4;
    uint8_t password_5;
    uint8_t password_6;
}ext_radar_cmd_t;
//子内容结束
/*============================================================================================*/


//24.选手端小地图交互数据:0X0302。发送频率：发送方触发发送，
//频率上限为 30Hz 发送范围：单一机器人
typedef struct
{
	//自定义数据 可改长度
    uint8_t data[30];
}ext_custom_robot_data_t;

//机器人可通过图传链路向对应的操作手选手端连接的自定义控制器发送数据（RMUL 暂不适用）
//命令码ID：0x0309
typedef struct
{
    //自定义数据
    uint8_t data[30];
}ext_robot_custom_data_t;

typedef struct
{
    uint8_t data[300];
}ext_robot_custom_client_data_t;

typedef struct
{
    uint8_t data[30];
}ext_client_custom_robot_data_t;

//25.选手端小地图交互数据:0X0303。发送频率：选手端触发发送 发送范围：单一机器人
typedef struct
{
	//目标位置x 轴坐标，单位m  当发送目标机器人ID 时，该值为0
    float target_position_x;
	//目标位置y 轴坐标，单位m  当发送目标机器人ID 时，该值为0
    float target_position_y;
	//云台手按下的键盘按键通用键值  无按键按下，则为0
    uint8_t cmd_keyboard;
	//对方机器人ID  当发送坐标数据时，该值为0
    uint8_t target_robot_id;
	//信息来源ID  信息来源的ID，ID 对应关系详见附录
    uint16_t cmd_source;
}ext_map_command_t;


    //26.选手端小地图可接收机器人数据:0X0305（2026 V1.3.0 修订：新增对方/己方空中机器人，扩至48字节）
typedef struct
{
    //对方英雄机器人x 位置坐标，单位：cm
    uint16_t opponent_hero_position_x;
    //对方英雄机器人y 位置坐标，单位：cm
    uint16_t opponent_hero_position_y;
    //对方工程机器人x 位置坐标，单位：cm
    uint16_t opponent_engineer_position_x;
    //对方工程机器人y 位置坐标，单位：cm
    uint16_t opponent_engineer_position_y;
    //对方3 号步兵机器人x 位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_x;
    //对方3 号步兵机器人y 位置坐标，单位：cm
    uint16_t opponent_infantry_3_position_y;
    //对方4 号步兵机器人x 位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_x;
    //对方4 号步兵机器人y 位置坐标，单位：cm
    uint16_t opponent_infantry_4_position_y;
    //对方6 号空中机器人x 位置坐标，单位：cm
    uint16_t opponent_aerial_position_x;
    //对方6 号空中机器人y 位置坐标，单位：cm
    uint16_t opponent_aerial_position_y;
    //对方哨兵机器人x 位置坐标，单位：cm
    uint16_t opponent_sentry_position_x;
    //对方哨兵机器人y 位置坐标，单位：cm
    uint16_t opponent_sentry_position_y;
    //己方英雄机器人x 位置坐标，单位：cm
    uint16_t ally_hero_position_x;
    //己方英雄机器人y 位置坐标，单位：cm
    uint16_t ally_hero_position_y;
    //己方工程机器人x 位置坐标，单位：cm
    uint16_t ally_engineer_position_x;
    //己方工程机器人y 位置坐标，单位：cm
    uint16_t ally_engineer_position_y;
    //己方3 号步兵机器人x 位置坐标，单位：cm
    uint16_t ally_infantry_3_position_x;
    //己方3 号步兵机器人y 位置坐标，单位：cm
    uint16_t ally_infantry_3_position_y;
    //己方4 号步兵机器人x 位置坐标，单位：cm
    uint16_t ally_infantry_4_position_x;
    //己方4 号步兵机器人y 位置坐标，单位：cm
    uint16_t ally_infantry_4_position_y;
    //己方6 号空中机器人x 位置坐标, 单位：cm
    uint16_t ally_aerial_position_x;
    //己方6 号空中机器人y 位置坐标, 单位：cm
    uint16_t ally_aerial_position_y;
    //己方哨兵机器人x 位置坐标，单位：cm
    uint16_t ally_sentry_position_x;
    //己方哨兵机器人y 位置坐标，单位：cm
    uint16_t ally_sentry_position_y;
    /*备注:
        当x、y 超出边界时显示在对应边缘处，
        当x、y 均为0时，视为未发送此机器人坐标。
    */
}ext_map_robot_data_t;

//25.选手端小地图接收哨兵数据:0X0307。发送频率：1Hz 发送范围：对应操作手选手端
typedef struct
{
	/*1：到目标点攻击
      2：到目标点防守
      3：移动到目标点*/
     uint8_t intention;
	//路径起点 x 轴坐标，单位：dm
     uint16_t start_position_x;
	/*note:小地图左下角为坐标原点，水平向右为 X 轴正方向，竖直向上为 Y 轴正方向。
显示位置将按照场地尺寸与小地图尺寸等比缩放，超出边界的位置将在边界处显示*/
	
    //路径起点 y 轴坐标，单位：dm
	 uint16_t start_position_y;
	//路径点 x轴增量数组，单位：dm
     int8_t delta_x[49];
	//路径点 y轴增量数组，单位：dm
	/*note:增量相较于上一个点位进行计算，共 49 个新点位，X 与 Y 轴增量对应组成点位*/
     int8_t delta_y[49];
	//发送者ID，备注：需与自身 ID 匹配，ID 编号详见附录
     uint16_t sender_id;
}ext_map_data_t;

//27.选手端小地图接收机器人数据:0X0308。发送频率：3Hz 发送范围：己方选手端
typedef struct
{ 
	/*发送者的ID。备注：需要校验发送者的ID正确性*/
    uint16_t sender_id;
	/*接收者的ID。备注：需要校验接收者的ID正确性，仅支持发送己方选手端*/
    uint16_t receiver_id;
	/*字符 备注：以utf-16格式编码发送，支持显示中文。编码发送时请注意数
    据的大小端问题*/
    uint8_t user_data[30];
} ext_custom_info_t;
#if 0

//28.图传遥控信息标识：0x0304。发送频率：30Hz。
typedef struct
{
    /*鼠标x轴移动速度，负值标识向左移动*/
    int16_t mouse_x;
    /*鼠标y轴移动速度，负值标识向下移动*/
    int16_t mouse_y;
    /*鼠标滚轮移动速度，负值标识向后滚动*/
    int16_t mouse_z;
    /*鼠标左键是否按下：0为未按下；1为按下*/
    int8_t left_button_down;
    /*鼠标右键是否按下：0为未按下，1为按下*/
    int8_t right_button_down;
    /*
    键盘信息
	键盘按键信息，每个bit对应一个按键，0为未按下，1为按下：
    bit 0：键盘W是否按下
    bit 1：键盘S是否按下
    bit 2：键盘A是否按下
    bit 3：键盘D是否按下
    bit 4：键盘SHIFT是否按下
    bit 5：键盘CTRL是否按下
    bit 6：键盘Q是否按下
    bit 7：键盘E是否按下
    bit 8：键盘R是否按下
    bit 9：键盘F是否按下
    bit 10：键盘G是否按下
    bit 11：键盘Z是否按下
    bit 12：键盘X是否按下
    bit 13：键盘C是否按下
    bit 14：键盘V是否按下
    bit 15：键盘B是否按下
    */
    uint16_t keyboard_value;
    /*保留位*/
    uint16_t reserved;
}ext_remote_control_t;
#endif

//29.自定义控制器与选手端交互数据:0x0306，发送方触发发送，频率上限为 30Hz
typedef struct
{
	/*键盘键值：
 bit 0-7：按键1键值
 bit 8-15：按键2键值*/
uint16_t key_value;
	/*bit 0-11：鼠标X轴像素位置
 bit 12-15：鼠标左键状态*/
 uint16_t x_position:12;
 uint16_t mouse_left:4;
	/*bit 0-11：鼠标Y轴像素位置
 bit 12-15：鼠标右键状态*/
 uint16_t y_position:12;
 uint16_t mouse_right:4;
	//保留位
 uint16_t reserved;
}ext_custom_client_data_t;
#pragma pack()




extern ext_game_status_t                ext_game_status;            //0X0001
extern ext_game_result_t                ext_game_result;            //0X0002
extern ext_game_robot_HP_t              ext_game_robot_HP;          //0X0003
extern ext_event_data_t                 ext_event_data;             //0X0101
extern ext_referee_warning_t            ext_referee_warning;        //0X0104
extern ext_dart_info_t                  ext_dart_info;              //0X0105
extern ext_robot_status_t               ext_game_robot_status;      //0X0201
extern ext_power_heat_data_t            ext_power_heat_data;        //0X0202
extern ext_robot_pos_t                  ext_robot_pos;              //0X0203
extern ext_buff_t                       ext_buff;                   //0X0204
extern ext_hurt_data_t                  ext_hurt_data;              //0X0206
extern ext_shoot_data_t                 ext_shoot_data;             //0X0207
extern ext_projectile_allowance_t       ext_projectile_allowance;   //0X0208
extern ext_rfid_status_t                ext_rfid_status;            //0X0209
extern ext_dart_client_cmd_t            ext_dart_client_cmd;        //0X020A
extern ext_ground_robot_position_t      ext_ground_robot_pos;       //0X020B
extern ext_radar_mark_data_t            ext_radar_mark_data;        //0X020C
extern ext_sentry_info_t                ext_sentry_info;            //0X020D
extern ext_radar_info_t                 ext_radar_info;             //0X020E
extern ext_robot_interaction_data_t     ext_robot_interaction_data; //0X0301
extern ext_map_command_t                ext_map_command;            //0X0303
extern ext_map_robot_data_t             ext_map_robot_data;         //0X0305
extern ext_map_data_t                   ext_map_data;               //0X0307
extern ext_custom_info_t                ext_custom_info;            //0X0308
extern ext_custom_robot_data_t          ext_custom_robot_data;      //0X0302
extern ext_robot_custom_data_t          ext_robot_custom_data;      //0X0309
extern ext_client_custom_robot_data_t   ext_client_custom_robot_data; //0X0311
extern ext_custom_client_data_t         ext_custom_client_data;     //0X0306


//void RefereeConnection_Init(void);
//void Referee_IDLECallback(UART_HandleTypeDef *huart);
void RefereeReceive(uint16_t judge_receive_counter,uint8_t* judge_receive_buffer);
void Referee_Receive_Data_Processing(uint16_t SOF, uint16_t CmdID, uint8_t* judge_receive_buffer);

#endif

#ifndef _REFEREE_CRC_H__
#define _REFEREE_CRC_H__

/*-------------------------------------------CRC校验---------------------------------------------------*/
/**
  * @brief  裁判系统数据校验
  * @param  __RECEIVEBUFFER__：  接收到的裁判系统数据头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval 1：                  校验正确
  * @retval 0：                  校验错误
  * @note	None
  */
#define Verify_CRC_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__) (Verify_CRC8_Check_Sum(__RECEIVEBUFFER__, JUDGE_CRC8_OFFSET + 1) && Verify_CRC16_Check_Sum(__RECEIVEBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)))

/**
  * @brief  裁判系统添加校验
  * @param  __TRANSMITBUFFER__： 发送到裁判系统的数据中头帧所在地址
  * @param  __DATALENGTH__：     一帧数据内的数据量/Bytes（内容）
  * @retval None
  * @note	None
  */
#define Append_CRC_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__)                       \
do                                                                                     \
{                                                                                      \
    Append_CRC8_Check_Sum(__TRANSMITBUFFER__, JUDGE_CRC8_OFFSET + 1);                  \
    Append_CRC16_Check_Sum(__TRANSMITBUFFER__, __DATALENGTH__ + JUDGE_DATA_LENGTH(0)); \
} while (0U)

/*--------------------------------------------------校验函数--------------------------------------------------*/
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);

/*--------------------------------------------------校验函数--------------------------------------------------*/
#endif
