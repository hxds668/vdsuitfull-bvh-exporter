#pragma once
#ifndef VDMOCAPSDK_VDSUITMINI_DATATYPE_H
#define VDMOCAPSDK_VDSUITMINI_DATATYPE_H

#include <cstdint> 

#define NODES_BODY 23
#define NODES_HAND 20
#define PC_NODES_VIRTUAL 18

#define PC_FINGERS_VIRTUAL 5		//五指

namespace VDSuitMiniDevice
{
	/**
	* @brief
	*   Body nodes name and its index.
	*/
	typedef enum BODYNODES
	{
		BN_Hips = 0,                    ///< Hips
		BN_RightUpperLeg,               ///< Right Upper Leg
		BN_RightLowerLeg,               ///< Right Lower Leg
		BN_RightFoot,                   ///< Right Foot
		BN_RightToe,                    ///< Right Toe
		BN_LeftUpperLeg,                ///< Left Upper Leg
		BN_LeftLowerLeg,                ///< Left Lower Leg
		BN_LeftFoot,                    ///< Left Foot
		BN_LeftToe,                     ///< Left Toe
		BN_Spine,                       ///< Spine
		BN_Spine1,                      ///< Spine1
		BN_Spine2,                      ///< Spine2
		BN_Spine3,                      ///< Spine3 -- Back
		BN_Neck,                        ///< Neck
		BN_Head,                        ///< Head
		BN_RightShoulder,               ///< Right Shoulder
		BN_RightUpperArm,               ///< Right Upper Arm
		BN_RightLowerArm,               ///< Right Lower Arm
		BN_RightHand,                   ///< Right Hand
		BN_LeftShoulder,                ///< Left Shoulder
		BN_LeftUpperArm,                ///< Left Upper Arm
		BN_LeftLowerArm,                ///< Left Lower Arm
		BN_LeftHand,                    ///< Left Hand
	}_BodyNodes_;


	/**
	* @brief
	*   Hand nodes name and its index.
	*/
	typedef enum HANDNODES
	{
		HN_Hand = 0,
		HN_ThumbFinger,
		HN_ThumbFinger1,
		HN_ThumbFinger2,
		HN_IndexFinger,
		HN_IndexFinger1,
		HN_IndexFinger2,
		HN_IndexFinger3,
		HN_MiddleFinger,
		HN_MiddleFinger1,
		HN_MiddleFinger2,
		HN_MiddleFinger3,
		HN_RingFinger,
		HN_RingFinger1,
		HN_RingFinger2,
		HN_RingFinger3,
		HN_PinkyFinger,
		HN_PinkyFinger1,
		HN_PinkyFinger2,
		HN_PinkyFinger3,
	}_HandNodes_;



	// 设备类型
	typedef enum DEVICETYPE
	{
		DT_NONE = -1,
		DT_Auto = 0,
		//DT_VDSuit = 1,
		DT_VDSuitMini = 2,  //无腿
		DT_VDSuitFull = 3,   //包括腿

	}_DeviceType_;

	//手部设备类型
	typedef enum HANDDEVICETYPE
	{
		HAND_DEVICE_TYPE_GLOVE = 0,    // 手套传感器
		HAND_DEVICE_TYPE_JOYSTICK = 1  // 摇杆传感器
	}_HandDeviceType_;

	//按键枚举
	typedef enum KEY {
		Key1 = 1,
		Key2 = 2,
		Key3 = 3,
		Key4 = 4,
		Key5 = 5
	}_Key_;

	typedef enum BASEGESTURE {
		Gesture_1 = 1,      
		Gesture_2 = 2,      
		Gesture_3 = 3,      
		Gesture_4 = 4,      
		Gesture_5 = 5,      
		Gesture_6 = 6,		
		Gesture_7 = 7,		
		Gesture_8 = 8,		
		Gesture_9 = 9,		
		Gesture_10 = 10,	
		Gesture_11 = 11,	
		Gesture_12 = 12,
		Gesture_13 = 13,
		Gesture_14 = 14,
		Gesture_15 = 15,
		Gesture_16 = 16,
		Gesture_17 = 17,
		Gesture_18 = 18,
		Gesture_19 = 19,
		Gesture_20 = 20,

		Gesture_Custom1 = 21,  // 用户自定义手势1
		Gesture_Custom2 = 22,  // 用户自定义手势2
		Gesture_Custom3 = 23,  // 用户自定义手势3
		Gesture_Custom4 = 24,   // 用户自定义手势4
		Gesture_Custom5,
		Gesture_Custom6,
		Gesture_Custom7,
		Gesture_Custom8,
		Gesture_Custom9,
		Gesture_Custom10
	}_BaseGesture_;


	typedef enum SENSORSTATE {
		SS_NONE = 0,                    //NONE. 不获取该传感器信息。
		SS_Well,                        //well.工作正常
		SS_NoData,                      //no data.无数据
		SS_UnReady,                     //on initial.传感器未就绪
		SS_BadMag,                      //the magnetic field is disturbed.磁场受干扰
	}_SensorState_;



	typedef enum WORLDSPACE {
		WS_Geo = 0,                        //表示世界坐标系为地理坐标系
		WS_Unity,                          //表示世界坐标系为Unity世界坐标系
		WS_UE4,                            //表示世界坐标系为UE4世界坐标系
	}_WorldSpace_;



	typedef enum CALIBRATIONMODE {
		CM_Apose = 0, //Apose标定模式，所述Apose：站立，双腿伸直平行，双脚掌指向正前方，双手自然垂下平行且掌心相对，手指伸直，大拇指与食指呈45度夹角关系，另外在标定过程中不可往前或往后倾斜，也不可弯腰驼背。
		CM_Ppose,   //Ppose标定，所述Ppose：在Apose站立的基础上，左右手同时水平朝正前方平举且平行，掌心向下，手指伸直，大拇指与食指呈45度夹角关系，基于Apose标定后的pose标定，用于手部的标定。
	}_CalibrationMode_;



	typedef enum CALIBRATIONSTATE {
		CS_UnStart = 0,                    //未开始标定状态
		CS_InPose,                         //处于pose标定状态
		CS_Successed,                      //标定成功
		CS_Failed,                         //标定失败
	}_CalibrationState_;



	typedef enum FREQUENCY {
		HZ_60 = 60,
		HZ_72 = 72,
		HZ_80 = 80,
		HZ_96 = 96,
		//HZ_120 = 120,
	}_Frequency_;

	typedef struct MAGCORRECTRESULT
	{
		bool isFinished = false;  //是否结束
		bool isHaveSucceed = false;  //是否有校准成功的传感器，若为false，则下面的数据无效
		float progress = 0;
		int bLength = 0;
		_BodyNodes_ bFailedNodes[NODES_BODY] = { BN_Hips };
		int rLength = 0;
		_HandNodes_ rFailedNodes[NODES_HAND] = { HN_Hand };
		int lLength = 0;
		_HandNodes_ lFailedNodes[NODES_HAND] = { HN_Hand };
	}_MagCorrectResult_;



	typedef struct CALIBRATIONPROGRESS {
		_CalibrationState_ state;
		float progress;                         //在 state 为 CS_Apose 时有效
	}_CalibrationProgress_;



	typedef struct MOCAPDATA {
		bool isUpdate;                                   //true表示设备数据已更新
		int frameIndex;                                  //帧序号
		float devicePower;                               //设备电量
		int frequency;                                   //设备数据传输频率

		_SensorState_ sensorState_body[NODES_BODY];      //设备全身（不包括手指）各传感器工作状态（按_BodyNodes_序号排列，下同）	
		float position_body[NODES_BODY][3]/*xyz-m*/;     //全身（不包括手指）各节点坐标，单位m
		float quaternion_body[NODES_BODY][4]/*wxyz*/;    //全身（不包括手指）各节点四元数

		_SensorState_ sensorState_rHand[NODES_HAND];     //设备双手（手掌及手指）各传感器工作状态（按_HandNodes_序号排列，下同）	
		float position_rHand[NODES_HAND][3]/*xyz-m*/;    //双手（手掌及手指）各节点坐标，单位m
		float quaternion_rHand[NODES_HAND][4]/*wxyz*/;   //双手（手掌及手指）各节点四元数

		_SensorState_ sensorState_lHand[NODES_HAND];     //设备双手（手掌及手指）各传感器工作状态（按_HandNodes_序号排列，下同）	
		float position_lHand[NODES_HAND][3]/*xyz-m*/;    //双手（手掌及手指）各节点坐标，单位m
		float quaternion_lHand[NODES_HAND][4]/*wxyz*/;   //双手（手掌及手指）各节点四元数

		int nsResult;                                  //其它数据

		uint8_t rKeyStatus[5];    // 按键：1=按下，0=未按下
		uint8_t rJoystickX;       // 摇杆X轴：实时ADC值
		uint8_t rJoystickY;       // 摇杆Y轴：实时ADC值
		uint8_t lKeyStatus[5];    // 左手同样
		uint8_t lJoystickX;
		uint8_t lJoystickY;

	}_MocapData_;

	typedef struct MOCAPDATA_WITH_VIRTUAL {
		bool isUpdate;                                   //true表示设备数据已更新
		int frameIndex;                                  //帧序号
		float devicePower;                               //设备电量
		int frequency;                                   //设备数据传输频率

		_SensorState_ sensorState_body[NODES_BODY];      //设备全身（不包括手指）各传感器工作状态（按_BodyNodes_序号排列，下同）	
		float position_body[NODES_BODY][3]/*xyz-m*/;     //全身（不包括手指）各节点坐标，单位m
		float quaternion_body[NODES_BODY][4]/*wxyz*/;    //全身（不包括手指）各节点四元数

		_SensorState_ sensorState_rHand[NODES_HAND];     //设备双手（手掌及手指）各传感器工作状态（按_HandNodes_序号排列，下同）	
		float position_rHand[NODES_HAND][3]/*xyz-m*/;    //双手（手掌及手指）各节点坐标，单位m
		float quaternion_rHand[NODES_HAND][4]/*wxyz*/;   //双手（手掌及手指）各节点四元数

		float position_rHandVirtual[PC_FINGERS_VIRTUAL][3] = { 0 };		//指尖坐标

		_SensorState_ sensorState_lHand[NODES_HAND];     //设备双手（手掌及手指）各传感器工作状态（按_HandNodes_序号排列，下同）	
		float position_lHand[NODES_HAND][3]/*xyz-m*/;    //双手（手掌及手指）各节点坐标，单位m
		float quaternion_lHand[NODES_HAND][4]/*wxyz*/;   //双手（手掌及手指）各节点四元数

		float position_lHandVirtual[PC_FINGERS_VIRTUAL][3] = { 0 };		//左手指尖坐标

		int nsResult;                                  //其它数据

		uint8_t rKeyStatus[5];    // 按键：1=按下，0=未按下
		uint8_t rJoystickX;       // 摇杆X轴：实时ADC值
		uint8_t rJoystickY;       // 摇杆Y轴：实时ADC值
		uint8_t lKeyStatus[5];    // 左手同样
		uint8_t lJoystickX;
		uint8_t lJoystickY;

	}_MocapDataWithVirtual_;

	
	typedef enum GESTURE {
		GESTURE_NONE = 0,  //未知手势
		GESTURE_1,         //剪刀手
		GESTURE_2,         //拳头
		GESTURE_3,         //掌
		GESTURE_4,         //OK
		GESTURE_5,         //点赞
		GESTURE_6,         //手枪状
		GESTURE_7,         //大拇指、食指、小指伸直，其它手指握拢
		GESTURE_8,         //拿，相比于抓，手握的更拢
		GESTURE_9,         //抓
		GESTURE_10,        //大拇指与食指伸直并拢
		GESTURE_11,        //食指伸直，其它手指握拢
	}_Gesture_;




	//
	typedef struct VERSION
	{
		unsigned char Project_Name[26] = { 0 };
		unsigned char Author_Organization[128] = { 0 };
		unsigned char Author_Domain[26] = { 0 };
		unsigned char Author_Maintainer[26] = { 0 };
		unsigned char Version[26] = { 0 };
		unsigned char Version_Major;
		unsigned char Version_Minor;
		unsigned char Version_Patch;
	}_Version_;


	typedef enum NoticeMode
	{
		NOPRITF = 0,
		NOTICE_PRINTF,
		DEBUG_PRINTF,
	}_NoticeMode_;



	

	typedef void(*VDMOCAPDATA_CALLBACK)(_MocapData_ mocapData); //定义函数回调指针

	typedef void(*VDMOCAPDATA_VIRTUAL_CALLBACK)(_MocapDataWithVirtual_ mocapData);		//回调指针，动捕数据带虚拟点

	typedef void(*DEVICEBREAK_CALLBACK)();

	//多人回调接口
	typedef void(*VDMOCAPDATA_CALLBACK_Mult)(_MocapData_ mocapData, int index); //定义函数回调指针

	typedef void(*VDMOCAPDATA_VIRTUAL_CALLBACK_Mult)(_MocapDataWithVirtual_ mocapData, int index);

	typedef void(*DEVICEBREAK_CALLBACK_Mult)(int index);


}//end namespace

#endif
