/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyMotion.h
*   @brief      华数III型二次开发接口 - 业务接口 - 运动功能代理
*   @details    提供了III型控制器运动功能相关业务接口。
*
*   @author     
*   @date       2018/06/29
*   @version    
*   
*/

#pragma once
/**
*   @skip DLL_EXPORT
*/
#ifdef _LINUX_
#define DLL_EXPORT __attribute__((visibility("default")))
#else
#define DLL_EXPORT _declspec(dllexport) 
#endif

#include "CommDef.h"
#include "Hsc3Def.h"
#include <stdint.h>


class CommApi;

/**
*   @class      ProxyMotion
*   @brief      业务接口 - 运动功能代理
*   @details    提供接口包含：对安全开关、模式、坐标系等对象的操作，还有单轴运动、运动定位、校准、标定。
*   @date       2018/06/29
*/
class DLL_EXPORT ProxyMotion
{
public:
    /**
     * @brief   构造函数
     * @details 注：确保传入已构造的通信客户端。
     * @param   pNet    通信客户端
     */
    ProxyMotion(CommApi * pNet);

    ~ProxyMotion();

    // 全局作用域

	/**
     * @brief   设置操作模式
     * @param   mode    模式
     */
    HMCErrCode setOpMode(OpMode mode);

    /**
     * @brief   获取操作模式
     * @param[out]  mode    模式
     */
    HMCErrCode getOpMode(OpMode & mode);
	
	/**
     * @brief   设置手动运行模式
     * @param   mode    模式
     */
	HMCErrCode setManualMode(ManualMode mode);

    /**
     * @brief   获取手动运行模式
     * @param[out]  mode    模式
     */
    HMCErrCode getManualMode(ManualMode & mode);
	
    /**
     * @brief   设置手动运行增量距离
     * @param   length  距离大小（单位根据坐标系确定，关节：°；笛卡尔：mm）
     */
	HMCErrCode setInchLen(double length);

    /**
     * @brief   获取手动运行增量距离
     * @param[out]  length  距离大小（单位根据坐标系确定，关节：°；笛卡尔：mm）
     */
    HMCErrCode getInchLen(double & length);
	
    /**
     * @brief   获取手动运行状态
     * @param[out]  state   状态
     */
    HMCErrCode getManualStat(ManState & state);
	
    /**
     * @brief   设置自动运行模式
     * @param   mode    模式
     */
	HMCErrCode setAutoMode(AutoMode mode);

    /**
     * @brief   获取自动运行模式
     * @param[out]  mode    模式
     */
    HMCErrCode getAutoMode(AutoMode & mode);
	
    /**
     * @brief   设置自动运行倍率
     * @param   vord    倍率（1~100，单位：\%）
     */
	HMCErrCode setVord(int32_t vord);

    /**
     * @brief   获取自动运行倍率
     * @param[out]  vord    倍率（1~100，单位：\%）
     */
    HMCErrCode getVord(int32_t & vord);
	
	/**
     * @brief   设置手动运行倍率
     * @param   vord    倍率（1~100，单位：\%）
     */
	HMCErrCode setJogVord(int32_t vord);

    /**
     * @brief   获取手动运行倍率
     * @param[out]  vord    倍率（1~100，单位：\%）
     */
    HMCErrCode getJogVord(int32_t & vord);
	
	/**
     * @brief   操作急停
     * @param   estop   是否急停
     */
	HMCErrCode setEstop(bool estop);

    /**
     * @brief   获取是否处于急停状态
     * @param[out]  estop   急停状态
     */
    HMCErrCode getEstop(bool & estop);
	

    // 组作用域

    /**
     * @brief   组使能
     * @param   gpId    组号（0..n-1）
     * @param   en      是否使能
     */
	HMCErrCode setGpEn(int8_t gpId, bool en);

    /**
     * @brief   获取是否处于使能状态
     * @param   gpId    组号（0..n-1）
     * @param[out]  en      使能状态
     */
    HMCErrCode getGpEn(int8_t gpId, bool & en);
	 
    /**
     * @brief   组复位
     * @param   gpId    组号（0..n-1）
     */
	HMCErrCode gpReset(int8_t gpId);

    /**
     * @brief   零点校准
     * @param   gpId    组号（0..n-1）
     * @param   pos     零点设定值
     * @param   mask    掩码，低9位有效，最低位代表组中第1个轴，第9位代表组中第9个轴。
     */
    HMCErrCode setHomePosition(int8_t gpId, const JntData& pos, int32_t mask);

    /**
     * @brief   获取零点
     * @param   gpId    组号（0..n-1）
	 * @param[out]  pos     零点设定值
     */
    HMCErrCode getHomePosition(int8_t gpId, JntData & pos);

    /**
     * @brief   设置软限位
     * @param   gpId    组号（0..n-1）
     * @param   pos     正向软限位设定值
	 * @param   neg     负向软限位设定值
     * @param   mask    使能掩码，低9位有效，最低位代表组中第1个轴，第9位代表组中第9个轴。
     */
    HMCErrCode setJointLimit(int8_t gpId, const JntData & pos, const JntData & neg, int32_t mask);
	
	/**
     * @brief   获取软限位
     * @param   gpId    组号（0..n-1）
     * @param[out]   pos     正向软限位设定值
     * @param[out]   neg     负向软限位设定值
     * @param   mask    使能掩码，低9位有效，最低位代表组中第1个轴，第9位代表组中第9个轴。
     */
    HMCErrCode getJointLimit(int8_t gpId, JntData & pos, JntData & neg, int32_t & mask);

	/**
     * @brief   单轴手动运动
	 * @param   gpId    组号（0..n-1）
	 * @param   axId    轴号（0..5为内部轴，6..8为附加轴）
	 * @param   direc   方向
     */
	HMCErrCode startJog(int8_t gpId, int8_t axId, DirectType direc);

    /**
     * @brief   停止手动运动
     * @details 注：可停止单轴手动运动、运动定位运动。
	 * @param   gpId    组号（0..n-1）
     * @see     startJog moveTo
     */
	HMCErrCode stopJog(int8_t gpId);
	
	/**
     * @brief   运动定位
	 * @param   gpId    组号（0..n-1）
	 * @param   point   目标点
	 * @param   isLinear    <em>true</em>  - 直线运动
     *                      <em>false</em> - 关节运动
     */
	HMCErrCode moveTo(int8_t gpId, const GeneralPos & point, bool isLinear);

    /**
     * @brief   设置工作坐标系
     * @param   gpId    组号（0..n-1）
     * @param   frame   坐标系
     */
	HMCErrCode setWorkFrame(int8_t gpId, FrameType frame);

    /**
     * @brief   获取工作坐标系
     * @param   gpId    组号（0..n-1）
     * @param[out]  frame   坐标系
     */
    HMCErrCode getWorkFrame(int8_t gpId, FrameType & frame);

    /**
     * @brief   获取内部轴数
     * @param   gpId    组号（0..n-1）
     * @param[out]  cnt     轴数
     */
	HMCErrCode getRobAxisCount(int8_t gpId, int32_t & cnt);

    /**
     * @brief   获取附加轴数
     * @param   gpId    组号（0..n-1）
     * @param[out]  cnt     轴数
     */
	HMCErrCode getAuxAxisCount(int8_t gpId, int32_t & cnt);

    /**
     * @brief   获取附加轴起始轴号
     * @param   gpId    组号（0..n-1）
     * @param[out]  index     起始轴号
     */
	HMCErrCode getAuxAxisBegin(int8_t gpId, int32_t & index);
	
    /**
     * @brief   获取关节坐标点数据
     * @param   gpId    组号（0..n-1）
     * @param[out]  pos     点数据（注：最多6轴数据）
     */
    HMCErrCode getJntData(int8_t gpId, JntData & data);

    /**
     * @brief   获取附加轴坐标点数据
     * @param   gpId    组号（0..n-1）
     * @param[out]  data     点数据
     */
	HMCErrCode getAuxData(int8_t gpId, AuxData & data);

    /**
     * @brief   获取笛卡尔坐标点数据
     * @param   gpId    组号（0..n-1）
     * @param[out]  pos     点数据（注：XYZABC）
     */
	HMCErrCode getLocData(int8_t gpId, LocData & data);

    /**
     * @brief   获取形态
     * @param   gpId    组号（0..n-1）
     * @param[out]  config  形态
     */
	HMCErrCode getConfig(int8_t gpId, int32_t & config);

    /**
     * @brief   设置工具坐标数据
     * @param   gpId    组号（0..n-1）
     * @param   index   工具坐标索引（0..15）
     * @param   pos     点数据
     */
	HMCErrCode setTool(int8_t gpId, int8_t index, const LocData & pos);

    /**
     * @brief   获取工具坐标数据
     * @param   gpId    组号（0..n-1）
     * @param   index   工具坐标索引（0..15）
     * @param[out]  pos     点数据
     */
	HMCErrCode getTool(int8_t gpId, int8_t index, LocData & pos);
	
	/**
     * @brief   设置工件坐标数据
     * @param   gpId    组号（0..n-1）
     * @param   index   工件坐标索引（0..15）
     * @param   pos     点数据
     */
	HMCErrCode setWorkpiece(int8_t gpId, int8_t index, const LocData & pos);

    /**
     * @brief   获取工件坐标数据
     * @param   gpId    组号（0..n-1）
     * @param   index   工件坐标索引（0..15）
     * @param[out]  pos     点数据
     */
	HMCErrCode getWorkpiece(int8_t gpId, int8_t index, LocData & pos);
	
	/**
     * @brief   设置工具号
     * @param   gpId    组号（0..n-1）
     * @param   num     工具坐标索引（0..15）
     */
	HMCErrCode setToolNum(int8_t gpId, int8_t num);

    /**
     * @brief   获取工具号
     * @param   gpId    组号（0..n-1）
     * @param[out]   num     工具坐标索引（0..15）
     */
	HMCErrCode getToolNum(int8_t gpId, int8_t & num);
    
	/**
     * @brief   设置工件号
     * @param   gpId    组号（0..n-1）
     * @param   num     工件坐标索引（0..15）
     */
	HMCErrCode setWorkpieceNum(int8_t gpId, int8_t num);

    /**
     * @brief   获取工件号
     * @param   gpId    组号（0..n-1）
     * @param[out]   num     工件坐标索引（0..15）
     */
	HMCErrCode getWorkpieceNum(int8_t gpId, int8_t & num);

    /**
     * @brief   获取轴组数据
     * @param   gpId    组号（0..n-1）
     * @param[out]   data     轴组数据
     */
	HMCErrCode getGroupData(int8_t gpId, GroupData & data);

    /**
     * @brief   设置机型（需重启）
     * @param   gpId    组号（0..n-1）
     * @param   strTypeName    机型名字
     */
    HMCErrCode setRobType(int8_t gpId, std::string strTypeName);

    /**
     * @brief   获取可用机型名字
     * @param[out]   listName     机型名字列表
     */
	HMCErrCode getRobTypeNameList(std::vector<std::string> & listName);

    /**
     * @brief   获取轴组配置
     * @param   gpId    组号（0..n-1）
     * @param[out]   config     轴组配置
     */
	HMCErrCode getGroupConfig(int8_t gpId, GroupConfig & config);

    /**
     * @brief   标定
     * @param   what    标定对象（<em>"TOOL"</em> - 工具，<em>"BASE"</em> - 工件）
     * @param   gpId    组号（0..n-1）
     * @param   num     工具号/工件号
     * @param   type    标定方法编号 （注：参考运动层定义）
     * @param   strData 标定数据（格式：半角分号分隔的点数据队列，例“{1,2,};{3,4,};{5,6,}”）
     * @param[out]  strResult   标定结果（格式：例“ret=0x0, data={7,8,}”，其中“ret”是标定返回值，“data”是标定结果）
     */
	HMCErrCode gpCalibrate(const std::string & what, int8_t gpId, int8_t num, int8_t type, const std::string & strData, std::string & strResult);

private:
    inline HMCErrCode getGlobalCommand(std::string cmd, std::string & ret);
    inline HMCErrCode getGroupCommand(std::string cmd, int8_t gpId, std::string & ret);

private:
    CommApi * m_pNet;
};

