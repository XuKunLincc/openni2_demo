/**
*   Copyright (C) 2018 华数机器人
*
*   @file       ProxyExt.h
*   @brief      华数III型二次开发接口 - 业务接口 - 外部操作代理
*   @details    提供了III型控制器外部模式相关业务接口。
*
*   @author     
*   @date       2018/10/16
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

#include "Hsc3Def.h"
#include "CommDef.h"
#include <stdint.h>

class CommApi;

/**
*   @class      ProxyExt
*   @brief      业务接口 - 外部操作代理
*   @details    提供接口包含：外部操作接口。
*   @date       2018/10/16
*/
class DLL_EXPORT ProxyExt
{
public:
    /**
     * @brief   构造函数
     * @details 注：确保传入已构造的通信客户端。
     * @param   pNet    通信客户端
     */
    ProxyExt(CommApi * pNet);

    ~ProxyExt();

	/**
     * @brief   获取外部输入配置数据
     * @details 
     * @param[out]   extInData   输入信号配置信息列表
     * @see     addExtIn delExtIn
     */
	HMCErrCode getExtIn(std::list<HookExtIn> & extInData);
		
	/**
     * @brief   添加外部输入配置
     * @details 
     * @param   extInItem   待添加项
     * @see     getExtIn delExtIn
     */
	HMCErrCode addExtIn(const HookExtIn & extInItem);
		
	/**
     * @brief   删除外部输入配置
     * @details 
     * @param   extInItem   待删除项
     * @see     getExtIn addExtIn
     */
	HMCErrCode delExtIn(const HookExtIn & extInItem);
		
	/**
     * @brief   获取外部输出配置数据
     * @details 
     * @param[out]   extOutData   输出信号配置信息列表
     * @see     addExtOut delExtOut
     */
	HMCErrCode getExtOut(std::list<HookExtOut> & extOutData);
		
	/**
     * @brief   添加外部输出配置
     * @details 
     * @param   extOutItem   待添加项
     * @see     getExtOut delExtOut
     */
	HMCErrCode addExtOut(const HookExtOut & extOutItem);
		
	/**
     * @brief   删除外部输出配置
     * @details 
     * @param   extOutItem   待删除项
     * @see     getExtOut addExtOut
     */
	HMCErrCode delExtOut(const HookExtOut & extOutItem);
		
	/**
     * @brief   获取外部程序
     * @details 可由外部信号控制的程序
     * @param[out]   progName   程序名
     * @see     setExtProgName
     */
	HMCErrCode getExtProgName(std::string & progName);
		
	/**
     * @brief   设置外部程序
     * @details 可由外部信号控制的程序
     * @param   progName   程序名
     * @see     getExtProgName
     */
	HMCErrCode setExtProgName(const std::string & progName);
		
	/**
     * @brief   获取引用寄存器数据
     * @details 
     * @param[out]   refRegData   数据列表
     * @see     modifyRefRegData delRefRegData
     */
	HMCErrCode getRefRegData(std::list<RefRegData> & refRegData);
		
	/**
     * @brief   修改（添加）引用寄存器数据
     * @details 
     * @param   modData   待修改的数据
     * @see     getRefRegData delRefRegData
     */
	HMCErrCode modifyRefRegData(const RefRegModData & modData);
		
	/**
     * @brief   删除引用寄存器数据
     * @details 
     * @param   iRef   引用信号索引
     * @see     getRefRegData modifyRefRegData
     */
	HMCErrCode delRefRegData(int iRef);
	
private:

	inline HMCErrCode execContackOnHook(const std::string & strIntent, const std::string & strMsg, std::string & strResponse);

private:
    CommApi * m_pNet;
};