#ifndef _DDS_TYPE_H
#define _DDS_TYPE_H

#include <string>
#include <iostream>
#include <atomic>
#include <vector>
#include <map>
#include <unordered_map>
#include <list>
#include <queue>
#include <memory>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <functional>
#include <future>

#define DDS_SUCCESS                  (0)
#define DDS_FAILURE                 (-1)

#define WAIT_FOREVER            (-1)

#define MAX_MEM_GRP_CNT     	(16)

typedef struct _DataBuffer_t
{
	uint64_t		m_u64TimeStamp;
	int32_t			m_iBufSize;
	int32_t			m_iBufLen;
	uint8_t*		m_pBufPtr;
	void*			m_pPriv;				/* 内部使用，不可修改 */
}DataBuffer_t, *DataBufferPtr;

typedef struct _ExtraDataInfo_t
{
	uint64_t		m_u64TimeStamp;	
}ExtraDataInfo_t;

typedef struct
{
    int32_t         m_iGrpId;						/* 组ID */
    int32_t         m_iMemSize;						/* 组内存块大小 */
    int32_t         m_iMaxCnt;						/* 组内存块最大个数 */
}MemGroup_t;

typedef struct
{
    int32_t     	m_iGrpCnt;						/* 内存池分组个数 */
    MemGroup_t  	m_astMemGrp[MAX_MEM_GRP_CNT]; 	/* 内存池组信息 */
}MemPoolLayout_t;

typedef struct
{
	bool			m_bUserCfg;						/*true-使用用户配置, 此时需要用户指定内存池布局; false-使用默认配置*/
	MemPoolLayout_t	m_stMemPoolLayout;
}ShmConfig_t;

typedef struct _DomainConfigParam_t
{
	int32_t					m_i32DomainId;			/* 域ID */

	std::string				m_strMultiIp;			/* 组播IP */
		
	int32_t					m_iMultiPort;			/* 组播端口 */

	int32_t					m_iDisPeriod;			/* 订阅发现周期 */

	std::vector<int32_t>	m_vecSocketPorts;		/* TCP/UDP 端口 */

	std::vector<int32_t>	m_vecUnixSocketIdx;		/* 域套接字Index */

	std::vector<int32_t>	m_vecShmKey;			/* 共享内存KEY */

	int32_t 				m_iMempoolSize;			/* 内存池最大大小 */

	std::string				m_strEthDev;			/* 网卡设备名 */
}DomainConfigParam_t;


typedef struct _DataTransChnInfo_t
{
	int32_t                          m_iTransType;            /* 数据通道传输方式 */
	int32_t	                         m_iDataPort;		      /* 数据端口号 */
} DataTransInfo_t;

typedef struct _DdsInfo_t
{
	std::string				         m_strTopic;			  /* 主题 */
	int32_t					         m_iRoleType;			  /* 角色类型 */
	int32_t					         m_iTcpPort;			  /* TCP端口号 */
	std::vector<DataTransInfo_t>     m_vecDataTransInfo;      /* 数据通道传输信息 */
	int32_t					         m_iDomainId;			  /* 域ID */
	int32_t                          m_iClientNum;            /* 发布者客户端连接个数 */
	int32_t                          m_iTopicState;           /* 主题订阅状态 */
	std::vector<int32_t>             m_vecFps;                /* 帧率 */
	int64_t                          m_iLostPkgCnt;           /* 丢包数量 */
} DdsInfo_t;

using UserDataCallBack = std::function<int32_t(void *, int32_t, ExtraDataInfo_t *)>;

#endif