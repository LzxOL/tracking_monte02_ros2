#ifndef _YLLOG_H
#define _YLLOG_H
#include <stdint.h>
#include <string.h>
#ifdef __cplusplus
extern "C"
{
#endif

#define MAX_LOG_LEN         (256)

#define LOGGER_CONS         (1 << 0)                        /* 日志终端输出功能ID */                  
#define LOGGER_FILE         (1 << 1)                        /* 日志文件记录功能ID */ 
#define LOGGER_NET          (1 << 2)                        /* 日志网络上传功能ID */ 
#define LOGGER_CONS_FILE    (LOGGER_CONS | LOGGER_FILE)     /* 日志终端和文件记录功能ID */
#define LOGGER (LOGGER_CONS | LOGGER_FILE | LOGGER_NET)

#define CONS_LEVEL(l) ((l) & 0xFF)
#define FILE_LEVEL(l) (((l) >> 8) & 0xFF)
#define NET_LEVEL(l)  (((l) >> 16) & 0xFF)

/*******************************************************************************
** 单条日志输出样式枚举
** SIMPLE: [时间][等级] - 内容.
** NORMAL: [时间][等级] - 内容. [进程ID]
** DETAIL: [时间][等级] - 内容. [运行时间][logger名称][进程ID]
********************************************************************************/
typedef enum
{
    LOG_PATTERN_SIMPLE = 0,
    LOG_PATTERN_NORMAL,
    LOG_PATTERN_DETAIL,
    LOG_PATTERN_BUTT
}LogPattern_e;

/***********************************************************************************
** 日志等级
************************************************************************************/
typedef enum
{
    LOG_LVL_FATAL = 0,
    LOG_LVL_ERROR,
    LOG_LVL_WARN,
    LOG_LVL_INFO,
    LOG_LVL_DEBUG,
    LOG_LVL_TRACE,
    LOG_LVL_BUTT
}LogLevel_e;

/************************************************************************************
** 终端输出配置参数
*************************************************************************************/
typedef struct _ConsoleParam_t
{
    int32_t         m_iPattern;                             /* @ LogPattern_e */               
    int32_t         m_iLevel;                               /* @ LogLevel_e */
    char            m_acLoggerName[MAX_LOG_LEN];            /* logger名称 */
}LogConsoleParam_t;

/***********************************************************************************
** 日志文件回滚方式
** INDEX: 按索引方式,例如: tom0.log,tom1.log,tom2.log, ... 
** TIME: 按时间方式,例如: tom_2022-03-25-11-22.log, tom_2022-03-25-11-23.log, ...
**************************************************************************************/
typedef enum
{
    FILE_ROLL_INDEX = 0,                                    
    FILE_ROLL_TIME,
    FILE_ROLL_BUTT
}LogFileRollType_e;

/************************************************************************************
** 日志文件回滚index方式配置参数
*************************************************************************************/
typedef struct
{
    int32_t         m_iMaxFileSize;                         /* 单个文件最大大小，单位字节 */
    int32_t         m_iMaxBackupIndex;                      /* 日志文件的最大个数，超过个数回滚 */
    bool            m_bImmediateFlush;                      /* 写磁盘是否立即flush */
    int32_t         m_iBufferSize;                          /* 日志缓存大小 */
    bool            m_bLockFile;                            /* 日志文件进程间互斥 */
    int32_t         m_iPattern;                             /* @ LogPattern_e */
    int32_t         m_iLevel;                               /* @ LogLevel_e */
    char            m_acFilePath[MAX_LOG_LEN];              /* 日志文件存放目录 */
    char            m_acFileName[MAX_LOG_LEN];              /* 日志文件名称 */
    char            m_acLoggerName[MAX_LOG_LEN];            /* 日志记录器名称 */
}LogFileRollIndex_t;

/************************************************************************************
** 日志文件回滚time方式策略
** DAILY: 按天生成日志文件, 例如 2022-03-25.log, 2022-03-26.log, ...
** HOURLY: 按小时生成日志文件, 例如 2022-03-25_11.log, 2022-03-25_12.log, ...
** MINUTELY: 按分钟生成日志文件, 例如 2022-03-25_1123.log, 2022-03-25_1124.log, ...
*************************************************************************************/
typedef enum
{
    ROLL_TIME_DAILY = 0,
    ROLL_TIME_HOURLY,
    ROLL_TIME_MINUTELY,
    ROLL_TIME_BUTT
}LogRollTimeSched_e;

/************************************************************************************
** 日志文件回滚time方式配置参数
*************************************************************************************/
typedef struct
{
    int32_t             m_iMaxHistory;                      /* 最大历史日志文件个数，超过个数回滚 */
    bool                m_bImmediateFlush;                  /* 写磁盘是否立即flush */
    int32_t             m_iBufferSize;                      /* 日志缓存大小 */
    bool                m_bLockFile;                        /* 日志文件进程间互斥 */
    bool                m_bClearHistoryOnStart;             /* 日志启动时是否删除旧文件 */
    int32_t             m_iTimeSched;                       /* @ LogRollTimeSched_e */
    int32_t             m_iPattern;                         /* @ LogPattern_e */
    int32_t             m_iLevel;                           /* @ LogLevel_e */
    char                m_acFilePath[MAX_LOG_LEN];          /* 日志文件存放目录 */
    char                m_acFileName[MAX_LOG_LEN];          /* 日志文件名称 */
    char                m_acLoggerName[MAX_LOG_LEN];        /* 日志记录器名称 */ 
}LogFileRollTime_t;

/************************************************************************************
** 日志文件配置参数
*************************************************************************************/
typedef struct FileParam_t
{
    int32_t                     m_iRollType;                /* @ LogFileRollType_e */
    union 
    {
        LogFileRollIndex_t      m_stRollIndexParam;         /* @ LogFileRollIndex_t */     
        LogFileRollTime_t       m_stRollTimeParam;          /* @ LogFileRollTime_t */ 
    }m_unParam;
}LogFileParam_t;

/************************************************************************************
** 日志网络上传配置参数
*************************************************************************************/
typedef struct
{
    int32_t         m_iPort;                                /* 日志服务器端口号 */
    char            m_acServAddr[32];                       /* 日志服务器IP地址 */
    int32_t         m_iPattern;                             /* @ LogPattern_e */
    int32_t         m_iLevel;                               /* @ LogLevel_e */                                    
    char            m_acLoggerName[MAX_LOG_LEN];            /* 日志记录器名称 */        
}LogNetParam_t;

/************************************************************************************
** 日志模块初始化配置参数
*************************************************************************************/
typedef struct _LogParam_t
{
    LogConsoleParam_t  m_stConParam;                        /* @ LogConsoleParam_t */               
    LogFileParam_t     m_stFileParam;                       /* @ LogFileParam_t */ 
    LogNetParam_t      m_stNetParam;                        /* @ LogNetParam_t */               
}LogParam_t;

/*************************************************************************************
* Function          : YLLOG_GetDefaultParam  
* Description       : 获取日志模块默认参数.     
* Input             : pstLogParam: 日志模块初始化参数.
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_GetDefaultParam(LogParam_t *pstLogParam);

/*************************************************************************************
* Function          : YLLOG_Init  
* Description       : 日志模块初始化.可以选择开启三种功能(终端/文件/网络)中任意一种或者多种功能.     
* Input             : iLoggerSet: 日志功能集,按位操作.
                      pstLogParam: 日志模块初始化参数,如果为NULL,则使用默认参数.              
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_Init(int32_t iLoggerSet, LogParam_t *pstLogParam);

/*************************************************************************************
* Function          : YLLOG_UnInit  
* Description       : 日志模块去初始化.    
* Input             : 无             
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_UnInit();

/*************************************************************************************
* Function          : YLLOG_SetLevel  
* Description       : 设置日志等级.可以选择设置三种功能(终端/文件/网络)中任意一种或者多种功能的等级.   
* Input             : iLoggerSet: 日志功能集,按位操作. 
                      iLevel: 日志等级.            
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_SetLevel(int32_t iLoggerSet, int32_t iLevel);

/*************************************************************************************
* Function          : YLLOG_GetLevel  
* Description       : 获取日志等级.可以选择获取三种功能(终端/文件/网络)中任意一种或者多种功能的等级.   
* Input             : iLoggerSet: 日志功能集. 
* Output            : piLevel: 日志功能的等级,每8位对应一种功能的等级.
                               |1BYTE(NET)|1BYTE(FILE)|1BYTE(CONS)|.    
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_GetLevel(int32_t iLoggerSet, int32_t *piLevel);

/*************************************************************************************
* Function          : YLLOG_Log  
* Description       : 记录单条日志.可以选择三种功能(终端/文件/网络)中任意一种或者多种功能记录日志.   
* Input             : iLoggerSet: 日志功能集,按位操作. 
                      iLevel: 日志等级.        
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/

int32_t YLLOG_Log(int32_t iLoggerSet, int32_t iLevel, const char *pFmt, ...);


/*************************************************************************************
* Function          : YLLOG_Log_verbose  
* Description       : 记录单条日志.可以选择三种功能(终端/文件/网络)中任意一种或者多种功能记录日志.   
* Input             : iLoggerSet: 日志功能集,按位操作. 
                      iLevel: 日志等级.        
                      pFileName: 文件名. 
                      pFuncName: 函数名.
                      iLineNum: 行号.  
* Output            : 无      
* Return            : 0:成功, -1:失败
* Others            :
**************************************************************************************/
int32_t YLLOG_Log_verbose(int32_t iLoggerSet, int32_t iLevel, const char *pFileName, 
                        const char *pFuncName, int32_t iLineNum, const char *pFmt, ...);


#if !defined(__FILENAME__)
#define __FILENAME__                                        \
    ({                                                        \
    strrchr(__FILE__, '/')    ? strrchr(__FILE__, '/') + 1  \
    : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 \
                                : __FILE__;                   \
    })
#endif

#define LINE_INFO   __FILENAME__,__FUNCTION__, __LINE__

#define YLLOG_FAT(fmt, ...) \
do \
{\
    YLLOG_Log(LOGGER, LOG_LVL_FATAL, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_ERR(fmt, ...) \
do \
{\
    YLLOG_Log(LOGGER, LOG_LVL_ERROR, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_WARN(fmt, ...) \
do \
{\
    YLLOG_Log(LOGGER, LOG_LVL_WARN, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_INFO(fmt, ...) \
do \
{\
    YLLOG_Log(LOGGER, LOG_LVL_INFO, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_DBG(fmt, ...) \
do \
{\
    YLLOG_Log(LOGGER, LOG_LVL_DEBUG, fmt, ##__VA_ARGS__); \
}while(0)


// Log verbose info 

#define YLLOG_FAT_V(fmt, ...) \
do \
{\
    YLLOG_Log_verbose(LOGGER, LOG_LVL_FATAL, LINE_INFO, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_ERR_V(fmt, ...) \
do \
{\
    YLLOG_Log_verbose(LOGGER, LOG_LVL_ERROR, LINE_INFO, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_WARN_V(fmt, ...) \
do \
{\
    YLLOG_Log_verbose(LOGGER, LOG_LVL_WARN, LINE_INFO, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_INFO_V(fmt, ...) \
do \
{\
    YLLOG_Log_verbose(LOGGER, LOG_LVL_INFO, LINE_INFO, fmt, ##__VA_ARGS__); \
}while(0)

#define YLLOG_DBG_V(fmt, ...) \
do \
{\
    YLLOG_Log_verbose(LOGGER, LOG_LVL_DEBUG, LINE_INFO, fmt, ##__VA_ARGS__); \
}while(0)


#ifdef __cplusplus
}
#endif

#endif