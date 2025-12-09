#ifndef _DATA_READER_H
#define _DATA_READER_H

#include "dds_type.h"

class CDataReaderImpl;
class CSubscriberImpl;

class CDataReader
{
    friend class CDataReaderImpl;
    friend class CSubscriberImpl;

protected:
    CDataReader(CDataReaderImpl *pRdImpl);

    virtual ~CDataReader();

public:
    /**
     * @brief 读取数据.
     * @param pBuf 数据buffer指针,存放读取的数据.
     * @param iSize 数据buffer的大小.
     * @param iTimeOut 读数据超时时间.
     *                  0: 如果没有数据立即返回.
     *                 >0: 等待一定时间,在等待期间如果有数据立即返回,否则直到超时才返回.
     *                 -1: 阻塞等待直到读取到数据才返回. 
     * @param pstExtraInfo 数据附加信息地址指针,由用户传入.
     * @return  >0: 读取数据成功,返回实际有效数据长度. 
     *         <=0: 读取数据失败.
     */
    int32_t read(void *pBuf, int32_t iSize, int32_t iTimeOut, ExtraDataInfo_t *pstExtraInfo = nullptr);

    /**
     * @brief 从底层获取一帧数据buffer,与releaseDataBuffer成对使用.
     * @param iTimeOut 获取数据帧buffer超时时间.
     *                  0: 如果没有数据立即返回.
     *                 >0: 等待一定时间,在等待期间如果有数据立即返回,否则直到超时才返回.
     *                 -1: 阻塞等待直到获取到一帧数据才返回. 
     * @param pstExtraInfo 数据附加信息地址指针,由用户传入.
     * @return 成功返回数据帧buffer指针,失败返回nullptr.
     */
    DataBufferPtr getDataBuffer(int32_t iTimeOut, ExtraDataInfo_t *pstExtraInfo = nullptr);
    
    /**
     * @brief 释放数据帧buffer.
     * @param pBuf 数据帧buffer指针.
     * @return 无.
     */
    void releaseDataBuffer(DataBufferPtr pBuf);

    /**
     * @brief 注册数据回调函数
     * @param funDataCb 回调函数
     * @return 无.
    */
    void registerUserDataCallBack(UserDataCallBack funDataCb);

    /**
     * @brief 获取底层缓存的数据帧buffer个数.
     * @param 无.
     * @return 缓存的数据帧buffer个数.
     */
    int32_t getBufferCount();

    /**
     * @brief: 数据通道是否已建立连接.
     * @param 无.
     * @return true: 连接成功, false: 未连接.
     */
    bool isConnect();

private:
    CDataReader(const CDataReader&) = delete;
    
    CDataReader& operator=(const CDataReader&) = delete;

private:
    CDataReaderImpl*     m_pImpl;
};


#endif