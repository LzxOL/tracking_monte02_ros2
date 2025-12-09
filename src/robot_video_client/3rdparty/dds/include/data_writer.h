#ifndef _DATA_WRITER_H
#define _DATA_WRITER_H

#include "dds_type.h"

class CDataWriterImpl;
class CPublisherImpl;

class CDataWriter
{
    friend class CDataWriterImpl;
    friend class CPublisherImpl;

protected:
    CDataWriter(CDataWriterImpl *pWrImpl);

    virtual ~CDataWriter();

public:
    /**
     * @brief 写数据.
     * @param pData 数据缓存地址指针.
     * @param iLen 数据长度.
     * @return 0: 成功,-1: 失败.
     */
    int32_t write(void *pData, int32_t iLen);

    /**
     * @brief 从底层内存池获取一个空闲数据帧buffer,与putFullDataBuffer成对使用.
     * @param iSize buffer容量.
     * @param iTimeOut 预留,暂时无用.
     * @return 成功返回buffer指针，失败返回nullptr.
     */
    DataBufferPtr getEmptyDataBuffer(int32_t iSize, int32_t iTimeOut = 0);
    
    /**
     * @brief 将数据帧buffer放入发送缓存.
     * @param pBuf 数据帧buffer指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t putFullDataBuffer(DataBufferPtr pBuf);

    /**
     * @brief: 数据通道是否已建立连接.
     * @param 无.
     * @return true: 连接成功, false: 未连接.
     */
    bool isConnect();

private:
    CDataWriter(const CDataWriter&) = delete;
    
    CDataWriter& operator=(const CDataWriter&) = delete;

private:
    CDataWriterImpl*    m_pImpl;
};


#endif
