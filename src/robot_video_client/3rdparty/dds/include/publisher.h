/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-25 17:30:49
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-01 20:51:44
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

#ifndef _PUBLISHER_H
#define _PUBLISHER_H

#include <iostream>
#include <memory>

class CTopic;
class CDataWriter;
class CPublisherImpl;
class CParticipantImpl;

class CPublisher
{
    friend class CPublisherImpl;
    friend class CParticipantImpl;

protected:
    CPublisher(CPublisherImpl *pPubImpl);

    virtual ~CPublisher();

public:
    /**
     * @brief 创建数据写者DATAWRITER.
     * @param pTopic TOPIC对象指针.
     * @return 成功返回CDataWriter对象指针,失败返回nullptr.
     */
    CDataWriter*    createDataWriter(CTopic *pTopic);

    /**
     * @brief 删除数据写者DATAWRITER.
     * @param pWriter CDataWriter对象指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t         deleteDataWriter(CDataWriter *pWriter);

private:
    CPublisher(const CPublisher&) = delete;
    
    CPublisher& operator=(const CPublisher&) = delete;

private:
    CPublisherImpl*  m_pImpl;
};







#endif