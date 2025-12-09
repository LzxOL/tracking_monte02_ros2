#ifndef _SUBSCRIBER_H
#define _SUBSCRIBER_H

#include <iostream>
#include <memory>

class CTopic;
class CDataReader;
class CSubscriberImpl;
class CParticipantImpl;

class CSubscriber
{
    friend class CSubscriberImpl;
    friend class CParticipantImpl;

protected:
    CSubscriber(CSubscriberImpl *pSubImpl);

    virtual ~CSubscriber();

public:
    /**
     * @brief 创建数据读者DATAREADER.
     * @param pTopic TOPIC对象指针.
     * @param iBufCnt 本读者数据帧缓存队列最大长度.
     * @return 成功返回CDataReader对象指针,失败返回nullptr.
     */
    CDataReader*    createDataReader(CTopic* pTopic, int32_t iBufCnt = 4);

    /**
     * @brief 删除数据读者DATAREADER.
     * @param pReader CDataReader对象指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t         deleteDataReader(CDataReader *pReader);

private:
    CSubscriber(const CSubscriber&) = delete;
    
    CSubscriber& operator=(const CSubscriber&) = delete;

private:
    CSubscriberImpl*     m_pImpl;
};

#endif