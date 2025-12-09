#ifndef _PARTICIPANT_H
#define _PARTICIPANT_H

#include "dds_type.h"

class CTopic;
class CPublisher;
class CSubscriber;
class CParticipantImpl;

class CParticipant
{
    friend class CParticipantImpl;

public:
    CParticipant(DomainConfigParam_t & stCfgParam);

    CParticipant(const std::string& strConfigPath);

    virtual ~CParticipant(); 

    /**
     * @brief 创建主题TOPIC.
     * @param strTopicName TOPIC名称.
     * @param pstShmCfg 如果SOC内部使用共享内存通信,需要设置此参数,仅对数据写者生效.
     * @return 成功返回CTopic对象指针,失败返回nullptr.
     */
    CTopic*         createTopic(const std::string &strTopicName, ShmConfig_t *pstShmCfg = nullptr);

    /**
     * @brief 删除主题TOPIC.
     * @param pTopic CTopic对象指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t         deleteTopic(CTopic *pTopic);        

    /**
     * @brief 创建发布者PUBLISHER.
     * @param 无.
     * @return 成功返回CPublisher对象指针,失败返回nullptr.
     */
    CPublisher*     createPublisher();

    /**
     * @brief 删除发布者PUBLISHER.
     * @param pPublisher CPublisher对象指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t         deletePublisher(CPublisher* pPublisher);

    /**
     * @brief 创建订阅者SUBSCRIBER.
     * @param 无.
     * @return 成功返回CSubscriber对象指针,失败返回nullptr.
     */
    CSubscriber*    createSubscriber();
    
    /**
     * @brief 删除订阅者SUBSCRIBER.
     * @param pPublisher CSubscriber对象指针.
     * @return 0: 成功, -1: 失败.
     */
    int32_t         deleteSubscriber(CSubscriber* pSubscriber);
    
    /**
     * @brief 获取域ID
     * @param 无
     * @return 返回域ID
     */
    int32_t     getDomainId();

    /**
     * @brief 获取设备IP
     * @param 无
     * @return 返回设备IP
     */
    std::string getDevIp();

    /**
     * @brief 获取DDS信息
     * @param strTopicName TOPIC名称
     * @param iRoleTye 角色类型
     * @param stDdsInfo DDS信息结构体
     * @return 0: 成功, -1: 失败.
     */
    int32_t     getDdsInfo(const std::string& strTopicName, int32_t iRoleTye, DdsInfo_t& stDdsInfo);

protected:
    CParticipant(const CParticipant&) = delete;
    
    CParticipant& operator=(const CParticipant&) = delete;

private:
    CParticipantImpl*   m_pImpl;
};


#endif