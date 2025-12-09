#ifndef _TOPIC_H
#define _TOPIC_H

#include <iostream>
#include <memory>
#include <atomic>

#include "topic_desc.h"

class CTopicImpl;
class CParticipantImpl;

class CTopic : public CTopicDesc
{
    friend class CTopicImpl;
    friend class CParticipantImpl;

protected:
    CTopic(const std::string& strTopicName, CTopicImpl *pTopicImpl);

    virtual ~CTopic();

public:
    /**
     * @brief 获取TOPIC名称.
     * @param 无.
     * @return TOPIC名称.
     */
    std::string& getTopicName();

    CTopicDescImpl* getTopicDescImpl() const;

private:
    CTopic(const CTopic&) = delete;
    
    CTopic& operator=(const CTopic&) = delete; 

protected:
    CTopicImpl*     m_pImpl;
};

#endif