/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-25 17:30:49
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-01 20:51:59
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

#ifndef _TOPIC_DESC_H
#define _TOPIC_DESC_H

#include <iostream>
#include <atomic>

class CTopicDescImpl;

class CTopicDesc
{
protected:
    CTopicDesc(const std::string& strName):
                m_strName(strName)
    {

    }

    virtual ~CTopicDesc(){};

public:
    /**
     * @brief 获取TOPIC名称.
     * @param 无.
     * @return TOPIC名称.
     */
    const std::string& getName() const
    {
        return m_strName;
    }

    virtual CTopicDescImpl* getTopicDescImpl() const = 0;

protected:
    std::string             m_strName;
};


#endif