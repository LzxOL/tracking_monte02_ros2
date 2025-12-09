/*
 * @Description: p
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-24 16:07:43
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-03-25 17:55:34
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once

#include "dds_type.h"
#include "participant.h"
#include "topic.h"
#include "topic_desc.h"
#include "publisher.h"
#include "subscriber.h"
#include "data_writer.h"
#include "data_reader.h"

class CDDSWrapper {
public:
    CDDSWrapper(const std::string &strConfigFile) {
        m_pParticipant = new CParticipant(strConfigFile);
        m_pPublisher = m_pParticipant->createPublisher();
        m_pSubscriber = m_pParticipant->createSubscriber();
    }

    ~CDDSWrapper() {
        std::lock_guard<std::mutex> lock(m_mtx);

        for (auto writer : m_vecDataWriter) {
            m_pPublisher->deleteDataWriter(writer);
        }
        m_vecDataWriter.clear();

        for (auto reader : m_vecDataReader) {
            m_pSubscriber->deleteDataReader(reader);
        }
        m_vecDataReader.clear();

        for (auto &[name, topic] : m_vecTopic) {
            if (topic) {
                m_pParticipant->deleteTopic(topic);
            }
        }
        m_vecTopic.clear();

        if (m_pPublisher) {
            m_pParticipant->deletePublisher(m_pPublisher);
            m_pPublisher = nullptr;
        }
        if (m_pSubscriber) {
            m_pParticipant->deleteSubscriber(m_pSubscriber);
            m_pSubscriber = nullptr;
        }

        delete m_pParticipant;
        m_pParticipant = nullptr;
    }

    CDataWriter *createDataWriter(const std::string &strTopicName) {
        std::lock_guard<std::mutex> lock(m_mtx);

        if(!m_pPublisher)
        {
          return nullptr;
        }

        CTopic *pTopic = getOrCreateTopic(strTopicName);
        if (!pTopic) return nullptr;

        CDataWriter *pDataWriter = m_pPublisher->createDataWriter(pTopic);
        if (pDataWriter) {
            m_vecDataWriter.push_back(pDataWriter);
        }

        std::cout << "createDataWriter: " << strTopicName << std::endl;

        return pDataWriter;
    }

    void deleteDataWriter(CDataWriter *pDataWriter) {
        std::lock_guard<std::mutex> lock(m_mtx);

        if(!m_pPublisher)
        {
          return;
        }

        if (pDataWriter) {
            m_pPublisher->deleteDataWriter(pDataWriter);
            m_vecDataWriter.erase(std::remove(m_vecDataWriter.begin(), m_vecDataWriter.end(), pDataWriter), m_vecDataWriter.end());
        }
    }

    CDataReader *createDataReader(const std::string &strTopicName) {
        std::lock_guard<std::mutex> lock(m_mtx);

        if(!m_pSubscriber)
        {
          return nullptr;
        }

        CTopic *pTopic = getOrCreateTopic(strTopicName);
        if (!pTopic) return nullptr;

        CDataReader *pDataReader = m_pSubscriber->createDataReader(pTopic);
        if (pDataReader) {
            m_vecDataReader.push_back(pDataReader);
        }

        std::cout << "createDataReader: " << strTopicName << std::endl;

        return pDataReader;
    }

    void deleteDataReader(CDataReader *pDataReader) {
        std::lock_guard<std::mutex> lock(m_mtx);

        if(!m_pSubscriber)
        {
          return;
        }

        if (pDataReader) {
            m_pSubscriber->deleteDataReader(pDataReader);
            m_vecDataReader.erase(std::remove(m_vecDataReader.begin(), m_vecDataReader.end(), pDataReader), m_vecDataReader.end());
        }
    }

private:
    CDDSWrapper(const CDDSWrapper &) = delete;
    CDDSWrapper &operator=(const CDDSWrapper &) = delete;

    CTopic *getOrCreateTopic(const std::string &strTopicName) {
        auto it = m_vecTopic.find(strTopicName);
        if (it != m_vecTopic.end()) {
            return it->second;
        }

        CTopic *pTopic = m_pParticipant->createTopic(strTopicName);
        if (pTopic) {
            m_vecTopic[strTopicName] = pTopic;
        }
        return pTopic;
    }

private:
    CParticipant *m_pParticipant = nullptr;
    CPublisher *m_pPublisher = nullptr;
    CSubscriber *m_pSubscriber = nullptr;
    std::unordered_map<std::string, CTopic *> m_vecTopic;
    std::vector<CDataWriter *> m_vecDataWriter;
    std::vector<CDataReader *> m_vecDataReader;
    std::mutex m_mtx;
};