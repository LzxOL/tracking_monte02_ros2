/*
 * @Description:
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-10 10:43:34
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-18 21:45:32
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */

 #pragma once

 #include <atomic>
 #include <condition_variable>
 #include <functional>
 #include <future>
 #include <iostream>
 #include <mutex>
 #include <queue>
 #include <thread>
 
 #define DATA_QUEUE_WAIT_FOREVER (-1)
 
 template <typename T>
 class DataQueue {
  public:
   /**
    * @description:
    * @param {size_t} size
    * @param {function<void(T)>} funCb
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   DataQueue(size_t size = 10, std::function<void(T)> funCb = nullptr, const std::string& tag="")
       : m_size(size), m_funCb(funCb), m_tag(tag) {
     m_size = size > 0 ? size : 10;
   }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   virtual ~DataQueue() {
     while (m_queData.size() > 0) {
       pop_();
     }
   }
 
   /**
    * @description:
    * @param {T} data
    * @return {*}
    * @author:
    */
   bool enqueue(T &&data) {
     std::unique_lock<std::mutex> lck(m_lck);
 
     if (m_queData.size() == m_size) {
       if (!m_tag.empty()) {
        std::cout << m_tag << ": Warning: data queue is full, lost one!!!" << std::endl;
       }
       pop_();
     }

    //  std::cout << "m_cacheSize: " << m_cacheSize << std::endl;
 
     m_queData.push(std::move(data));
     m_cond.notify_all();
 
     return true;
   }
 
   /**
    * @description:
    * @param {int32_t} timeout
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   bool dequeue(T &data, int32_t timeout = 0) {
     std::unique_lock<std::mutex> lck(m_lck);
 
     if (timeout > 0) {
       m_cond.wait_for(lck, std::chrono::milliseconds(timeout),
                       [this]() { return !m_queData.empty(); });
     } else if (timeout == DATA_QUEUE_WAIT_FOREVER) {
       m_cond.wait(lck, [this]() { return !m_queData.empty(); });
     } else if (timeout < DATA_QUEUE_WAIT_FOREVER) {
       std::cout << m_tag << ": Error: timeout is invalid!!!" << std::endl;
       return false;
     }
 
     if (m_queData.size() > 0) {
       data = m_queData.front();
       m_queData.pop();
       return true;
     }
 
     return false;
   }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   bool peek(T &data) {
     std::unique_lock<std::mutex> lck(m_lck);
 
     if (m_queData.empty()) {
       return false;
     }
 
     data = m_queData.front();
 
     return true;
   }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   void pop() {
     std::unique_lock<std::mutex> lck(m_lck);
 
     if (m_queData.size() > 0) {
       pop_();
     }
   }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   int32_t getCount() const {
    std::unique_lock<std::mutex> lck(m_lck);
    return m_queData.size();
   }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   bool isFull() const { 
    std::unique_lock<std::mutex> lck(m_lck);
    return m_queData.size() == m_size; 
  }
 
   /**
    * @description:
    * @return {*}
    * @author: hongyuan.liu@corenetic.ai
    */
   bool isEmpty() const { 
    std::unique_lock<std::mutex> lck(m_lck);
    return m_queData.size() == 0;
   }
 
  private:
   void pop_() {
     T tmp = m_queData.front();
     m_queData.pop();
 
     if (m_funCb) {
        m_funCb(tmp);
     }
   }
 
  private:
   size_t m_size = 0;
   std::function<void(T)> m_funCb = nullptr;
   std::string m_tag = "";
 
   mutable std::mutex m_lck;
   std::condition_variable m_cond;
   std::queue<T> m_queData;
 };