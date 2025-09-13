/*
 * ring_queue.h
 *
 *  Created on: Mar 11, 2025
 *      Author: 19325
 */

#ifndef RING_QUEUE_H_
#define RING_QUEUE_H_

#include "sr_portable.h"

template<typename T>
class RingQueue {
public:
    // 构造与析构。
    RingQueue();
    virtual ~RingQueue();
    // 设置存储空间，及其大小。
    void setBuff(T *buff_val, S16 buff_len);
    // 从尾部加入一个数据。
    void push(T &d);
    // 从头部弹出一个数据
    T pop(void);
    // 头部数据。
    T head(Boolean &is_ok);
    // 尾部数据。
    T tail(Boolean &is_ok);
    // 当前数据个数。
    S16 size(void);
    // 获取指定位置的数据。
    T at(S16 index);
    // 空/满判断。
    inline Boolean isEmpty(void);
    inline Boolean isFull(void);

private:
    T *buff;
    S16 len;
    S16 write_index;
    S16 read_index;
};

/**************************************************************************************************
 * 构造。
 *************************************************************************************************/
template<typename T>
RingQueue<T>::RingQueue() {
    buff = NULL;
    len = 0;
    write_index = 0;
    read_index = 0;
}

/**************************************************************************************************
 * 析构。
 *************************************************************************************************/
template<typename T>
RingQueue<T>::~RingQueue() {

}

/**************************************************************************************************
 * 功能： 设置存储空间，及其大小。
 * 参数：
 *      buff： 数据空间。
 *      len：数据空间长度。
 *************************************************************************************************/
template<typename T>
void RingQueue<T>::setBuff(T *buff_val, S16 buff_len) {
    buff = buff_val;
    len = buff_len;
    write_index = 0;
    read_index = 0;
}

/**************************************************************************************************
 * 功能： 从尾部加入一个数据。
 * 参数：
 *      d： 新数据。
 *************************************************************************************************/
template<typename T>
void RingQueue<T>::push(T &d) {
    if (isFull() == SR_TRUE) {
        return;
    }

    buff[write_index] = d;
    write_index = (write_index + 1) % len;
}

/**************************************************************************************************
 * 功能： 从头部弹出一个数据。
 * 返回： 头部数据。
 *************************************************************************************************/
template<typename T>
T RingQueue<T>::pop(void) {
    T d = T();
    if (isEmpty() == SR_TRUE) {
        return d;
    }

    d = buff[read_index];
    read_index = (read_index + 1) % len;
    return d;
}

/**************************************************************************************************
 * 功能： 得到头部数据。
 * 返回： 头部数据。
 *************************************************************************************************/
template<typename T>
T RingQueue<T>::head(Boolean &is_oK) {
    T d = T();
    if (isEmpty() == SR_TRUE) {
        is_oK = SR_FALSE;
        return d;
    }

    is_oK = SR_TRUE;
    return buff[read_index];
}

/**************************************************************************************************
 * 功能： 得到尾部数据。
 * 返回： 尾部数据。
 *************************************************************************************************/
template<typename T>
T RingQueue<T>::tail(Boolean &is_oK) {
    T d = T();
    if (isEmpty() == SR_TRUE) {
        is_oK = SR_FALSE;
        return d;
    }

    is_oK = SR_TRUE;
    return buff[(write_index - 1 + len) % len];
}

/**************************************************************************************************
 * 功能： 当前数据个数。
 * 返回： 数据个数。
 *************************************************************************************************/
template<typename T>
S16 RingQueue<T>::size(void) {
    S16 wi = write_index;
    S16 ri = read_index;
    return (wi >= ri) ? (wi - ri) : (len - ri + wi);
}

/**************************************************************************************************
 * 功能： 获取指定位置的数据。
 * 参数：
 *      index：数据位置。
 * 返回： 指定位置的数据。
 *************************************************************************************************/
template<typename T>
T RingQueue<T>::at(S16 index) {
    return buff[(read_index + index) % len];
}

/**************************************************************************************************
 * 功能： 是否空。
 * 返回：
 *      true，空； false，非空。
 *************************************************************************************************/
template<typename T>
inline Boolean RingQueue<T>::isEmpty(void) {
    return (read_index == write_index) ? SR_TRUE: SR_FALSE;
}

/**************************************************************************************************
 * 功能： 是否满。
 * 返回：
 *      true，满； false，非满。
 *************************************************************************************************/
template<typename T>
inline Boolean RingQueue<T>::isFull(void) {
    // 使用少存放一个数据的方式判断满。
    S16 wi = write_index;
    S16 ri = read_index;
    return ((wi + 1) % len == ri) ? SR_TRUE: SR_FALSE;
}



#endif /* RING_QUEUE_H_ */
