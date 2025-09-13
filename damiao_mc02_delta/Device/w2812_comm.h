/*
 * w2812_comm.h
 *
 *  Created on: Mar 10, 2025
 *      Author: 19325
 */

#ifndef W2812_COMM_H_
#define W2812_COMM_H_

#include "sr_portable.h"

class W2812Comm {
public:
    static W2812Comm &construction(void);

    void ctrlColor(U8 r, U8 g, U8 b);
private:
    W2812Comm();
    W2812Comm(const W2812Comm&);
    W2812Comm &operator=(const W2812Comm&);

};

#define w2812_comm (W2812Comm::construction())

#endif /* W2812_COMM_H_ */
