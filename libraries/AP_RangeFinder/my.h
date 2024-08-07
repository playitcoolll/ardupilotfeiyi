#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>
#include <AP_Param/AP_Param.h>




class AP_aoassa_Backend;

class aoassa
{



public:
    aoassa();

    /* Do not allow copies */
    CLASS_NO_COPY(aoassa);



    // detect and initialise any available rangefinders
    void init();

    // update state of all rangefinders. Should be called at around
    // 10Hz from main loop
    void update(void);



    float get_aoa() const {
        return _aoa;
    }

    // AP_Float get_ssa() const {
    //     return _ssa;
    // }
    static aoassa *get_singleton() {
         return _singleton; 
    }

    float _aoa;
    float _ssa;
    
protected:

    


private:
    static aoassa *_singleton;
    uint32_t _last_update_ms;
};

namespace AP {
    aoassa *Aoassa();
};
