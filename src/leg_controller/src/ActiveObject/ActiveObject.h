#ifndef ACTIVE_OBJECT_H
#define ACTIVE_OBJECT_H

#include <iostream>
#include <unistd.h>
#include <queue>

namespace ao
{
    enum Status
    {
        TRAN_STATUS, 
        HANDLED_STATUS, 
        IGNORED_STATUS, 
        INIT_STATUS
    };

    enum ReservedSignal
    { 
        INIT_SIG, 
        ENTRY_SIG, 
        EXIT_SIG, 
        DEFAULT_SIG,
        USER_SIG 
    };

    class Event
    {
        public:
            uint16_t _signal;
            Event(uint16_t signal){_signal = signal;}
    };


    class ActiveObject
    {
        public:
            typedef Status (ActiveObject::*StateHandler)(const Event * const);
            Status (ActiveObject::*_state)(const Event * const);

            ActiveObject();
            void loopEvent();
            void pushEvent(const Event * const event);

        private:    
            std::queue<Event> _event_queue;

            void dispatch(const Event * const event);
    };
}



#endif /* ACTIVE_OBJECT_H */