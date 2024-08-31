#include "ActiveObject.h"

namespace ao
{
    ActiveObject::ActiveObject()
    {
    }

    void ActiveObject::pushEvent(const Event * const event)
    {
        this->_event_queue.push(Event(event->_signal));
    }

    void ActiveObject::dispatch(const Event * const event)
    {
        Status status;
        Status (ActiveObject::*prev_state)(const Event * const) = _state;
        static Event const entry_event(ENTRY_SIG);
        static Event const exit_event(EXIT_SIG);

        status = (this->*_state)(event);
        if(status == TRAN_STATUS)
        {
            (this->*prev_state)((Event *)&exit_event);
            (this->*_state)((Event *)&entry_event);
        }
    }

    void ActiveObject::loopEvent()
    {
        Event event(DEFAULT_SIG);
        if(!_event_queue.empty())
        {
            event = _event_queue.front();
            _event_queue.pop();
        }

        this->dispatch((Event *)&event);
    }
}

