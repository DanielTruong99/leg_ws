#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H
    #include <iostream>
    

    namespace motor_config
    {
        class MotorConfig
        {
            public:
                std::string topic_name;
                uint8_t index;
                MotorConfig(std::string topic_name, uint8_t index) : topic_name(topic_name), index(index) {}
        };

    }

#endif /* MOTOR_CONFIG_H */