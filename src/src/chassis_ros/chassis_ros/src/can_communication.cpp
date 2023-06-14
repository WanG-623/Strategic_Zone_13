#include "can_communication.h"


//namespace chassis_can {
CHASSIS_CAN::CHASSIS_CAN() :
    can_("can2") {
}

CHASSIS_CAN::CHASSIS_CAN(std::string port) :
    can_(port.c_str()) {
}

CHASSIS_CAN::~CHASSIS_CAN() {
}


bool CHASSIS_CAN::init(const char* device, int bitRate)
{
    return 1;
}
void CHASSIS_CAN::close()
{

}
bool CHASSIS_CAN::setEntry(int index, const float* data_address)
{
    uint32_t frame_id = 01;
    uint8_t dlc = 6;
    uint8_t data[6] = {0};
    float value = *data_address;
    data[0] = 01;
    data[1] = index;
    can_msg msg;
    msg.f= value;
    data[2] = msg.m.a;
    data[3] = msg.m.b;
    data[4] = msg.m.c;
    data[5] = msg.m.d;
    can_.write(frame_id,dlc,data);
    uint32_t frame_id_fb;
    uint8_t dlc_fb;
    uint8_t data_fb[6] = {0};
    bool read_status = can_.read(&frame_id_fb, &dlc_fb, data_fb);
    if (!read_status) {
        return false;
    }
    //ToDo 检验can应答 
    return true;
    }
bool CHASSIS_CAN::setEntry(int index, const int* data_address)
{
    uint32_t frame_id = 01;
    uint8_t dlc = 6;
    uint8_t data[6] = {0};
    int value = *data_address;
    data[0] = 01;
    data[1] = index;
    can_msg_int msg;
    msg.i= value;
    data[2] = msg.m.a;
    data[3] = msg.m.b;
    data[4] = msg.m.c;
    data[5] = msg.m.d;
    can_.write(frame_id,dlc,data);
    uint32_t frame_id_fb;
    uint8_t dlc_fb;
    uint8_t data_fb[6] = {0};
    bool read_status = can_.read(&frame_id_fb, &dlc_fb, data_fb);
    if (!read_status) {
        return false;
    }
    //ToDo 检验can应答 
    return true;
    }
bool CHASSIS_CAN::setEntry(int index, const char* data_address)
    {
    //ToDO
    printf("ToDo");
    }

bool CHASSIS_CAN::getEntry(int index, float* data_address)
    {
     uint32_t frame_id = 01;
    uint8_t dlc = 2;
    uint8_t data[2] = {0};
    //float* value;
    can_msg msg;


    data[0] = 02;
    data[1] = index;
    can_.write(frame_id,dlc,data);
    uint32_t frame_id_fb;
    uint8_t dlc_fb;
    uint8_t data_fb[6] = {0};
    for (int i = 0; i < 2; i++)
    {
        bool read_status = can_.read(&frame_id_fb, &dlc_fb, data_fb);
        if (!read_status ) {
            return false;
        }
        if (frame_id_fb=10)
        {
            //value = reinterpret_cast<float*>(&data_fb);
            msg.m.a=data_fb[2];
            msg.m.b=data_fb[3];
            msg.m.c=data_fb[4];
            msg.m.d=data_fb[5];
            //printf("%f \r\n",msg.e);

            memcpy(data_address,&msg,sizeof(float)); 
            return true;      
        }
        
    }

    //ToDo 检验can应答
    return false; 
}
bool CHASSIS_CAN::getEntry(int index, int* data_address)
    {
    uint32_t frame_id = 01;
    uint8_t dlc = 2;
    uint8_t data[2] = {0};
    //float* value;
    can_msg_int msg;


    data[0] = 02;
    data[1] = index;
    can_.write(frame_id,dlc,data);
    uint32_t frame_id_fb;
    uint8_t dlc_fb;
    uint8_t data_fb[6] = {0};
    for (int i = 0; i < 2; i++)
    {
        bool read_status = can_.read(&frame_id_fb, &dlc_fb, data_fb);
        if (!read_status ) {
            return false;
        }
        if (frame_id_fb=10)
        {
            //value = reinterpret_cast<float*>(&data_fb);
            msg.m.a=data_fb[2];
            msg.m.b=data_fb[3];
            msg.m.c=data_fb[4];
            msg.m.d=data_fb[5];
            //printf("%f \r\n",msg.e);

            memcpy(data_address,&msg,sizeof(int)); 
            return true;      
        }
        
    }

    //ToDo 检验can应答
    return false;
    }
bool CHASSIS_CAN::getEntry(int index, char* data_address)
{
    printf("ToDo");
}
//}
