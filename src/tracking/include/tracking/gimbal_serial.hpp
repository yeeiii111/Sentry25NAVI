#ifndef GIMBAL_SERIAL_H
#define GIMBAL_SERIAL_H
#include  <string>
namespace gimbal_serial_msg{

struct serial_receive_msg
{
    serial_receive_msg()
    : header(0)
    , team(0)
    , goal(0)
    , tailer(0){}

    uint8_t header;
    uint8_t team;
    uint8_t goal;
    uint8_t tailer;
};

// struct serial_send_msg
// {
//     serial_send_msg()
//     : header(0)
//     , v_x(0)
//     , v_y(0)
//     , v_z(0)
//     , w_z(0)
//     , tailer(0){}

//     uint8_t header;
//     float   v_x;
//     float   v_y;
//     float   v_z;//0可以不跟随云台；1跟随云台；2抵达目的地，大yaw可由自瞄停下
//     float   w_z;
//     uint8_t tailer;
// }__attribute__((packed));
struct serial_send_msg
{
    serial_send_msg()
    : header(0)
    , v_x(0)
    , v_y(0)
    , v_z(0)
    , w_z(0)
    , stuck_trigger(0)
    , tailer(0){}

    uint8_t header;
    float   v_x;
    float   v_y;
    float   v_z;//0可以不跟随云台；1跟随云台；2抵达目的地，大yaw可由自瞄停下
    float   w_z;
    uint8_t stuck_trigger; // 新增，serial.cpp有用到
    uint8_t tailer;
}__attribute__((packed));
}
#endif