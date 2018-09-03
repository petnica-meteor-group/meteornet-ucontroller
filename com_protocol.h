/*
 * com_protocol.h
 *
 *  Copyright 2018 Vladimir Nikolić
 */

#ifndef COMM_PROTOCOL_H
#define COMM_PROTOCOL_H

enum Command {
    SHUTTER_OPEN = 0,
    SHUTTER_CLOSE,
    CAMERA_TURN_ON,
    CAMERA_TURN_OFF,
    DHT_INFO_GET,
    PSU_STATUS_GET,
    CAMERA_VOLTAGE_GET
};

#endif /* COM_PROTOCOL_H */
