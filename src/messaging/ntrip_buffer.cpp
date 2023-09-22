
/********************************************************************************
 * File Name:   ntrip_buffer.cpp
 * Description: definition for ntrip_buffer.h
 *
 * Author:      Austin Johnson
 * Date:        8/18/23
 *
 * License:     MIT License
 *
 * Note:        
 ********************************************************************************/



#include "ntrip_buffer.h"
#include <cstring>

port_buffer global_ntrip_buffer;
port_buffer global_config_buffer;

port_buffer::port_buffer()
{
    this->buffer = new uint8_t[NTRIP_BUFFER_SIZE];
    this->read_ready = false;
    this->bytes_used = 0;
}

port_buffer::~port_buffer()
{
    delete[] this->buffer;
}

void port_buffer::add_data_to_buffer(const uint8_t *buf, int len)
{
    this->clear_buffer();

    for (int i = 0; i < len; i++)
    {
        this->buffer[i] = buf[i];
    }

    this->read_ready = true;
    this->bytes_used = len;
}

void port_buffer::clear_buffer()
{
    memset(this->buffer, 0, NTRIP_BUFFER_SIZE*sizeof(uint8_t));
    this->read_ready = false;
    this->bytes_used = 0;
}

int port_buffer::get_buffer_length()
{
    return this->bytes_used;
}

const uint8_t *port_buffer::get_buffer()
{
    return this->buffer;
}

bool port_buffer::is_read_ready()
{
    return this->read_ready;
}

void port_buffer::set_read_ready_false()
{
    this->read_ready = false;
}

