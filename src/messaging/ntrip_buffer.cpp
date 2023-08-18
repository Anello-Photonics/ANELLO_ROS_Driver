
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

NTRIP_buffer global_ntrip_buffer;

NTRIP_buffer::NTRIP_buffer()
{
    this->buffer = new uint8_t[NTRIP_BUFFER_SIZE];
    this->read_ready = false;
    this->bytes_used = 0;
}

NTRIP_buffer::~NTRIP_buffer()
{
    delete[] this->buffer;
}

void NTRIP_buffer::add_ntrip_data(const uint8_t *buf, int len)
{
    this->clear_buffer();

    for (int i = 0; i < len; i++)
    {
        this->buffer[i] = buf[i];
    }

    this->read_ready = true;
    this->bytes_used = len;
}

void NTRIP_buffer::clear_buffer()
{
    memset(this->buffer, 0, NTRIP_BUFFER_SIZE*sizeof(uint8_t));
    this->read_ready = false;
    this->bytes_used = 0;
}

int NTRIP_buffer::get_buffer_length()
{
    return this->bytes_used;
}

const uint8_t *NTRIP_buffer::get_buffer()
{
    return this->buffer;
}

bool NTRIP_buffer::is_read_ready()
{
    return this->read_ready;
}

void NTRIP_buffer::set_read_ready_false()
{
    this->read_ready = false;
}

