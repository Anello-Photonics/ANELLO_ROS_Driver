#ifndef NTRIP_BUFFER_H
#define NTRIP_BUFFER_H
/********************************************************************************
 * File Name:   ntrip_buffer.h
 * Description: class declaration and header file for ntrip_buffer.cpp
 *
 * Author:      Austin Johnson
 * Date:        8/18/23
 *
 * License:     MIT License
 *
 * Note:        
 ********************************************************************************/

#include <cstdint>

#ifndef NTRIP_BUFFER_SIZE
#define NTRIP_BUFFER_SIZE 4*180
#endif

class NTRIP_buffer {
private:
    bool read_ready;
    uint8_t *buffer;
	uint32_t bytes_used;
public:
	NTRIP_buffer();
	~NTRIP_buffer();
	void add_ntrip_data(const uint8_t *buf, int len);
	void set_read_ready_false();
	void clear_buffer();
	int get_buffer_length();
	const uint8_t *get_buffer();
    bool is_read_ready();
};

extern NTRIP_buffer global_ntrip_buffer;

#endif