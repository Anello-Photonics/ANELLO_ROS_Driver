#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

class base_interface {

public:
    /*
    * Parameters:
    * char *buf : char array that will be written over with as much anello unit data as possible 
    * size_t buf_len : size of the buffer in 'char *buf'
    *
    * Return:
    * Number of bytes written 'char *buf'
    */
    virtual size_t get_data(char *buf, size_t buf_len) { return 0; }
};



#endif
