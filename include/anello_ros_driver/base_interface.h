#ifndef BASE_INTERFACE_H
#define BASE_INTERFACE_H

class base_interface {

public:
    virtual size_t get_data(char *buf, size_t buf_len) { return 0; }
};



#endif
