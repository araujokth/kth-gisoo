
#ifndef TEST_SERIAL_H
#define TEST_SERIAL_H

typedef nx_struct test_serial_msg {
nx_uint16_t counter;
nx_uint16_t counter2[7];

} test_serial_msg_t;

enum {
  AM_TEST_SERIAL_MSG = 6, 
};

#endif
