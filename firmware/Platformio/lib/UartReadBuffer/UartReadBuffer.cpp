#include "UartReadBuffer.h"

UartReadBuffer::UartReadBuffer()
  : data_{0},
    write_index_(0),
    read_index_(0)
{

}

uint32_t UartReadBuffer::get_size() const
{
  return write_index_;
}

uint32_t UartReadBuffer::get_max_size() const
{
  return MAX_SIZE;
}

bool UartReadBuffer::peek(uint8_t& byte) const
{
  const bool return_value = write_index_ > read_index_;
  if(return_value)
  {
    byte = data_[read_index_];
  }
  return return_value;
}

bool UartReadBuffer::advance()
{
  const bool return_value = write_index_ > read_index_;
  if(return_value)
  {
    ++read_index_;
  }
  return return_value;
}

bool UartReadBuffer::advance(const uint32_t N)
{
  const uint32_t new_read_index = read_index_ + N;
  const bool return_value = write_index_ > new_read_index;
  if(return_value)
  {
    read_index_ = new_read_index;
  }
  return return_value;
}

bool UartReadBuffer::pop(uint8_t& byte)
{
  const bool return_value = write_index_ > read_index_;
  if(return_value) {
    byte = data_[read_index_];
    ++read_index_;
  }
  return return_value;
}

uint8_t* UartReadBuffer::get_data_array()
{
  return data_;
}

uint32_t& UartReadBuffer::get_bytes_written()
{
  return write_index_;
}

void UartReadBuffer::clear()
{
  read_index_ = 0;
  write_index_ = 0;
}

bool UartReadBuffer::push(uint8_t& byte)
{
  const bool return_value = MAX_SIZE > write_index_;
  if(return_value)
  {
    data_[write_index_] = byte;
    ++write_index_;
  }
  return return_value;
}