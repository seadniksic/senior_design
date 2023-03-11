#ifndef UARTWRITEBUFFER_H_
#define UARTWRITEBUFFER_H_

#include "WriteBufferInterface.h"

class UartWriteBuffer : public ::EmbeddedProto::WriteBufferInterface
{
    //! Store a maximum of MAX_SIZE bytes in the buffer
    static constexpr uint32_t MAX_SIZE = 50;

  public:
    UartWriteBuffer() = default;
    ~UartWriteBuffer() override = default;

    //!  ::EmbeddedProto::WriteBufferInterface::clear()
    virtual void clear() override;

    //!  ::EmbeddedProto::WriteBufferInterface::get_size()
    virtual uint32_t get_size() const override;

    //!  ::EmbeddedProto::WriteBufferInterface::get_max_size()
    virtual uint32_t get_max_size() const override;

    //!  ::EmbeddedProto::WriteBufferInterface::get_available_size()
    virtual uint32_t get_available_size() const override;

    //!  ::EmbeddedProto::WriteBufferInterface::push()
    virtual bool push(const uint8_t byte) override;

    //!  ::EmbeddedProto::WriteBufferInterface::push()
    virtual bool push(const uint8_t* bytes, const uint32_t length) override;

    //! Return a pointer to the data array.
    uint8_t* get_data();

  private:

    //! The array in which the serialized data is stored.
    uint8_t data_[MAX_SIZE];

    //! The number of bytes currently serialized in the array.
    uint32_t write_index_;

};


#endif
