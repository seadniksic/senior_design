/*
 *  Copyright (C) 2020-2023 Embedded AMS B.V. - All Rights Reserved
 *
 *  This file is part of Embedded Proto.
 *
 *  Embedded Proto is open source software: you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License as published
 *  by the Free Software Foundation, version 3 of the license.
 *
 *  Embedded Proto  is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with Embedded Proto. If not, see <https://www.gnu.org/licenses/>.
 *
 *  For commercial and closed source application please visit:
 *  <https://EmbeddedProto.com/license/>.
 *
 *  Embedded AMS B.V.
 *  Info:
 *    info at EmbeddedProto dot com
 *
 *  Postal address:
 *    Johan Huizingalaan 763a
 *    1066 VH, Amsterdam
 *    the Netherlands
 */

// This file is generated. Please do not edit!
#ifndef UART_MESSAGES_H
#define UART_MESSAGES_H

#include <cstdint>
#include <MessageInterface.h>
#include <WireFormatter.h>
#include <Fields.h>
#include <MessageSizeCalculator.h>
#include <ReadBufferSection.h>
#include <RepeatedFieldFixedSize.h>
#include <FieldStringBytes.h>
#include <Errors.h>
#include <Defines.h>
#include <limits>

// Include external proto definitions


class GUI_Data final: public ::EmbeddedProto::MessageInterface
{
  public:
    GUI_Data() = default;
    GUI_Data(const GUI_Data& rhs )
    {
      set_cpu_temp(rhs.get_cpu_temp());
    }

    GUI_Data(const GUI_Data&& rhs ) noexcept
    {
      set_cpu_temp(rhs.get_cpu_temp());
    }

    ~GUI_Data() override = default;

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      CPU_TEMP = 1
    };

    GUI_Data& operator=(const GUI_Data& rhs)
    {
      set_cpu_temp(rhs.get_cpu_temp());
      return *this;
    }

    GUI_Data& operator=(const GUI_Data&& rhs) noexcept
    {
      set_cpu_temp(rhs.get_cpu_temp());
      return *this;
    }

    static constexpr char const* CPU_TEMP_NAME = "cpu_temp";
    inline void clear_cpu_temp() { cpu_temp_.clear(); }
    inline void set_cpu_temp(const int32_t& value) { cpu_temp_ = value; }
    inline void set_cpu_temp(const int32_t&& value) { cpu_temp_ = value; }
    inline int32_t& mutable_cpu_temp() { return cpu_temp_.get(); }
    inline const int32_t& get_cpu_temp() const { return cpu_temp_.get(); }
    inline int32_t cpu_temp() const { return cpu_temp_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if((0 != cpu_temp_.get()) && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = cpu_temp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CPU_TEMP), buffer, false);
      }

      return return_value;
    };

    ::EmbeddedProto::Error deserialize(::EmbeddedProto::ReadBufferInterface& buffer) override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;
      ::EmbeddedProto::WireFormatter::WireType wire_type = ::EmbeddedProto::WireFormatter::WireType::VARINT;
      uint32_t id_number = 0;
      FieldNumber id_tag = FieldNumber::NOT_SET;

      ::EmbeddedProto::Error tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
      while((::EmbeddedProto::Error::NO_ERRORS == return_value) && (::EmbeddedProto::Error::NO_ERRORS == tag_value))
      {
        id_tag = static_cast<FieldNumber>(id_number);
        switch(id_tag)
        {
          case FieldNumber::CPU_TEMP:
            return_value = cpu_temp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::NOT_SET:
            return_value = ::EmbeddedProto::Error::INVALID_FIELD_ID;
            break;

          default:
            return_value = skip_unknown_field(buffer, wire_type);
            break;
        }

        if(::EmbeddedProto::Error::NO_ERRORS == return_value)
        {
          // Read the next tag.
          tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
        }
      }

      // When an error was detect while reading the tag but no other errors where found, set it in the return value.
      if((::EmbeddedProto::Error::NO_ERRORS == return_value)
         && (::EmbeddedProto::Error::NO_ERRORS != tag_value)
         && (::EmbeddedProto::Error::END_OF_BUFFER != tag_value)) // The end of the buffer is not an array in this case.
      {
        return_value = tag_value;
      }

      return return_value;
    };

    void clear() override
    {
      clear_cpu_temp();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::CPU_TEMP:
          name = CPU_TEMP_NAME;
          break;
        default:
          name = "Invalid FieldNumber";
          break;
      }
      return name;
    }

#ifdef MSG_TO_STRING

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str) const
    {
      return this->to_string(str, 0, nullptr, true);
    }

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str, const uint32_t indent_level, char const* name, const bool first_field) const override
    {
      ::EmbeddedProto::string_view left_chars = str;
      int32_t n_chars_used = 0;

      if(!first_field)
      {
        // Add a comma behind the previous field.
        n_chars_used = snprintf(left_chars.data, left_chars.size, ",\n");
        if(0 < n_chars_used)
        {
          // Update the character pointer and characters left in the array.
          left_chars.data += n_chars_used;
          left_chars.size -= n_chars_used;
        }
      }

      if(nullptr != name)
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "\"%s\": {\n", name);
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s\"%s\": {\n", indent_level, " ", name);
        }
      }
      else
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "{\n");
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s{\n", indent_level, " ");
        }
      }
      
      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      left_chars = cpu_temp_.to_string(left_chars, indent_level + 2, CPU_TEMP_NAME, true);
  
      if( 0 == indent_level) 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n}");
      }
      else 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n%*s}", indent_level, " ");
      }

      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      return left_chars;
    }

#endif // End of MSG_TO_STRING

  private:


      EmbeddedProto::int32 cpu_temp_ = 0;

};

class Joystick_Input final: public ::EmbeddedProto::MessageInterface
{
  public:
    Joystick_Input() = default;
    Joystick_Input(const Joystick_Input& rhs )
    {
      if(rhs.has_button())
      {
        set_button(rhs.get_button());
      }
      else
      {
        clear_button();
      }

      if(rhs.has_LJOY_X())
      {
        set_LJOY_X(rhs.get_LJOY_X());
      }
      else
      {
        clear_LJOY_X();
      }

      if(rhs.has_LJOY_Y())
      {
        set_LJOY_Y(rhs.get_LJOY_Y());
      }
      else
      {
        clear_LJOY_Y();
      }

      if(rhs.has_RJOY_X())
      {
        set_RJOY_X(rhs.get_RJOY_X());
      }
      else
      {
        clear_RJOY_X();
      }

      if(rhs.has_RJOY_Y())
      {
        set_RJOY_Y(rhs.get_RJOY_Y());
      }
      else
      {
        clear_RJOY_Y();
      }

      if(rhs.has_TR())
      {
        set_TR(rhs.get_TR());
      }
      else
      {
        clear_TR();
      }

      if(rhs.has_TL())
      {
        set_TL(rhs.get_TL());
      }
      else
      {
        clear_TL();
      }

    }

    Joystick_Input(const Joystick_Input&& rhs ) noexcept
    {
      if(rhs.has_button())
      {
        set_button(rhs.get_button());
      }
      else
      {
        clear_button();
      }

      if(rhs.has_LJOY_X())
      {
        set_LJOY_X(rhs.get_LJOY_X());
      }
      else
      {
        clear_LJOY_X();
      }

      if(rhs.has_LJOY_Y())
      {
        set_LJOY_Y(rhs.get_LJOY_Y());
      }
      else
      {
        clear_LJOY_Y();
      }

      if(rhs.has_RJOY_X())
      {
        set_RJOY_X(rhs.get_RJOY_X());
      }
      else
      {
        clear_RJOY_X();
      }

      if(rhs.has_RJOY_Y())
      {
        set_RJOY_Y(rhs.get_RJOY_Y());
      }
      else
      {
        clear_RJOY_Y();
      }

      if(rhs.has_TR())
      {
        set_TR(rhs.get_TR());
      }
      else
      {
        clear_TR();
      }

      if(rhs.has_TL())
      {
        set_TL(rhs.get_TL());
      }
      else
      {
        clear_TL();
      }

    }

    ~Joystick_Input() override = default;

    enum class Buttons : uint32_t
    {
      NONE = 0,
      BTN_A = 1,
      BTN_B = 2,
      BTN_X = 4,
      BTN_Y = 8,
      BTN_START = 16,
      BTN_SELECT = 32,
      DPAD_UP = 64,
      DPAD_DOWN = 128,
      DPAD_LEFT = 256,
      DPAD_RIGHT = 512,
      BTN_THUMBR = 1024,
      BTN_THUMBL = 2048,
      BTN_TR = 4096,
      BTN_TL = 8192
    };

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      BUTTON = 1,
      LJOY_X = 2,
      LJOY_Y = 3,
      RJOY_X = 4,
      RJOY_Y = 5,
      TR = 6,
      TL = 7
    };

    Joystick_Input& operator=(const Joystick_Input& rhs)
    {
      if(rhs.has_button())
      {
        set_button(rhs.get_button());
      }
      else
      {
        clear_button();
      }

      if(rhs.has_LJOY_X())
      {
        set_LJOY_X(rhs.get_LJOY_X());
      }
      else
      {
        clear_LJOY_X();
      }

      if(rhs.has_LJOY_Y())
      {
        set_LJOY_Y(rhs.get_LJOY_Y());
      }
      else
      {
        clear_LJOY_Y();
      }

      if(rhs.has_RJOY_X())
      {
        set_RJOY_X(rhs.get_RJOY_X());
      }
      else
      {
        clear_RJOY_X();
      }

      if(rhs.has_RJOY_Y())
      {
        set_RJOY_Y(rhs.get_RJOY_Y());
      }
      else
      {
        clear_RJOY_Y();
      }

      if(rhs.has_TR())
      {
        set_TR(rhs.get_TR());
      }
      else
      {
        clear_TR();
      }

      if(rhs.has_TL())
      {
        set_TL(rhs.get_TL());
      }
      else
      {
        clear_TL();
      }

      return *this;
    }

    Joystick_Input& operator=(const Joystick_Input&& rhs) noexcept
    {
      if(rhs.has_button())
      {
        set_button(rhs.get_button());
      }
      else
      {
        clear_button();
      }
      
      if(rhs.has_LJOY_X())
      {
        set_LJOY_X(rhs.get_LJOY_X());
      }
      else
      {
        clear_LJOY_X();
      }
      
      if(rhs.has_LJOY_Y())
      {
        set_LJOY_Y(rhs.get_LJOY_Y());
      }
      else
      {
        clear_LJOY_Y();
      }
      
      if(rhs.has_RJOY_X())
      {
        set_RJOY_X(rhs.get_RJOY_X());
      }
      else
      {
        clear_RJOY_X();
      }
      
      if(rhs.has_RJOY_Y())
      {
        set_RJOY_Y(rhs.get_RJOY_Y());
      }
      else
      {
        clear_RJOY_Y();
      }
      
      if(rhs.has_TR())
      {
        set_TR(rhs.get_TR());
      }
      else
      {
        clear_TR();
      }
      
      if(rhs.has_TL())
      {
        set_TL(rhs.get_TL());
      }
      else
      {
        clear_TL();
      }
      
      return *this;
    }

    static constexpr char const* BUTTON_NAME = "button";
    inline bool has_button() const
    {
      return 0 != (presence::mask(presence::fields::BUTTON) & presence_[presence::index(presence::fields::BUTTON)]);
    }
    inline void clear_button()
    {
      presence_[presence::index(presence::fields::BUTTON)] &= ~(presence::mask(presence::fields::BUTTON));
      button_.clear();
    }
    inline void set_button(const Buttons& value)
    {
      presence_[presence::index(presence::fields::BUTTON)] |= presence::mask(presence::fields::BUTTON);
      button_ = value;
    }
    inline void set_button(const Buttons&& value)
    {
      presence_[presence::index(presence::fields::BUTTON)] |= presence::mask(presence::fields::BUTTON);
      button_ = value;
    }
    inline const Buttons& get_button() const { return button_.get(); }
    inline Buttons button() const { return button_.get(); }

    static constexpr char const* LJOY_X_NAME = "LJOY_X";
    inline bool has_LJOY_X() const
    {
      return 0 != (presence::mask(presence::fields::LJOY_X) & presence_[presence::index(presence::fields::LJOY_X)]);
    }
    inline void clear_LJOY_X()
    {
      presence_[presence::index(presence::fields::LJOY_X)] &= ~(presence::mask(presence::fields::LJOY_X));
      LJOY_X_.clear();
    }
    inline void set_LJOY_X(const int32_t& value)
    {
      presence_[presence::index(presence::fields::LJOY_X)] |= presence::mask(presence::fields::LJOY_X);
      LJOY_X_ = value;
    }
    inline void set_LJOY_X(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::LJOY_X)] |= presence::mask(presence::fields::LJOY_X);
      LJOY_X_ = value;
    }
    inline int32_t& mutable_LJOY_X()
    {
      presence_[presence::index(presence::fields::LJOY_X)] |= presence::mask(presence::fields::LJOY_X);
      return LJOY_X_.get();
    }
    inline const int32_t& get_LJOY_X() const { return LJOY_X_.get(); }
    inline int32_t LJOY_X() const { return LJOY_X_.get(); }

    static constexpr char const* LJOY_Y_NAME = "LJOY_Y";
    inline bool has_LJOY_Y() const
    {
      return 0 != (presence::mask(presence::fields::LJOY_Y) & presence_[presence::index(presence::fields::LJOY_Y)]);
    }
    inline void clear_LJOY_Y()
    {
      presence_[presence::index(presence::fields::LJOY_Y)] &= ~(presence::mask(presence::fields::LJOY_Y));
      LJOY_Y_.clear();
    }
    inline void set_LJOY_Y(const int32_t& value)
    {
      presence_[presence::index(presence::fields::LJOY_Y)] |= presence::mask(presence::fields::LJOY_Y);
      LJOY_Y_ = value;
    }
    inline void set_LJOY_Y(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::LJOY_Y)] |= presence::mask(presence::fields::LJOY_Y);
      LJOY_Y_ = value;
    }
    inline int32_t& mutable_LJOY_Y()
    {
      presence_[presence::index(presence::fields::LJOY_Y)] |= presence::mask(presence::fields::LJOY_Y);
      return LJOY_Y_.get();
    }
    inline const int32_t& get_LJOY_Y() const { return LJOY_Y_.get(); }
    inline int32_t LJOY_Y() const { return LJOY_Y_.get(); }

    static constexpr char const* RJOY_X_NAME = "RJOY_X";
    inline bool has_RJOY_X() const
    {
      return 0 != (presence::mask(presence::fields::RJOY_X) & presence_[presence::index(presence::fields::RJOY_X)]);
    }
    inline void clear_RJOY_X()
    {
      presence_[presence::index(presence::fields::RJOY_X)] &= ~(presence::mask(presence::fields::RJOY_X));
      RJOY_X_.clear();
    }
    inline void set_RJOY_X(const int32_t& value)
    {
      presence_[presence::index(presence::fields::RJOY_X)] |= presence::mask(presence::fields::RJOY_X);
      RJOY_X_ = value;
    }
    inline void set_RJOY_X(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::RJOY_X)] |= presence::mask(presence::fields::RJOY_X);
      RJOY_X_ = value;
    }
    inline int32_t& mutable_RJOY_X()
    {
      presence_[presence::index(presence::fields::RJOY_X)] |= presence::mask(presence::fields::RJOY_X);
      return RJOY_X_.get();
    }
    inline const int32_t& get_RJOY_X() const { return RJOY_X_.get(); }
    inline int32_t RJOY_X() const { return RJOY_X_.get(); }

    static constexpr char const* RJOY_Y_NAME = "RJOY_Y";
    inline bool has_RJOY_Y() const
    {
      return 0 != (presence::mask(presence::fields::RJOY_Y) & presence_[presence::index(presence::fields::RJOY_Y)]);
    }
    inline void clear_RJOY_Y()
    {
      presence_[presence::index(presence::fields::RJOY_Y)] &= ~(presence::mask(presence::fields::RJOY_Y));
      RJOY_Y_.clear();
    }
    inline void set_RJOY_Y(const int32_t& value)
    {
      presence_[presence::index(presence::fields::RJOY_Y)] |= presence::mask(presence::fields::RJOY_Y);
      RJOY_Y_ = value;
    }
    inline void set_RJOY_Y(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::RJOY_Y)] |= presence::mask(presence::fields::RJOY_Y);
      RJOY_Y_ = value;
    }
    inline int32_t& mutable_RJOY_Y()
    {
      presence_[presence::index(presence::fields::RJOY_Y)] |= presence::mask(presence::fields::RJOY_Y);
      return RJOY_Y_.get();
    }
    inline const int32_t& get_RJOY_Y() const { return RJOY_Y_.get(); }
    inline int32_t RJOY_Y() const { return RJOY_Y_.get(); }

    static constexpr char const* TR_NAME = "TR";
    inline bool has_TR() const
    {
      return 0 != (presence::mask(presence::fields::TR) & presence_[presence::index(presence::fields::TR)]);
    }
    inline void clear_TR()
    {
      presence_[presence::index(presence::fields::TR)] &= ~(presence::mask(presence::fields::TR));
      TR_.clear();
    }
    inline void set_TR(const int32_t& value)
    {
      presence_[presence::index(presence::fields::TR)] |= presence::mask(presence::fields::TR);
      TR_ = value;
    }
    inline void set_TR(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::TR)] |= presence::mask(presence::fields::TR);
      TR_ = value;
    }
    inline int32_t& mutable_TR()
    {
      presence_[presence::index(presence::fields::TR)] |= presence::mask(presence::fields::TR);
      return TR_.get();
    }
    inline const int32_t& get_TR() const { return TR_.get(); }
    inline int32_t TR() const { return TR_.get(); }

    static constexpr char const* TL_NAME = "TL";
    inline bool has_TL() const
    {
      return 0 != (presence::mask(presence::fields::TL) & presence_[presence::index(presence::fields::TL)]);
    }
    inline void clear_TL()
    {
      presence_[presence::index(presence::fields::TL)] &= ~(presence::mask(presence::fields::TL));
      TL_.clear();
    }
    inline void set_TL(const int32_t& value)
    {
      presence_[presence::index(presence::fields::TL)] |= presence::mask(presence::fields::TL);
      TL_ = value;
    }
    inline void set_TL(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::TL)] |= presence::mask(presence::fields::TL);
      TL_ = value;
    }
    inline int32_t& mutable_TL()
    {
      presence_[presence::index(presence::fields::TL)] |= presence::mask(presence::fields::TL);
      return TL_.get();
    }
    inline const int32_t& get_TL() const { return TL_.get(); }
    inline int32_t TL() const { return TL_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if(has_button() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = button_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BUTTON), buffer, true);
      }

      if(has_LJOY_X() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = LJOY_X_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LJOY_X), buffer, true);
      }

      if(has_LJOY_Y() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = LJOY_Y_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LJOY_Y), buffer, true);
      }

      if(has_RJOY_X() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = RJOY_X_.serialize_with_id(static_cast<uint32_t>(FieldNumber::RJOY_X), buffer, true);
      }

      if(has_RJOY_Y() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = RJOY_Y_.serialize_with_id(static_cast<uint32_t>(FieldNumber::RJOY_Y), buffer, true);
      }

      if(has_TR() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = TR_.serialize_with_id(static_cast<uint32_t>(FieldNumber::TR), buffer, true);
      }

      if(has_TL() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = TL_.serialize_with_id(static_cast<uint32_t>(FieldNumber::TL), buffer, true);
      }

      return return_value;
    };

    ::EmbeddedProto::Error deserialize(::EmbeddedProto::ReadBufferInterface& buffer) override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;
      ::EmbeddedProto::WireFormatter::WireType wire_type = ::EmbeddedProto::WireFormatter::WireType::VARINT;
      uint32_t id_number = 0;
      FieldNumber id_tag = FieldNumber::NOT_SET;

      ::EmbeddedProto::Error tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
      while((::EmbeddedProto::Error::NO_ERRORS == return_value) && (::EmbeddedProto::Error::NO_ERRORS == tag_value))
      {
        id_tag = static_cast<FieldNumber>(id_number);
        switch(id_tag)
        {
          case FieldNumber::BUTTON:
            presence_[presence::index(presence::fields::BUTTON)] |= presence::mask(presence::fields::BUTTON);
            return_value = button_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LJOY_X:
            presence_[presence::index(presence::fields::LJOY_X)] |= presence::mask(presence::fields::LJOY_X);
            return_value = LJOY_X_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LJOY_Y:
            presence_[presence::index(presence::fields::LJOY_Y)] |= presence::mask(presence::fields::LJOY_Y);
            return_value = LJOY_Y_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::RJOY_X:
            presence_[presence::index(presence::fields::RJOY_X)] |= presence::mask(presence::fields::RJOY_X);
            return_value = RJOY_X_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::RJOY_Y:
            presence_[presence::index(presence::fields::RJOY_Y)] |= presence::mask(presence::fields::RJOY_Y);
            return_value = RJOY_Y_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::TR:
            presence_[presence::index(presence::fields::TR)] |= presence::mask(presence::fields::TR);
            return_value = TR_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::TL:
            presence_[presence::index(presence::fields::TL)] |= presence::mask(presence::fields::TL);
            return_value = TL_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::NOT_SET:
            return_value = ::EmbeddedProto::Error::INVALID_FIELD_ID;
            break;

          default:
            return_value = skip_unknown_field(buffer, wire_type);
            break;
        }

        if(::EmbeddedProto::Error::NO_ERRORS == return_value)
        {
          // Read the next tag.
          tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
        }
      }

      // When an error was detect while reading the tag but no other errors where found, set it in the return value.
      if((::EmbeddedProto::Error::NO_ERRORS == return_value)
         && (::EmbeddedProto::Error::NO_ERRORS != tag_value)
         && (::EmbeddedProto::Error::END_OF_BUFFER != tag_value)) // The end of the buffer is not an array in this case.
      {
        return_value = tag_value;
      }

      return return_value;
    };

    void clear() override
    {
      clear_button();
      clear_LJOY_X();
      clear_LJOY_Y();
      clear_RJOY_X();
      clear_RJOY_Y();
      clear_TR();
      clear_TL();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::BUTTON:
          name = BUTTON_NAME;
          break;
        case FieldNumber::LJOY_X:
          name = LJOY_X_NAME;
          break;
        case FieldNumber::LJOY_Y:
          name = LJOY_Y_NAME;
          break;
        case FieldNumber::RJOY_X:
          name = RJOY_X_NAME;
          break;
        case FieldNumber::RJOY_Y:
          name = RJOY_Y_NAME;
          break;
        case FieldNumber::TR:
          name = TR_NAME;
          break;
        case FieldNumber::TL:
          name = TL_NAME;
          break;
        default:
          name = "Invalid FieldNumber";
          break;
      }
      return name;
    }

#ifdef MSG_TO_STRING

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str) const
    {
      return this->to_string(str, 0, nullptr, true);
    }

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str, const uint32_t indent_level, char const* name, const bool first_field) const override
    {
      ::EmbeddedProto::string_view left_chars = str;
      int32_t n_chars_used = 0;

      if(!first_field)
      {
        // Add a comma behind the previous field.
        n_chars_used = snprintf(left_chars.data, left_chars.size, ",\n");
        if(0 < n_chars_used)
        {
          // Update the character pointer and characters left in the array.
          left_chars.data += n_chars_used;
          left_chars.size -= n_chars_used;
        }
      }

      if(nullptr != name)
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "\"%s\": {\n", name);
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s\"%s\": {\n", indent_level, " ", name);
        }
      }
      else
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "{\n");
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s{\n", indent_level, " ");
        }
      }
      
      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      left_chars = button_.to_string(left_chars, indent_level + 2, BUTTON_NAME, true);
      left_chars = LJOY_X_.to_string(left_chars, indent_level + 2, LJOY_X_NAME, false);
      left_chars = LJOY_Y_.to_string(left_chars, indent_level + 2, LJOY_Y_NAME, false);
      left_chars = RJOY_X_.to_string(left_chars, indent_level + 2, RJOY_X_NAME, false);
      left_chars = RJOY_Y_.to_string(left_chars, indent_level + 2, RJOY_Y_NAME, false);
      left_chars = TR_.to_string(left_chars, indent_level + 2, TR_NAME, false);
      left_chars = TL_.to_string(left_chars, indent_level + 2, TL_NAME, false);
  
      if( 0 == indent_level) 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n}");
      }
      else 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n%*s}", indent_level, " ");
      }

      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      return left_chars;
    }

#endif // End of MSG_TO_STRING

  private:

      // Define constants for tracking the presence of fields.
      // Use a struct to scope the variables from user fields as namespaces are not allowed within classes.
      struct presence
      {
        // An enumeration with all the fields for which presence has to be tracked.
        enum class fields : uint32_t
        {
          BUTTON,
          LJOY_X,
          LJOY_Y,
          RJOY_X,
          RJOY_Y,
          TR,
          TL
        };

        // The number of fields for which presence has to be tracked.
        static constexpr uint32_t N_FIELDS = 7;

        // Which type are we using to track presence.
        using TYPE = uint32_t;

        // How many bits are there in the presence type.
        static constexpr uint32_t N_BITS = std::numeric_limits<TYPE>::digits;

        // How many variables of TYPE do we need to bit mask all presence fields.
        static constexpr uint32_t SIZE = (N_FIELDS / N_BITS) + ((N_FIELDS % N_BITS) > 0 ? 1 : 0);

        // Obtain the index of a given field in the presence array.
        static constexpr uint32_t index(const fields& field) { return static_cast<uint32_t>(field) / N_BITS; }

        // Obtain the bit mask for the given field assuming we are at the correct index in the presence array.
        static constexpr TYPE mask(const fields& field)
        {
          return static_cast<uint32_t>(0x01) << (static_cast<uint32_t>(field) % N_BITS);
        }
      };

      // Create an array in which the presence flags are stored.
      typename presence::TYPE presence_[presence::SIZE] = {0};

      EmbeddedProto::enumeration<Buttons> button_ = static_cast<Buttons>(0);
      EmbeddedProto::int32 LJOY_X_ = 0;
      EmbeddedProto::int32 LJOY_Y_ = 0;
      EmbeddedProto::int32 RJOY_X_ = 0;
      EmbeddedProto::int32 RJOY_Y_ = 0;
      EmbeddedProto::int32 TR_ = 0;
      EmbeddedProto::int32 TL_ = 0;

};

class SLAM_Data final: public ::EmbeddedProto::MessageInterface
{
  public:
    SLAM_Data() = default;
    SLAM_Data(const SLAM_Data& rhs )
    {
      if(rhs.has_lia_x())
      {
        set_lia_x(rhs.get_lia_x());
      }
      else
      {
        clear_lia_x();
      }

      if(rhs.has_lia_y())
      {
        set_lia_y(rhs.get_lia_y());
      }
      else
      {
        clear_lia_y();
      }

      if(rhs.has_lia_z())
      {
        set_lia_z(rhs.get_lia_z());
      }
      else
      {
        clear_lia_z();
      }

      if(rhs.has_eul_y())
      {
        set_eul_y(rhs.get_eul_y());
      }
      else
      {
        clear_eul_y();
      }

      if(rhs.has_eul_p())
      {
        set_eul_p(rhs.get_eul_p());
      }
      else
      {
        clear_eul_p();
      }

      if(rhs.has_eul_r())
      {
        set_eul_r(rhs.get_eul_r());
      }
      else
      {
        clear_eul_r();
      }

      if(rhs.has_pan())
      {
        set_pan(rhs.get_pan());
      }
      else
      {
        clear_pan();
      }

      if(rhs.has_tilt())
      {
        set_tilt(rhs.get_tilt());
      }
      else
      {
        clear_tilt();
      }

    }

    SLAM_Data(const SLAM_Data&& rhs ) noexcept
    {
      if(rhs.has_lia_x())
      {
        set_lia_x(rhs.get_lia_x());
      }
      else
      {
        clear_lia_x();
      }

      if(rhs.has_lia_y())
      {
        set_lia_y(rhs.get_lia_y());
      }
      else
      {
        clear_lia_y();
      }

      if(rhs.has_lia_z())
      {
        set_lia_z(rhs.get_lia_z());
      }
      else
      {
        clear_lia_z();
      }

      if(rhs.has_eul_y())
      {
        set_eul_y(rhs.get_eul_y());
      }
      else
      {
        clear_eul_y();
      }

      if(rhs.has_eul_p())
      {
        set_eul_p(rhs.get_eul_p());
      }
      else
      {
        clear_eul_p();
      }

      if(rhs.has_eul_r())
      {
        set_eul_r(rhs.get_eul_r());
      }
      else
      {
        clear_eul_r();
      }

      if(rhs.has_pan())
      {
        set_pan(rhs.get_pan());
      }
      else
      {
        clear_pan();
      }

      if(rhs.has_tilt())
      {
        set_tilt(rhs.get_tilt());
      }
      else
      {
        clear_tilt();
      }

    }

    ~SLAM_Data() override = default;

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      LIA_X = 1,
      LIA_Y = 2,
      LIA_Z = 3,
      EUL_Y = 4,
      EUL_P = 5,
      EUL_R = 6,
      PAN = 7,
      TILT = 8
    };

    SLAM_Data& operator=(const SLAM_Data& rhs)
    {
      if(rhs.has_lia_x())
      {
        set_lia_x(rhs.get_lia_x());
      }
      else
      {
        clear_lia_x();
      }

      if(rhs.has_lia_y())
      {
        set_lia_y(rhs.get_lia_y());
      }
      else
      {
        clear_lia_y();
      }

      if(rhs.has_lia_z())
      {
        set_lia_z(rhs.get_lia_z());
      }
      else
      {
        clear_lia_z();
      }

      if(rhs.has_eul_y())
      {
        set_eul_y(rhs.get_eul_y());
      }
      else
      {
        clear_eul_y();
      }

      if(rhs.has_eul_p())
      {
        set_eul_p(rhs.get_eul_p());
      }
      else
      {
        clear_eul_p();
      }

      if(rhs.has_eul_r())
      {
        set_eul_r(rhs.get_eul_r());
      }
      else
      {
        clear_eul_r();
      }

      if(rhs.has_pan())
      {
        set_pan(rhs.get_pan());
      }
      else
      {
        clear_pan();
      }

      if(rhs.has_tilt())
      {
        set_tilt(rhs.get_tilt());
      }
      else
      {
        clear_tilt();
      }

      return *this;
    }

    SLAM_Data& operator=(const SLAM_Data&& rhs) noexcept
    {
      if(rhs.has_lia_x())
      {
        set_lia_x(rhs.get_lia_x());
      }
      else
      {
        clear_lia_x();
      }
      
      if(rhs.has_lia_y())
      {
        set_lia_y(rhs.get_lia_y());
      }
      else
      {
        clear_lia_y();
      }
      
      if(rhs.has_lia_z())
      {
        set_lia_z(rhs.get_lia_z());
      }
      else
      {
        clear_lia_z();
      }
      
      if(rhs.has_eul_y())
      {
        set_eul_y(rhs.get_eul_y());
      }
      else
      {
        clear_eul_y();
      }
      
      if(rhs.has_eul_p())
      {
        set_eul_p(rhs.get_eul_p());
      }
      else
      {
        clear_eul_p();
      }
      
      if(rhs.has_eul_r())
      {
        set_eul_r(rhs.get_eul_r());
      }
      else
      {
        clear_eul_r();
      }
      
      if(rhs.has_pan())
      {
        set_pan(rhs.get_pan());
      }
      else
      {
        clear_pan();
      }
      
      if(rhs.has_tilt())
      {
        set_tilt(rhs.get_tilt());
      }
      else
      {
        clear_tilt();
      }
      
      return *this;
    }

    static constexpr char const* LIA_X_NAME = "lia_x";
    inline bool has_lia_x() const
    {
      return 0 != (presence::mask(presence::fields::LIA_X) & presence_[presence::index(presence::fields::LIA_X)]);
    }
    inline void clear_lia_x()
    {
      presence_[presence::index(presence::fields::LIA_X)] &= ~(presence::mask(presence::fields::LIA_X));
      lia_x_.clear();
    }
    inline void set_lia_x(const float& value)
    {
      presence_[presence::index(presence::fields::LIA_X)] |= presence::mask(presence::fields::LIA_X);
      lia_x_ = value;
    }
    inline void set_lia_x(const float&& value)
    {
      presence_[presence::index(presence::fields::LIA_X)] |= presence::mask(presence::fields::LIA_X);
      lia_x_ = value;
    }
    inline float& mutable_lia_x()
    {
      presence_[presence::index(presence::fields::LIA_X)] |= presence::mask(presence::fields::LIA_X);
      return lia_x_.get();
    }
    inline const float& get_lia_x() const { return lia_x_.get(); }
    inline float lia_x() const { return lia_x_.get(); }

    static constexpr char const* LIA_Y_NAME = "lia_y";
    inline bool has_lia_y() const
    {
      return 0 != (presence::mask(presence::fields::LIA_Y) & presence_[presence::index(presence::fields::LIA_Y)]);
    }
    inline void clear_lia_y()
    {
      presence_[presence::index(presence::fields::LIA_Y)] &= ~(presence::mask(presence::fields::LIA_Y));
      lia_y_.clear();
    }
    inline void set_lia_y(const float& value)
    {
      presence_[presence::index(presence::fields::LIA_Y)] |= presence::mask(presence::fields::LIA_Y);
      lia_y_ = value;
    }
    inline void set_lia_y(const float&& value)
    {
      presence_[presence::index(presence::fields::LIA_Y)] |= presence::mask(presence::fields::LIA_Y);
      lia_y_ = value;
    }
    inline float& mutable_lia_y()
    {
      presence_[presence::index(presence::fields::LIA_Y)] |= presence::mask(presence::fields::LIA_Y);
      return lia_y_.get();
    }
    inline const float& get_lia_y() const { return lia_y_.get(); }
    inline float lia_y() const { return lia_y_.get(); }

    static constexpr char const* LIA_Z_NAME = "lia_z";
    inline bool has_lia_z() const
    {
      return 0 != (presence::mask(presence::fields::LIA_Z) & presence_[presence::index(presence::fields::LIA_Z)]);
    }
    inline void clear_lia_z()
    {
      presence_[presence::index(presence::fields::LIA_Z)] &= ~(presence::mask(presence::fields::LIA_Z));
      lia_z_.clear();
    }
    inline void set_lia_z(const float& value)
    {
      presence_[presence::index(presence::fields::LIA_Z)] |= presence::mask(presence::fields::LIA_Z);
      lia_z_ = value;
    }
    inline void set_lia_z(const float&& value)
    {
      presence_[presence::index(presence::fields::LIA_Z)] |= presence::mask(presence::fields::LIA_Z);
      lia_z_ = value;
    }
    inline float& mutable_lia_z()
    {
      presence_[presence::index(presence::fields::LIA_Z)] |= presence::mask(presence::fields::LIA_Z);
      return lia_z_.get();
    }
    inline const float& get_lia_z() const { return lia_z_.get(); }
    inline float lia_z() const { return lia_z_.get(); }

    static constexpr char const* EUL_Y_NAME = "eul_y";
    inline bool has_eul_y() const
    {
      return 0 != (presence::mask(presence::fields::EUL_Y) & presence_[presence::index(presence::fields::EUL_Y)]);
    }
    inline void clear_eul_y()
    {
      presence_[presence::index(presence::fields::EUL_Y)] &= ~(presence::mask(presence::fields::EUL_Y));
      eul_y_.clear();
    }
    inline void set_eul_y(const float& value)
    {
      presence_[presence::index(presence::fields::EUL_Y)] |= presence::mask(presence::fields::EUL_Y);
      eul_y_ = value;
    }
    inline void set_eul_y(const float&& value)
    {
      presence_[presence::index(presence::fields::EUL_Y)] |= presence::mask(presence::fields::EUL_Y);
      eul_y_ = value;
    }
    inline float& mutable_eul_y()
    {
      presence_[presence::index(presence::fields::EUL_Y)] |= presence::mask(presence::fields::EUL_Y);
      return eul_y_.get();
    }
    inline const float& get_eul_y() const { return eul_y_.get(); }
    inline float eul_y() const { return eul_y_.get(); }

    static constexpr char const* EUL_P_NAME = "eul_p";
    inline bool has_eul_p() const
    {
      return 0 != (presence::mask(presence::fields::EUL_P) & presence_[presence::index(presence::fields::EUL_P)]);
    }
    inline void clear_eul_p()
    {
      presence_[presence::index(presence::fields::EUL_P)] &= ~(presence::mask(presence::fields::EUL_P));
      eul_p_.clear();
    }
    inline void set_eul_p(const float& value)
    {
      presence_[presence::index(presence::fields::EUL_P)] |= presence::mask(presence::fields::EUL_P);
      eul_p_ = value;
    }
    inline void set_eul_p(const float&& value)
    {
      presence_[presence::index(presence::fields::EUL_P)] |= presence::mask(presence::fields::EUL_P);
      eul_p_ = value;
    }
    inline float& mutable_eul_p()
    {
      presence_[presence::index(presence::fields::EUL_P)] |= presence::mask(presence::fields::EUL_P);
      return eul_p_.get();
    }
    inline const float& get_eul_p() const { return eul_p_.get(); }
    inline float eul_p() const { return eul_p_.get(); }

    static constexpr char const* EUL_R_NAME = "eul_r";
    inline bool has_eul_r() const
    {
      return 0 != (presence::mask(presence::fields::EUL_R) & presence_[presence::index(presence::fields::EUL_R)]);
    }
    inline void clear_eul_r()
    {
      presence_[presence::index(presence::fields::EUL_R)] &= ~(presence::mask(presence::fields::EUL_R));
      eul_r_.clear();
    }
    inline void set_eul_r(const float& value)
    {
      presence_[presence::index(presence::fields::EUL_R)] |= presence::mask(presence::fields::EUL_R);
      eul_r_ = value;
    }
    inline void set_eul_r(const float&& value)
    {
      presence_[presence::index(presence::fields::EUL_R)] |= presence::mask(presence::fields::EUL_R);
      eul_r_ = value;
    }
    inline float& mutable_eul_r()
    {
      presence_[presence::index(presence::fields::EUL_R)] |= presence::mask(presence::fields::EUL_R);
      return eul_r_.get();
    }
    inline const float& get_eul_r() const { return eul_r_.get(); }
    inline float eul_r() const { return eul_r_.get(); }

    static constexpr char const* PAN_NAME = "pan";
    inline bool has_pan() const
    {
      return 0 != (presence::mask(presence::fields::PAN) & presence_[presence::index(presence::fields::PAN)]);
    }
    inline void clear_pan()
    {
      presence_[presence::index(presence::fields::PAN)] &= ~(presence::mask(presence::fields::PAN));
      pan_.clear();
    }
    inline void set_pan(const int32_t& value)
    {
      presence_[presence::index(presence::fields::PAN)] |= presence::mask(presence::fields::PAN);
      pan_ = value;
    }
    inline void set_pan(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::PAN)] |= presence::mask(presence::fields::PAN);
      pan_ = value;
    }
    inline int32_t& mutable_pan()
    {
      presence_[presence::index(presence::fields::PAN)] |= presence::mask(presence::fields::PAN);
      return pan_.get();
    }
    inline const int32_t& get_pan() const { return pan_.get(); }
    inline int32_t pan() const { return pan_.get(); }

    static constexpr char const* TILT_NAME = "tilt";
    inline bool has_tilt() const
    {
      return 0 != (presence::mask(presence::fields::TILT) & presence_[presence::index(presence::fields::TILT)]);
    }
    inline void clear_tilt()
    {
      presence_[presence::index(presence::fields::TILT)] &= ~(presence::mask(presence::fields::TILT));
      tilt_.clear();
    }
    inline void set_tilt(const int32_t& value)
    {
      presence_[presence::index(presence::fields::TILT)] |= presence::mask(presence::fields::TILT);
      tilt_ = value;
    }
    inline void set_tilt(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::TILT)] |= presence::mask(presence::fields::TILT);
      tilt_ = value;
    }
    inline int32_t& mutable_tilt()
    {
      presence_[presence::index(presence::fields::TILT)] |= presence::mask(presence::fields::TILT);
      return tilt_.get();
    }
    inline const int32_t& get_tilt() const { return tilt_.get(); }
    inline int32_t tilt() const { return tilt_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if(has_lia_x() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = lia_x_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LIA_X), buffer, true);
      }

      if(has_lia_y() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = lia_y_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LIA_Y), buffer, true);
      }

      if(has_lia_z() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = lia_z_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LIA_Z), buffer, true);
      }

      if(has_eul_y() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = eul_y_.serialize_with_id(static_cast<uint32_t>(FieldNumber::EUL_Y), buffer, true);
      }

      if(has_eul_p() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = eul_p_.serialize_with_id(static_cast<uint32_t>(FieldNumber::EUL_P), buffer, true);
      }

      if(has_eul_r() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = eul_r_.serialize_with_id(static_cast<uint32_t>(FieldNumber::EUL_R), buffer, true);
      }

      if(has_pan() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = pan_.serialize_with_id(static_cast<uint32_t>(FieldNumber::PAN), buffer, true);
      }

      if(has_tilt() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = tilt_.serialize_with_id(static_cast<uint32_t>(FieldNumber::TILT), buffer, true);
      }

      return return_value;
    };

    ::EmbeddedProto::Error deserialize(::EmbeddedProto::ReadBufferInterface& buffer) override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;
      ::EmbeddedProto::WireFormatter::WireType wire_type = ::EmbeddedProto::WireFormatter::WireType::VARINT;
      uint32_t id_number = 0;
      FieldNumber id_tag = FieldNumber::NOT_SET;

      ::EmbeddedProto::Error tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
      while((::EmbeddedProto::Error::NO_ERRORS == return_value) && (::EmbeddedProto::Error::NO_ERRORS == tag_value))
      {
        id_tag = static_cast<FieldNumber>(id_number);
        switch(id_tag)
        {
          case FieldNumber::LIA_X:
            presence_[presence::index(presence::fields::LIA_X)] |= presence::mask(presence::fields::LIA_X);
            return_value = lia_x_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LIA_Y:
            presence_[presence::index(presence::fields::LIA_Y)] |= presence::mask(presence::fields::LIA_Y);
            return_value = lia_y_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LIA_Z:
            presence_[presence::index(presence::fields::LIA_Z)] |= presence::mask(presence::fields::LIA_Z);
            return_value = lia_z_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::EUL_Y:
            presence_[presence::index(presence::fields::EUL_Y)] |= presence::mask(presence::fields::EUL_Y);
            return_value = eul_y_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::EUL_P:
            presence_[presence::index(presence::fields::EUL_P)] |= presence::mask(presence::fields::EUL_P);
            return_value = eul_p_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::EUL_R:
            presence_[presence::index(presence::fields::EUL_R)] |= presence::mask(presence::fields::EUL_R);
            return_value = eul_r_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::PAN:
            presence_[presence::index(presence::fields::PAN)] |= presence::mask(presence::fields::PAN);
            return_value = pan_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::TILT:
            presence_[presence::index(presence::fields::TILT)] |= presence::mask(presence::fields::TILT);
            return_value = tilt_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::NOT_SET:
            return_value = ::EmbeddedProto::Error::INVALID_FIELD_ID;
            break;

          default:
            return_value = skip_unknown_field(buffer, wire_type);
            break;
        }

        if(::EmbeddedProto::Error::NO_ERRORS == return_value)
        {
          // Read the next tag.
          tag_value = ::EmbeddedProto::WireFormatter::DeserializeTag(buffer, wire_type, id_number);
        }
      }

      // When an error was detect while reading the tag but no other errors where found, set it in the return value.
      if((::EmbeddedProto::Error::NO_ERRORS == return_value)
         && (::EmbeddedProto::Error::NO_ERRORS != tag_value)
         && (::EmbeddedProto::Error::END_OF_BUFFER != tag_value)) // The end of the buffer is not an array in this case.
      {
        return_value = tag_value;
      }

      return return_value;
    };

    void clear() override
    {
      clear_lia_x();
      clear_lia_y();
      clear_lia_z();
      clear_eul_y();
      clear_eul_p();
      clear_eul_r();
      clear_pan();
      clear_tilt();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::LIA_X:
          name = LIA_X_NAME;
          break;
        case FieldNumber::LIA_Y:
          name = LIA_Y_NAME;
          break;
        case FieldNumber::LIA_Z:
          name = LIA_Z_NAME;
          break;
        case FieldNumber::EUL_Y:
          name = EUL_Y_NAME;
          break;
        case FieldNumber::EUL_P:
          name = EUL_P_NAME;
          break;
        case FieldNumber::EUL_R:
          name = EUL_R_NAME;
          break;
        case FieldNumber::PAN:
          name = PAN_NAME;
          break;
        case FieldNumber::TILT:
          name = TILT_NAME;
          break;
        default:
          name = "Invalid FieldNumber";
          break;
      }
      return name;
    }

#ifdef MSG_TO_STRING

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str) const
    {
      return this->to_string(str, 0, nullptr, true);
    }

    ::EmbeddedProto::string_view to_string(::EmbeddedProto::string_view& str, const uint32_t indent_level, char const* name, const bool first_field) const override
    {
      ::EmbeddedProto::string_view left_chars = str;
      int32_t n_chars_used = 0;

      if(!first_field)
      {
        // Add a comma behind the previous field.
        n_chars_used = snprintf(left_chars.data, left_chars.size, ",\n");
        if(0 < n_chars_used)
        {
          // Update the character pointer and characters left in the array.
          left_chars.data += n_chars_used;
          left_chars.size -= n_chars_used;
        }
      }

      if(nullptr != name)
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "\"%s\": {\n", name);
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s\"%s\": {\n", indent_level, " ", name);
        }
      }
      else
      {
        if( 0 == indent_level)
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "{\n");
        }
        else
        {
          n_chars_used = snprintf(left_chars.data, left_chars.size, "%*s{\n", indent_level, " ");
        }
      }
      
      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      left_chars = lia_x_.to_string(left_chars, indent_level + 2, LIA_X_NAME, true);
      left_chars = lia_y_.to_string(left_chars, indent_level + 2, LIA_Y_NAME, false);
      left_chars = lia_z_.to_string(left_chars, indent_level + 2, LIA_Z_NAME, false);
      left_chars = eul_y_.to_string(left_chars, indent_level + 2, EUL_Y_NAME, false);
      left_chars = eul_p_.to_string(left_chars, indent_level + 2, EUL_P_NAME, false);
      left_chars = eul_r_.to_string(left_chars, indent_level + 2, EUL_R_NAME, false);
      left_chars = pan_.to_string(left_chars, indent_level + 2, PAN_NAME, false);
      left_chars = tilt_.to_string(left_chars, indent_level + 2, TILT_NAME, false);
  
      if( 0 == indent_level) 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n}");
      }
      else 
      {
        n_chars_used = snprintf(left_chars.data, left_chars.size, "\n%*s}", indent_level, " ");
      }

      if(0 < n_chars_used)
      {
        left_chars.data += n_chars_used;
        left_chars.size -= n_chars_used;
      }

      return left_chars;
    }

#endif // End of MSG_TO_STRING

  private:

      // Define constants for tracking the presence of fields.
      // Use a struct to scope the variables from user fields as namespaces are not allowed within classes.
      struct presence
      {
        // An enumeration with all the fields for which presence has to be tracked.
        enum class fields : uint32_t
        {
          LIA_X,
          LIA_Y,
          LIA_Z,
          EUL_Y,
          EUL_P,
          EUL_R,
          PAN,
          TILT
        };

        // The number of fields for which presence has to be tracked.
        static constexpr uint32_t N_FIELDS = 8;

        // Which type are we using to track presence.
        using TYPE = uint32_t;

        // How many bits are there in the presence type.
        static constexpr uint32_t N_BITS = std::numeric_limits<TYPE>::digits;

        // How many variables of TYPE do we need to bit mask all presence fields.
        static constexpr uint32_t SIZE = (N_FIELDS / N_BITS) + ((N_FIELDS % N_BITS) > 0 ? 1 : 0);

        // Obtain the index of a given field in the presence array.
        static constexpr uint32_t index(const fields& field) { return static_cast<uint32_t>(field) / N_BITS; }

        // Obtain the bit mask for the given field assuming we are at the correct index in the presence array.
        static constexpr TYPE mask(const fields& field)
        {
          return static_cast<uint32_t>(0x01) << (static_cast<uint32_t>(field) % N_BITS);
        }
      };

      // Create an array in which the presence flags are stored.
      typename presence::TYPE presence_[presence::SIZE] = {0};

      EmbeddedProto::floatfixed lia_x_ = 0.0;
      EmbeddedProto::floatfixed lia_y_ = 0.0;
      EmbeddedProto::floatfixed lia_z_ = 0.0;
      EmbeddedProto::floatfixed eul_y_ = 0.0;
      EmbeddedProto::floatfixed eul_p_ = 0.0;
      EmbeddedProto::floatfixed eul_r_ = 0.0;
      EmbeddedProto::int32 pan_ = 0;
      EmbeddedProto::int32 tilt_ = 0;

};

#endif // UART_MESSAGES_H