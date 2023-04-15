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
      if(rhs.has_cpu_temp())
      {
        set_cpu_temp(rhs.get_cpu_temp());
      }
      else
      {
        clear_cpu_temp();
      }

      if(rhs.has_calib_status())
      {
        set_calib_status(rhs.get_calib_status());
      }
      else
      {
        clear_calib_status();
      }

      if(rhs.has_hts_temp())
      {
        set_hts_temp(rhs.get_hts_temp());
      }
      else
      {
        clear_hts_temp();
      }

      if(rhs.has_hts_humidity())
      {
        set_hts_humidity(rhs.get_hts_humidity());
      }
      else
      {
        clear_hts_humidity();
      }

      if(rhs.has_batt_temp())
      {
        set_batt_temp(rhs.get_batt_temp());
      }
      else
      {
        clear_batt_temp();
      }

      if(rhs.has_curr_servo_pan())
      {
        set_curr_servo_pan(rhs.get_curr_servo_pan());
      }
      else
      {
        clear_curr_servo_pan();
      }

      if(rhs.has_curr_servo_tilt())
      {
        set_curr_servo_tilt(rhs.get_curr_servo_tilt());
      }
      else
      {
        clear_curr_servo_tilt();
      }

      if(rhs.has_home_servo_pan())
      {
        set_home_servo_pan(rhs.get_home_servo_pan());
      }
      else
      {
        clear_home_servo_pan();
      }

      if(rhs.has_home_servo_tilt())
      {
        set_home_servo_tilt(rhs.get_home_servo_tilt());
      }
      else
      {
        clear_home_servo_tilt();
      }

      if(rhs.has_loco_status())
      {
        set_loco_status(rhs.get_loco_status());
      }
      else
      {
        clear_loco_status();
      }

      if(rhs.has_dev_status())
      {
        set_dev_status(rhs.get_dev_status());
      }
      else
      {
        clear_dev_status();
      }

      if(rhs.has_i2c_msg_failed())
      {
        set_i2c_msg_failed(rhs.get_i2c_msg_failed());
      }
      else
      {
        clear_i2c_msg_failed();
      }

      if(rhs.has_i2c_msg_passed())
      {
        set_i2c_msg_passed(rhs.get_i2c_msg_passed());
      }
      else
      {
        clear_i2c_msg_passed();
      }

      if(rhs.has_uart_msg_failed())
      {
        set_uart_msg_failed(rhs.get_uart_msg_failed());
      }
      else
      {
        clear_uart_msg_failed();
      }

      if(rhs.has_uart_msg_passed())
      {
        set_uart_msg_passed(rhs.get_uart_msg_passed());
      }
      else
      {
        clear_uart_msg_passed();
      }

      if(rhs.has_lost_sync_byte())
      {
        set_lost_sync_byte(rhs.get_lost_sync_byte());
      }
      else
      {
        clear_lost_sync_byte();
      }

      if(rhs.has_reseting_comms())
      {
        set_reseting_comms(rhs.get_reseting_comms());
      }
      else
      {
        clear_reseting_comms();
      }

      if(rhs.has_uptime())
      {
        set_uptime(rhs.get_uptime());
      }
      else
      {
        clear_uptime();
      }

    }

    GUI_Data(const GUI_Data&& rhs ) noexcept
    {
      if(rhs.has_cpu_temp())
      {
        set_cpu_temp(rhs.get_cpu_temp());
      }
      else
      {
        clear_cpu_temp();
      }

      if(rhs.has_calib_status())
      {
        set_calib_status(rhs.get_calib_status());
      }
      else
      {
        clear_calib_status();
      }

      if(rhs.has_hts_temp())
      {
        set_hts_temp(rhs.get_hts_temp());
      }
      else
      {
        clear_hts_temp();
      }

      if(rhs.has_hts_humidity())
      {
        set_hts_humidity(rhs.get_hts_humidity());
      }
      else
      {
        clear_hts_humidity();
      }

      if(rhs.has_batt_temp())
      {
        set_batt_temp(rhs.get_batt_temp());
      }
      else
      {
        clear_batt_temp();
      }

      if(rhs.has_curr_servo_pan())
      {
        set_curr_servo_pan(rhs.get_curr_servo_pan());
      }
      else
      {
        clear_curr_servo_pan();
      }

      if(rhs.has_curr_servo_tilt())
      {
        set_curr_servo_tilt(rhs.get_curr_servo_tilt());
      }
      else
      {
        clear_curr_servo_tilt();
      }

      if(rhs.has_home_servo_pan())
      {
        set_home_servo_pan(rhs.get_home_servo_pan());
      }
      else
      {
        clear_home_servo_pan();
      }

      if(rhs.has_home_servo_tilt())
      {
        set_home_servo_tilt(rhs.get_home_servo_tilt());
      }
      else
      {
        clear_home_servo_tilt();
      }

      if(rhs.has_loco_status())
      {
        set_loco_status(rhs.get_loco_status());
      }
      else
      {
        clear_loco_status();
      }

      if(rhs.has_dev_status())
      {
        set_dev_status(rhs.get_dev_status());
      }
      else
      {
        clear_dev_status();
      }

      if(rhs.has_i2c_msg_failed())
      {
        set_i2c_msg_failed(rhs.get_i2c_msg_failed());
      }
      else
      {
        clear_i2c_msg_failed();
      }

      if(rhs.has_i2c_msg_passed())
      {
        set_i2c_msg_passed(rhs.get_i2c_msg_passed());
      }
      else
      {
        clear_i2c_msg_passed();
      }

      if(rhs.has_uart_msg_failed())
      {
        set_uart_msg_failed(rhs.get_uart_msg_failed());
      }
      else
      {
        clear_uart_msg_failed();
      }

      if(rhs.has_uart_msg_passed())
      {
        set_uart_msg_passed(rhs.get_uart_msg_passed());
      }
      else
      {
        clear_uart_msg_passed();
      }

      if(rhs.has_lost_sync_byte())
      {
        set_lost_sync_byte(rhs.get_lost_sync_byte());
      }
      else
      {
        clear_lost_sync_byte();
      }

      if(rhs.has_reseting_comms())
      {
        set_reseting_comms(rhs.get_reseting_comms());
      }
      else
      {
        clear_reseting_comms();
      }

      if(rhs.has_uptime())
      {
        set_uptime(rhs.get_uptime());
      }
      else
      {
        clear_uptime();
      }

    }

    ~GUI_Data() override = default;

    enum class Locomotion_Status : uint32_t
    {
      NONE = 0,
      CONTROLS_INHIBIT = 1,
      NORMAL_DRIVING_MODE = 2
    };

    enum class Device_Status : uint32_t
    {
      NONE_1 = 0,
      BNO_INIT_SUCCESS = 1,
      HTS_INIT_SUCCESS = 2
    };

    enum class FieldNumber : uint32_t
    {
      NOT_SET = 0,
      CPU_TEMP = 1,
      CALIB_STATUS = 2,
      HTS_TEMP = 3,
      HTS_HUMIDITY = 4,
      BATT_TEMP = 5,
      CURR_SERVO_PAN = 6,
      CURR_SERVO_TILT = 7,
      HOME_SERVO_PAN = 8,
      HOME_SERVO_TILT = 9,
      LOCO_STATUS = 10,
      DEV_STATUS = 11,
      I2C_MSG_FAILED = 12,
      I2C_MSG_PASSED = 13,
      UART_MSG_FAILED = 14,
      UART_MSG_PASSED = 15,
      LOST_SYNC_BYTE = 16,
      RESETING_COMMS = 17,
      UPTIME = 18
    };

    GUI_Data& operator=(const GUI_Data& rhs)
    {
      if(rhs.has_cpu_temp())
      {
        set_cpu_temp(rhs.get_cpu_temp());
      }
      else
      {
        clear_cpu_temp();
      }

      if(rhs.has_calib_status())
      {
        set_calib_status(rhs.get_calib_status());
      }
      else
      {
        clear_calib_status();
      }

      if(rhs.has_hts_temp())
      {
        set_hts_temp(rhs.get_hts_temp());
      }
      else
      {
        clear_hts_temp();
      }

      if(rhs.has_hts_humidity())
      {
        set_hts_humidity(rhs.get_hts_humidity());
      }
      else
      {
        clear_hts_humidity();
      }

      if(rhs.has_batt_temp())
      {
        set_batt_temp(rhs.get_batt_temp());
      }
      else
      {
        clear_batt_temp();
      }

      if(rhs.has_curr_servo_pan())
      {
        set_curr_servo_pan(rhs.get_curr_servo_pan());
      }
      else
      {
        clear_curr_servo_pan();
      }

      if(rhs.has_curr_servo_tilt())
      {
        set_curr_servo_tilt(rhs.get_curr_servo_tilt());
      }
      else
      {
        clear_curr_servo_tilt();
      }

      if(rhs.has_home_servo_pan())
      {
        set_home_servo_pan(rhs.get_home_servo_pan());
      }
      else
      {
        clear_home_servo_pan();
      }

      if(rhs.has_home_servo_tilt())
      {
        set_home_servo_tilt(rhs.get_home_servo_tilt());
      }
      else
      {
        clear_home_servo_tilt();
      }

      if(rhs.has_loco_status())
      {
        set_loco_status(rhs.get_loco_status());
      }
      else
      {
        clear_loco_status();
      }

      if(rhs.has_dev_status())
      {
        set_dev_status(rhs.get_dev_status());
      }
      else
      {
        clear_dev_status();
      }

      if(rhs.has_i2c_msg_failed())
      {
        set_i2c_msg_failed(rhs.get_i2c_msg_failed());
      }
      else
      {
        clear_i2c_msg_failed();
      }

      if(rhs.has_i2c_msg_passed())
      {
        set_i2c_msg_passed(rhs.get_i2c_msg_passed());
      }
      else
      {
        clear_i2c_msg_passed();
      }

      if(rhs.has_uart_msg_failed())
      {
        set_uart_msg_failed(rhs.get_uart_msg_failed());
      }
      else
      {
        clear_uart_msg_failed();
      }

      if(rhs.has_uart_msg_passed())
      {
        set_uart_msg_passed(rhs.get_uart_msg_passed());
      }
      else
      {
        clear_uart_msg_passed();
      }

      if(rhs.has_lost_sync_byte())
      {
        set_lost_sync_byte(rhs.get_lost_sync_byte());
      }
      else
      {
        clear_lost_sync_byte();
      }

      if(rhs.has_reseting_comms())
      {
        set_reseting_comms(rhs.get_reseting_comms());
      }
      else
      {
        clear_reseting_comms();
      }

      if(rhs.has_uptime())
      {
        set_uptime(rhs.get_uptime());
      }
      else
      {
        clear_uptime();
      }

      return *this;
    }

    GUI_Data& operator=(const GUI_Data&& rhs) noexcept
    {
      if(rhs.has_cpu_temp())
      {
        set_cpu_temp(rhs.get_cpu_temp());
      }
      else
      {
        clear_cpu_temp();
      }
      
      if(rhs.has_calib_status())
      {
        set_calib_status(rhs.get_calib_status());
      }
      else
      {
        clear_calib_status();
      }
      
      if(rhs.has_hts_temp())
      {
        set_hts_temp(rhs.get_hts_temp());
      }
      else
      {
        clear_hts_temp();
      }
      
      if(rhs.has_hts_humidity())
      {
        set_hts_humidity(rhs.get_hts_humidity());
      }
      else
      {
        clear_hts_humidity();
      }
      
      if(rhs.has_batt_temp())
      {
        set_batt_temp(rhs.get_batt_temp());
      }
      else
      {
        clear_batt_temp();
      }
      
      if(rhs.has_curr_servo_pan())
      {
        set_curr_servo_pan(rhs.get_curr_servo_pan());
      }
      else
      {
        clear_curr_servo_pan();
      }
      
      if(rhs.has_curr_servo_tilt())
      {
        set_curr_servo_tilt(rhs.get_curr_servo_tilt());
      }
      else
      {
        clear_curr_servo_tilt();
      }
      
      if(rhs.has_home_servo_pan())
      {
        set_home_servo_pan(rhs.get_home_servo_pan());
      }
      else
      {
        clear_home_servo_pan();
      }
      
      if(rhs.has_home_servo_tilt())
      {
        set_home_servo_tilt(rhs.get_home_servo_tilt());
      }
      else
      {
        clear_home_servo_tilt();
      }
      
      if(rhs.has_loco_status())
      {
        set_loco_status(rhs.get_loco_status());
      }
      else
      {
        clear_loco_status();
      }
      
      if(rhs.has_dev_status())
      {
        set_dev_status(rhs.get_dev_status());
      }
      else
      {
        clear_dev_status();
      }
      
      if(rhs.has_i2c_msg_failed())
      {
        set_i2c_msg_failed(rhs.get_i2c_msg_failed());
      }
      else
      {
        clear_i2c_msg_failed();
      }
      
      if(rhs.has_i2c_msg_passed())
      {
        set_i2c_msg_passed(rhs.get_i2c_msg_passed());
      }
      else
      {
        clear_i2c_msg_passed();
      }
      
      if(rhs.has_uart_msg_failed())
      {
        set_uart_msg_failed(rhs.get_uart_msg_failed());
      }
      else
      {
        clear_uart_msg_failed();
      }
      
      if(rhs.has_uart_msg_passed())
      {
        set_uart_msg_passed(rhs.get_uart_msg_passed());
      }
      else
      {
        clear_uart_msg_passed();
      }
      
      if(rhs.has_lost_sync_byte())
      {
        set_lost_sync_byte(rhs.get_lost_sync_byte());
      }
      else
      {
        clear_lost_sync_byte();
      }
      
      if(rhs.has_reseting_comms())
      {
        set_reseting_comms(rhs.get_reseting_comms());
      }
      else
      {
        clear_reseting_comms();
      }
      
      if(rhs.has_uptime())
      {
        set_uptime(rhs.get_uptime());
      }
      else
      {
        clear_uptime();
      }
      
      return *this;
    }

    static constexpr char const* CPU_TEMP_NAME = "cpu_temp";
    inline bool has_cpu_temp() const
    {
      return 0 != (presence::mask(presence::fields::CPU_TEMP) & presence_[presence::index(presence::fields::CPU_TEMP)]);
    }
    inline void clear_cpu_temp()
    {
      presence_[presence::index(presence::fields::CPU_TEMP)] &= ~(presence::mask(presence::fields::CPU_TEMP));
      cpu_temp_.clear();
    }
    inline void set_cpu_temp(const int32_t& value)
    {
      presence_[presence::index(presence::fields::CPU_TEMP)] |= presence::mask(presence::fields::CPU_TEMP);
      cpu_temp_ = value;
    }
    inline void set_cpu_temp(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::CPU_TEMP)] |= presence::mask(presence::fields::CPU_TEMP);
      cpu_temp_ = value;
    }
    inline int32_t& mutable_cpu_temp()
    {
      presence_[presence::index(presence::fields::CPU_TEMP)] |= presence::mask(presence::fields::CPU_TEMP);
      return cpu_temp_.get();
    }
    inline const int32_t& get_cpu_temp() const { return cpu_temp_.get(); }
    inline int32_t cpu_temp() const { return cpu_temp_.get(); }

    static constexpr char const* CALIB_STATUS_NAME = "calib_status";
    inline bool has_calib_status() const
    {
      return 0 != (presence::mask(presence::fields::CALIB_STATUS) & presence_[presence::index(presence::fields::CALIB_STATUS)]);
    }
    inline void clear_calib_status()
    {
      presence_[presence::index(presence::fields::CALIB_STATUS)] &= ~(presence::mask(presence::fields::CALIB_STATUS));
      calib_status_.clear();
    }
    inline void set_calib_status(const int32_t& value)
    {
      presence_[presence::index(presence::fields::CALIB_STATUS)] |= presence::mask(presence::fields::CALIB_STATUS);
      calib_status_ = value;
    }
    inline void set_calib_status(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::CALIB_STATUS)] |= presence::mask(presence::fields::CALIB_STATUS);
      calib_status_ = value;
    }
    inline int32_t& mutable_calib_status()
    {
      presence_[presence::index(presence::fields::CALIB_STATUS)] |= presence::mask(presence::fields::CALIB_STATUS);
      return calib_status_.get();
    }
    inline const int32_t& get_calib_status() const { return calib_status_.get(); }
    inline int32_t calib_status() const { return calib_status_.get(); }

    static constexpr char const* HTS_TEMP_NAME = "hts_temp";
    inline bool has_hts_temp() const
    {
      return 0 != (presence::mask(presence::fields::HTS_TEMP) & presence_[presence::index(presence::fields::HTS_TEMP)]);
    }
    inline void clear_hts_temp()
    {
      presence_[presence::index(presence::fields::HTS_TEMP)] &= ~(presence::mask(presence::fields::HTS_TEMP));
      hts_temp_.clear();
    }
    inline void set_hts_temp(const int32_t& value)
    {
      presence_[presence::index(presence::fields::HTS_TEMP)] |= presence::mask(presence::fields::HTS_TEMP);
      hts_temp_ = value;
    }
    inline void set_hts_temp(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::HTS_TEMP)] |= presence::mask(presence::fields::HTS_TEMP);
      hts_temp_ = value;
    }
    inline int32_t& mutable_hts_temp()
    {
      presence_[presence::index(presence::fields::HTS_TEMP)] |= presence::mask(presence::fields::HTS_TEMP);
      return hts_temp_.get();
    }
    inline const int32_t& get_hts_temp() const { return hts_temp_.get(); }
    inline int32_t hts_temp() const { return hts_temp_.get(); }

    static constexpr char const* HTS_HUMIDITY_NAME = "hts_humidity";
    inline bool has_hts_humidity() const
    {
      return 0 != (presence::mask(presence::fields::HTS_HUMIDITY) & presence_[presence::index(presence::fields::HTS_HUMIDITY)]);
    }
    inline void clear_hts_humidity()
    {
      presence_[presence::index(presence::fields::HTS_HUMIDITY)] &= ~(presence::mask(presence::fields::HTS_HUMIDITY));
      hts_humidity_.clear();
    }
    inline void set_hts_humidity(const int32_t& value)
    {
      presence_[presence::index(presence::fields::HTS_HUMIDITY)] |= presence::mask(presence::fields::HTS_HUMIDITY);
      hts_humidity_ = value;
    }
    inline void set_hts_humidity(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::HTS_HUMIDITY)] |= presence::mask(presence::fields::HTS_HUMIDITY);
      hts_humidity_ = value;
    }
    inline int32_t& mutable_hts_humidity()
    {
      presence_[presence::index(presence::fields::HTS_HUMIDITY)] |= presence::mask(presence::fields::HTS_HUMIDITY);
      return hts_humidity_.get();
    }
    inline const int32_t& get_hts_humidity() const { return hts_humidity_.get(); }
    inline int32_t hts_humidity() const { return hts_humidity_.get(); }

    static constexpr char const* BATT_TEMP_NAME = "batt_temp";
    inline bool has_batt_temp() const
    {
      return 0 != (presence::mask(presence::fields::BATT_TEMP) & presence_[presence::index(presence::fields::BATT_TEMP)]);
    }
    inline void clear_batt_temp()
    {
      presence_[presence::index(presence::fields::BATT_TEMP)] &= ~(presence::mask(presence::fields::BATT_TEMP));
      batt_temp_.clear();
    }
    inline void set_batt_temp(const int32_t& value)
    {
      presence_[presence::index(presence::fields::BATT_TEMP)] |= presence::mask(presence::fields::BATT_TEMP);
      batt_temp_ = value;
    }
    inline void set_batt_temp(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::BATT_TEMP)] |= presence::mask(presence::fields::BATT_TEMP);
      batt_temp_ = value;
    }
    inline int32_t& mutable_batt_temp()
    {
      presence_[presence::index(presence::fields::BATT_TEMP)] |= presence::mask(presence::fields::BATT_TEMP);
      return batt_temp_.get();
    }
    inline const int32_t& get_batt_temp() const { return batt_temp_.get(); }
    inline int32_t batt_temp() const { return batt_temp_.get(); }

    static constexpr char const* CURR_SERVO_PAN_NAME = "curr_servo_pan";
    inline bool has_curr_servo_pan() const
    {
      return 0 != (presence::mask(presence::fields::CURR_SERVO_PAN) & presence_[presence::index(presence::fields::CURR_SERVO_PAN)]);
    }
    inline void clear_curr_servo_pan()
    {
      presence_[presence::index(presence::fields::CURR_SERVO_PAN)] &= ~(presence::mask(presence::fields::CURR_SERVO_PAN));
      curr_servo_pan_.clear();
    }
    inline void set_curr_servo_pan(const int32_t& value)
    {
      presence_[presence::index(presence::fields::CURR_SERVO_PAN)] |= presence::mask(presence::fields::CURR_SERVO_PAN);
      curr_servo_pan_ = value;
    }
    inline void set_curr_servo_pan(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::CURR_SERVO_PAN)] |= presence::mask(presence::fields::CURR_SERVO_PAN);
      curr_servo_pan_ = value;
    }
    inline int32_t& mutable_curr_servo_pan()
    {
      presence_[presence::index(presence::fields::CURR_SERVO_PAN)] |= presence::mask(presence::fields::CURR_SERVO_PAN);
      return curr_servo_pan_.get();
    }
    inline const int32_t& get_curr_servo_pan() const { return curr_servo_pan_.get(); }
    inline int32_t curr_servo_pan() const { return curr_servo_pan_.get(); }

    static constexpr char const* CURR_SERVO_TILT_NAME = "curr_servo_tilt";
    inline bool has_curr_servo_tilt() const
    {
      return 0 != (presence::mask(presence::fields::CURR_SERVO_TILT) & presence_[presence::index(presence::fields::CURR_SERVO_TILT)]);
    }
    inline void clear_curr_servo_tilt()
    {
      presence_[presence::index(presence::fields::CURR_SERVO_TILT)] &= ~(presence::mask(presence::fields::CURR_SERVO_TILT));
      curr_servo_tilt_.clear();
    }
    inline void set_curr_servo_tilt(const int32_t& value)
    {
      presence_[presence::index(presence::fields::CURR_SERVO_TILT)] |= presence::mask(presence::fields::CURR_SERVO_TILT);
      curr_servo_tilt_ = value;
    }
    inline void set_curr_servo_tilt(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::CURR_SERVO_TILT)] |= presence::mask(presence::fields::CURR_SERVO_TILT);
      curr_servo_tilt_ = value;
    }
    inline int32_t& mutable_curr_servo_tilt()
    {
      presence_[presence::index(presence::fields::CURR_SERVO_TILT)] |= presence::mask(presence::fields::CURR_SERVO_TILT);
      return curr_servo_tilt_.get();
    }
    inline const int32_t& get_curr_servo_tilt() const { return curr_servo_tilt_.get(); }
    inline int32_t curr_servo_tilt() const { return curr_servo_tilt_.get(); }

    static constexpr char const* HOME_SERVO_PAN_NAME = "home_servo_pan";
    inline bool has_home_servo_pan() const
    {
      return 0 != (presence::mask(presence::fields::HOME_SERVO_PAN) & presence_[presence::index(presence::fields::HOME_SERVO_PAN)]);
    }
    inline void clear_home_servo_pan()
    {
      presence_[presence::index(presence::fields::HOME_SERVO_PAN)] &= ~(presence::mask(presence::fields::HOME_SERVO_PAN));
      home_servo_pan_.clear();
    }
    inline void set_home_servo_pan(const int32_t& value)
    {
      presence_[presence::index(presence::fields::HOME_SERVO_PAN)] |= presence::mask(presence::fields::HOME_SERVO_PAN);
      home_servo_pan_ = value;
    }
    inline void set_home_servo_pan(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::HOME_SERVO_PAN)] |= presence::mask(presence::fields::HOME_SERVO_PAN);
      home_servo_pan_ = value;
    }
    inline int32_t& mutable_home_servo_pan()
    {
      presence_[presence::index(presence::fields::HOME_SERVO_PAN)] |= presence::mask(presence::fields::HOME_SERVO_PAN);
      return home_servo_pan_.get();
    }
    inline const int32_t& get_home_servo_pan() const { return home_servo_pan_.get(); }
    inline int32_t home_servo_pan() const { return home_servo_pan_.get(); }

    static constexpr char const* HOME_SERVO_TILT_NAME = "home_servo_tilt";
    inline bool has_home_servo_tilt() const
    {
      return 0 != (presence::mask(presence::fields::HOME_SERVO_TILT) & presence_[presence::index(presence::fields::HOME_SERVO_TILT)]);
    }
    inline void clear_home_servo_tilt()
    {
      presence_[presence::index(presence::fields::HOME_SERVO_TILT)] &= ~(presence::mask(presence::fields::HOME_SERVO_TILT));
      home_servo_tilt_.clear();
    }
    inline void set_home_servo_tilt(const int32_t& value)
    {
      presence_[presence::index(presence::fields::HOME_SERVO_TILT)] |= presence::mask(presence::fields::HOME_SERVO_TILT);
      home_servo_tilt_ = value;
    }
    inline void set_home_servo_tilt(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::HOME_SERVO_TILT)] |= presence::mask(presence::fields::HOME_SERVO_TILT);
      home_servo_tilt_ = value;
    }
    inline int32_t& mutable_home_servo_tilt()
    {
      presence_[presence::index(presence::fields::HOME_SERVO_TILT)] |= presence::mask(presence::fields::HOME_SERVO_TILT);
      return home_servo_tilt_.get();
    }
    inline const int32_t& get_home_servo_tilt() const { return home_servo_tilt_.get(); }
    inline int32_t home_servo_tilt() const { return home_servo_tilt_.get(); }

    static constexpr char const* LOCO_STATUS_NAME = "loco_status";
    inline bool has_loco_status() const
    {
      return 0 != (presence::mask(presence::fields::LOCO_STATUS) & presence_[presence::index(presence::fields::LOCO_STATUS)]);
    }
    inline void clear_loco_status()
    {
      presence_[presence::index(presence::fields::LOCO_STATUS)] &= ~(presence::mask(presence::fields::LOCO_STATUS));
      loco_status_.clear();
    }
    inline void set_loco_status(const Locomotion_Status& value)
    {
      presence_[presence::index(presence::fields::LOCO_STATUS)] |= presence::mask(presence::fields::LOCO_STATUS);
      loco_status_ = value;
    }
    inline void set_loco_status(const Locomotion_Status&& value)
    {
      presence_[presence::index(presence::fields::LOCO_STATUS)] |= presence::mask(presence::fields::LOCO_STATUS);
      loco_status_ = value;
    }
    inline const Locomotion_Status& get_loco_status() const { return loco_status_.get(); }
    inline Locomotion_Status loco_status() const { return loco_status_.get(); }

    static constexpr char const* DEV_STATUS_NAME = "dev_status";
    inline bool has_dev_status() const
    {
      return 0 != (presence::mask(presence::fields::DEV_STATUS) & presence_[presence::index(presence::fields::DEV_STATUS)]);
    }
    inline void clear_dev_status()
    {
      presence_[presence::index(presence::fields::DEV_STATUS)] &= ~(presence::mask(presence::fields::DEV_STATUS));
      dev_status_.clear();
    }
    inline void set_dev_status(const Device_Status& value)
    {
      presence_[presence::index(presence::fields::DEV_STATUS)] |= presence::mask(presence::fields::DEV_STATUS);
      dev_status_ = value;
    }
    inline void set_dev_status(const Device_Status&& value)
    {
      presence_[presence::index(presence::fields::DEV_STATUS)] |= presence::mask(presence::fields::DEV_STATUS);
      dev_status_ = value;
    }
    inline const Device_Status& get_dev_status() const { return dev_status_.get(); }
    inline Device_Status dev_status() const { return dev_status_.get(); }

    static constexpr char const* I2C_MSG_FAILED_NAME = "i2c_msg_failed";
    inline bool has_i2c_msg_failed() const
    {
      return 0 != (presence::mask(presence::fields::I2C_MSG_FAILED) & presence_[presence::index(presence::fields::I2C_MSG_FAILED)]);
    }
    inline void clear_i2c_msg_failed()
    {
      presence_[presence::index(presence::fields::I2C_MSG_FAILED)] &= ~(presence::mask(presence::fields::I2C_MSG_FAILED));
      i2c_msg_failed_.clear();
    }
    inline void set_i2c_msg_failed(const int32_t& value)
    {
      presence_[presence::index(presence::fields::I2C_MSG_FAILED)] |= presence::mask(presence::fields::I2C_MSG_FAILED);
      i2c_msg_failed_ = value;
    }
    inline void set_i2c_msg_failed(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::I2C_MSG_FAILED)] |= presence::mask(presence::fields::I2C_MSG_FAILED);
      i2c_msg_failed_ = value;
    }
    inline int32_t& mutable_i2c_msg_failed()
    {
      presence_[presence::index(presence::fields::I2C_MSG_FAILED)] |= presence::mask(presence::fields::I2C_MSG_FAILED);
      return i2c_msg_failed_.get();
    }
    inline const int32_t& get_i2c_msg_failed() const { return i2c_msg_failed_.get(); }
    inline int32_t i2c_msg_failed() const { return i2c_msg_failed_.get(); }

    static constexpr char const* I2C_MSG_PASSED_NAME = "i2c_msg_passed";
    inline bool has_i2c_msg_passed() const
    {
      return 0 != (presence::mask(presence::fields::I2C_MSG_PASSED) & presence_[presence::index(presence::fields::I2C_MSG_PASSED)]);
    }
    inline void clear_i2c_msg_passed()
    {
      presence_[presence::index(presence::fields::I2C_MSG_PASSED)] &= ~(presence::mask(presence::fields::I2C_MSG_PASSED));
      i2c_msg_passed_.clear();
    }
    inline void set_i2c_msg_passed(const int32_t& value)
    {
      presence_[presence::index(presence::fields::I2C_MSG_PASSED)] |= presence::mask(presence::fields::I2C_MSG_PASSED);
      i2c_msg_passed_ = value;
    }
    inline void set_i2c_msg_passed(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::I2C_MSG_PASSED)] |= presence::mask(presence::fields::I2C_MSG_PASSED);
      i2c_msg_passed_ = value;
    }
    inline int32_t& mutable_i2c_msg_passed()
    {
      presence_[presence::index(presence::fields::I2C_MSG_PASSED)] |= presence::mask(presence::fields::I2C_MSG_PASSED);
      return i2c_msg_passed_.get();
    }
    inline const int32_t& get_i2c_msg_passed() const { return i2c_msg_passed_.get(); }
    inline int32_t i2c_msg_passed() const { return i2c_msg_passed_.get(); }

    static constexpr char const* UART_MSG_FAILED_NAME = "uart_msg_failed";
    inline bool has_uart_msg_failed() const
    {
      return 0 != (presence::mask(presence::fields::UART_MSG_FAILED) & presence_[presence::index(presence::fields::UART_MSG_FAILED)]);
    }
    inline void clear_uart_msg_failed()
    {
      presence_[presence::index(presence::fields::UART_MSG_FAILED)] &= ~(presence::mask(presence::fields::UART_MSG_FAILED));
      uart_msg_failed_.clear();
    }
    inline void set_uart_msg_failed(const int32_t& value)
    {
      presence_[presence::index(presence::fields::UART_MSG_FAILED)] |= presence::mask(presence::fields::UART_MSG_FAILED);
      uart_msg_failed_ = value;
    }
    inline void set_uart_msg_failed(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::UART_MSG_FAILED)] |= presence::mask(presence::fields::UART_MSG_FAILED);
      uart_msg_failed_ = value;
    }
    inline int32_t& mutable_uart_msg_failed()
    {
      presence_[presence::index(presence::fields::UART_MSG_FAILED)] |= presence::mask(presence::fields::UART_MSG_FAILED);
      return uart_msg_failed_.get();
    }
    inline const int32_t& get_uart_msg_failed() const { return uart_msg_failed_.get(); }
    inline int32_t uart_msg_failed() const { return uart_msg_failed_.get(); }

    static constexpr char const* UART_MSG_PASSED_NAME = "uart_msg_passed";
    inline bool has_uart_msg_passed() const
    {
      return 0 != (presence::mask(presence::fields::UART_MSG_PASSED) & presence_[presence::index(presence::fields::UART_MSG_PASSED)]);
    }
    inline void clear_uart_msg_passed()
    {
      presence_[presence::index(presence::fields::UART_MSG_PASSED)] &= ~(presence::mask(presence::fields::UART_MSG_PASSED));
      uart_msg_passed_.clear();
    }
    inline void set_uart_msg_passed(const int32_t& value)
    {
      presence_[presence::index(presence::fields::UART_MSG_PASSED)] |= presence::mask(presence::fields::UART_MSG_PASSED);
      uart_msg_passed_ = value;
    }
    inline void set_uart_msg_passed(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::UART_MSG_PASSED)] |= presence::mask(presence::fields::UART_MSG_PASSED);
      uart_msg_passed_ = value;
    }
    inline int32_t& mutable_uart_msg_passed()
    {
      presence_[presence::index(presence::fields::UART_MSG_PASSED)] |= presence::mask(presence::fields::UART_MSG_PASSED);
      return uart_msg_passed_.get();
    }
    inline const int32_t& get_uart_msg_passed() const { return uart_msg_passed_.get(); }
    inline int32_t uart_msg_passed() const { return uart_msg_passed_.get(); }

    static constexpr char const* LOST_SYNC_BYTE_NAME = "lost_sync_byte";
    inline bool has_lost_sync_byte() const
    {
      return 0 != (presence::mask(presence::fields::LOST_SYNC_BYTE) & presence_[presence::index(presence::fields::LOST_SYNC_BYTE)]);
    }
    inline void clear_lost_sync_byte()
    {
      presence_[presence::index(presence::fields::LOST_SYNC_BYTE)] &= ~(presence::mask(presence::fields::LOST_SYNC_BYTE));
      lost_sync_byte_.clear();
    }
    inline void set_lost_sync_byte(const int32_t& value)
    {
      presence_[presence::index(presence::fields::LOST_SYNC_BYTE)] |= presence::mask(presence::fields::LOST_SYNC_BYTE);
      lost_sync_byte_ = value;
    }
    inline void set_lost_sync_byte(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::LOST_SYNC_BYTE)] |= presence::mask(presence::fields::LOST_SYNC_BYTE);
      lost_sync_byte_ = value;
    }
    inline int32_t& mutable_lost_sync_byte()
    {
      presence_[presence::index(presence::fields::LOST_SYNC_BYTE)] |= presence::mask(presence::fields::LOST_SYNC_BYTE);
      return lost_sync_byte_.get();
    }
    inline const int32_t& get_lost_sync_byte() const { return lost_sync_byte_.get(); }
    inline int32_t lost_sync_byte() const { return lost_sync_byte_.get(); }

    static constexpr char const* RESETING_COMMS_NAME = "reseting_comms";
    inline bool has_reseting_comms() const
    {
      return 0 != (presence::mask(presence::fields::RESETING_COMMS) & presence_[presence::index(presence::fields::RESETING_COMMS)]);
    }
    inline void clear_reseting_comms()
    {
      presence_[presence::index(presence::fields::RESETING_COMMS)] &= ~(presence::mask(presence::fields::RESETING_COMMS));
      reseting_comms_.clear();
    }
    inline void set_reseting_comms(const int32_t& value)
    {
      presence_[presence::index(presence::fields::RESETING_COMMS)] |= presence::mask(presence::fields::RESETING_COMMS);
      reseting_comms_ = value;
    }
    inline void set_reseting_comms(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::RESETING_COMMS)] |= presence::mask(presence::fields::RESETING_COMMS);
      reseting_comms_ = value;
    }
    inline int32_t& mutable_reseting_comms()
    {
      presence_[presence::index(presence::fields::RESETING_COMMS)] |= presence::mask(presence::fields::RESETING_COMMS);
      return reseting_comms_.get();
    }
    inline const int32_t& get_reseting_comms() const { return reseting_comms_.get(); }
    inline int32_t reseting_comms() const { return reseting_comms_.get(); }

    static constexpr char const* UPTIME_NAME = "uptime";
    inline bool has_uptime() const
    {
      return 0 != (presence::mask(presence::fields::UPTIME) & presence_[presence::index(presence::fields::UPTIME)]);
    }
    inline void clear_uptime()
    {
      presence_[presence::index(presence::fields::UPTIME)] &= ~(presence::mask(presence::fields::UPTIME));
      uptime_.clear();
    }
    inline void set_uptime(const int32_t& value)
    {
      presence_[presence::index(presence::fields::UPTIME)] |= presence::mask(presence::fields::UPTIME);
      uptime_ = value;
    }
    inline void set_uptime(const int32_t&& value)
    {
      presence_[presence::index(presence::fields::UPTIME)] |= presence::mask(presence::fields::UPTIME);
      uptime_ = value;
    }
    inline int32_t& mutable_uptime()
    {
      presence_[presence::index(presence::fields::UPTIME)] |= presence::mask(presence::fields::UPTIME);
      return uptime_.get();
    }
    inline const int32_t& get_uptime() const { return uptime_.get(); }
    inline int32_t uptime() const { return uptime_.get(); }


    ::EmbeddedProto::Error serialize(::EmbeddedProto::WriteBufferInterface& buffer) const override
    {
      ::EmbeddedProto::Error return_value = ::EmbeddedProto::Error::NO_ERRORS;

      if(has_cpu_temp() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = cpu_temp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CPU_TEMP), buffer, true);
      }

      if(has_calib_status() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = calib_status_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CALIB_STATUS), buffer, true);
      }

      if(has_hts_temp() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = hts_temp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::HTS_TEMP), buffer, true);
      }

      if(has_hts_humidity() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = hts_humidity_.serialize_with_id(static_cast<uint32_t>(FieldNumber::HTS_HUMIDITY), buffer, true);
      }

      if(has_batt_temp() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = batt_temp_.serialize_with_id(static_cast<uint32_t>(FieldNumber::BATT_TEMP), buffer, true);
      }

      if(has_curr_servo_pan() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = curr_servo_pan_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURR_SERVO_PAN), buffer, true);
      }

      if(has_curr_servo_tilt() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = curr_servo_tilt_.serialize_with_id(static_cast<uint32_t>(FieldNumber::CURR_SERVO_TILT), buffer, true);
      }

      if(has_home_servo_pan() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = home_servo_pan_.serialize_with_id(static_cast<uint32_t>(FieldNumber::HOME_SERVO_PAN), buffer, true);
      }

      if(has_home_servo_tilt() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = home_servo_tilt_.serialize_with_id(static_cast<uint32_t>(FieldNumber::HOME_SERVO_TILT), buffer, true);
      }

      if(has_loco_status() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = loco_status_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LOCO_STATUS), buffer, true);
      }

      if(has_dev_status() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = dev_status_.serialize_with_id(static_cast<uint32_t>(FieldNumber::DEV_STATUS), buffer, true);
      }

      if(has_i2c_msg_failed() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = i2c_msg_failed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::I2C_MSG_FAILED), buffer, true);
      }

      if(has_i2c_msg_passed() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = i2c_msg_passed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::I2C_MSG_PASSED), buffer, true);
      }

      if(has_uart_msg_failed() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = uart_msg_failed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::UART_MSG_FAILED), buffer, true);
      }

      if(has_uart_msg_passed() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = uart_msg_passed_.serialize_with_id(static_cast<uint32_t>(FieldNumber::UART_MSG_PASSED), buffer, true);
      }

      if(has_lost_sync_byte() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = lost_sync_byte_.serialize_with_id(static_cast<uint32_t>(FieldNumber::LOST_SYNC_BYTE), buffer, true);
      }

      if(has_reseting_comms() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = reseting_comms_.serialize_with_id(static_cast<uint32_t>(FieldNumber::RESETING_COMMS), buffer, true);
      }

      if(has_uptime() && (::EmbeddedProto::Error::NO_ERRORS == return_value))
      {
        return_value = uptime_.serialize_with_id(static_cast<uint32_t>(FieldNumber::UPTIME), buffer, true);
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
            presence_[presence::index(presence::fields::CPU_TEMP)] |= presence::mask(presence::fields::CPU_TEMP);
            return_value = cpu_temp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CALIB_STATUS:
            presence_[presence::index(presence::fields::CALIB_STATUS)] |= presence::mask(presence::fields::CALIB_STATUS);
            return_value = calib_status_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::HTS_TEMP:
            presence_[presence::index(presence::fields::HTS_TEMP)] |= presence::mask(presence::fields::HTS_TEMP);
            return_value = hts_temp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::HTS_HUMIDITY:
            presence_[presence::index(presence::fields::HTS_HUMIDITY)] |= presence::mask(presence::fields::HTS_HUMIDITY);
            return_value = hts_humidity_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::BATT_TEMP:
            presence_[presence::index(presence::fields::BATT_TEMP)] |= presence::mask(presence::fields::BATT_TEMP);
            return_value = batt_temp_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURR_SERVO_PAN:
            presence_[presence::index(presence::fields::CURR_SERVO_PAN)] |= presence::mask(presence::fields::CURR_SERVO_PAN);
            return_value = curr_servo_pan_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::CURR_SERVO_TILT:
            presence_[presence::index(presence::fields::CURR_SERVO_TILT)] |= presence::mask(presence::fields::CURR_SERVO_TILT);
            return_value = curr_servo_tilt_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::HOME_SERVO_PAN:
            presence_[presence::index(presence::fields::HOME_SERVO_PAN)] |= presence::mask(presence::fields::HOME_SERVO_PAN);
            return_value = home_servo_pan_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::HOME_SERVO_TILT:
            presence_[presence::index(presence::fields::HOME_SERVO_TILT)] |= presence::mask(presence::fields::HOME_SERVO_TILT);
            return_value = home_servo_tilt_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LOCO_STATUS:
            presence_[presence::index(presence::fields::LOCO_STATUS)] |= presence::mask(presence::fields::LOCO_STATUS);
            return_value = loco_status_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::DEV_STATUS:
            presence_[presence::index(presence::fields::DEV_STATUS)] |= presence::mask(presence::fields::DEV_STATUS);
            return_value = dev_status_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::I2C_MSG_FAILED:
            presence_[presence::index(presence::fields::I2C_MSG_FAILED)] |= presence::mask(presence::fields::I2C_MSG_FAILED);
            return_value = i2c_msg_failed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::I2C_MSG_PASSED:
            presence_[presence::index(presence::fields::I2C_MSG_PASSED)] |= presence::mask(presence::fields::I2C_MSG_PASSED);
            return_value = i2c_msg_passed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::UART_MSG_FAILED:
            presence_[presence::index(presence::fields::UART_MSG_FAILED)] |= presence::mask(presence::fields::UART_MSG_FAILED);
            return_value = uart_msg_failed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::UART_MSG_PASSED:
            presence_[presence::index(presence::fields::UART_MSG_PASSED)] |= presence::mask(presence::fields::UART_MSG_PASSED);
            return_value = uart_msg_passed_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::LOST_SYNC_BYTE:
            presence_[presence::index(presence::fields::LOST_SYNC_BYTE)] |= presence::mask(presence::fields::LOST_SYNC_BYTE);
            return_value = lost_sync_byte_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::RESETING_COMMS:
            presence_[presence::index(presence::fields::RESETING_COMMS)] |= presence::mask(presence::fields::RESETING_COMMS);
            return_value = reseting_comms_.deserialize_check_type(buffer, wire_type);
            break;

          case FieldNumber::UPTIME:
            presence_[presence::index(presence::fields::UPTIME)] |= presence::mask(presence::fields::UPTIME);
            return_value = uptime_.deserialize_check_type(buffer, wire_type);
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
      clear_calib_status();
      clear_hts_temp();
      clear_hts_humidity();
      clear_batt_temp();
      clear_curr_servo_pan();
      clear_curr_servo_tilt();
      clear_home_servo_pan();
      clear_home_servo_tilt();
      clear_loco_status();
      clear_dev_status();
      clear_i2c_msg_failed();
      clear_i2c_msg_passed();
      clear_uart_msg_failed();
      clear_uart_msg_passed();
      clear_lost_sync_byte();
      clear_reseting_comms();
      clear_uptime();

    }

    static char const* field_number_to_name(const FieldNumber fieldNumber)
    {
      char const* name = nullptr;
      switch(fieldNumber)
      {
        case FieldNumber::CPU_TEMP:
          name = CPU_TEMP_NAME;
          break;
        case FieldNumber::CALIB_STATUS:
          name = CALIB_STATUS_NAME;
          break;
        case FieldNumber::HTS_TEMP:
          name = HTS_TEMP_NAME;
          break;
        case FieldNumber::HTS_HUMIDITY:
          name = HTS_HUMIDITY_NAME;
          break;
        case FieldNumber::BATT_TEMP:
          name = BATT_TEMP_NAME;
          break;
        case FieldNumber::CURR_SERVO_PAN:
          name = CURR_SERVO_PAN_NAME;
          break;
        case FieldNumber::CURR_SERVO_TILT:
          name = CURR_SERVO_TILT_NAME;
          break;
        case FieldNumber::HOME_SERVO_PAN:
          name = HOME_SERVO_PAN_NAME;
          break;
        case FieldNumber::HOME_SERVO_TILT:
          name = HOME_SERVO_TILT_NAME;
          break;
        case FieldNumber::LOCO_STATUS:
          name = LOCO_STATUS_NAME;
          break;
        case FieldNumber::DEV_STATUS:
          name = DEV_STATUS_NAME;
          break;
        case FieldNumber::I2C_MSG_FAILED:
          name = I2C_MSG_FAILED_NAME;
          break;
        case FieldNumber::I2C_MSG_PASSED:
          name = I2C_MSG_PASSED_NAME;
          break;
        case FieldNumber::UART_MSG_FAILED:
          name = UART_MSG_FAILED_NAME;
          break;
        case FieldNumber::UART_MSG_PASSED:
          name = UART_MSG_PASSED_NAME;
          break;
        case FieldNumber::LOST_SYNC_BYTE:
          name = LOST_SYNC_BYTE_NAME;
          break;
        case FieldNumber::RESETING_COMMS:
          name = RESETING_COMMS_NAME;
          break;
        case FieldNumber::UPTIME:
          name = UPTIME_NAME;
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
      left_chars = calib_status_.to_string(left_chars, indent_level + 2, CALIB_STATUS_NAME, false);
      left_chars = hts_temp_.to_string(left_chars, indent_level + 2, HTS_TEMP_NAME, false);
      left_chars = hts_humidity_.to_string(left_chars, indent_level + 2, HTS_HUMIDITY_NAME, false);
      left_chars = batt_temp_.to_string(left_chars, indent_level + 2, BATT_TEMP_NAME, false);
      left_chars = curr_servo_pan_.to_string(left_chars, indent_level + 2, CURR_SERVO_PAN_NAME, false);
      left_chars = curr_servo_tilt_.to_string(left_chars, indent_level + 2, CURR_SERVO_TILT_NAME, false);
      left_chars = home_servo_pan_.to_string(left_chars, indent_level + 2, HOME_SERVO_PAN_NAME, false);
      left_chars = home_servo_tilt_.to_string(left_chars, indent_level + 2, HOME_SERVO_TILT_NAME, false);
      left_chars = loco_status_.to_string(left_chars, indent_level + 2, LOCO_STATUS_NAME, false);
      left_chars = dev_status_.to_string(left_chars, indent_level + 2, DEV_STATUS_NAME, false);
      left_chars = i2c_msg_failed_.to_string(left_chars, indent_level + 2, I2C_MSG_FAILED_NAME, false);
      left_chars = i2c_msg_passed_.to_string(left_chars, indent_level + 2, I2C_MSG_PASSED_NAME, false);
      left_chars = uart_msg_failed_.to_string(left_chars, indent_level + 2, UART_MSG_FAILED_NAME, false);
      left_chars = uart_msg_passed_.to_string(left_chars, indent_level + 2, UART_MSG_PASSED_NAME, false);
      left_chars = lost_sync_byte_.to_string(left_chars, indent_level + 2, LOST_SYNC_BYTE_NAME, false);
      left_chars = reseting_comms_.to_string(left_chars, indent_level + 2, RESETING_COMMS_NAME, false);
      left_chars = uptime_.to_string(left_chars, indent_level + 2, UPTIME_NAME, false);
  
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
          CPU_TEMP,
          CALIB_STATUS,
          HTS_TEMP,
          HTS_HUMIDITY,
          BATT_TEMP,
          CURR_SERVO_PAN,
          CURR_SERVO_TILT,
          HOME_SERVO_PAN,
          HOME_SERVO_TILT,
          LOCO_STATUS,
          DEV_STATUS,
          I2C_MSG_FAILED,
          I2C_MSG_PASSED,
          UART_MSG_FAILED,
          UART_MSG_PASSED,
          LOST_SYNC_BYTE,
          RESETING_COMMS,
          UPTIME
        };

        // The number of fields for which presence has to be tracked.
        static constexpr uint32_t N_FIELDS = 18;

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

      EmbeddedProto::sint32 cpu_temp_ = 0;
      EmbeddedProto::sint32 calib_status_ = 0;
      EmbeddedProto::sint32 hts_temp_ = 0;
      EmbeddedProto::sint32 hts_humidity_ = 0;
      EmbeddedProto::sint32 batt_temp_ = 0;
      EmbeddedProto::sint32 curr_servo_pan_ = 0;
      EmbeddedProto::sint32 curr_servo_tilt_ = 0;
      EmbeddedProto::sint32 home_servo_pan_ = 0;
      EmbeddedProto::sint32 home_servo_tilt_ = 0;
      EmbeddedProto::enumeration<Locomotion_Status> loco_status_ = static_cast<Locomotion_Status>(0);
      EmbeddedProto::enumeration<Device_Status> dev_status_ = static_cast<Device_Status>(0);
      EmbeddedProto::sint32 i2c_msg_failed_ = 0;
      EmbeddedProto::sint32 i2c_msg_passed_ = 0;
      EmbeddedProto::sint32 uart_msg_failed_ = 0;
      EmbeddedProto::sint32 uart_msg_passed_ = 0;
      EmbeddedProto::sint32 lost_sync_byte_ = 0;
      EmbeddedProto::sint32 reseting_comms_ = 0;
      EmbeddedProto::sint32 uptime_ = 0;

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
      EmbeddedProto::sint32 LJOY_X_ = 0;
      EmbeddedProto::sint32 LJOY_Y_ = 0;
      EmbeddedProto::sint32 RJOY_X_ = 0;
      EmbeddedProto::sint32 RJOY_Y_ = 0;
      EmbeddedProto::sint32 TR_ = 0;
      EmbeddedProto::sint32 TL_ = 0;

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
      EmbeddedProto::sint32 pan_ = 0;
      EmbeddedProto::sint32 tilt_ = 0;

};

#endif // UART_MESSAGES_H