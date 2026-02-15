#ifndef DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <sstream>
#include <string>
#include <cstdlib>

// IMPORTANT: inline sebab ini header (elak multiple definition)
inline LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate
                << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

class ArduinoComms
{
public:
  ArduinoComms() = default;

  void connect(const std::string & serial_device, int32_t baud_rate, int32_t timeout_ms)
  {
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  // existing: send raw & read line until '\n'
  std::string send_msg(const std::string & msg_to_send, bool print_output = false)
  {
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(msg_to_send);

    std::string response;
    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      std::cerr << "send_msg() ReadLine timeout." << std::endl;
    }

    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }
    return response;
  }

  // NEW: send cmd + auto add '\r' + read 1 line reply
  bool send_and_readline(const std::string & cmd, std::string & response, bool print_output = false)
  {
    response.clear();
    if (!connected()) return false;

    std::string tx = cmd;
    if (tx.empty()) tx = "\r";
    if (tx.back() != '\r') tx.push_back('\r');

    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(tx);

    try
    {
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout &)
    {
      if (print_output)
      {
        std::cerr << "send_and_readline() timeout. Sent: " << tx << std::endl;
      }
      return false;
    }

    // trim belakang \r\n dan whitespace
    while (!response.empty() &&
           (response.back() == '\n' || response.back() == '\r' ||
            response.back() == ' '  || response.back() == '\t'))
    {
      response.pop_back();
    }
    // trim depan whitespace
    while (!response.empty() &&
           (response.front() == ' ' || response.front() == '\t' ||
            response.front() == '\r' || response.front() == '\n'))
    {
      response.erase(response.begin());
    }

    if (print_output)
    {
      std::cout << "Sent: " << tx << " Recv: " << response << std::endl;
    }
    return true;
  }

  void read_encoder_values(int & val_1, int & val_2)
  {
    std::string response = send_msg("e\r");

    const std::string delimiter = " ";
    size_t del_pos = response.find(delimiter);
    std::string token_1 = response.substr(0, del_pos);
    std::string token_2 = (del_pos == std::string::npos) ? "" : response.substr(del_pos + delimiter.length());

    val_1 = std::atoi(token_1.c_str());
    val_2 = std::atoi(token_2.c_str());
  }

  void set_motor_values(int val_1, int val_2)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << "\r";
    (void)send_msg(ss.str());
  }

  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    (void)send_msg(ss.str());
  }

private:
  LibSerial::SerialPort serial_conn_;
  int timeout_ms_{1000};
};

#endif  // DIFFDRIVE_ARDUINO_ARDUINO_COMMS_HPP
