#include <iostream>
#include <cstdio>
#include <cstdlib>

int main(int argc, char* argv[]) {
  std::string ssid, password;

  std::cout << "Enter the SSID of the hotspot: ";
  std::cin >> ssid;
  
  std::cout << "Enter the password for the hotspot: ";
  std::cin >> password;
  
  std::string cmd = "nmcli dev wifi hotspot ifname wlan0 ssid " + ssid + " password " + password;
  int ret = system(cmd.c_str());
  
  if (ret == 0) {
    std::cout << "WiFi hotspot created successfully!" << std::endl;
  } else {
    std::cout << "Failed to create WiFi hotspot" << std::endl;
  }

  return 0;
}
