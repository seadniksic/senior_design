#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

#define ipAddress "10.42.0.1"

int main() {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == -1) {
    std::cerr << "Failed to create socket." << std::endl;
    return 1;
  }

  struct sockaddr_in server_address;
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(8080);
  server_address.sin_addr.s_addr = inet_addr(ipAddress);

  char data[] = "Hello, World!";
  int data_len = sizeof(data) / sizeof(data[0]);
  int send_result = sendto(sock, data, data_len, 0, (struct sockaddr *) &server_address, sizeof(server_address));
  if (send_result == -1) {
    std::cerr << "Failed to send data." << std::endl;
    return 1;
  }

  std::cout << "Data sent successfully." << std::endl;

  close(sock);
  return 0;
}
