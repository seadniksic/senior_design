#include <iostream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

int main() {

  int sock = socket(AF_INET, SOCK_DGRAM, 0);

  if (sock == -1) {
    std::cerr << "Failed to create socket." << std::endl;
    return 1;
  }

  struct sockaddr_in server_address;
  server_address.sin_family = AF_INET;
  server_address.sin_port = htons(8080);
  server_address.sin_addr.s_addr = htonl(INADDR_ANY);

  int bind_result = bind(sock, (struct sockaddr *) &server_address, sizeof(server_address));
  if (bind_result == -1) {
    std::cerr << "Failed to bind socket to address." << std::endl;
    return 1;
  }

  char buffer[1024];
  int bytes_received = recvfrom(sock, buffer, sizeof(buffer), 0, nullptr, nullptr);
  if (bytes_received == -1) {
    std::cerr << "Failed to receive data." << std::endl;
    return 1;
  }

  buffer[bytes_received] = '\0';
  std::cout << "Received: " << buffer << std::endl;

  close(sock);
  return 0;
}
