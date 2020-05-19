/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#pragma once
#ifndef __STREAM_SOCKET_H__
#define __STREAM_SOCKET_H__

#ifdef _WIN32
#undef UNICODE

#define WIN32_LEAN_AND_MEAN
#ifdef MINGW_COMPILE
#define _WIN32_WINNT 0x0600
#endif
#endif

#include <string>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif


class I_Stream_Socket {
public:
	~I_Stream_Socket();
// methods
	virtual void InitConnection() = 0;

	void Recv(char* recvbuf, const int& N_byte);
	void Send(const char* sendbuf, const int& N_byte);

	void Recv(int* data);
	void Send(const int& data);

	void Recv(std::string* buff_rcv);
	void Send(const std::string& buffer_snd);
protected:
#ifdef _WIN32


	I_Stream_Socket(const std::string& server_address, const int& port) :
		mAddress_server(server_address), mPort(std::to_string(port)), mConnection(INVALID_SOCKET) {};
// data
	SOCKET			mConnection; 
	std::string		mAddress_server;
	std::string		mPort;


#else


	I_Stream_Socket(const std::string& server_address, const int& port_to_use) : 
		mAddress_server(server_address) , port(port_to_use) {};
// data
	std::string		mAddress_server;
	int port;
	int sockfd;


#endif
};



class Stream_to_Server : public I_Stream_Socket {
public:
	Stream_to_Server(const std::string& server_address, const int& port):
		I_Stream_Socket(server_address, port) {}; //use "" for local host
	Stream_to_Server(const int& port) : Stream_to_Server("", port) {}; //localhost is assumed

	void InitConnection();
};



class Stream_to_Client : public I_Stream_Socket {
public:
	Stream_to_Client(const int& port) : I_Stream_Socket("", port) {};

	void InitConnection();
};

#endif // !__SOCKET_MY_H__