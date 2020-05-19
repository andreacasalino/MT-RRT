/**
 * Author:    Andrea Casalino
 * Created:   16.05.2019
*
* report any bug to andrecasa91@gmail.com.
 **/
 
#include "Stream_Socket.h"
#include <iostream>
#include <malloc.h>
using namespace std;

#ifdef _WIN32
#elif  __linux__
#include "Stream_Socket.h"
#include <strings.h> //only for bzero
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <unistd.h>
#include <iostream>
#include <malloc.h>
#else
#error Os not supported: only Windows and Linux are supported
#endif

#ifdef _WIN32
#ifndef MINGW_COMPILE
#pragma comment (lib, "Ws2_32.lib")
#endif
#endif


#ifdef _WIN32
I_Stream_Socket::~I_Stream_Socket() {
	// cleanup
	closesocket(this->mConnection);
	WSACleanup();
}

void I_Stream_Socket::Recv(char* recvbuf, const int& N_byte) {

	if (N_byte <= 0) {
		std::cout << "N_byte cannot be less than 0 \n";
		abort();
	}

	int iResult = recv(this->mConnection, recvbuf, N_byte, 0);
	if (iResult < 0) throw 0;

}

void I_Stream_Socket::Send(const char* sendbuf, const int& N_byte) {

	if (N_byte <= 0) {
		std::cout << "N_byte cannot be less than 0 \n";
		abort();
	}

	int iSendResult = send(this->mConnection, sendbuf, N_byte, 0);
	if (iSendResult == SOCKET_ERROR) throw 0;

}
#elif  __linux__
I_Stream_Socket::~I_Stream_Socket() {
	// cleanup
	//TODO
	cout << "Clean up not implemented when destroying the socket\n";
}

void I_Stream_Socket::Recv(char* recvbuf, const int& N_byte) {

	if (N_byte <= 0) cerr << "N_byte cannot be less than 0 \n";

	int nbytes = recv(this->sockfd, &recvbuf[0], N_byte, 0);
	if (nbytes < 0) cerr << "ERROR reading from socket";

}

void I_Stream_Socket::Send(const char* sendbuf, const int& N_byte) {

	if (N_byte <= 0)  cerr << "N_byte cannot be less than 0 \n";

	int nbytes = send(this->sockfd, &sendbuf[0], N_byte, 0);
	if (nbytes < 0) cerr << "ERROR sending to socket";

}
#endif

void I_Stream_Socket::Recv(int* data) {

	char Bytes[4];

	// Receiving current node
	this->Recv(&Bytes[0], 4);
	*data = 0;
	*data = 0x000000FF & Bytes[3];
	*data = ((*data << 8) & 0x0000FFFF) | (0x000000FF & Bytes[2]);
	*data = ((*data << 8) & 0x00FFFFFF) | (0x000000FF & Bytes[1]);
	*data = ((*data << 8) & 0xFFFFFFFF) | (0x000000FF & Bytes[0]);

}

void I_Stream_Socket::Send(const int& data) {

	char Bytes[4];
	int myint = data;

	// Sending delta
	for (int i = 0; i < 4; i++) Bytes[i] = (myint >> 8 * i) & 0xFF;
	this->Send(&Bytes[0], 4);

}

void I_Stream_Socket::Recv(std::string* buff_rcv) {

	int buff_size;
	this->Recv(&buff_size);
	buff_rcv->resize(buff_size);
	char* temp = (char*)malloc(buff_size * sizeof(char));
	this->Recv(&temp[0], buff_size);
	for (size_t k = 0; k < buff_size; k++)
		(*buff_rcv)[k] = temp[k];
	free(temp);

}

void I_Stream_Socket::Send(const std::string& buff_snd) {

	this->Send((int)buff_snd.size());
	this->Send(buff_snd.c_str(), (int)buff_snd.size());

}



#ifdef _WIN32
void Stream_to_Server::InitConnection() {

	while (true) { //blocking operation: retry the connection to the server till succeed

		WSADATA wsaData; //Create a WSADATA object called wsaData.
		//Call WSAStartup and return its value as an integer and check for errors.
		this->mConnection = INVALID_SOCKET;
		//CREATING A SOCKET FOR THE CLIENT
		/*Declare an addrinfo object that contains a sockaddr structure and initialize these values.
		For this application, the Internet address family is unspecified so that either an IPv6 or IPv4 address can be
		returned. The application requests the socket type to be a stream socket for the TCP protocol.*/
		struct addrinfo* result = NULL, * ptr = NULL, hints;
		//Send and recv functions used by the client once a connection is established.
		/*The send and recv functions both return an integer value of the number of bytes
		sent or received, respectively, or an error. Each function also takes the same parameters:
		the active socket, a char buffer, the number of bytes to send or receive, and any flags to use.*/
		int iResult;
		// Initialize Winsock
		iResult = WSAStartup(MAKEWORD(2, 2), &wsaData); //WSAStartup function initiates use of the Winsock DLL by a process.
		if (iResult != 0) {
			printf("WSAStartup failed with error: %d\n", iResult);
			abort();
		}
		ZeroMemory(&hints, sizeof(hints));
		hints.ai_family = AF_UNSPEC; //AF_UNSPEC either an IPv6 or IPv4 address can be returned.
		hints.ai_socktype = SOCK_STREAM; //The application requests the socket type to be a stream socket for the TCP protocol
		hints.ai_protocol = IPPROTO_TCP; //TCP protocol
		if (this->mAddress_server.compare("") == 0) //loaclhost
			iResult = getaddrinfo("127.0.0.1", this->mPort.c_str(), &hints, &result);
		else
			iResult = getaddrinfo(this->mAddress_server.c_str(), this->mPort.c_str(), &hints, &result);
		if (iResult != 0) {
			printf("getaddrinfo failed with error: %d\n", iResult);
			WSACleanup();
			abort();
		}
		/*Call the socket function and return its value to the ConnectSocket variable. For this application, use the
		first IP address returned by the call to getaddrinfo that matched the address family, socket type, and protocol
		specified in the hints parameter. In this example, a TCP stream socket was specified with a socket type of
		SOCK_STREAM and a protocol of IPPROTO_TCP. The address family was left unspecified (AF_UNSPEC), so the returned
		IP address could be either an IPv6 or IPv4 address for the server.*/

		/*If the client application wants to connect using only IPv6 or IPv4, then the address family needs to be set
		to AF_INET6 for IPv6 or AF_INET for IPv4 in the hints parameter.*/ //SEE LINE 44

		// Attempt to connect to an addres until one succeds
		for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {

			// Create a SOCKET for connecting to server
			this->mConnection = socket(ptr->ai_family, ptr->ai_socktype,
				ptr->ai_protocol);

			//Check for errors to ensure that the socket is a valid socket.
			/*If the socket call fails, it returns INVALID_SOCKET. The if statement in the previous code is used to catch
			any errors that may have occurred while creating the socket. WSAGetLastError returns an error number associated
			with the last error that occurred.*/
			if (this->mConnection == INVALID_SOCKET) {
				printf("socket failed with error: %ld\n", WSAGetLastError());
				//freeaddrinfo(result);
				WSACleanup(); //WSACleanup is used to terminate the use of the WS2_32 DLL.
				abort();
			}

			// Connect to server.
			iResult = connect(this->mConnection, ptr->ai_addr, (int)ptr->ai_addrlen);
			if (iResult == SOCKET_ERROR) {
				closesocket(this->mConnection);
				this->mConnection = INVALID_SOCKET;
				continue;
			}

			break;
		}

		// Should really try the next address returned by getaddrinfo
		// if the connect call failed
		// But for this simple example we just free the resources
		// returned by getaddrinfo and print an error message

		freeaddrinfo(result);

		if (this->mConnection == INVALID_SOCKET) {
			//printf("Unable to connect to server!\n");
			WSACleanup();
		}
		else break;

	}

}

void Stream_to_Client::InitConnection() {

	WSADATA wsaData;
	int iResult;

	SOCKET ListenSocket = INVALID_SOCKET;
	this->mConnection = INVALID_SOCKET;

	struct addrinfo* result = NULL;
	struct addrinfo hints;

	// Initialize Winsock
	iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != 0) {
		printf("WSAStartup failed with error: %d\n", iResult);
		abort();
	}

	ZeroMemory(&hints, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP; //TCP protocol
	hints.ai_flags = AI_PASSIVE;

	// Resolve the server address and port
	if (this->mAddress_server.compare("") == 0) //loaclhost
		iResult = getaddrinfo(NULL, this->mPort.c_str(), &hints, &result);
	else
		iResult = getaddrinfo(this->mAddress_server.c_str(), this->mPort.c_str(), &hints, &result);

	if (iResult != 0) {
		printf("getaddrinfo failed with error: %d\n", iResult);
		WSACleanup();
		abort();
	}

	// Create a SOCKET for connecting to server
	ListenSocket = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
	if (ListenSocket == INVALID_SOCKET) {
		printf("socket failed with error: %ld\n", WSAGetLastError());
		freeaddrinfo(result);
		WSACleanup();
		abort();
	}

	// Setup the TCP listening socket
	iResult = bind(ListenSocket, result->ai_addr, (int)result->ai_addrlen);
	if (iResult == SOCKET_ERROR) {
		printf("bind failed with error: %d\n", WSAGetLastError());
		freeaddrinfo(result);
		closesocket(ListenSocket);
		WSACleanup();
		abort();
	}

	freeaddrinfo(result);

	iResult = listen(ListenSocket, SOMAXCONN);
	if (iResult == SOCKET_ERROR) {
		printf("listen failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		abort();
	}

	// Accept a client socket
	this->mConnection = accept(ListenSocket, NULL, NULL);
	if (this->mConnection == INVALID_SOCKET) {
		printf("accept failed with error: %d\n", WSAGetLastError());
		closesocket(ListenSocket);
		WSACleanup();
		abort();
	}

	// No longer need server socket
	closesocket(ListenSocket);

}
#elif __linux__
#include <strings.h> //only for bzero
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <unistd.h>
#include <arpa/inet.h>

void Stream_to_Server::InitConnection() {
	
    struct sockaddr_in serv_addr; 
	this->sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (this->sockfd < 0) cerr << "ERROR opening socket";
	
	bzero((char*)&serv_addr, sizeof(serv_addr));
	serv_addr.sin_family = AF_INET;
	if(inet_pton(AF_INET, this->mAddress_server.c_str(), &serv_addr.sin_addr)<=0) cerr << "Server not found";
	serv_addr.sin_port = htons(this->port);	
	if(connect(this->sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) cerr << "Connection failed";
	
};

void Stream_to_Client::InitConnection() {
#ifdef PRINT_CONNECTION_INFO
	cout << "Creating a new strem\n";
#endif

	struct sockaddr_in serv_addr, cli_addr;

	int sockfd = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd < 0) cerr << "ERROR opening socket";
	//disable Nagle algorithm
	int flag = 1;
	int result = setsockopt(sockfd, IPPROTO_TCP, TCP_NODELAY, (char*)&flag, sizeof(int));
	if (result < 0) cerr << "ERROR setting option socket";

	bzero((char*)&serv_addr, sizeof(serv_addr));
	//portno = atoi(argv[1]);
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(this->port);
	socklen_t clilen;
#ifdef PRINT_CONNECTION_INFO
	cout << "Waiting for the client to ask the connection\n";
#endif
	if (bind(sockfd, (struct sockaddr*) & serv_addr, sizeof(serv_addr)) < 0) cerr << "ERROR on binding";

	listen(sockfd, 5);
	clilen = sizeof(cli_addr);
	this->sockfd = accept(sockfd, (struct sockaddr*) & cli_addr, &clilen);
	if (this->sockfd < 0) cerr << "ERROR on accept";

#ifdef PRINT_CONNECTION_INFO
	cout << "Connection done\n";
#endif
}
#endif
