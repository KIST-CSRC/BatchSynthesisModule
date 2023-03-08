#include "TCPServer.h"
#include <iostream>
#include <string>
#include <sstream>
#include "AXL.h"
#include "AXM.h"
#include <conio.h>
#include <vector>


#define AXIS_0 0
#define AXIS_1 1
#define AXIS_2 2

using namespace std;
const int MAX_BUFFER_SIZE = 4096;			//Constant value for the buffer size = where we will store the data received.

DWORD Code = AxlOpen(7);
DWORD uStatus;

// Declare All of Position (vial holder on stirring machine)
//double d_vial_Position[8][2] = { {26334,596956}, {221730,598583}, {421733,598583},
//								 {622436,596173}, {21352,402285}, {219677,402285},
//	                             {422472,404625}, {619053,406185} };
double d_vial_Position[16][2] = { {875280,109500}, {804801,108642}, {714305,108642}, {643024,109322}, 
								 {872612,182150}, {805784,181563}, {714245,180589}, {644397,180267},
								 {874856,271525}, {803285,271525}, {712367,271780}, {643526,271387},
								 {873083,341829}, {803350,342219}, {711573,342433}, {642103,343346}};
double d_z1_Position = 200000, d_z2_Position = 168000;

// Declare linear actuator velocity
double dMaxVelocity[3] = { 7500000, 7500000, 1200000 };
double dMaxAccel[3] = { 80000, 80000, 100000 };
double dMaxDecel[3] = { 80000, 80000, 100000 };


TCPServer::TCPServer(string ipAddress, int port)
	: listenerIPAddress(ipAddress), listenerPort(port) {
}

TCPServer::~TCPServer() {
	cleanupWinsock();			//Cleanup Winsock when the server shuts down. 
}


//Function to check whether we were able to initialize Winsock & start the server. 
bool TCPServer::initWinsock() {

	WSADATA data;
	WORD ver = MAKEWORD(2, 2);

	int wsInit = WSAStartup(ver, &data);

	if (wsInit != 0) {
		cout << "Error: can't initialize Winsock." << std::endl;
		return false;
	}
	
	if (Code == AXT_RT_SUCCESS)
	{
		printf("Library is initialzed.\n");
		// search motion module
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			if (uStatus == STATUS_EXIST)
			{
				printf("Motion Modlue can read in our computer.\n");

				//Servo On
				AxmSignalServoOn(AXIS_0, ENABLE);
				AxmSignalServoOn(AXIS_1, ENABLE);
				AxmSignalServoOn(AXIS_2, ENABLE);

				// read Save_Para.mot and Load all of parameter in our memory
				char pFilename[20] = "configuration.mot";
				AxmMotLoadParaAll(pFilename);

				double AXIS_0_InitPos, AXIS_0_InitVel, AXIS_0_InitAccel, AXIS_0_InitDecel;
				AxmMotGetParaLoad(AXIS_0, &AXIS_0_InitPos, &AXIS_0_InitVel, &AXIS_0_InitAccel, &AXIS_0_InitDecel);
				cout << AXIS_0 << ": x original location(" << AXIS_0_InitPos << "), original velocity(" <<
					AXIS_0_InitVel << "), original acceleration(" << AXIS_0_InitAccel << "), original Deacceleration(" << AXIS_0_InitDecel << ")\n";

				double AXIS_1_InitPos, AXIS_1_InitVel, AXIS_1_InitAccel, AXIS_1_InitDecel;
				AxmMotGetParaLoad(AXIS_1, &AXIS_1_InitPos, &AXIS_1_InitVel, &AXIS_1_InitAccel, &AXIS_1_InitDecel);
				cout << AXIS_1 << ": y original location(" << AXIS_1_InitPos << "), original velocity(" <<
					AXIS_1_InitVel << "), original acceleration(" << AXIS_1_InitAccel << "), original Deacceleration(" << AXIS_1_InitDecel << ")\n";

				double AXIS_2_InitPos, AXIS_2_InitVel, AXIS_2_InitAccel, AXIS_2_InitDecel;
				AxmMotGetParaLoad(AXIS_0, &AXIS_2_InitPos, &AXIS_2_InitVel, &AXIS_2_InitAccel, &AXIS_2_InitDecel);
				cout << AXIS_2 << ": z original location(" << AXIS_2_InitPos << "), original velocity(" <<
					AXIS_2_InitVel << "), original acceleration(" << AXIS_2_InitAccel << "), original Deacceleration(" << AXIS_2_InitDecel << ")\n";

				cout << "Check!!" << endl;
			}
			else
				printf("AxmInfoIsMotionModule() : ERROR ( NOT STATUS_EXIST )code 0x%x\n", Code);
		}
		else
			printf("AxmInfoIsMotionModule() : ERROR ( Return FALSE ) code 0x%x\n", Code);
	}
	else
		printf("AxlOpen() : ERROR code 0x%x\n", Code);

	return true;
}


//Function that creates a listening socket of the server. 
SOCKET TCPServer::createSocket() {

	SOCKET listeningSocket = socket(AF_INET, SOCK_STREAM, 0);	//AF_INET = IPv4. 

	if (listeningSocket != INVALID_SOCKET) {

		sockaddr_in hint;		//Structure used to bind IP address & port to specific socket. 
		hint.sin_family = AF_INET;		//Tell hint that we are IPv4 addresses. 
		hint.sin_port = htons(listenerPort);	//Tell hint what port we are using. 
		inet_pton(AF_INET, listenerIPAddress.c_str(), &hint.sin_addr); 	//Converts IP string to bytes & pass it to our hint. hint.sin_addr is the buffer. 

		int bindCheck = bind(listeningSocket, (sockaddr*)&hint, sizeof(hint));	//Bind listeningSocket to the hint structure. We're telling it what IP address family & port to use. 

		if (bindCheck != SOCKET_ERROR) {			//If bind OK:

			int listenCheck = listen(listeningSocket, SOMAXCONN);	//Tell the socket is for listening. 
			if (listenCheck == SOCKET_ERROR)
				return -1;
		}

		else
			return -1;

		return listeningSocket;
	}

}

int TCPServer::callLinearActuator_pos(int x, int y, int z) {
	int status = 0;
	
	/*
	set linear actuator's parameter

	lAxesNo : our the number of X, Y, Z  Acutator
	*/
	long lAxesNo[3] = { 0,1,2 };
				
	// set Absolute Movement type
	for (long nAxisNo = 0; nAxisNo <= 2; nAxisNo++) {
		AxmMotSetAbsRelMode(nAxisNo, 0); //set absolute type
		AxmMotSetProfileMode(nAxisNo, 0); // 0 : Sym Trapezoide
	}
				
	AxmMovePos(AXIS_0, x, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
	AxmMovePos(AXIS_1, y, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
	AxmMovePos(AXIS_2, z, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
	
	/*
	if (up_status == 1) {
		AxmMovePos(AXIS_2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
	}
	*/
				
	// waiting for motion until all of movement is ready!
	cout << "Move!!" << endl;

	BOOL WaitForDone = TRUE;
	DWORD uStatus0, uStatus1, uStatus2;

	while (WaitForDone) {
		AxmStatusReadInMotion(AXIS_0, &uStatus0);
		AxmStatusReadInMotion(AXIS_1, &uStatus1);
		AxmStatusReadInMotion(AXIS_2, &uStatus2);
		if (uStatus0 == 0 && uStatus1 == 0 && uStatus2 == 0) {
			WaitForDone = FALSE;
			status = 1;
		}
						
	}
	printf("All Movement is finished.\n");
			
	return status;
}

void TCPServer::callHome_pos() {
	// search Home Location
	AxmHomeSetStart(AXIS_0);
	AxmHomeSetStart(AXIS_1);
	AxmHomeSetStart(AXIS_2);

	// check current home location progress
	DWORD uHomeResult_0, uHomeResult_1, uHomeResult_2;
	DWORD uHomeMainStepNumber, uHomeStepNumber_0;
	DWORD uHomeStepNumber_1, uHomeStepNumber_2;
	AxmHomeGetResult(AXIS_0, &uHomeResult_0);
	AxmHomeGetResult(AXIS_1, &uHomeResult_1);
	AxmHomeGetResult(AXIS_2, &uHomeResult_2);

	// process in progress : 0, process finished : 1
	while (uHomeResult_0 == HOME_SEARCHING || uHomeResult_1 == HOME_SEARCHING || uHomeResult_2 == HOME_SEARCHING)
	{
		AxmHomeGetResult(AXIS_0, &uHomeResult_0);
		AxmHomeGetResult(AXIS_1, &uHomeResult_1);
		AxmHomeGetResult(AXIS_2, &uHomeResult_2);
		AxmHomeGetRate(AXIS_0, &uHomeMainStepNumber, &uHomeStepNumber_0);
		AxmHomeGetRate(AXIS_1, &uHomeMainStepNumber, &uHomeStepNumber_1);
		AxmHomeGetRate(AXIS_2, &uHomeMainStepNumber, &uHomeStepNumber_2);
		printf("\r ongoing progress... x Axis : %d%, y Axis : %d, z Axis : %d\n", uHomeStepNumber_0,
			uHomeStepNumber_1, uHomeStepNumber_2);
	}
}

//Function doing the main work of the server -> evaluates sockets & either accepts connections or receives data. 
void TCPServer::run() {

	char buf[MAX_BUFFER_SIZE];		//Create the buffer to receive the data from the clients. 
	SOCKET listeningSocket = createSocket();		//Create the listening socket for the server. 
	cout << "Linear Actuator server connection is created!" << endl;

	while (true) {
		cout << "listeningSocket" << listeningSocket << endl;
		cout << "INVALID_SOCKET" << INVALID_SOCKET << endl;
		if (listeningSocket == INVALID_SOCKET) {
			break;
		}

		fd_set master;				//File descriptor storing all the sockets.
		FD_ZERO(&master);			//Empty file file descriptor. 

		FD_SET(listeningSocket, &master);		//Add listening socket to file descriptor. 

		while (true) {

			fd_set copy = master;	//Create new file descriptor bc the file descriptor gets destroyed every time. 
			int socketCount = select(0, &copy, nullptr, nullptr, nullptr);				//Select() determines status of sockets & returns the sockets doing "work".

			for (int i = 0; i < socketCount; i++) {				//Server can only accept connection & receive msg from client. 

				SOCKET sock = copy.fd_array[i];					//Loop through all the sockets in the file descriptor, identified as "active". 

				if (sock == listeningSocket) {				//Case 1: accept new connection.

					SOCKET client = accept(listeningSocket, nullptr, nullptr);		//Accept incoming connection & identify it as a new client. 
					FD_SET(client, &master);		//Add new connection to list of sockets.  

					string welcomeMsg = "Connection accepted.\n";			//Notify client that he entered the chat. 
					cout << "Listening requests" << endl;			//Log connection on server side. 
				}
				else {										//Case 2: receive a msg.	
					ZeroMemory(buf, MAX_BUFFER_SIZE);		//Clear the buffer before receiving data. 
					int bytesReceived = recv(sock, buf, MAX_BUFFER_SIZE, 0);	//Receive data into buf & put it into bytesReceived. 

					if (bytesReceived <= 0) {	//No msg = drop client. 
						closesocket(sock);
						FD_CLR(sock, &master);	//Remove connection from file director.
					}
					else {						//Send msg to other clients & not listening socket.
						for (int i = 0; i < master.fd_count; i++) {			//Loop through the sockets. 
							SOCKET outSock = master.fd_array[i];
							if (outSock != listeningSocket) {
								if (outSock == sock) {		//If the current socket is the one that sent the message:
									if (bytesReceived > 0) {
										string message = string(buf, 0, bytesReceived);
										//cout << message << endl;
										string msgSent = "";

										if (message == "closed") {
											this->callHome_pos();
											// Terminate Library
											if (AxlClose())
												printf("\nTerminate Library.\n");
											else
												printf("Library is not terminated normally.\n");

											msgSent = "Terminated.";
											send(outSock, msgSent.c_str(), msgSent.size() + 1, 0);
											this->cleanupWinsock();
											exit(0);
										}
										else if (message == "home") {
											this->callHome_pos();
											msgSent = "Move back to home location.";
											send(outSock, msgSent.c_str(), msgSent.size() + 1, 0);
										}
										else if (message == "hello") {
											msgSent = "Hello World!! Succeed to connection to main computer!";
											send(outSock, msgSent.c_str(), msgSent.size() + 1, 0);
										}
										else {
											vector<string> v;
											stringstream ss(message);

											while (ss.good()) {
												string substr;
												getline(ss, substr, ',');
												v.push_back(substr);
											}

											cout << stoi(v[0]) << ", " << stoi(v[1]) << ", " << stoi(v[2]);
											int status = this->callLinearActuator_pos(stoi(v[0]), stoi(v[1]), stoi(v[2]));

											if (status == 1) {
												msgSent = "X: " + v[0] + ", Y: " + v[1] + ", Z: " + v[2]
													+ " is done -> for injection will be started.";
												send(outSock, msgSent.c_str(), msgSent.size() + 1, 0);
												cout << msgSent << endl;
												cout << "Done~\n" << endl;
											}

											/*
											int status = this->callLinearActuator_pos(stoi(v[0]), stoi(v[1]), stoi(v[2]));

											if (status == 1) {
												if (v[1] == "1")
													msgSent = "[Linear Actuator] : move position " + v[0] + " is done -> for injection is done.";
												else if (v[2] == "1")
													msgSent = "[Linear Actuator] : move position " + v[0] + " is done -> for injection will be started.";
												send(outSock, msgSent.c_str(), msgSent.size() + 1, 0);
												cout << msgSent << endl;
												cout << "Done~\n" << endl;
											}
											*/
										}
									}
								}
							}
						}
					}
				}
			}
		}
	}
}


//Function to send the message to a specific client. 
void TCPServer::sendMsg(int clientSocket, std::string msg) {

	send(clientSocket, msg.c_str(), msg.size() + 1, 0);

}


void TCPServer::cleanupWinsock() {

	WSACleanup();

}
