// CUBE_test.cpp : This file contains the 'main' function. Program execution begins and ends there.
//
// cube.cpp : 이 파일에는 'main' 함수가 포함됩니다. 거기서 프로그램 실행이 시작되고 종료됩니다.
//
#include <iostream>
#include "AXL.h"
#include "AXM.h"
#include <conio.h>
#include "stdio.h"

#define AXIS_0 0
#define AXIS_1 1
#define AXIS_2 2

using namespace std;

void test(void)
{
	// Library is initialzed.
	// 7 means IRQ. IRQ is auto-set by using PCI
	DWORD Code = AxlOpen(7);
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
				cout << AXIS_1 << " : y original location(" << AXIS_1_InitPos << "), original velocity(" <<
					AXIS_1_InitVel << "), original acceleration(" << AXIS_1_InitAccel << "), original Deacceleration(" << AXIS_1_InitDecel << ")\n";

				double AXIS_2_InitPos, AXIS_2_InitVel, AXIS_2_InitAccel, AXIS_2_InitDecel;
				AxmMotGetParaLoad(AXIS_0, &AXIS_2_InitPos, &AXIS_2_InitVel, &AXIS_2_InitAccel, &AXIS_2_InitDecel);
				cout << AXIS_2 << ": z original location(" << AXIS_2_InitPos << "), original velocity(" <<
					AXIS_2_InitVel << "), 초기original acceleration(" << AXIS_2_InitAccel << "), original Deacceleration(" << AXIS_2_InitDecel << ")\n";

				cout << "Check!!" << endl;

				// set linear actuator's parameter
				 
				// lAxesNo : our the number of X, Y, Z  Acutator
				
				long lAxesNo[3] = { 0,1,2 };
				double dMaxVelocity[3], dMaxAccel[3], dMaxDecel[3];
				// set Absolute Movement type
				for (long nAxisNo = 0; nAxisNo <= 2; nAxisNo++) {
					AxmMotSetAbsRelMode(nAxisNo, 0); //set absolute type
					AxmMotSetProfileMode(nAxisNo, 0); // 0 : Sym Trapezoide
				}
				
				dMaxVelocity[0] = 300000; dMaxVelocity[1] = 300000; dMaxVelocity[2] = 50000;
				dMaxAccel[0] = 60000; dMaxAccel[1] = 60000; dMaxAccel[2] = 60000;
				dMaxDecel[0] = 60000; dMaxDecel[1] = 60000; dMaxDecel[2] = 60000;

				// Declare All of Position (vial holder on stirring machine)
				double d_1_Position[2], d_2_Position[2], d_3_Position[2], d_4_Position[2], d_z1_Position;
				double d_5_Position[2], d_6_Position[2], d_7_Position[2], d_8_Position[2], d_z2_Position;
				d_1_Position[0] = 26334, d_1_Position[1] = 596956;
				d_2_Position[0] = 221730, d_2_Position[1] = 598583;
				d_3_Position[0] = 421733, d_3_Position[1] = 598583;
				d_4_Position[0] = 622436, d_4_Position[1] = 596173;
				d_5_Position[0] = 21352, d_5_Position[1] = 402285;
				d_6_Position[0] = 219677, d_6_Position[1] = 402285;
				d_7_Position[0] = 422472, d_7_Position[1] = 404625;
				d_8_Position[0] = 619053, d_8_Position[1] = 406185;
				d_z1_Position = 168000;
				d_z2_Position = 200000;

				// vial 1
				AxmMovePos(0, d_1_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_1_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 2
				AxmMovePos(0, d_2_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_2_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 3
				AxmMovePos(0, d_3_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_3_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 4
				AxmMovePos(0, d_4_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_4_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 5
				AxmMovePos(0, d_5_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_5_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 6
				AxmMovePos(0, d_6_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_6_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 7
				AxmMovePos(0, d_7_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_7_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				// vial 8
				AxmMovePos(0, d_8_Position[0], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(1, d_8_Position[1], dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z1_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);
				AxmMovePos(2, d_z2_Position, dMaxVelocity[0], dMaxAccel[0], dMaxDecel[0]);

				// waiting for motion until all of movement is ready!
				cout << "Move!!" << endl;

				BOOL WaitForDone = TRUE;
				DWORD uStatus0, uStatus1, uStatus2;

				while (WaitForDone) {
					AxmStatusReadInMotion(AXIS_0, &uStatus0);
					AxmStatusReadInMotion(AXIS_1, &uStatus1);
					AxmStatusReadInMotion(AXIS_2, &uStatus2);
					if (uStatus0 == 0 && uStatus1 == 0 && uStatus2 == 0)
						WaitForDone = FALSE;
				}
				printf("All Movement is finished.\n");


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
					printf("\r ongoing progress... x Axis : %d%, y Axis : %d, z Axis : %d", uHomeStepNumber_0,
						uHomeStepNumber_1, uHomeStepNumber_2);
				}
				
				
			}
			else
				printf("AxmInfoIsMotionModule() : ERROR ( NOT STATUS_EXIST )code 0x%x\n", Code);
		}
		else
			printf("AxmInfoIsMotionModule() : ERROR ( Return FALSE ) code 0x%x\n", Code);
	}
	else
		printf("AxlOpen() : ERROR code 0x%x\n", Code);

	// Terminate Library
	if (AxlClose())
		printf("\nTerminate Library.\n");
	else
		printf("Library is not terminated normally.\n");
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
