/*
int main()
{
	// 라이브러리를 초기화한다.
	// 7은 IRQ를 뜻한다. PCI에서 자동으로 IRQ가 설정된다.
	DWORD Code = AxlOpen(7);
	if (Code == AXT_RT_SUCCESS)
	{
		printf("라이브러리가 초기화되었습니다.\n");
		//모션 모듈이 있는지 검사.
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			if (uStatus == STATUS_EXIST)
			{
				printf("모션 모듈이 존재합니다.\n");
				//시스템에 장착된 축 개수를 확인.
				long lpAxisCount;
				AxmInfoGetAxisCount(&lpAxisCount);
				printf("시스템에장착된축개수 : %d \n", lpAxisCount);
				//0번 축의 보드 번호, 모듈 위치, 모듈 ID를 확인한다.
				long lBoardNo, lModulePos;
				long lAxisNo = 0;
				DWORD uModuleID;
				AxmInfoGetAxis(lAxisNo, &lBoardNo, &lModulePos, &uModuleID);
				printf("0번 축의 보드 번호 : %x, 모듈 위치 : %x, 모듈 ID : %x\n",
					lBoardNo, lModulePos, uModuleID);
				//0번 보드의 1번 모듈에서 시작 축 번호를 확인한다.
				long lFirstAxisNo;
				AxmInfoGetFirstAxisNo(lBoardNo, lModulePos, &lFirstAxisNo);
				printf("%d번 베이스 보드의 %d번째 모듈의 시작 축 번호 : %d\n",
					lBoardNo, lModulePos, lFirstAxisNo);
			}
			else
				printf("AxmInfoIsMotionModule() : ERROR ( NOT STATUS_EXIST ) code 0x%x\n", Code);
		}
		else
			printf("AxmInfoIsMotionModule() : ERROR ( Return FALSE ) code 0x%x\n", Code);
	}
	else
		printf("AxlOpen() : ERROR code 0x%x\n", Code);

	//Save_Para.mot 파일에 저장되어 있는 모든 축의 모션 정보를 읽어와서 설정한다.
	char pFilename[20] = "configuration.mot";
	AxmMotLoadParaAll(pFilename);

	double AXIS_0_InitPos, AXIS_0_InitVel, AXIS_0_InitAccel, AXIS_0_InitDecel;
	AxmMotGetParaLoad(AXIS_0, &AXIS_0_InitPos, &AXIS_0_InitVel, &AXIS_0_InitAccel, &AXIS_0_InitDecel);
	cout << AXIS_0 << "축 초기 위치(" << AXIS_0_InitPos << "), 초기 속도(" <<
		AXIS_0_InitVel << "), 초기 가속도(" << AXIS_0_InitAccel << "), 초기 감속도(" << AXIS_0_InitDecel << ")\n";

	double AXIS_1_InitPos, AXIS_1_InitVel, AXIS_1_InitAccel, AXIS_1_InitDecel;
	AxmMotGetParaLoad(AXIS_1, &AXIS_1_InitPos, &AXIS_1_InitVel, &AXIS_1_InitAccel, &AXIS_1_InitDecel);
	cout << AXIS_0 << "축 초기 위치(" << AXIS_1_InitPos << "), 초기 속도(" <<
		AXIS_1_InitVel << "), 초기 가속도(" << AXIS_1_InitAccel << "), 초기 감속도(" << AXIS_1_InitDecel << ")\n";

	double AXIS_2_InitPos, AXIS_2_InitVel, AXIS_2_InitAccel, AXIS_2_InitDecel;
	AxmMotGetParaLoad(AXIS_0, &AXIS_2_InitPos, &AXIS_2_InitVel, &AXIS_2_InitAccel, &AXIS_2_InitDecel);
	cout << AXIS_0 << "축 초기 위치(" << AXIS_2_InitPos << "), 초기 속도(" <<
		AXIS_2_InitVel << "), 초기 가속도(" << AXIS_2_InitAccel << "), 초기 감속도(" << AXIS_2_InitDecel << ")\n";

	//서보 드라이브가 On되어 있는지를 확인한다.
	for(long i=0 ; i<=2 ; i++)
	{
		DWORD uUse = ENABLE;
		AxmSignalServoOn(i, uUse);
		AxmSignalIsServoOn(i, &uUse);
		if (uUse == ENABLE) {
			cout << "서보 드라이브가 ON 되어있습니다.\n";
		}
		else if (uUse == DISABLE) {
			cout << "서보 드라이브가 OFF 되어있습니다.\n";
		}
	}

	// 설정된 이동 모드를 읽는 함수.
	for (long i = 0; i <= 2; i++)
	{
		DWORD uAbsRelMode;
		AxmMotGetAbsRelMode(i, &uAbsRelMode);
		if (uAbsRelMode == 0)
			cout << "absolute mode \n";
		else if(uAbsRelMode == 1)
			cout << "relative mode \n";
	}

	// Set Home Location depending on MOT file

	DWORD uHomeResult;
	DWORD uHomeMainStepNumber, uHomeStepNumber;

	for (int i = 0; i <= 2; i++)
	{
		AxmHomeGetResult(i, &uHomeResult);
		if (uHomeResult) //원점 검색 진행 중 : 0, 원점 검색 완료 : 1
		{
			AxmHomeSetStart(i);
			printf("\n원점 검색을 시작한다.\n");
		}
		else if (uHomeResult == HOME_SEARCHING)//원점 검색 진행 중 : 0
		{
			AxmMoveSStop(i);
			printf("\n원점 검색을 종료한다.\n");
		}
	}


	// Real Movement
	// 절대 좌표로 2000만큼 100의 속도와 200의 가속율로 사다리꼴 구동, 구동이 시작되면 함수를 빠져나온다.
	long lAxis[3]; double dPos[3], dVel[3], dAccel[3];
	for (int i = 0; i <= 2; i++) {
		AxmMotSetProfileMode(i, 3);
		AxmMotSetAbsRelMode(i, 1);
		lAxis[i] = i;
		dPos[i] = 500000;
		dVel[i] = 10000;
		dAccel[i] = 10000;
	}

	AxmMoveStartMultiPos(3, lAxis, dPos, dVel, dAccel, dAccel);
	//모션이 모두 종료될 때까지 기다린다.
	BOOL WaitForDone = TRUE;
	DWORD uStatus0, uStatus1, uStatus2;
	while (WaitForDone)
	{
		AxmStatusReadInMotion(AXIS_0, &uStatus0);
		AxmStatusReadInMotion(AXIS_1, &uStatus1);
		AxmStatusReadInMotion(AXIS_2, &uStatus2);

		if(uStatus1 == 0 && uStatus2 == 0 && uStatus2 == 0)
			WaitForDone = FALSE;
	}


	// AxmMovePos(0, 500000, 100, 200, 200);

	// AxmMovePos(1, 500000, 100, 200, 200);

	// AxmMovePos(2, 500000, 100, 200, 200);


	for (long i = 0; i < 3; i++)
	{
		long lAxisNo = i;
		double InitPos, InitVel, InitAccel, InitDecel;
		AxmMotGetParaLoad(lAxisNo, &InitPos, &InitVel, &InitAccel, &InitDecel);
		printf(" % d축 초기 위치(% f), 초기 속도(% f), 초기 가속도(% f), 초기 감속도(% f)\n",
			lAxisNo, InitPos, InitVel, InitAccel, InitDecel);
	}


	// 라이브러리를 종료한다.
	if (AxlClose())
		printf("라이브러리가 종료되었습니다.\n");
	else
		printf("라이브러리가 정상적으로 종료되지 않았습니다.\n");

	return 0;
}
*/