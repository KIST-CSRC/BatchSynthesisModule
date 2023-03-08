/*
int main()
{
	// ���̺귯���� �ʱ�ȭ�Ѵ�.
	// 7�� IRQ�� ���Ѵ�. PCI���� �ڵ����� IRQ�� �����ȴ�.
	DWORD Code = AxlOpen(7);
	if (Code == AXT_RT_SUCCESS)
	{
		printf("���̺귯���� �ʱ�ȭ�Ǿ����ϴ�.\n");
		//��� ����� �ִ��� �˻�.
		DWORD uStatus;
		Code = AxmInfoIsMotionModule(&uStatus);
		if (Code == AXT_RT_SUCCESS)
		{
			if (uStatus == STATUS_EXIST)
			{
				printf("��� ����� �����մϴ�.\n");
				//�ý��ۿ� ������ �� ������ Ȯ��.
				long lpAxisCount;
				AxmInfoGetAxisCount(&lpAxisCount);
				printf("�ý��ۿ��������ళ�� : %d \n", lpAxisCount);
				//0�� ���� ���� ��ȣ, ��� ��ġ, ��� ID�� Ȯ���Ѵ�.
				long lBoardNo, lModulePos;
				long lAxisNo = 0;
				DWORD uModuleID;
				AxmInfoGetAxis(lAxisNo, &lBoardNo, &lModulePos, &uModuleID);
				printf("0�� ���� ���� ��ȣ : %x, ��� ��ġ : %x, ��� ID : %x\n",
					lBoardNo, lModulePos, uModuleID);
				//0�� ������ 1�� ��⿡�� ���� �� ��ȣ�� Ȯ���Ѵ�.
				long lFirstAxisNo;
				AxmInfoGetFirstAxisNo(lBoardNo, lModulePos, &lFirstAxisNo);
				printf("%d�� ���̽� ������ %d��° ����� ���� �� ��ȣ : %d\n",
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

	//Save_Para.mot ���Ͽ� ����Ǿ� �ִ� ��� ���� ��� ������ �о�ͼ� �����Ѵ�.
	char pFilename[20] = "configuration.mot";
	AxmMotLoadParaAll(pFilename);

	double AXIS_0_InitPos, AXIS_0_InitVel, AXIS_0_InitAccel, AXIS_0_InitDecel;
	AxmMotGetParaLoad(AXIS_0, &AXIS_0_InitPos, &AXIS_0_InitVel, &AXIS_0_InitAccel, &AXIS_0_InitDecel);
	cout << AXIS_0 << "�� �ʱ� ��ġ(" << AXIS_0_InitPos << "), �ʱ� �ӵ�(" <<
		AXIS_0_InitVel << "), �ʱ� ���ӵ�(" << AXIS_0_InitAccel << "), �ʱ� ���ӵ�(" << AXIS_0_InitDecel << ")\n";

	double AXIS_1_InitPos, AXIS_1_InitVel, AXIS_1_InitAccel, AXIS_1_InitDecel;
	AxmMotGetParaLoad(AXIS_1, &AXIS_1_InitPos, &AXIS_1_InitVel, &AXIS_1_InitAccel, &AXIS_1_InitDecel);
	cout << AXIS_0 << "�� �ʱ� ��ġ(" << AXIS_1_InitPos << "), �ʱ� �ӵ�(" <<
		AXIS_1_InitVel << "), �ʱ� ���ӵ�(" << AXIS_1_InitAccel << "), �ʱ� ���ӵ�(" << AXIS_1_InitDecel << ")\n";

	double AXIS_2_InitPos, AXIS_2_InitVel, AXIS_2_InitAccel, AXIS_2_InitDecel;
	AxmMotGetParaLoad(AXIS_0, &AXIS_2_InitPos, &AXIS_2_InitVel, &AXIS_2_InitAccel, &AXIS_2_InitDecel);
	cout << AXIS_0 << "�� �ʱ� ��ġ(" << AXIS_2_InitPos << "), �ʱ� �ӵ�(" <<
		AXIS_2_InitVel << "), �ʱ� ���ӵ�(" << AXIS_2_InitAccel << "), �ʱ� ���ӵ�(" << AXIS_2_InitDecel << ")\n";

	//���� ����̺갡 On�Ǿ� �ִ����� Ȯ���Ѵ�.
	for(long i=0 ; i<=2 ; i++)
	{
		DWORD uUse = ENABLE;
		AxmSignalServoOn(i, uUse);
		AxmSignalIsServoOn(i, &uUse);
		if (uUse == ENABLE) {
			cout << "���� ����̺갡 ON �Ǿ��ֽ��ϴ�.\n";
		}
		else if (uUse == DISABLE) {
			cout << "���� ����̺갡 OFF �Ǿ��ֽ��ϴ�.\n";
		}
	}

	// ������ �̵� ��带 �д� �Լ�.
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
		if (uHomeResult) //���� �˻� ���� �� : 0, ���� �˻� �Ϸ� : 1
		{
			AxmHomeSetStart(i);
			printf("\n���� �˻��� �����Ѵ�.\n");
		}
		else if (uHomeResult == HOME_SEARCHING)//���� �˻� ���� �� : 0
		{
			AxmMoveSStop(i);
			printf("\n���� �˻��� �����Ѵ�.\n");
		}
	}


	// Real Movement
	// ���� ��ǥ�� 2000��ŭ 100�� �ӵ��� 200�� �������� ��ٸ��� ����, ������ ���۵Ǹ� �Լ��� �������´�.
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
	//����� ��� ����� ������ ��ٸ���.
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
		printf(" % d�� �ʱ� ��ġ(% f), �ʱ� �ӵ�(% f), �ʱ� ���ӵ�(% f), �ʱ� ���ӵ�(% f)\n",
			lAxisNo, InitPos, InitVel, InitAccel, InitDecel);
	}


	// ���̺귯���� �����Ѵ�.
	if (AxlClose())
		printf("���̺귯���� ����Ǿ����ϴ�.\n");
	else
		printf("���̺귯���� ���������� ������� �ʾҽ��ϴ�.\n");

	return 0;
}
*/