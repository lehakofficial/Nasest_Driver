/*
���� �������� ��������� ��� ������� DC/DC
���� �������� - 06.06.2023
*/
//��������� ��� ������ ������� ������� ���
#define	PWM_TO_POWER_SCALE		635			/*���������� ���������� �������� �������� ������� � ��������*/
#define	PWM_TO_VOLTAG_SCALE		1			/*���������� ���������� �������� �������� ������� � ���������� - �������� 0,00329 � �� ��*/
#define	PWM_TO_CURRENT_SCALE		1			/*���������� ���������� �������� �������� ������� � ���*/
#define PWM_IN_MIN	699			/*����������� ������� �������� ���*/
#define PWM_IN_MAX	5699		/*������������ ������� �������� ���*/
#define PWM_ZERO		0				/*�������� ������� ��� ����� �� �����������. �������� � ��������� ��������� �������� �����������*/
#define PWM_IN_CALIBRATION_MIN	300			/*����������� ������� �������� ��� ��� ������ ����������*/
#define PWM_IN_CALIBRATION_MAX	600			/*������������ ������� �������� ��� ��� ������ ����������*/
#define PWM_IN_MAX_POWER_MIN	5899			/*����������� ������� �������� ��� ��� ������ ������������ ��������*/
#define PWM_IN_MAX_POWER_MAX	6199			/*������������ ������� �������� ��� ��� ������ ������������ ��������*/
#define PWM_IN_CONTROL_PERIOD_POWER_MIN	6299			/*����������� ������� �������� ������� ��� ��� ������ �������� 10 ���*/
#define PWM_IN_CONTROL_PERIOD_POWER_MAX	6499			/*������������ ������� �������� ������� ��� ��� ������ �������� 10 ���*/
#define PWM_IN_CONTROL_PERIOD_VOLTAG_MIN	7899			/*����������� ������� �������� ������� ��� ��� ������ ���������� 8 ���*/
#define PWM_IN_CONTROL_PERIOD_VOLTAG_MAX	8099			/*������������ ������� �������� ������� ��� ��� ������ ���������� 8 ���*/
#define PWM_IN_CONTROL_PERIOD_CURRENT_MIN	7010			/*����������� ������� �������� ������� ��� ��� ������ ���� 9 ���*/
#define PWM_IN_CONTROL_PERIOD_CURRENT_MAX	7210			/*������������ ������� �������� ������� ��� ��� ������ ���� 9 ���*/
#define PWM_IN_COUNT_CONTROL_MAX	10			/*������������ �������� ������������ �������� ����� ���*/

//������ ���
#define COUNT_ADC_CALIBRATE_START	12500	/*���������� �������� �� ������ �������� ���������� ���, ����� ����������� �������. �������� 0,5 �������*/
#define COUNT_ADC_CALIBRATE		28884			/*28884 - 12500 = 16384 (2^14) ������� ��� ��� ���������� �������� �� ������ 25 ��� �������� 0,5 �������*/

/*��� �����������*/
#define PWR_LIMIT		400					/*������������ ��� ��� �������*/
#define PWR_MIN			22					/*�������� ��� ����� �� �������� ��������*/
#define V_INTEGR_MAX 18677760		/*��������� 18677760 = 50 (������������ ��� ������� � ��� ��� �����������) * 5,7 (���������� �����������) * 65536 (������� ����� ����������� 16 ��������)*/
#define I_INTEGR_MAX 12451840		/*��������� 18677760 = 50 (������������ ��� ������� � ��� ��� �����������) * 3.8 (���������� �����������) * 65536 (������� ����� ����������� 16 ��������)*/
#define MAX_V_PILTIE 2430	/*������������ �������� ��� ������� ��� ������� ���������� ���������� �� 8,0�*/