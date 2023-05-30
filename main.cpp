#include "serial.h"
#include "bla21.h"

int main(int argc, char** argv){
	int  portno;
	char port[256];

	printf("input COM port number: ");
	scanf("%d", &portno);
	sprintf(port, "\\\\.\\COM%d", portno);

	// open serial port
	Serial serial;
	if(serial.Open(port)){
		printf("serial port opened\n");
	}
	else{
		printf("failed to open serial port");
		return -1;
	}

	printf(
		"\n"
		"Very simple Futaba BLA21 communication tool\n"
		"Accepted commands:  \n"
		" read               read returned CAN frame and show status\n"
		" enable             enable torque of speficied servo\n"
		" disable            disable torque of specified servo\n"
		" setposition        set goal position of a single servo (uses GetSet)\n"
		" setpositionforall  set goal position of multiple servos (uses ArrayCommand)\n"
		" verbose            toggle verbose option\n"
		" quit               exit program\n"
		"\n");

	BLA21 servo;
	servo.OpenChannel(&serial);

	while(true){
		printf(">>");
		
		char cmd[256];
		scanf("%s", cmd);
		
		if(strcmp(cmd, "quit") == 0){
			break;
		}
		if(strcmp(cmd, "verbose") == 0){
			servo.verbose = !servo.verbose;
		}
		if(strcmp(cmd, "sethostid") == 0){
			printf("input CAN id of host (1-127, avoid conflict with CAN ids of servos): ");
			scanf("%d", servo.hostId);
		}
		if(strcmp(cmd, "read") == 0){
			servo.Read(&serial);
		}
		if(strcmp(cmd, "enable") == 0){
			printf(" id: ");
			
			int id;
			scanf("%d", &id);
			
			servo.EnableTorque(&serial, id, true);
		}
		if(strcmp(cmd, "disable") == 0){
			printf(" id: ");
			
			int id;
			scanf("%d", &id);
			
			servo.EnableTorque(&serial, id, false);
		}
		if(strcmp(cmd, "setposition") == 0){
			int   id;
			float pos;

			printf(" id: ");
			scanf("%d", &id);
			printf(" goal position [deg]: ");
			scanf("%f", &pos);
			
			servo.SetGoalPosition(&serial, id, pos);
		}
		if(strcmp(cmd, "setpositionforall") == 0){
			int num;
			vector<int> idArray;
			vector<float> posArray;

			printf(" num: ");
			scanf("%d", &num);
			idArray.resize(num);
			posArray.resize(num);

			for(int i = 0; i < num; i++){
				printf(" id[%d]: ", i);
				scanf("%d", &idArray[i]);
				printf(" goal position[%d] [deg]: ", i);
				scanf("%f", &posArray[i]);
			}

			servo.SetGoalPositionForAll(&serial, idArray, posArray);
		}
	}
	
	servo.CloseChannel(&serial);
	serial.Close();
	
	return 0;
}

