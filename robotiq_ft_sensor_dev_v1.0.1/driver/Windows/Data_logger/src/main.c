/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/*
 *  \file main.c
 *  \date July 8, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotic.com>
 *  \maintener Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "../../../src/rq_sensor_com.h"
#include "../../../src/rq_sensor_state.h"
#include "../../../src/rq_int.h"
#include <string.h>
#include <stdio.h>
#include <time.h>
#include <math.h>
#include <stdlib.h>
#include <windows.h>
#include "../../../src/Thread/rq_thread.h"
#define REGLE_TEMPS 10 //Au dix secondes

/**
 * \fn static void wait_for_other_connection()
 * \brief Fonction qui attend une connection a une pince
 */
static void wait_for_other_connection(){
	INT_8 ret;
	while(1){
		Sleep(1000);// Attend 1 seconde
		ret = rq_sensor_state();
		if(ret == 0){
			break;
		}
	}
}

/**
 * \fn void get_data(void)
 * \brief Fonction qui permet de recuperer les force appliquer sur le capteur
 * \param chr_return string permettant de retourner les force.
 */
static void get_data(INT_8 * chr_return){
	INT_8 i;
	INT_8 floatData[50];
	for(i = 0; i < 6; i++){
		sprintf(floatData, "%f", rq_state_get_received_data(i));
		if(i == 0){
			strcat(chr_return, floatData);
		}
		else{
			strcat(chr_return," , ");
			strcat(chr_return,floatData);
		}
	}
}

 int main(){
 	//Si on est pas capable d'init on retourne en erreur
	INT_8 ret = rq_sensor_state();
	if(ret == -1){
		wait_for_other_connection();
	}

	//Permet d'aller lire les infos de base du capteur
	ret = rq_sensor_state();
	if(ret == -1){
		wait_for_other_connection();
	}

	//Permet d'aller demarrer le stream
	ret = rq_sensor_state();
	if(ret == -1){
		wait_for_other_connection();
	}
	float nb = 0;
	char input[40];
	while(nb == 0){
		fputs("Please enter the number of data vectors to record during 10 seconds (1-1000): ", stdout);
		fflush(stdout);
		fgets(input, sizeof(input),stdin);
		nb = atoi(input);
		if(nb == 0){
			fputs("Invalid number\n", stdout);
		}
	}

	FILE *fp = NULL;
	while(fp == NULL){
		fputs("Name of the file in which the data will be stored: ", stdout);
		fflush(stdout);
		fgets(input, sizeof(input),stdin);
		if(strcmp(input, "\n") == 0){
			strcpy(input,"Data.csv");
		}
		else if(strstr(input,".csv") == NULL){
			input[strlen(input) -1] = 0;
			strcat(input,".csv");
		}
		else{
			input[strlen(input) -1] = 0;
		}

		fp = fopen(input,"w");
		if(fp == NULL){
			fputs("Cannot open or create the file/Impossible d'ouvrir ou creer le fichier\n", stdout);
		}
	}
	fprintf(fp,"t(s),FX,FY,FZ,MX,MY,MZ");
	INT_8 bufVarHighLvl[30];
	rq_com_get_str_firmware_version(bufVarHighLvl);
	printf("Firmware version: %s\n", bufVarHighLvl);
	rq_com_get_str_serial_number(bufVarHighLvl);
	printf("Serial Number: %s\n", bufVarHighLvl);
	rq_com_get_str_production_year(bufVarHighLvl);
	printf("Production Year: %s\n", bufVarHighLvl);
	printf("t(s),FX,FY,FZ,MX,MY,MZ\n");
	INT_8 bufStream[512];
	clock_t start, diff;
	int timerVerif = 1;
	start = clock();
 	while(1){
 		ret = rq_sensor_state();
 		if(ret == -1){
 			wait_for_other_connection();
 		}
 		if(rq_sensor_get_current_state() == RQ_STATE_RUN){
 			diff = clock() - start;
 			if(floor(((((float)diff)/CLOCKS_PER_SEC)+(REGLE_TEMPS/nb))/(REGLE_TEMPS/nb)) >= timerVerif){
 				fprintf(fp,"\n");
 				fprintf(fp,"%f,",((float)diff)/CLOCKS_PER_SEC);
 				printf("%f, ",((float)diff)/CLOCKS_PER_SEC);
 				strcpy(bufStream,"");
 				get_data(bufStream);
 				printf("%s\n", bufStream);
 				fprintf(fp,bufStream);
 				timerVerif++;
 			}
 		}
 	}
 	return 0;
 }
