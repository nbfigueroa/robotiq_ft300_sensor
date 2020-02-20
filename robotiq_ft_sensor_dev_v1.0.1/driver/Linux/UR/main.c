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
 *  \date June 18, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotic.com>
 *  \maintener Nicolas Lauzier <nicolas@robotiq.com>
 */

#include "../../src/rq_sensor_com.h"
#include "../../src/rq_sensor_state.h"
#include "../src/rq_sensor_socket.h"
#include "../../src/rq_int.h"
#include <string.h>
#include <stdio.h>
#include "../../src/Thread/rq_thread.h"

/**
 * \fn static void decode_message_and_do(SOCKET csock, char *buff)
 * \brief Decode the message received and execute the operation. 
 * \param csock, client's SOCKET
 * \param *buff, message to decode and execute
 */
static void decode_message_and_do(SOCKET csock, UINT_8 *buff){
	if(buff == NULL || csock == 0 || strlen(buff) < 7){
		return;
	}
	UINT_8 get_or_set[3];
	strncpy(get_or_set, &buff[0], 3);
	if(strstr(get_or_set, "GET")){
		UINT_8 nom_get_var[4];
		strncpy(nom_get_var, &buff[4], strlen(buff) -3);
		UINT_8 buffer[512];
		rq_state_get_command(nom_get_var, buffer);
		write_client(csock, buffer);
	}
	else if (strstr(buff, "SET ZRO")){
		rq_set_zero();
	}
	set_socket_message_to_null();
}

/**
 * \fn static void wait_for_other_connection()
 * \brief Fonction qui attend une connection a une pince
 */
static void wait_for_other_connection(){
	INT_8 ret;
	while(1){
		usleep(1000000);//Attend 1 seconde.
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
			strcpy(chr_return, "( ");
			strcat(chr_return, floatData);
		}
		else{
			strcat(chr_return," , ");
			strcat(chr_return,floatData);
		}
		if(i == 5){
			strcat(chr_return, " )");
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
	int8_t buffer[512];
	int8_t bufStream[512];
	SOCKET sock = 0;
	while (sock == 0){
		sock = init_connection();
	}

	SOCKET sockStream = get_socket_stream();
	SOCKET csockAcc = get_client_sock_accessor();
	SOCKET csockStream = get_client_sock_stream();

	ret = create_Thread(Read_socket);
	UINT_8 cpt = 0;
	while(ret == -1 && cpt < 5){
		cpt++;
		ret = create_Thread(Read_socket);
	}
	if (ret == -1){
		printf("Thread: Impossible de creer un thread\n");
		return -1;
	}
 	while(1){
 		strcpy(buffer,"");
		socket_message(buffer);
		csockAcc = get_client_sock_accessor();
		if(strcmp(buffer, "") != 0)
		{
			decode_message_and_do(csockAcc, buffer);
		}
 		ret = rq_sensor_state();
 		if(ret == -1){
 			wait_for_other_connection();
 		}
 		if(rq_sensor_get_current_state() == RQ_STATE_RUN){
			csockStream = get_client_sock_stream();
 			strcpy(bufStream,"");
 			get_data(bufStream);
			write_client(csockStream,bufStream);
 		}
 	}
 	end_connection(sock);
 	end_connection(sockStream);
 	return 0;
 }
