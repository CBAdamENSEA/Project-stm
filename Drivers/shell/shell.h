/*
 * shell.h
 *
 *  Created on: 7 juin 2019
 *      Author: laurent
 */

#ifndef INC_LIB_SHELL_SHELL_H_
#define INC_LIB_SHELL_SHELL_H_

#include <stdint.h>
#include "cmsis_os.h"


#define UART_DEVICE huart2 //huart3 stlink
#define ARGC_MAX 8
#define BUFFER_SIZE 40
#define SHELL_FUNC_LIST_MAX_SIZE 64


struct h_shell_t;
struct h_shell;

typedef uint8_t (* drv_shell_transmit_t)(char * pData, uint16_t size);// pointeur sur fonction
typedef uint8_t (* drv_shell_receive_t)(char * pData, uint16_t size);


typedef struct drv_shell_struct
{
	drv_shell_transmit_t drv_shell_transmit;
	drv_shell_receive_t drv_shell_receive;
} drv_shell_t;

typedef struct{
	char c;
	int (* func)(struct h_shell* h_shell,int argc, char ** argv);
	char * description;
} shell_func_t;

typedef struct{

	int shell_func_list_size ;
	shell_func_t shell_func_list[SHELL_FUNC_LIST_MAX_SIZE];
	char print_buffer[BUFFER_SIZE];
	SemaphoreHandle_t sem_uart_read ;
	drv_shell_t drv_shell_struct ;
} h_shell_t;



void shell_init();
int shell_add(h_shell_t * h_shell,char c, int (* pfunc)(int argc, char ** argv), char * description);
int shell_run(h_shell_t * h_shell);
// commentaire documentation!
void shell_uart_receive_irq_cb(h_shell_t * h_shell);

#endif /* INC_LIB_SHELL_SHELL_H_ */
