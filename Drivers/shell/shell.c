/*
 * shell.c
 *
 *  Created on: 7 juin 2019
 *      Author: Laurent Fiack
 */

#include "shell.h"

#include "cmsis_os.h"

#include <stdio.h>

#include "usart.h"
#include "gpio.h"

extern UART_HandleTypeDef huart2; //huart3 stlink





void shell_uart_receive_irq_cb(h_shell_t *h_shell)
{
	BaseType_t pxHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(h_shell->sem_uart_read, &pxHigherPriorityTaskWoken);

	portYIELD_FROM_ISR(pxHigherPriorityTaskWoken);
}

static char uart_read(h_shell_t *h_shell) {
	char c;

	//HAL_UART_Receive_IT(&huart3, (uint8_t*)(&c), 1);//stlink
	HAL_UART_Receive_IT(&huart2, (uint8_t*)(&c), 1);
	// il faut mettre la tâche shell dans l'état bloqué, jusqu'à l'interruption de réception de caractère
	// prendre un sémaphore vide par exemple
	xSemaphoreTake(h_shell->sem_uart_read, portMAX_DELAY);

	return c;
}

static int uart_write(char * s, uint16_t size) {
	//HAL_UART_Transmit(&huart3, (uint8_t*)s, size, HAL_MAX_DELAY); stlink
	HAL_UART_Transmit(&huart2, (uint8_t*)s, size, HAL_MAX_DELAY);
	return size;
}

static int sh_help(h_shell_t *h_shell,int argc, char ** argv) {
	int i;
	for(i = 0 ; i <h_shell->shell_func_list_size ; i++) {
		int size;
		size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: %s\r\n", h_shell->shell_func_list[i].c,h_shell-> shell_func_list[i].description);
		uart_write(h_shell->print_buffer, size);
	}

	return 0;
}

void shell_init(h_shell_t *h_shell) {
	int size = 0;

	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n\r\n== Démonstration Robot CARY ==\r\n");
	uart_write(h_shell->print_buffer, size);

	h_shell->sem_uart_read = xSemaphoreCreateBinary();
	if (h_shell->sem_uart_read == NULL)
	{
		printf("Error semaphore shell\r\n");
		while(1);
	}

	shell_add(h_shell,'h', sh_help, "Help");
}

int shell_add(h_shell_t *h_shell,char c, int (* pfunc)(int argc, char ** argv), char * description) {
	if (h_shell->shell_func_list_size < SHELL_FUNC_LIST_MAX_SIZE) {
		h_shell->shell_func_list[h_shell->shell_func_list_size].c = c;
		h_shell->shell_func_list[h_shell->shell_func_list_size].func = pfunc;
		h_shell->shell_func_list[h_shell->shell_func_list_size].description = description;
		h_shell->shell_func_list_size++;
		return 0;
	}

	return -1;
}

static int shell_exec(char * buf,h_shell_t *h_shell) {
	int i;

	char c = buf[0];

	int argc;
	char * argv[ARGC_MAX];
	char *p;

	for(i = 0 ; i < h_shell->shell_func_list_size ; i++) {
		if (h_shell->shell_func_list[i].c == c) {
			argc = 1;
			argv[0] = buf;

			for(p = buf ; *p != '\0' && argc < ARGC_MAX ; p++){
				if(*p == ' ') {
					*p = '\0';
					argv[argc++] = p+1;
				}
			}

			return h_shell->shell_func_list[i].func(h_shell, argc, argv);
		}
	}

	int size;
	size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "%c: no such command\r\n", c);
	uart_write(h_shell->print_buffer, size);
	return -1;
}

static char backspace[] = "\b \b";
static char prompt[] = "> ";

int shell_run(h_shell_t *h_shell) {
	int reading = 0;
	int pos = 0;

	static char cmd_buffer[BUFFER_SIZE];

	while (1) {
		uart_write(prompt, 2);
		reading = 1;

		while(reading) {
			char c = uart_read(h_shell);
			int size;

			switch (c) {
			//process RETURN key
			case '\r':
				//case '\n':
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, "\r\n");
				uart_write(h_shell->print_buffer, size);
				cmd_buffer[pos++] = 0;     //add \0 char at end of string
				size = snprintf (h_shell->print_buffer, BUFFER_SIZE, ":%s\r\n", cmd_buffer);
				uart_write(h_shell->print_buffer, size);
				reading = 0;        //exit read loop
				pos = 0;            //reset buffer
				break;
				//backspace
			case '\b':
				if (pos > 0) {      //is there a char to delete?
					pos--;          //remove it in buffer

					uart_write(backspace, 3);	// delete the char on the terminal
				}
				break;
				//other characters
			default:
				//only store characters if buffer has space
				if (pos < BUFFER_SIZE) {
					uart_write(&c, 1);
					cmd_buffer[pos++] = c; //store
				}
			}
		}
		shell_exec(cmd_buffer,h_shell);
	}
	return 0;
}
