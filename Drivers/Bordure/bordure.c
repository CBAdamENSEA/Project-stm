/*
 * bordure.c
 *
 *  Created on: Jan 15, 2023
 *      Author: cheik
 */
#include "bordure.h"


uint8_t init_bords(bords_t * bords)
{
	bords->left.state=0;
	bords->right.state=0;
	bords->detect=0;
	return 1;
}

uint8_t update_bords(bords_t * bords)
{
	if (HAL_GPIO_ReadPin(OPTQ1_GPIO_Port, OPTQ1_Pin)==GPIO_PIN_SET)
		bords->left.state=1;
	else
		bords->left.state=0;
	if(HAL_GPIO_ReadPin(OPTQ2_GPIO_Port, OPTQ2_Pin)==GPIO_PIN_SET)
		bords->right.state=1;
	else
		bords->right.state=0;
	return ((bords->left.state)|(bords->right.state));

}

