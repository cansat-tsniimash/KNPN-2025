/*
 * freakheader.h
 *
 *  Created on: 13 окт. 2023 г.
 *      Author: Install
 */

#ifndef FREAKHEADER_H_
#define FREAKHEADER_H_

typedef struct{
	double pressure;
	double bme_pres_grow;
	uint32_t time_steady;
	double temperature;
	double humidity;
	double altitude;
}bme_important_shit_t;

#endif /* FREAKHEADER_H_ */
