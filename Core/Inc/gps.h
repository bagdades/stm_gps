#ifndef  PARSER_INC
#define  PARSER_INC

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#define 	TIME_BUF_LEN		10
#define 	LATITUDE_BUF_LEN	15
#define 	LONGITUDE_BUF_LEN	15
#define 	SATELITE_BUF_LEN	3
#define 	HEIGHT_BUF_LEN		5

#define 	TRUE					1
#define 	FALSE					0
#define 	CHAR_NEWLINE			'\n'
#define 	CHAR_RETURN				'\r'
#define 	RETURN_NEWLINE			"\r\n"

#define     USART_RX_BUFFER_SIZE    100

struct GPGGA_t
{
	uint8_t time[TIME_BUF_LEN];
	uint8_t latitude[LATITUDE_BUF_LEN];
	uint8_t longitude[LONGITUDE_BUF_LEN];
	uint8_t dir_latit;
	uint8_t dir_longit;
	uint8_t quality;
	uint8_t num_satelites[SATELITE_BUF_LEN];
	uint8_t height[HEIGHT_BUF_LEN];
};

typedef struct GPGGA_t GPGGA_t;

//Prototypes
uint8_t gps_parsing(GPGGA_t *data_gps, uint8_t* buff);

#endif   /* ----- #ifndef PARSER_INC  ----- */
