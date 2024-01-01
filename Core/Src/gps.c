/*
 * =====================================================================================
 *
 *       Filename:  parser.c
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  01/01/21 10:25:37
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  vovan (), volodumurkoval0@gmail.com
 *   Organization:  
 *
 * =====================================================================================
 */

#include "gps.h"
#include <stdint.h>

/* --------------------------------------------------------------------------*/
/**
* @brief search separator in buffer
*
* @param sep
* @param buf
* @param size_buf
* @param start_pos
*
* @return index after separator
*/
/* ----------------------------------------------------------------------------*/
static uint8_t search_sep(uint8_t sep, uint8_t* buf, uint8_t size_buf, uint8_t start_pos)
{
	uint8_t s_pos = start_pos;
	while(buf[s_pos] != sep && s_pos < size_buf) s_pos++;
	s_pos++;
	return s_pos;
}

/* --------------------------------------------------------------------------*/
/**
* @brief copy patch from input buffer to outbuffer
*
* @param in_buf
* @param in_idx
* @param sep
* @param out_buf
* @param size_outBuf
*
* @return  index of buffer
*/
/* ----------------------------------------------------------------------------*/
static uint8_t copy_buf(uint8_t* in_buf, uint8_t in_idx, uint8_t sep, uint8_t* out_buf, uint8_t size_outBuf)
{
	uint8_t idx = in_idx;
	uint8_t i;
	for (i = 0; i < size_outBuf && in_buf[idx] != sep; i++) {
		/* if(in_buf[idx] == '.') {idx++; i--; continue;} */
		/* out_buf[i] = in_buf[idx]; */
		out_buf[i] = in_buf[idx];
		/* out_buf[i] = in_buf[idx] & 0x0F; */
		idx++;
	}
	return idx;
}

static void change_sep(uint8_t* buf, uint8_t sep, uint8_t buf_size)
{
	uint8_t i;
	for (i = 0; i < buf_size; i++) {
		if(buf[i] == sep)
			buf[i] = ' ';
	}
}

uint8_t gps_parsing(GPGGA_t *data_gps, uint8_t* buff)
{
	uint8_t i = 0;
	uint8_t sep = ',';
	if(strstr((char*)buff, "$GPGGA"))
	{
		memset(data_gps->num_satelites, 0, SATELITE_BUF_LEN);
		memset(data_gps->longitude, 0, LONGITUDE_BUF_LEN);
		memset(data_gps->latitude, 0, LATITUDE_BUF_LEN);
		data_gps->dir_latit = 0;
		data_gps->dir_longit = 0;
		data_gps->quality = 0;
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		i = copy_buf(buff, i, sep, (uint8_t*)&data_gps->time, TIME_BUF_LEN);
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		i = copy_buf(buff, i, sep, (uint8_t*)&data_gps->latitude, LATITUDE_BUF_LEN);
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		if(buff[i] != sep) data_gps->dir_latit = buff[i];
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		i = copy_buf(buff, i, sep, (uint8_t*)&data_gps->longitude, LONGITUDE_BUF_LEN);
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		if(buff[i] != sep) data_gps->dir_longit = buff[i];
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		if(buff[i] != sep) data_gps->quality = buff[i] & 0x0F;
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		i = copy_buf(buff, i, sep, (uint8_t*)&data_gps->num_satelites, SATELITE_BUF_LEN);
		i = search_sep(sep, buff, USART_RX_BUFFER_SIZE, i);
		i = copy_buf(buff, i, sep, (uint8_t*)&data_gps->height, HEIGHT_BUF_LEN);
		change_sep((uint8_t*)buff, ',', USART_RX_BUFFER_SIZE);
        return 1;
	} else return 0;
}

