/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"

#include "shared_var.h"

int my_id = 4; //CHANGE THIS TO THE DESIRED ID BEFORE PROGRAMMING THE DEVICE (1-STARTING DEVICE AS INITIATOR, 2,3,4 responders
#define  MAX_ITER 50 //NUMBER OF MEASUREMENTS

#define APP_NAME "SS TWR INIT v1.3"

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 100
/*5,6 destination 7,8 source*/
/* Frames used in the ranging process INITIATOR. See NOTE 1,2 below. */
static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'X', 'I', 'X', 'S',  0, 0, 0, 0, 0, 0, 0, 0};//4 additional bytes to store mean and std from the initiator
static uint8 tx_poll_msg2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', 'X', 'I', 'X', 'F', 0, 0, 0, 0, 0, 0, 0, 0}; //finish message to stop reception from responder
//static uint8 rx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', '1', 'R', 'X', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* Frames used in the ranging process RESPONDER. See NOTE 2,3 below. */
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'I', 'X', 'R', 'X', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//static uint8 rx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', '4', 'I', 'X', 0xE0, 0, 0};
//static uint8 rx_poll_msg2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'R', '4', 'I', 'X', 0xE3, 0, 0}; //last message from initiator
/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

#define SOURCE_IDX 8
#define DEST_IDX 6

#define MEAN_IDX 12
#define STD_IDX 14
/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 25
static uint8 rx_buffer[RX_BUF_LEN];
//responder
#define RX_BUF_LEN2 29
static uint8 rx_buffer2[RX_BUF_LEN2];

// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
#define POLL_RX_TO_RESP_TX_DLY_UUS  1100

/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;

/* Declaration of static functions. */
//static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
//static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);


/*Transactions Counters */
static volatile int tx_count = 0 ; // Successful transmit counter
static volatile int rx_count = 0 ; // Successful receive counter 

//auxiliar variables
int resp_cnt = 0;
int init_cnt = 0;
int last = 0;
bool change = false;
/*byte 0 my_id, byte 1/2 Initiator_ID, byte 3 first responder_id, byte 4/5 first responder'smean, 6/7 first responder's SD*/
/*uint8 measurements_vector[4][4][4] = {
{{  0,    0,    0,    0  },{'M12','M12','S12','S12'},{'M13','M13','S13','S13'},{'M14','M14','S14','S14'}},
{{'M21','M21','S21','S21'},{  0,    0,    0,    0  },{'M23','M23','S23','S23'},{'M24','M24','S24','S24'}},
{{'M31','M31','S31','S31'},{'M32','M32','S32','S32'},{  0,    0,    0,    0  },{'M34','M34','S34','S34'}},
{{'M41','M41','S41','S41'},{'M42','M42','S42','S42'},{'M43','M43','S43','S43'},{  0,    0,    0,    0  }}
};*/
uint8 measurements_vector[4][4][4] = {0};

#define M1_IDX 0
#define S1_IDX 2

int exter_resp_id = 1;
int exter_init_id = 1;

//variables for mean and std calculation
float mean = 0;
uint8 mean_lsb = 0;
uint8 mean_msb = 0;
float total = 0;
float SD = 0;
uint8 SD_lsb = 0;
uint8 SD_msb = 0;

//Measurements vector
float measurements[MAX_ITER];
//responder and initiators ids
int resp_id = 1;

/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ss_init_run(void)
{
  tx_poll_msg[SOURCE_IDX] = my_id;
  tx_poll_msg2[SOURCE_IDX] = my_id;
  /* Loop forever initiating ranging exchanges. */

  if(init_cnt<MAX_ITER)
  {
    if(tx_poll_msg[SOURCE_IDX] == 1)
    {
      if(last==0)
        resp_id = 4;
      if(last==1)
        resp_id = 3;
      if(last==2)
        resp_id = 2;
    }
    else if(tx_poll_msg[SOURCE_IDX] == 2)
    {
      if(last==0)
        resp_id = 1;
      if(last==1)
        resp_id = 4;
      if(last==2)
        resp_id = 3;
    }
    else if(tx_poll_msg[SOURCE_IDX] == 3)
    {
      if(last==0)
        resp_id = 2;
      if(last==1)
        resp_id = 1;
      if(last==2)
        resp_id = 4;
    }
    else if(tx_poll_msg[SOURCE_IDX] == 4)
    {
      if(last==0)
        resp_id = 3;
      if(last==1)
        resp_id = 2;
      if(last==2)
        resp_id = 1;
    }
    tx_poll_msg[DEST_IDX] = resp_id;
    /* Write frame data to DW1000 and prepare transmission. See NOTE 3 below. */
    tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
  }
  else
  {
    last++;
    printf("sending different\r\n");
    mean = total/MAX_ITER;
    mean_lsb = (int)(mean*100) % 256;
    mean_msb = (int)(mean*100 / 256);

    printf("MEAN most significant byte = %d", mean_msb);
    printf("MEAN least significant byte = %d", mean_lsb);

    SD = 0;
    for (int i = 0; i < MAX_ITER; ++i)
    {
        SD += pow(measurements[i] - mean, 2);
    }
    SD = sqrt(SD / MAX_ITER);
    printf("standard deviation = %f", SD);
    SD_lsb = (int)(SD*100) % 256;
    SD_msb = (int)(SD*100 / 256);
    printf("SD most significant byte = %d", SD_msb);
    printf("SD least significant byte = %d", SD_lsb);

    tx_poll_msg2[MEAN_IDX] = mean_msb;
    tx_poll_msg2[MEAN_IDX + 1] = mean_lsb;
    tx_poll_msg2[STD_IDX] = SD_msb;
    tx_poll_msg2[STD_IDX + 1] = SD_lsb;
    tx_poll_msg2[DEST_IDX] = resp_id;
    tx_poll_msg2[ALL_MSG_SN_IDX] = frame_seq_nb;
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
    dwt_writetxdata(sizeof(tx_poll_msg2), tx_poll_msg2, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(sizeof(tx_poll_msg2), 0, 1); /* Zero offset in TX buffer, ranging. */
    
    measurements_vector[my_id - 1][resp_id - 1][M1_IDX]= mean_msb;
    measurements_vector[my_id - 1][resp_id - 1][M1_IDX + 1]= mean_lsb;
    measurements_vector[my_id - 1][resp_id - 1][S1_IDX]= SD_msb;
    measurements_vector[my_id - 1][resp_id - 1][S1_IDX + 1]= SD_lsb;
    for(int i = 0; i < 4; i++)
    {
      for(int j = 0; j < 4; j++)
      {
        for(int k = 0; k < 4; k++)
        {
          printf("%d , ", measurements_vector[i][j][k]);
        }
      }
    }
    init_cnt = 0;
    if(last == 3)
    {
      last = 0;
      change = true;
    }
  }
  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
  * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
  tx_count++;
  printf("Transmission # : %d\r\n",tx_count);


  /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 4 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};

    #if 0  // include if required to help debug timeouts.
    int temp = 0;		
    if(status_reg & SYS_STATUS_RXFCG )
    temp =1;
    else if(status_reg & SYS_STATUS_ALL_RX_TO )
    temp =2;
    if(status_reg & SYS_STATUS_ALL_RX_ERR )
    temp =3;
    #endif

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  if (status_reg & SYS_STATUS_RXFCG)
  {		
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
   
    if (frame_len <= RX_BUF_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    //if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
    if((rx_buffer[SOURCE_IDX]==resp_id)&&(rx_buffer[DEST_IDX] == tx_poll_msg[SOURCE_IDX]) && (rx_buffer[DEST_IDX - 1] == 'I'))
    {	
      rx_count++;
      printf("Reception # : %d\r\n",rx_count);
      printf("Responder n%d\r\n", resp_id);
      uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      int32 rtd_init, rtd_resp;
      float clockOffsetRatio ;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 5 below. */
      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 7 below. */
      clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6) ;

      /* Get timestamps embedded in response message. */
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS; // Specifying 1.0f and 2.0f are floats to clear warning 
      distance = tof * SPEED_OF_LIGHT;
      printf("Distance : %f\r\n",distance);
      measurements[init_cnt] = distance;
      total += measurements[init_cnt];
      init_cnt++;
    }
  }
  else
  {
    /* Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  /* Execute a delay between ranging exchanges. */
  //     deca_sleep(RNG_DELAY_MS);

  //	return(1);
  deca_sleep(RNG_DELAY_MS);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_get_ts(uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}

//**------------------------responder------------****//
int ss_resp_run(void)
{
  tx_resp_msg[SOURCE_IDX] = my_id;
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {};

    #if 0	  // Include to determine the type of timeout if required.
    int temp = 0;
    // (frame wait timeout and preamble detect timeout)
    if(status_reg & SYS_STATUS_RXRFTO )
    temp =1;
    else if(status_reg & SYS_STATUS_RXPTO )
    temp =2;
    #endif
  printf("entering rsp run\r\n");
  if (status_reg & SYS_STATUS_RXFCG)
  {
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN)
    {
      dwt_readrxdata(rx_buffer2, frame_len, 0);
    }

    /* Check that the frame is a poll sent by "SS TWR initiator" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer2[ALL_MSG_SN_IDX] = 0;
    //if (memcmp(rx_buffer2, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
    if((rx_buffer2[DEST_IDX-1]=='R') && ( rx_buffer2[DEST_IDX] == tx_resp_msg[SOURCE_IDX]))
    {
      printf("message received from init\r\n");
      if(rx_buffer2[SOURCE_IDX+1] == 'S')
      {
        printf("standard\n\r");
        uint32 resp_tx_time;
        int ret;

        /* Retrieve poll reception timestamp. */
        poll_rx_ts = get_rx_timestamp_u64();

        /* Compute final message transmission time. See NOTE 7 below. */
        resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
        dwt_setdelayedtrxtime(resp_tx_time);

        /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
        resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        /* Write all timestamps in the final message. See NOTE 8 below. */
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

        /* Write and send the response message. See NOTE 9 below. */
        tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_resp_msg[DEST_IDX] = rx_buffer2[SOURCE_IDX];
        dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0); /* Zero offset in TX buffer. See Note 5 below.*/
        dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1); /* Zero offset in TX buffer, ranging. */
        ret = dwt_starttx(DWT_START_TX_DELAYED);

        //ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

        /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
        if (ret == DWT_SUCCESS)
        {
        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {};

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;
        resp_cnt++;
        }
        else
        {
        /* If we end up in here then we have not succeded in transmitting the packet we sent up.
        POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. 
        For slower platforms where the SPI is at a slower speed or the processor is operating at a lower 
        frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
        Knowing the exact time when the responder is going to send its response is vital for time of flight 
        calculation. The specification of the time of respnse must allow the processor enough time to do its 
        calculations and put the packet in the Tx buffer. So more time is required for a slower system(processor).
        */

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
        }
      }//if standard message
      
    }//IF correct destination
    //if(memcmp(rx_buffer2, rx_poll_msg2, ALL_MSG_COMMON_LEN) == 0)
    if((rx_buffer2[SOURCE_IDX+1] == 'F')&&(rx_buffer2[DEST_IDX-1]=='R'))
    {
      exter_resp_id = rx_buffer2[DEST_IDX];
      exter_init_id = rx_buffer2[SOURCE_IDX];
      measurements_vector[exter_init_id-1][exter_resp_id-1][M1_IDX] = rx_buffer2[MEAN_IDX];
      measurements_vector[exter_init_id-1][exter_resp_id-1][M1_IDX+1] = rx_buffer2[MEAN_IDX+1];
      measurements_vector[exter_init_id-1][exter_resp_id-1][S1_IDX] = rx_buffer2[STD_IDX];
      measurements_vector[exter_init_id-1][exter_resp_id-1][S1_IDX+1] = rx_buffer2[STD_IDX+1];

      for(int i = 0; i < 4; i++)
      {
        //printf("{ ");
        for(int j = 0; j < 4; j++)
        {
          //printf("{ ");
          for(int k = 0; k < 4; k++)
          {
            printf("%d , ", measurements_vector[i][j][k]);
          }
          //printf("} ");
        }
        //printf("}\r\n ");
      }
      if(rx_buffer2[DEST_IDX] == tx_resp_msg[SOURCE_IDX])
      {
        if(((rx_buffer2[SOURCE_IDX] == 1)&&(tx_resp_msg[SOURCE_IDX] == 2))||((rx_buffer2[SOURCE_IDX] == 2)&&(tx_resp_msg[SOURCE_IDX] == 3))||((rx_buffer2[SOURCE_IDX] == 3)&&(tx_resp_msg[SOURCE_IDX] == 4))||((rx_buffer2[SOURCE_IDX] == 4)&&(tx_resp_msg[SOURCE_IDX] == 1)))
        {
          change = 1;
           printf("CHANGE\n\r");
        }
        else 
          printf("END of meassuring\n\r");
      }
    }//else last message
        
      
  }
  else
  {
    /* Clear RX error events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  return(1);		
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
*    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
*    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
*    bytes.
* 5. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 6. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 7. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
