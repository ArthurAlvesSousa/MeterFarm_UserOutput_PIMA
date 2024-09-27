/*
* PIMA Protocol
* PREÂMBULO | IDENTIFICADOR | TAMANHO | ESCOPO + ÍNDICE |   DADOS    |    CRC
*  2 bytes  |    5 bytes    |  1 byte |     2 bytes     |  n bytes   |   2 bytes
*
* TAMANHO = ESCOPO + DADOS
*
*/



#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include <math.h>


#define UART_NUM UART_NUM_2
#define TX_PIN GPIO_NUM_18
#define RX_PIN GPIO_NUM_5
#define BUF_SIZE 1024

#define PREAMBLE_SIZE 2
#define IDENTIFIER_SIZE 5
#define SIZE_DATA 1
#define SCOPE 2
#define CRC_SIZE 2

static const char *TAG = "UART_TASK";

void read_PIMA_task(void *pvParameters);

void app_main(void) {
    
    xTaskCreate(read_PIMA_task, "Read PIMA Task", 10240, NULL, 1, NULL);
    vTaskDelay(pdMS_TO_TICKS(1000));
}

uint16_t calculate_crc(uint8_t *data, int length) {
    uint16_t crc = 0x0000; // CRC inicializado com 0
    for (int i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

    uint8_t hex_to_bcd_u8(uint8_t bcd) { 
        return ((bcd & 0xF0)  >> 4 ) * 10 + (bcd & 0x0F); 
    }
/*
   uint8_t bcd_to_hex_u8(uint8_t bcd) { 
        return bcd - 0x30; 
    }

    uint16_t bcd_to_hex_u8(uint16_t bcd) {
        uint16_t hex = bcd_to_hex_u8((bcd & 0xFF00) >> 8) * 10;
        hex += bcd_to_hex_u8((bcd & 0x00FF));
    }
*/

void read_PIMA_task(void *pvParameters) {
    uint8_t data[BUF_SIZE];
    int length = 0;
    int state = 0;
    int data_length = 0;
    int index = 0;

    uint8_t preamble[PREAMBLE_SIZE] = {0xAA, 0x55};
    //uint8_t identifier[IDENTIFIER_SIZE];
    uint8_t size_byte = 0;
    //uint8_t *frame_data = NULL;

    uart_config_t uart_config = {
        .baud_rate = 2400,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    // Configure UART parameters
    uart_param_config(UART_NUM, &uart_config);
    uart_set_pin(UART_NUM, TX_PIN, RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM, BUF_SIZE, 0, 0, NULL, 0);
    //
    //If the ABNT UO reading is by the self-powered output, uncomment this line.
    //Otherwise, if it is an open collector output, comment on the line below
    ESP_ERROR_CHECK(uart_set_line_inverse(UART_NUM, UART_SIGNAL_TXD_INV  | UART_SIGNAL_RXD_INV));// Inverts the TX and RX sign
    vTaskDelay(pdMS_TO_TICKS(1000));
    uart_flush(UART_NUM);
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint8_t Scope_Index[2];
    //uint8_t *DataReceived = (uint8_t*) malloc();
    int i=0;
    uint64_t Data_Converted = 0;
    uint64_t Data_Converted_Integer_Part=0;
    uint8_t Data_Converted_Decimal_Part=0;

 
    while (1) {
        bool data_available = true;
        while (data_available) {
            uint8_t byte;
            int len = uart_read_bytes(UART_NUM, &byte, 1, pdMS_TO_TICKS(10));
            //printf("%x\n", byte);
            if (len > 0) {
                switch (state) {
                    case 0: // Looking for the first preamble byte
                        if (byte == preamble[0]) {
                            data[index++] = byte;
                            state = 1;
                        }
                        break;

                    case 1: // Looking for the second preamble byte
                        if (byte == preamble[1]) {
                            data[index++] = byte;
                            state = 2;
                        } else {
                            index = 0; // Reset and start over
                            state = 0;
                        }
                        break;

                    case 2: // Reading the identifier
                        data[index++] = byte;
                        if (index == PREAMBLE_SIZE + IDENTIFIER_SIZE) {
                            state = 3;
                        }
                        break;

                    case 3: // Reading the size byte
                        data[index++] = byte;
                        size_byte = byte;
                        data_length = size_byte - 2; // Subtracting Scope Size
                        state = 4;
                        break;

                    case 4: // Reading the Scope
                        
                        data[index++] = byte;
                        Scope_Index[i] = data[index - 2];

                        if (index == PREAMBLE_SIZE + IDENTIFIER_SIZE + SIZE_DATA + SCOPE) {
                            state = 5;
                            i=0;
                        }
                        break;

                    case 5: // Reading the frame data
                        data[index++] = byte;
                        if (index == PREAMBLE_SIZE + IDENTIFIER_SIZE + SIZE_DATA + SCOPE + data_length) {
                            state = 6;
                        }
                        break;

                    case 6: // Reading the CRC bytes
                        data[index++] = byte;
                        if (index == PREAMBLE_SIZE + IDENTIFIER_SIZE + SIZE_DATA + SCOPE + data_length + CRC_SIZE) {
                            // uint16_t calculated_crc = calculate_crc(data, index - CRC_SIZE);
                            // uint16_t received_crc = (data[index - 2] << 8) | data[index - 1];

                            uint16_t calculated_crc = calculate_crc(data + PREAMBLE_SIZE, index - PREAMBLE_SIZE - CRC_SIZE);
                            uint16_t received_crc = (data[index - 1] << 8) | data[index - 2];

                            if (calculated_crc == received_crc) {
                                ESP_LOGI(TAG, "Frame received successfully:");
                                state = 7;
                            } else {
                                ESP_LOGE(TAG, "CRC mismatch: calculated 0x%04X, received 0x%04X", calculated_crc, received_crc);
                                // Reset state for the next frame
                                index = 0;
                                state = 0;
                            }

                            for (int i = 0; i < index; i++) {
                                printf("%02X ", data[i]);
                            }
                            printf("\n");
                        }
                        break;
                    case 7:
                        Scope_Index[0] = data[8];
                        Scope_Index[1] = data[9];
                        //Scope_Index[i++] = byte;
                        uint8_t DataSize = data[7]-2;

                        
                        printf("scope: %02X|%02X\n", Scope_Index[0], Scope_Index[1]);

                        
                        if(Scope_Index[0] == 0x0B && Scope_Index[1] == 0x01){
                           //(first << 24) | (second << 16) | third

                            //Dat aReceived[0] = data[10] + data[11];
                            //printf("Dado recebido: %02X\n", DataReceived[0]);
                            //printf("Dado recebido: %02X\n", DataReceived[0], DataReceived[1]);
                            
                           
                            for(int i = 0; i<DataSize; i++){
                                Data_Converted+= hex_to_bcd_u8 (data[10+i]) * (pow(10, (DataSize-1-i)*2));
                                //printf("Raul: %02X\n" , bcd_to_hex_u9 (data[10+i]) );

                                

                            }

                            //printf("Tensão na fase A: %lld V\n", Data_Converted);
                            Data_Converted_Integer_Part = Data_Converted/pow(10, 1);
                            Data_Converted_Decimal_Part = Data_Converted - Data_Converted_Integer_Part*pow(10,1);
                            printf("Tensão na fase A: %lld.%d V\n", Data_Converted_Integer_Part, Data_Converted_Decimal_Part);
                            Data_Converted = 0;
                            for(int i=10; i<(10+data_length-2);i++){
                                printf(data[i]);
                            }
                            

                            
                        } 

                        index = 0;
                        state = 0;
                        //DataReceived = 0;
                        break;
                }
            } else {
                data_available = false;
            }
        }
        ESP_LOGI(TAG, "Waiting for bytes...");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}