/* 
** Program Name: rfPacketRx
** Author: Antonio Washington
** Description: This program acts as a reciever for the sensor data through RF.
** The program then transmits the data serially utilizing UART. This will then
** be processed by ARM processor or PC
*/
/***** Includes *****/
/* Standard C Libraries */
#include <stdlib.h>
#include <stddef.h>

/* POSIX Header files */
#include <unistd.h>

/* TI Drivers */
#include <ti/drivers/rf/RF.h>
#include <ti/drivers/PIN.h>
#include <ti/display/Display.h>
#include <ti/drivers/UART.h>

/* Driverlib Header files */
#include DeviceFamily_constructPath(driverlib/rf_prop_mailbox.h)

/* Board Header files */
#include "Board.h"

/* Application Header files */
#include "RFQueue.h"
#include "smartrf_settings/smartrf_settings.h"


/***** Defines *****/

/* Packet RX Configuration */
#define DATA_ENTRY_HEADER_SIZE 8  /* Constant header size of a Generic Data Entry */
#define MAX_LENGTH             30 /* Max length byte the radio will accept */
#define NUM_DATA_ENTRIES       2  /* NOTE: Only two data entries supported at the moment */
#define NUM_APPENDED_BYTES     2  /* The Data Entries data field will contain:
                                   * 1 Header byte (RF_cmdPropRx.rxConf.bIncludeHdr = 0x1)
                                   * Max 30 payload bytes
                                   * 1 status byte (RF_cmdPropRx.rxConf.bAppendStatus = 0x1) */


#define READ_8B  7
#define READ_16B 16

/***** Prototypes *****/
static void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e);

/***** Variable declarations *****/
static RF_Object rfObject;
static RF_Handle rfHandle;
static Display_Handle display;
static UART_Handle uart;
static UART_Params uartParams;

/* Pin driver handle */
static PIN_Handle ledPinHandle;
static PIN_State ledPinState;


/* Buffer which contains all Data Entries for receiving data.
 * Pragmas are needed to make sure this buffer is 4 byte aligned (requirement from the RF Core) */
#if defined(__TI_COMPILER_VERSION__)
#pragma DATA_ALIGN (rxDataEntryBuffer, 4);
static uint8_t 
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__IAR_SYSTEMS_ICC__)
#pragma data_alignment = 4
static uint8_t 
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH,
                                                  NUM_APPENDED_BYTES)];
#elif defined(__GNUC__)
static uint8_t 
rxDataEntryBuffer[RF_QUEUE_DATA_ENTRY_BUFFER_SIZE(NUM_DATA_ENTRIES,
                                                  MAX_LENGTH, 
                                                  NUM_APPENDED_BYTES)] 
                                                  __attribute__((aligned(4)));
#else
#error This compiler is not supported.
#endif

/* Receive dataQueue for RF Core to fill in data */
static dataQueue_t dataQueue;
static rfc_dataEntryGeneral_t* currentDataEntry;
static uint8_t packetLength;
static uint8_t* packetDataPointer;


static uint8_t packet[MAX_LENGTH + NUM_APPENDED_BYTES - 1]; /* The length byte is stored in a separate variable */

/*
 * Application LED pin configuration table:
 *   - All LEDs board LEDs are off.
 */
PIN_Config pinTable[] =
{
    Board_PIN_LED2 | PIN_GPIO_OUTPUT_EN | PIN_GPIO_LOW | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#if defined Board_CC1352R1_LAUNCHXL
    Board_DIO30_RFSW | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MAX,
#endif
	PIN_TERMINATE
};
/******* 72554 bytes of SRAM Available ********/
/***** 335958 bytes of Flash Available *****/

/***** Function definitions *****/

void *mainThread(void *arg0)
{
    RF_Params rfParams;
    RF_Params_init(&rfParams);
    UART_init();

    /* Open LED pins */
    ledPinHandle = PIN_open(&ledPinState, pinTable);
    if (ledPinHandle == NULL)
    {
        while(1);
    }
	
    /* Initialize UART For Serial Transmission */
    UART_Params_init(&uartParams);
    uartParams.writeDataMode = UART_DATA_TEXT;//UART_DATA_BINARY;
    uartParams.readDataMode = UART_DATA_TEXT;//UART_DATA_BINARY;
    uartParams.readReturnMode = UART_RETURN_FULL;
    uartParams.readEcho = UART_ECHO_OFF;
    uartParams.baudRate = 115200;
    uart = UART_open(Board_UART0, &uartParams);

    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
	
    /* Define Queue for data entries of RF Transmission */
    if( RFQueue_defineQueue(&dataQueue,
                            rxDataEntryBuffer,
                            sizeof(rxDataEntryBuffer),
                            NUM_DATA_ENTRIES,
                            MAX_LENGTH + NUM_APPENDED_BYTES))
    {
        /* Failed to allocate space for all data entries */
        while(1);
    }
    //Display_print0(display, 0, 0, "Test 2 BLAH...\n\n");

    /* Modify CMD_PROP_RX command for application needs */
    /* Set the Data Entity queue for received data */
    RF_cmdPropRx.pQueue = &dataQueue;           
    /* Discard ignored packets from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushIgnored = 1;  
    /* Discard packets with CRC error from Rx queue */
    RF_cmdPropRx.rxConf.bAutoFlushCrcErr = 1;   
    /* Implement packet length filtering to avoid PROP_ERROR_RXBUF */
    RF_cmdPropRx.maxPktLen = MAX_LENGTH;        
    RF_cmdPropRx.pktConf.bRepeatOk = 1;
    RF_cmdPropRx.pktConf.bRepeatNok = 1;

    /* Request access to the radio */
    rfHandle = RF_open(&rfObject, &RF_prop, 
                       (RF_RadioSetup*)&RF_cmdPropRadioDivSetup, &rfParams);

    /* Set the frequency */
    RF_postCmd(rfHandle, (RF_Op*)&RF_cmdFs, RF_PriorityNormal, NULL, 0);

    /* Enter RX mode and stay forever in RX */
    RF_EventMask terminationReason = RF_runCmd(rfHandle, (RF_Op*)&RF_cmdPropRx,
                                               RF_PriorityNormal, &callback,
                                               RF_EventRxEntryDone);


    switch(terminationReason)
    {
        case RF_EventLastCmdDone:
            // A stand-alone radio operation command or the last radio
            // operation command in a chain finished.
            //Display_print0(display, 0, 0, "Test 4...\n\n");
            break;
        case RF_EventCmdCancelled:
            // Command cancelled before it was started; it can be caused
            // by RF_cancelCmd() or RF_flushCmd().
//            Display_print0(display, 0, 0, "Command Cancel...\n\n");

            break;
        case RF_EventCmdAborted:
            // Abrupt command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
  //          Display_print0(display, 0, 0, "Test 5...\n\n");
            break;
        case RF_EventCmdStopped:
            // Graceful command termination caused by RF_cancelCmd() or
            // RF_flushCmd().
    //        Display_print0(display, 0, 0, "Test 6...\n\n");
            break;
        default:
            // Uncaught error event
      //      Display_print0(display, 0, 0, "Uncaught Error...\n\n");
            while(1);
    }
    //Display_print0(display, 0, 0, "Test 3 - After Term reason...\n\n");

    uint32_t cmdStatus = ((volatile RF_Op*)&RF_cmdPropRx)->status;
    switch(cmdStatus)
    {
        case PROP_DONE_OK:
            // Packet received with CRC OK
            break;
        case PROP_DONE_RXERR:
            // Packet received with CRC error
      //      Display_print0(display, 0, 0, "Receive Error...\n\n");

            break;
        case PROP_DONE_RXTIMEOUT:
            // Observed end trigger while in sync search
        //    Display_print0(display, 0, 0, "Timeout...\n\n");

            break;
        case PROP_DONE_BREAK:
            // Observed end trigger while receiving packet when the command is
            // configured with endType set to 1
            break;
        case PROP_DONE_ENDED:
            // Received packet after having observed the end trigger; if the
            // command is configured with endType set to 0, the end trigger
            // will not terminate an ongoing reception
            break;
        case PROP_DONE_STOPPED:
            // received CMD_STOP after command started and, if sync found,
            // packet is received
            break;
        case PROP_DONE_ABORT:
            // Received CMD_ABORT after command started
            break;
        case PROP_ERROR_RXBUF:
            // No RX buffer large enough for the received data available at
            // the start of a packet
            break;
        case PROP_ERROR_RXFULL:
            // Out of RX buffer space during reception in a partial read
            break;
        case PROP_ERROR_PAR:
            // Observed illegal parameter
            break;
        case PROP_ERROR_NO_SETUP:
            // Command sent without setting up the radio in a supported
            // mode using CMD_PROP_RADIO_SETUP or CMD_RADIO_SETUP
            break;
        case PROP_ERROR_NO_FS:
            // Command sent without the synthesizer being programmed
            break;
        case PROP_ERROR_RXOVF:
            // RX overflow observed during operation
            break;
        default:
            // Uncaught error event - these could come from the
            // pool of states defined in rf_mailbox.h
            while(1);
    }

    while(1);
}
/*
** Callback function to fire whenever data is founf through RF transmission.
** Read data from RF signal then output the signal serially
*/
void callback(RF_Handle h, RF_CmdHandle ch, RF_EventMask e)
{
    if (e & RF_EventRxEntryDone)
    {
        /* Toggle pin to indicate RX */
        PIN_setOutputValue(ledPinHandle, Board_PIN_LED2,
                           !PIN_getOutputValue(Board_PIN_LED2));

        /* Get current unhandled data entry */
        currentDataEntry = RFQueue_getDataEntry();

        /* Handle the packet data, located at &currentDataEntry->data:
         * - Length is the first byte with the current configuration
         * - Data starts from the second byte */
        packetLength      = *(uint8_t*)(&currentDataEntry->data);
        packetDataPointer = (uint8_t*)(&currentDataEntry->data + 1);
        /* Copy the payload + the status byte to the packet variable */
        memcpy(packet, packetDataPointer, (packetLength + 1));
        /* 16 bit counter implementation */
        uint8_t packs[17+6] = {0x13,0x40,0x40,packet[1],packet[0]};
        /* Serial Transmission */

        // Single Sensor Pack Reference: packs[] = {0x13,0x40,0x40,packet[0],packet[1],0x00,packet[2],packet[3],'\n'}

        uint8_t i = 0;
        uint8_t offset = 5; // 5 for 2 byte counter and 4 for 1 byte counter

        for (i=0;i < READ_16B;i++){
            packs[offset++]=packet[2+i]; //offset for 5 initial bytes in pack array + packet offset of 1 for the counter
        }
        // Add Endline Character
       // packs[offset++]="\n";
        packs[offset++]='\n';
        UART_write(uart,packs,sizeof(packs));
        RFQueue_nextEntry();
    }
}
