/**
  ******************************************************************************
  * @file    bridge.h
  * @author  GPM-AppliTools-HWBoards Team
  * @brief   Header for bridge.cpp module
  ******************************************************************************
  * @attention
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/** @addtogroup BRIDGE
 * @{
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _BRIDGE_H
#define _BRIDGE_H
/* Includes ------------------------------------------------------------------*/
#include "stlink_device.h"
#include "stlink_fw_const_bridge.h"


/* Exported types and constants ----------------------------------------------*/
/** @addtogroup GENERAL
 * @{
 */
/// Bridge Error and Status
typedef enum {
	BRG_NO_ERR = 0,           ///< OK (no error)
	BRG_CONNECT_ERR,          ///< USB Connection error
	BRG_DLL_ERR,              ///< USB DLL error
	BRG_USB_COMM_ERR,         ///< USB Communication error
	BRG_NO_DEVICE,            ///< No Bridge device target found error
	BRG_OLD_FIRMWARE_WARNING, ///< Warning: current bridge firmware is not the last one available
	BRG_TARGET_CMD_ERR,       ///< Target communication or command error
	BRG_PARAM_ERR,            ///< Wrong parameters error
	BRG_CMD_NOT_SUPPORTED,    ///< Firmware command not supported by the current firmware version
	BRG_GET_INFO_ERR,         ///< Error getting STLink Bridge device information
	BRG_STLINK_SN_NOT_FOUND,  ///< Required STLink serial number not found error
	BRG_NO_STLINK,            ///< STLink Bridge device not opened error
	BRG_NOT_SUPPORTED,        ///< Parameter error
	BRG_PERMISSION_ERR,       ///< STLink Bridge device already in use by another program error
	BRG_ENUM_ERR,             ///< USB enumeration error
	BRG_COM_FREQ_MODIFIED,    ///< Warning: required frequency is not exactely the one applied
	BRG_COM_FREQ_NOT_SUPPORTED,///< Required frequency cannot be applied error
	BRG_SPI_ERR,              ///< SPI communication error
	BRG_I2C_ERR,              ///< I2C communication error
	BRG_CAN_ERR,              ///< CAN communication error
	BRG_TARGET_CMD_TIMEOUT,   ///< Timeout error during Bridge communication
	BRG_COM_INIT_NOT_DONE,    ///< Bridge Init function not called error
	BRG_COM_CMD_ORDER_ERR,    ///< Sequencial Bridge function order error
	BRG_BL_NACK_ERR,          ///< Bootloader NACK error
	BRG_VERIF_ERR,            ///< Data verification error
	BRG_MEM_ALLOC_ERR,        ///< Memory allocation error
	BRG_GPIO_ERR,             ///< GPIO communication error
	BRG_OVERRUN_ERR,          ///< Overrun error during bridge communication
	BRG_CMD_BUSY,             ///< Command busy: only Brg::GetLastReadWriteStatus() command allowed in that case
	BRG_CLOSE_ERR,            ///< Error during device Close
	BRG_INTERFACE_ERR,        ///< Unknown default error returned by STLinkInterface
	BRG_CMD_NOT_ALLOWED       ///< Command not allowed in current bridge state (e.g.: CAN/FDCAN exclusive)
} Brg_StatusT;

#define COM_SPI STLINK_SPI_COM   ///< 0x2 SPI Bridge communication parameter
#define COM_I2C STLINK_I2C_COM   ///< 0x3 I2C Bridge communication parameter
#define COM_CAN STLINK_CAN_COM   ///< 0x4 CAN Bridge communication parameter
#define COM_FDCAN STLINK_FDCAN_COM ///< 0x5 FDCAN Bridge communication parameter
#define COM_GPIO STLINK_GPIO_COM ///< 0x6 GPIO Bridge communication parameter
#define COM_UNDEF_ALL 0xFF       ///< 0xFF All or Undefined Bridge communication parameter

#define DEFAULT_CMD_TIMEOUT 0  ///< 0x0 Parameter to use default firmware timeout
// end group doxygen GENERAL
/** @} */
// -------------------------------- SPI ------------------------------------ //
/** @addtogroup SPI
 * @{
 */
/// Add or not a SPI delay between bytes (or words)
typedef enum {
	DEFAULT_NO_DELAY = 0, ///< Default case: no delay is inserted
	DELAY_FEW_MICROSEC = 1  ///< At least 4us delay is inserted
} Brg_DelayT;

// SPI Init parameters, see InitSPI() for the currently supported SPI mode
/// SPI Init lines connection configuration :  \n
///MASTER_SCK-SLAVE_SCK   ,   MASTER_NSS-SLAVE_NSS (if used)   ,   MASTER_GND-SLAVE_GND   \n Plus the connection of MISO and MOSI as follows:
///(see also \ref LIMITATION)
typedef enum {
	SPI_DIRECTION_2LINES_FULLDUPLEX = 0, ///< Fullduplex: MASTER_MISO-SLAVE_MISO , MASTER_MOSI-SLAVE_MOSI.
	SPI_DIRECTION_2LINES_RXONLY = 1, ///< Simplex read only: MASTER_MISO-SLAVE_MISO (master MOSI unused).
	SPI_DIRECTION_1LINE_RX = 2, ///< Halfduplex read only: bidirectional MASTER_MOSI used in receive-only mode (to receive from SLAVE_MISO),This mode is not functional if voltage adaptation is present in the used STLink hardware (e.g. with B-STLINK-VOLT or B-STLINK-ISOL). Because in this case the STLink MASTER_MOSI is always an output.
	SPI_DIRECTION_1LINE_TX = 3  ///< Halfduplex transmit only: bidirectional MASTER_MOSI used in transmit-only mode (to send to SLAVE_MOSI).
} Brg_SpiDirT;
/// SPI Init: mode configuration see also \ref LIMITATION
typedef enum {
	SPI_MODE_SLAVE = 0, ///< SPI Slave (Target) mode: not supported.
	SPI_MODE_MASTER = 1 ///< SPI Master (Controller) mode
} Brg_SpiModeT;
/// SPI Init: data size configuration
typedef enum {
	SPI_DATASIZE_16B = 0, ///< 16 Bit data word
	SPI_DATASIZE_8B = 1 ///< 8 Bit data byte
} Brg_SpiDataSizeT;
/// SPI Init: SCK polarity configuration (see also #Brg_SpiCphaT)
typedef enum {
	SPI_CPOL_LOW = 0, ///< SCK low polarity
	SPI_CPOL_HIGH = 1 ///< SCK High polarity
} Brg_SpiCpolT;
/// SPI Init: SCK phase configuration (see also #Brg_SpiCpolT)
typedef enum {
	SPI_CPHA_1EDGE = 0, ///< SCK 1st edge: data captured on 1st CLK edge (rising if CPOL low or falling if CPOL high),
						///<                data output on 2nd edge (falling if CPOL low)
	SPI_CPHA_2EDGE = 1  ///< SCK 2nd edge: data captured on 2nd CLK edge, data output on 3rd edge
} Brg_SpiCphaT;
/// SPI Init: data bit transmission order configuration
typedef enum {
	SPI_FIRSTBIT_LSB = 0, ///< LSB is output first
	SPI_FIRSTBIT_MSB = 1 ///< MSB is output first
} Brg_SpiFirstBitT;
/// SPI Init: frame format configuration see also \ref LIMITATION
typedef enum {
	SPI_FRF_MOTOROLA = 0, ///< Motorola format (default)
	SPI_FRF_TI = 1 ///< TI format, CPHA, CPOL, NSS are forced to TI mode (whatever their value)
} Brg_SpiFrfT;
/// SPI Init: NSS (Slave Select) management
typedef enum {
	SPI_NSS_SOFT = 0, ///< NSS manage by software (see Brg::SetSPIpinCS())
	SPI_NSS_HARD = 1 ///< NSS manage by hardware
} Brg_SpiNssT;
/// SPI Init: NSS configuration for SPI_NSS_HARD case in master mode
typedef enum {
	SPI_NSS_NO_PULSE = 0, ///< SPI NSS not pulsed between each data
	SPI_NSS_PULSE = 1 ///< SPI NSS pulse generated between 2 data
} Brg_SpiNssPulseT;
/// Brg::SetSPIpinCS() parameter: SPI NSS level for #SPI_NSS_SOFT case in master mode
typedef enum {
	SPI_NSS_LOW = 0, ///< Set SPI NSS low
	SPI_NSS_HIGH = 1 ///< Set SPI NSS high
} Brg_SpiNssLevelT;
/// SPI Init: baudrate, SPI IP input CLK prescaler, use Brg::GetSPIbaudratePrescal()
/// to get the correct prescaler according to required SCK frequency.
typedef enum {
	SPI_BAUDRATEPRESCALER_2 = 0,  ///< Prescaler = 2
	SPI_BAUDRATEPRESCALER_4 = 1,  ///< Prescaler = 4
	SPI_BAUDRATEPRESCALER_8 = 2,  ///< Prescaler = 8
	SPI_BAUDRATEPRESCALER_16 = 3, ///< Prescaler = 16
	SPI_BAUDRATEPRESCALER_32 = 4, ///< Prescaler = 32
	SPI_BAUDRATEPRESCALER_64 = 5, ///< Prescaler = 64
	SPI_BAUDRATEPRESCALER_128 = 6,///< Prescaler = 128
	SPI_BAUDRATEPRESCALER_256 = 7 ///< Prescaler = 256
} Brg_SpiBaudrateT;
/// SPI Init: CRC configuration see also \ref LIMITATION
typedef enum {
	SPI_CRC_DISABLE = 0, ///< CRC disabled (default)
	SPI_CRC_ENABLE = 1 ///< CRC enabled
} Brg_SpiCrcT;
/// SPI Init parameter structure, see also \ref LIMITATION
typedef struct {
	Brg_SpiDirT Direction;      ///< Data Line direction
	Brg_SpiModeT Mode;          ///< SPI mode
	Brg_SpiDataSizeT DataSize;  ///< Data size
	Brg_SpiCpolT Cpol;          ///< SCK Polarity
	Brg_SpiCphaT Cpha;          ///< SCK Phase
	Brg_SpiFirstBitT FirstBit;  ///< Data bit transmission order
	Brg_SpiFrfT FrameFormat;    ///< Frame format
	Brg_SpiNssT Nss;            ///< NSS configuration
	Brg_SpiNssPulseT NssPulse;  ///< NSS hardware pulse configuration
	Brg_SpiBaudrateT Baudrate;  ///< Prescaler for SCK
	Brg_SpiCrcT Crc;            ///< CRC configuration
	uint16_t CrcPoly;           ///< 0 or if #SPI_CRC_ENABLE: Odd value only
	Brg_DelayT SpiDelay;
	///< SpiDelay to be used to insert a delay of several microseconds between bytes (or words)
	///< when calling Read/Write functions.\n
	///< Note for smaller delay: NSS mode hardware pulse insert one SPI CLK cycle (when NSS is pulsed)\n
	///< Note for larger delay: call N time Read/Write function transmitting 1 byte instead of 1 time
	///<                        Read/write function transmitting N bytes (~200us delay)
} Brg_SpiInitT;
// end group doxygen SPI
/** @} */
// -------------------------------- I2C ------------------------------------ //
/** @addtogroup I2C
 * @{
 */
// I2C Init parameters
/// I2C Init: enable or disable filter configuration
typedef enum {
	I2C_FILTER_DISABLE = 0, ///< Filter disabled
	I2C_FILTER_ENABLE = 1   ///< Filter enabled
} Brg_I2cFilterT;

/// AddressingMode : for slave address for master mode and Own address for slave mode, see also \ref LIMITATION
typedef enum {
	I2C_ADDR_7BIT = 0, ///< 7 bit addressing
	I2C_ADDR_10BIT = 1 ///< 10 bit addressing
} Brg_I2cAddrModeT;
/// I2C Init parameter structure, see also \ref LIMITATION
typedef struct {
	uint32_t TimingReg;        ///< Call Brg::GetI2cTiming() to get TimingReg parameter according to required
						       ///< I2C speed and time configuration parameters.
	uint16_t  OwnAddr;         ///< OwnAddress1 <= 0X3FF (for slave mode)
	Brg_I2cAddrModeT AddrMode; ///< Address mode (for slave mode)
	Brg_I2cFilterT AnFilterEn; ///< Enable or disable Analog filter
	Brg_I2cFilterT DigitalFilterEn; ///< Enable or disable Digital filter
	uint8_t Dnf;               ///< Digital filter configuration: DNF (<=15) or 0 if digital filter OFF
} Brg_I2cInitT;
/// I2C Init: I2C mode parameter for Brg::GetI2cTiming()
typedef enum {
	I2C_STANDARD,   ///< I2C standard mode, 1-100KHz
	I2C_FAST,       ///< I2C fast mode, 1-400KHz
	I2C_FAST_PLUS   ///< I2C fast mode plus, 1-1000KHz
} I2cModeT;

#define I2C_10B_ADDR_FLAG  0x8000   ///< I2C 10bit addressing flag
#define I2C_10B_ADDR(_addr) (_addr|I2C_10B_ADDR_FLAG) ///< I2C 10bit addressing macro
#define I2C_7B_ADDR(_addr) (_addr)  ///< I2C 7bit addressing macro

/// I2C repeated start preparation: I2C PrepRestart parameter for Brg::StartWriteI2C(), Brg::ContWriteI2C(),
/// Brg::StartReadI2C(), Brg::ContReadI2C(), needed for repeated start management. \n\n
/// See \ref LIMITATION for I2C repeated start feature support by STLink firmwares.\n
/// I2C repeated start is supported in low-level routines thanks to #I2cTransPrepT and #I2cTransForceT.
/// #I2cTransPrepT is used, in Start or Cont routines, to prepare the generation of the repeated start
/// for next routine. #I2cTransForceT may be needed to force the restart generation.\n
/// Take care to use these parameters in the correct order to perform a valid sequence.
/// Start, Cont, Stop routines called with invalid sequences/parameters order abort the I2C transfer.\n
/// After having used #I2C_PREP_RESTART, it is mandatory to generate a RESTART in next call.\n
/// #I2C_FORCE_RESTART cannot be used if #I2C_PREP_RESTART is not used in previous call.\n
/// Using a Read after a Write (or vice versa), requires #I2C_PREP_RESTART to be used in previous call.\n
///
/// To perform repeated start sequence using the low-level I2C API, follow the following rules:
/// - Use #I2C_PREP_RESTART in the function call preceding the RESTART.\n
/// - Use #I2C_FORCE_RESTART or change the direction in the function call starting with a RESTART.\n
/// Write-Read repeated start examples, use either: \n
/// - Brg::StartWriteI2C(#I2C_PREP_RESTART) then Brg::StopReadI2C(#I2C_FORCE_RESTART)\n
/// - Brg::StartWriteI2C(#I2C_PREP_NONE) then Brg::ContWriteI2C(#I2C_PREP_RESTART)
///   then Brg::ContReadI2C(#I2C_FORCE_RESTART) then Brg::StopReadI2C(#I2C_FORCE_NONE)\n
/// - Brg::StartWriteI2C(#I2C_PREP_RESTART) then Brg::StopReadI2C(#I2C_FORCE_NONE),
///  #I2C_FORCE_RESTART is not mandatory because of direction change.\n
/// - Brg::StartWriteI2C(#I2C_PREP_NONE) then Brg::ContWriteI2C(#I2C_PREP_RESTART)
///   then Brg::ContReadI2C(#I2C_FORCE_NONE) then Brg::StopReadI2C(#I2C_FORCE_NONE)
///  #I2C_FORCE_RESTART is not mandatory because of direction change.\n
/// Read-Read repeated start examples, use either: \n
/// - Brg::StartReadI2C(#I2C_PREP_RESTART) then Brg::StopReadI2C(#I2C_FORCE_RESTART)\n
/// - Brg::StartReadI2C(#I2C_PREP_NONE) then Brg::ContReadI2C(#I2C_PREP_RESTART)
///   then Brg::ContReadI2C(#I2C_FORCE_RESTART) then Brg::StopReadI2C(#I2C_FORCE_NONE)\n
typedef enum {
	I2C_PREP_NONE = 0,   ///< (default) Next partial I2C transfer must be in the same direction without repeated start.\n
	                     ///< Example: after current Brg::StartWriteI2C() (or Brg::ContWriteI2C()),
						 ///< next call must be Brg::StopWriteI2C() (or Brg::ContWriteI2C()) with #I2C_FORCE_NONE.
	I2C_PREP_RESTART = 1 ///< Prepare the repeated start of the next transaction.\n
	                     ///< Next partial I2C transfer after this transaction must trigger a repeated start (RESTART + slave address):
	                     ///< either by forcing a restart with #I2C_FORCE_RESTART, either by changing of direction (e.g.: Read after Write).\n
	                     ///< Read after Write example: after current Brg::StartWriteI2C() (or Brg::ContWriteI2C()),
	                     ///< next call must be Brg::StopReadI2C() (or Brg::ContReadI2C()).\n
						 ///< Read after read example: after current Brg::StartReadI2C() (or Brg::ContReadI2C()),
	                     ///< next call to Brg::StopReadI2C() (or Brg::ContReadI2C()) must use #I2C_FORCE_RESTART.
} I2cTransPrepT;

/// I2C transfer force option: I2C ForceRestart parameter for Brg::StopWriteI2C(), Brg::ContWriteI2C(),
/// Brg::StopReadI2C(), Brg::ContReadI2C(), needed for repeated start management, see #I2cTransPrepT.
typedef enum {
	I2C_FORCE_NONE = 0,    ///< (default) No restart forced in current partial I2C transfer.
	I2C_FORCE_RESTART = 1  ///< Force a restart to be generated. Previous I2C transaction must have set #I2C_PREP_RESTART.
}I2cTransForceT;

// Private low level struct for partial transfer management
typedef enum {
	I2C_FULL_RW_TRANS = I2C_TRANS_TYPE_FULL, // 0, Default do full I2C transaction: START-slave ADDR-DATA-STOP
	// Partial I2C transaction: user needs to call Start, Cont (optional) and Stop in this order
	I2C_START_RW_TRANS = I2C_TRANS_TYPE_START, // 1, Do partial I2C transaction: START-slave ADDR-DATA
	I2C_CONT_RW_TRANS = I2C_TRANS_TYPE_CONT, // 2, Do partial I2C transaction: DATA
	I2C_STOP_RW_TRANS = I2C_TRANS_TYPE_STOP, // 3, Do partial I2C transaction: DATA-STOP
	// Full I2C transaction with write few bytes step before reading data using repeated start between write and read.
	I2C_FULL_RW_TRANS_WRITE_READ = I2C_TRANS_TYPE_FULL_WRITE_READ // 4, Do full Write-Read I2C transaction:
	                                                              // START-slave ADDR-DATAWRITE-RESTART-slave-DATAREAD-STOP
} Brg_I2cRWTransfer; // Used in case I2C transfer is split over several RW cmd
// end group doxygen I2C
/** @} */
// -------------------------------- CAN ------------------------------------ //
/** @addtogroup CAN
 * @{
 */
// group doxygen FDCAN (common CAN/FDCAN)
 /** @addtogroup FDCAN
  * @{
  */
/// CAN and FDCAN Init: Initialization type 
typedef enum {
	BRG_INIT_FULL, ///< Default: complete IP initialization (Filter reseted)
	BRG_REINIT,    ///< Reinitialization (only modify configuration parameters)
} Brg_InitTypeT;

/// CAN and FDCAN Init: CAN Bit Time configuration used in Brg::GetCANbaudratePrescal() and InitCAN()/InitFDCAN()
/// BS1 =PROP_SEG + PHASE_SEG1  BS1 max is 256_time_quantum
typedef struct {
	uint8_t PropSegInTq;   ///< PROP_SEG, CAN: 1 to 8 time quantum,\n FDCAN Nominal BitTime: 1 to max 256 time quantum for PropSegInTq + PhaseSeg1InTq,\n
	                       ///<                                       FDCAN: Data BitTime: 1 to max 32 time quantum for PropSegInTq + PhaseSeg1InTq,
	uint8_t PhaseSeg1InTq; ///< PHASE_SEG1, CAN: 1 to 8 time quantum,\n FDCAN Nominal BitTime: 1 to max 256 time quantum for PropSegInTq + PhaseSeg1InTq,\n
	                       ///<                                       FDCAN: Data BitTime: 1 to max 32 time quantum for PropSegInTq + PhaseSeg1InTq,
	uint8_t PhaseSeg2InTq; ///< PHASE_SEG2, CAN: 1 to 8 time quantum,\n FDCAN Nominal BitTime: 1 to max 128 time quantum,\n
	                       ///<                                       FDCAN: Data BitTime: 1 to max 16 time quantum,
	uint8_t SjwInTq;       ///< SJW, CAN: 1 to 4 time quantum (may be less than PhaseSeg1InTq and PhaseSeg2InTq),\n 
						   ///<      FDCAN Nominal BitTime: 1 to max 128 time quantum, (must be less than PHASE_SEG2)\n
						   ///<      FDCAN: Data BitTime: 1 to max 16 time quantum,(must be less than PHASE_SEG2)
} Brg_CanBitTimeConfT;
// end group doxygen FDCAN (common CAN/FDCAN)
/** @} */

/// CAN Init: possible CAN mode parameter for Brg::InitCAN(), see also \ref LIMITATION .\n
/// #CAN_MODE_NORMAL is the mode to be used by default.\n\n
///
///	In Loop Back Mode, the CAN Bridge treats its own transmitted messages as received
///	messages and stores them (if they pass acceptance filtering) in a Receive mailbox.
///	This mode is provided for self-test functions. To be independent of external events, the CAN
///	Core ignores acknowledge errors (no dominant bit sampled in the acknowledge slot of a
///	data / remote frame) in Loop Back Mode. In this mode, the CAN performs an internal
///	feedback from its Tx output to its Rx input. The actual value of the CANRX input pin is
///	disregarded by the bxCAN. The transmitted messages can be monitored on the CANTX pin.\n\n
///
///	In Silent mode, the CAN Bridge is able to receive valid data frames and valid remote frames, but
///	it sends only recessive bits on the CAN bus and it cannot start a transmission. If the CAN
///	has to send a dominant bit (ACK bit, overload flag, active error flag), the bit is rerouted
///	internally so that the CAN Core monitors this dominant bit, although the CAN bus may
///	remain in recessive state. Silent mode can be used to analyze the traffic on a CAN bus
///	without affecting it by the transmission of dominant bits (Acknowledge Bits, Error Frames).
typedef enum {
	CAN_MODE_NORMAL = 0,         ///< Normal mode
	CAN_MODE_LOOPBACK = 1,       ///< Loopback mode
	CAN_MODE_SILENT = 2,         ///< Silent mode
	CAN_MODE_SILENT_LOOPBACK = 3 ///< Silent Loopback mode
} Brg_CanModeT;

/// CAN init parameters for InitCAN(), see also \ref LIMITATION
typedef struct {
	Brg_CanBitTimeConfT BitTimeConf; ///< Bit time management
	Brg_CanModeT Mode;               ///< CAN mode (normal, loopback ...)
	uint32_t Prescaler;              ///< 1 to 1024 (use Brg::GetCANbaudratePrescal())
	bool bIsAbomEn;                  ///< Enable or disable automatic bus-off management
	bool bIsAwumEn;                  ///< Enable or disable automatic wake-up mode
	bool bIsNartEn;                  ///< Enable or disable no-automatic retransmission mode
	bool bIsRflmEn;                  ///< Enable or disable Receive FIFO Locked mode
	bool bIsTxfpEn;                  ///< Enable or disable transmit FIFO priority
} Brg_CanInitT;


// group doxygen FDCAN (common CAN/FDCAN)
 /** @addtogroup FDCAN
  * @{
  */
 /// CAN or FDCAN message format types
typedef enum {
	CAN_ID_STANDARD = 0, ///< Standard CAN or FDCAN message
	CAN_ID_EXTENDED = 1  ///< Extended CAN or FDCAN message
} Brg_CanMsgIdT;
/// CAN or FDCAN message data types, see also \ref LIMITATION
typedef enum {
	CAN_DATA_FRAME = 0, ///< Data (default)
	CAN_REMOTE_FRAME = 1 ///< Remote data
} Brg_CanMsgRtrT;
/// CAN or FDCAN receive fifo
typedef enum {
	CAN_MSG_RX_FIFO0 = 0, ///< Received CAN message in FIFO0
	CAN_MSG_RX_FIFO1 = 1  ///< Received CAN message in FIFO1
} Brg_CanRxFifoT;
/// CAN or FDCAN Rx message overrun status
typedef enum {
	CAN_RX_NO_OVERRUN = 0,   ///< No error
	CAN_RX_FIFO_OVERRUN = 1, ///< STLink CAN HW fifo overrun
	CAN_RX_BUFF_OVERRUN = 2  ///< STLink CAN Rx buffer overrun
} Brg_CanRxOverrunT;
// end group doxygen FDCAN (common CAN/FDCAN)
/** @} */

/// Structure for Rx CAN messages (except data field), see also \ref LIMITATION
typedef struct {
	Brg_CanMsgIdT IDE;  ///< Specifies if ID is standard (11-bit) or extended (29-bit) identifier.
	uint32_t ID;        ///< Identifier of the message (11bit or 29bit according to IDE).
	Brg_CanMsgRtrT RTR; ///< Remote Frame Request or data frame message type.
	uint8_t DLC;        ///< Data Length Code is the number of data bytes in the received message
	                    ///< or number of data bytes requested by RTR.
	Brg_CanRxFifoT Fifo; ///< Fifo in which the message was received (according to Filter initialization done
	                     ///< with Brg::InitFilterCAN(): AssignedFifo in #Brg_CanFilterConfT)
	Brg_CanRxOverrunT Overrun; ///< Indicate if overrun has occurred before this message.
	uint16_t TimeStamp;  ///< unused
} Brg_CanRxMsgT;

/// Structure for Tx CAN messages (except data field), see also \ref LIMITATION
typedef struct {
	Brg_CanMsgIdT IDE; ///< Specifies if ID is standard (11-bit) or extended (29-bit) identifier.
	uint32_t ID;       ///< Identifier of the message (11bit or 29bit according to IDE).
	                   ///< DLC: max 8. Data Length Code of requested data bytes when sending RTR .
	Brg_CanMsgRtrT RTR; ///< RTR: Specifies if Remote Transmission Request should be sent (DLC is used for
	                    ///< number of requested bytes), otherwise the data message will be sent.
	uint8_t DLC;        ///< Data Length Code, max 8. Number of requested data for RTR, unused for data Frame.
	                    ///< (for data frame Size parameter of Brg::WriteMsgCAN() is used as DLC)
} Brg_CanTxMsgT;

/// Filter mode \n
/// In mask mode the identifier is associated with mask to specify which
/// bits of the identifier are handled as "must match" or as "don't care".
/// i.e.: message is accepted if ID_msg & MASK_filter = ID_filter.\n
/// In identifier list mode, instead of defining an identifier and a mask,
/// two identifiers are specified, doubling the number of single identifiers.
/// All bits of the incoming identifier must match the bits specified in the filter.
/// i.e.: message is accepted if ID_msg = ID_filter.
typedef enum {
	CAN_FILTER_ID_MASK = 0, ///< Mask mode
	CAN_FILTER_ID_LIST = 1  ///< Identifier list mode
}Brg_CanFilterModeT;

/// Filter scale \n
/// To optimize and adapt the filters to the application needs, each filter bank
/// can be scaled independently. Depending on the filter scale a filter bank provides:\n
/// One 32-bit filter for the STDID[10:0], EXTID[17:0], IDE and RTR bits.\n
/// Two 16-bit filters for the STDID[10:0], RTR, IDE and EXTID[17:15] bits.
typedef enum {
	CAN_FILTER_16BIT = 0, ///< 16 bit filter
	CAN_FILTER_32BIT = 1  ///< 32 bit filter
}Brg_CanFilterScaleT;

/// Filter identifier or mask
typedef struct {
	Brg_CanMsgRtrT RTR; ///< Data type
	Brg_CanMsgIdT IDE;  ///< ID Format type
	uint32_t ID;        ///< Standard (max 0x7FF) or extended ID (max 0x1FFFFFFF)
}Brg_FilterBitsT;

/// Structure used to configure a given CAN filter with Brg::InitFilterCAN().\n
/// For 16-bit filters and extended ID, only ID[28:26,10:0] bits are used
/// in #Brg_FilterBitsT fields (ID[25:11] are unused).
/// According to FilterMode and FilterScale some #Brg_FilterBitsT fields
/// are used or not.\n See table below (x = used, - = unused)\n
///     <table>
///     <tr><th>Scale              <th>Mode    <th>Id[0] <th>Mask[0] <th>Id[1] <th>Mask[1] <th>Id[2] <th>Id[3]
///     <tr><td rowspan="2">32bit  <td>ID_MASK <td>x     <td>x       <td>-     <td>-       <td>-     <td>-
///     <tr>                       <td>ID_LIST <td>x     <td>-       <td>x     <td>-       <td>-     <td>-
///     <tr><td rowspan="2">16bit  <td>ID_MASK <td>x     <td>x       <td>x     <td>x       <td>-     <td>-
///     <tr>                       <td>ID_LIST <td>x     <td>-       <td>x     <td>-       <td>x     <td>x
///     </table>
// -------------------------------------------------------------------
// | Scale |  Mode   | Id[0] |Mask[0]| Id[1] |Mask[1]| Id[2] | Id[3] |
// -------------------------------------------------------------------
// |       | ID_MASK |   x   |   x   |   -   |   -   |   -   |   -   |
// | 32bit |----------------------------------------------------------
// |       | ID_LIST |   x   |   -   |   x   |   -   |   -   |   -   |
// -------------------------------------------------------------------
// |       | ID_MASK |   x   |   x   |   x   |   x   |   -   |   -   |
// | 16bit |----------------------------------------------------------
// |       | ID_LIST |   x   |   -   |   x   |   -   |   x   |   x   |
// -------------------------------------------------------------------
typedef struct {
	uint8_t FilterBankNb;    ///< Filter bank to configure: 0 to 13
	bool bIsFilterEn;        ///< FilterActivation: Enable or disable the filter
	Brg_CanFilterModeT FilterMode;   ///< ID_LIST or ID_MASK
	Brg_CanFilterScaleT FilterScale; ///< 16bit or 32bit
	Brg_FilterBitsT Id[4];   ///< Identifiers, Id[0] used in all cases,
	                         ///< Id[1] used only if 16bit (LIST or MASK) or 32bit and ID_LIST,
	                         ///< Id[2] used only if 16bit and ID_LIST,
	                         ///< Id[3] used only if 16bit and ID_LIST.
	Brg_FilterBitsT Mask[2]; ///< Masks, Mask[0] used only if ID_MASK (16bit or 32bit)
	                         ///< Mask[1] used only if 16bit and ID_MASK.
	Brg_CanRxFifoT AssignedFifo;     ///< Rx FIFO in which message is received
}Brg_CanFilterConfT;
// end group doxygen CAN
/** @} */

// ------------------------------- FCAN ------------------------------------ //
/** @addtogroup FDCAN
 * @{
 */
 
/// FDCAN Init: possible FDCAN mode parameter for Brg::InitFDCAN(), see also \ref LIMITATION .\n
/// #FDCAN_MODE_NORMAL is the mode to be used by default.\n\n
///
/// #FDCAN_MODE_BUS_MONITORING: In this mode, the FDCAN is able to receive valid data frames,
/// but cannot start a transmission. The bus monitoring mode can be used to analyze the traffic 
/// on a CAN bus without affecting it by the transmission of dominant bits.\n
///	#FDCAN_MODE_INT_LOOPBACK: This mode is provided for self-test functions, meaning the STLINK FDCAN
/// can be tested without affecting a running CAN system connected to the CAN_TX and
/// CAN_RX pins. In this mode, CAN_RX pin is disconnected from the FDCAN and
/// CAN_TX pin is held recessive.The FDCAN Bridge treats its own transmitted messages as received
///	messages and stores them (if they pass acceptance filtering) in a Receive mailbox.\n
/// #FDCAN_MODE_EXT_LOOPBACK: This mode is provided for hardware self-test. To be independent from external stimulation,
/// the FDCAN ignores acknowledge errors in loop back mode. In this mode the FDCAN performs an internal
/// feedback from its transmit output to its receive input. The actual value of the STLINK CAN_RX
/// input pin is disregarded by the FDCAN. The transmitted messages can be monitored at the
/// STLINK CAN_TX transmit pin.\n
/// #FDCAN_MODE_RESTRICTED: In this mode the node is able to receive data and remote frames and to give
/// acknowledge to valid frames, but it does not send data frames, remote frames, active error
/// frames, or overload frames.The restricted operation mode can be used in applications that adapt
/// themselves to different CAN bitrates. In this case the application tests different bitrates 
/// and leaves the restricted operation mode after it has received a valid frame. \n
///
typedef enum {
	FDCAN_MODE_NORMAL = 0,         ///< Normal mode (default)
	FDCAN_MODE_RESTRICTED = 1,     ///< Restricted Operation mode
	FDCAN_MODE_BUS_MONITORING = 2, ///< Bus Monitoring mode
	FDCAN_MODE_INT_LOOPBACK = 3,   ///< Internal Loopback mode
	FDCAN_MODE_EXT_LOOPBACK = 4    ///< External Loopback mode
} Brg_FdcanModeT;

/// FDCAN Init: possible FDCAN Frame parameter for Brg::InitFDCAN().
/// Use #FDCAN_FRAME_FD_BRS to send and receive all kinds of FD or classic CAN frame format.
/// Choice of the accepted/sent messages is individual per Tx messages or Rx filters.
/// Use other modes to limit the possible frame format for all Rx/Tx messages.
typedef enum {
	FDCAN_FRAME_CLASSIC = 0,     ///< Classic CAN to connect to a classic CAN bus only (reject FD format)
	FDCAN_FRAME_FD_NO_BRS = 1,   ///< FDCAN with no Bit Rate Switching, to connect to a FDCAN or CAN bus and ignore FDCAN BRS setting
	FDCAN_FRAME_FD_BRS = 2       ///< FDCAN with Bit Rate Switching (default to be used), to connect to a FDCAN or CAN bus without restriction.
} Brg_FdcanFrameModeT;

/// FDCAN Init: possible FDCAN fifo operating mode for Fifo0Mode and Fifo1Mode parameter for Brg::InitFDCAN().\n
typedef enum {
	FDCAN_FIFO_BLOCKING = 0,   ///< Fifo blocking mode when fifo is full new messages are discared.
	FDCAN_FIFO_OVERWRITE = 1   ///< Fifo overwrite mode when fifo is full the next message accepted for the FIFO will overwrite the oldest FIFO message.
} Brg_FdcanFifoModeT;

/// FDCAN init parameters for InitFDCAN(), see also \ref LIMITATION
typedef struct {
	Brg_CanBitTimeConfT NomBitTimeConf;  ///< Nominal bit time management
	Brg_CanBitTimeConfT DataBitTimeConf; ///< Data Bit time management for Data phase bitrate in #FDCAN_FRAME_FD_BRS (no impact on Data phase bitrate in classic CAN or FD CAN no BRS).
	uint32_t NomPrescaler;           ///< 1 to 512 (use Brg::GetFDCANbaudratePrescal())
	uint32_t DataPrescaler;          ///< 1 to 32 (use Brg::GetFDCANbaudratePrescal())
	Brg_FdcanModeT Mode;             ///< FDCAN mode (normal, loopback ...)
	Brg_FdcanFrameModeT FrameMode;   ///< Frame mode used to connect to the bus (CAN, FDCAN ...)
	Brg_FdcanFifoModeT Fifo0Mode;    ///< Choose Receive FIFO0 mode blocking (default) or overwrite
	Brg_FdcanFifoModeT Fifo1Mode;    ///< Choose Receive FIFO1 mode blocking (default) or overwrite
	bool bIsArEn;                    ///< Enable (default) or disable Automatic Retransmission mode
	bool bIsTxpEn;                   ///< Enable or disable (default) Transmit Pause feature
	bool bIsPexhEn;                  ///< Enable (default) or disable Protocol exception handling
	bool bIsTdcEn;                   ///< Enable or disable (default) Transceiver delay compensation, to be used for high data baudrate in FD-CAN BRS.
	                                 ///< Without transceiver delay compensation, the bitrate in the data phase of a CAN FD frame is limited by the transceivers loop delay.
									 ///< Detailed explanation in FDCAN-Transceiver delay compensation chapter in STM32H7 reference manual.
	uint8_t tdcOffset;               ///< 0-127 tq used only if bIsTdcEn is true, Offset value defining the distance between the measured delay from FDCAN_TX to FDCAN_RX and the secondary sample point (SSP).                               
	uint8_t tdcFilter;               ///< 0-127 tq used only if bIsTdcEn is true, Defines the minimum value for the SSP position, dominant edges on FDCAN_RX that would result in an earlier SSP position are ignored for transmitter delay measurements.
} Brg_FdcanInitT;

/// FDCAN message Error State Indicator (ESI) represents transmitter error state. Used in FDCAN frame format only.
typedef enum {
	FDCAN_ESI_ACTIVE = 0, ///< Transmitting node is error active (ESI bit dominant, no error)
	FDCAN_ESI_PASSIVE = 1  ///< Transmitting node is error passive (ESI bit recessive, error)
} Brg_FdcanEsiT;

/// FDCAN message Bitrate Switching (BRS) used to activate or not data bit timing during data phase
/// (#Brg_FdcanInitT.DataBitTimeConf), for #FDCAN_FRAME_FD_BRS frame format only.
typedef enum {
	FDCAN_BRS_OFF = 0, ///< BRS is disabled (nominal bit timing is used during data phase)
	FDCAN_BRS_ON = 1   ///< BRS is enabled (data bit timing is used during data phase)
} Brg_FdcanBrsT;

/// FDCAN message Frame format (FDF) indicates if the message is sent or received in FD or Classic CAN format.
typedef enum {
	FDCAN_F_CLASSIC_CAN = 0, ///< Frame transmitted/received in Classic CAN format (BRS, ESI ignored)
	FDCAN_F_FD_CAN = 1       ///< Frame transmitted/received in FD CAN format
} Brg_FdcanFdfT;

/// Structure for Tx FDCAN messages header (except data field) and part of Rx FDCAN messages header, see also \ref LIMITATION
typedef struct {
	uint32_t ID;       ///< Identifier of the message (11bit or 29bit according to IDE).\n
					   ///< Tx: DLC: max 8. Data Length Code of requested data bytes when sending RTR.
	Brg_CanMsgIdT IDE; ///< Specifies if ID is standard (11-bit) or extended (29-bit) identifier.
	Brg_CanMsgRtrT RTR; ///< RTR (Remote/data Frame) in Classic CAN, always Data Frame in FD CAN.
						///< Rx: Remote Frame Request or data frame message type.\n
						///< Tx: RTR: Specifies if Remote Transmission Request should be sent (DLC is used for
	                    ///< number of requested bytes), otherwise the data message will be sent.
	Brg_FdcanEsiT ESI;  ///< Transmitting node Error State Indicator (unused in CAN classic).
	Brg_FdcanBrsT BRS;  ///< Bitrate Switching Off or On during data phase.
	Brg_FdcanFdfT FDF;  ///< FD Frame Format: FD or Classic CAN.
	uint8_t DLC;        ///< Rx: Data Length Code up to 8 for CAN, up to 64 for FDCAN, is the number
	                    ///< of data bytes in the received message or number of data bytes requested by RTR.\n
						///< Tx: RTR Data Length Code, 0-8, 12, 16, 20, 24, 32, 48 or 64 bytes and max 8 in classic CAN.
	                    ///< Number of requested data for RTR, unused for data Frame.
	                    ///< (for data frame Size parameter of Brg::WriteMsgFDCAN() is used as DLC)
} Brg_FdcanMsgT;

/// Structure for Rx FDCAN messages (except data field), see also \ref LIMITATION
typedef struct {
	Brg_FdcanMsgT Header; ///< Common header fields between Rx and Tx messages
	uint8_t FilterNb;   ///< Matching filter Index: filter that accepted the message (among those initialized
						///< with Brg::InitFilterFDCAN())
	Brg_CanRxOverrunT Overrun; ///< Indicate if overrun has occurred before this message.
	uint16_t TimeStamp;  ///<  Rx Message time stamp, if timestamp enabled: 16bit counter value captured on SOF detection else 0.
} Brg_FdcanRxMsgT;

/// Filter mode \n
/// In range mode, two identifiers are specified, 
/// the filter matches for all received message identifier in the range defined by ID1 to ID2
/// i.e.: filter matches if ID1 <=ID_msg <= ID2.\n
/// In identifier list mode,
/// two identifiers are specified, doubling the number of single identifiers.
/// All bits of the incoming identifier must match the bits specified in the filter.
/// i.e.: filter matches if ID_msg = ID1 or ID_msg = ID2.\n
/// In mask mode the identifier (ID1) is associated with mask (ID2) to specify which
/// bits of the identifier are handled as "must match" or as "don't care".
/// i.e.: filter matches if ID_msg & ID2 = ID1 & ID2.\n
typedef enum {
	FDCAN_FILTER_ID_RANGE = 0, ///< Range mode: accept ID in  ID1 and ID2 range (ID1 <= ID2)
	FDCAN_FILTER_ID_LIST = 1,  ///< Identifier mode: accept ID matching ID1 or ID2 (ID1 and ID2 can be equal)
	FDCAN_FILTER_ID_MASK = 2   ///< Mask mode: accept ID matching ID1 with ID2 mask (if ID2 = 0, all msg IDs match)
}Brg_FdcanFilterModeT;

/// Structure used to configure a given FDCAN filter with Brg::InitFilterFDCAN().\n
/// FilterNb and IDE define a single filter (there is one filter0 standard and one fileter0 extended).
typedef struct {
	uint32_t ID1;                     ///< Standard (max 0x7FF) or extended ID (max 0x1FFFFFFF)
	uint32_t ID2;                     ///< Standard (max 0x7FF) or extended ID (max 0x1FFFFFFF)
	uint8_t FilterNb;                 ///< Filter to configure: 0-27 for filter IDE=#CAN_ID_STANDARD or 0-7 for IDE=#CAN_ID_EXTENDED
	Brg_CanMsgIdT IDE;                ///< Filter ID Format type: choose between standard FilterNb or extended FilterNb.
	Brg_FdcanFilterModeT FilterMode;  ///< Filter range, identifier or mask mode.
	bool bIsFilterEn;                 ///< Filter Activation: Enable or disable the filter
	bool bIsFilterReject;             ///< Filter Reject mode: Messages passing the filter are rejected
	Brg_CanRxFifoT AssignedFifo;      ///< Rx FIFO in which message is received (if bIsFilterEn = true and bIsFilterReject = false, else unused)
}Brg_FdcanFilterConfT;
// end group doxygen FDCAN
/** @} */

// -------------------------------- GPIO ----------------------------------- //
/** @addtogroup GPIO
 * @{
 */
/// GPIO mask
typedef enum {
	BRG_GPIO_0 = 0x01,  ///< GPIO Bridge I00
	BRG_GPIO_1 = 0x02,  ///< GPIO Bridge I01
	BRG_GPIO_2 = 0x04,  ///< GPIO Bridge I02
	BRG_GPIO_3 = 0x08,  ///< GPIO Bridge I03
	BRG_GPIO_ALL = 0x0F ///< All GPIO Bridge: I00 I01 I02 I03
}Brg_GpioMaskT;

#define BRG_GPIO_MAX_NB 4  ///< Max number of used GPIO Bridge

/// GPIO port mode configuration, see also \ref LIMITATION
typedef enum {
	GPIO_MODE_INPUT = 0,  ///< Input mode
	GPIO_MODE_OUTPUT = 1, ///< Ouptput mode
	GPIO_MODE_ANALOG = 3  ///< Analog mode
}Brg_GpioModeT;

/// GPIO port output speed configuration
typedef enum {
	GPIO_SPEED_LOW = 0,      ///< Low speed
	GPIO_SPEED_MEDIUM = 1,   ///< Medium speed
	GPIO_SPEED_HIGH = 2,     ///< High speed
	GPIO_SPEED_VERY_HIGH = 3 ///< Very high speed
}Brg_GpioSpeedT;

/// GPIO port pull-up/pull-down configuration
typedef enum {
	GPIO_NO_PULL = 0,  ///< No pull-up, no pull-down
	GPIO_PULL_UP = 1,  ///< Pull-up
	GPIO_PULL_DOWN = 2 ///< Pull-down
}Brg_GpioPullT;

/// GPIO port output type configuration
typedef enum {
	GPIO_OUTPUT_PUSHPULL = 0, ///< Output push-pull
	GPIO_OUTPUT_OPENDRAIN = 1 ///< Output open-drain
}Brg_GpioOutputT;

/// GPIO init configuration, see also \ref LIMITATION
typedef struct {
	Brg_GpioModeT Mode;         ///< GPIO port mode
	Brg_GpioSpeedT Speed;       ///< GPIO port output speed
	Brg_GpioPullT Pull;         ///< GPIO port pull-up/pull-down
	Brg_GpioOutputT OutputType; ///< GPIO port output type
}Brg_GpioConfT;

/// GPIO init parameters for Brg::InitGPIO(), see also \ref LIMITATION
typedef struct {
	uint8_t GpioMask; ///< GPIO(s) to be configured (one or several value of #Brg_GpioMaskT)
	uint8_t ConfigNb; ///< Number of #Brg_GpioConfT pointed by pGpioConf:\n
	                  ///< must be #BRG_GPIO_MAX_NB or 1 (if 1 pGpioConf[0] used for all gpios)
	Brg_GpioConfT *pGpioConf;  ///< Table of ConfigNb init configuration.\n
	                  ///< If #BRG_GPIO_MAX_NB, pGpioConf[0] for GPIO_0, .., pGpioConf[3] for GPIO_3. \n
	                  ///< GPIO(s) that are not present in GpioMask are not configured.
} Brg_GpioInitT;

/// GPIO Level for Brg::ReadGPIO() and Brg::SetResetGPIO() functions
typedef enum {
	GPIO_RESET = 0, ///< GPIO Low level
	GPIO_SET = 1    ///< GPIO High level
}Brg_GpioValT;
/** @} */
// ------------------------------------------------------------------------- //
/* Class -------------------------------------------------------------------- */
/// Bridge Class
class Brg : public StlinkDevice
{
public:
	// Brg constructor; StlinkIf is expected to be a reference to a
	// STLINK_BRIDGE instance
	Brg(STLinkInterface &StlinkIf);

	virtual ~Brg(void);

	/**
	 * @ingroup DEVICE
	 * @retval Current Bridge C++ API version.
	 */
	int GetBridgeApiVersion(void) const {
		return 1; // Current Bridge C++ API version v1
	}

	Brg_StatusT OpenStlink(int StlinkInstId=0);
	Brg_StatusT OpenStlink(const char *pSerialNumber, bool bStrict);
	Brg_StatusT CloseStlink(void);

	Brg_StatusT ST_GetVersionExt(Stlk_VersionExtT* pVersion);
	Brg_StatusT GetTargetVoltage(float *pVoltage);

	Brg_StatusT GetLastReadWriteStatus(uint16_t *pBytesWithoutError=NULL, uint32_t *pErrorInfo=NULL);
	Brg_StatusT CloseBridge(uint8_t BrgCom);
	Brg_StatusT GetClk(uint8_t BrgCom, uint32_t *pBrgInputClk, uint32_t *pStlHClk);

	Brg_StatusT InitSPI(const Brg_SpiInitT *pInitParams);
	Brg_StatusT GetSPIbaudratePrescal(uint32_t ReqSpiFreqKHz, Brg_SpiBaudrateT *pBaudrate, uint32_t *pFinalSpiFreqKHz);
	Brg_StatusT SetSPIpinCS(Brg_SpiNssLevelT NssLevel);
	Brg_StatusT ReadSPI(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT WriteSPI(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten);

	Brg_StatusT InitI2C(const Brg_I2cInitT *pInitParams);
	Brg_StatusT GetI2cTiming(I2cModeT I2CSpeedMode, int SpeedFrequency, int DNFn, int RiseTime,
	                         int FallTime, bool bAF, uint32_t *pTimingReg);
	Brg_StatusT ReadI2C(uint8_t *pBuffer, uint16_t Addr,
	                    uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT ReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                    uint16_t SizeInBytes, uint16_t *pSizeRead);
	Brg_StatusT StartReadI2C(uint8_t *pBuffer, uint16_t Addr,
	                         uint16_t SizeInBytes, uint16_t *pSizeRead, const I2cTransPrepT PrepRestart = I2C_PREP_NONE);
	Brg_StatusT StartReadI2C(uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                         uint16_t SizeInBytes, uint16_t *pSizeRead, const I2cTransPrepT PrepRestart = I2C_PREP_NONE);
	Brg_StatusT ContReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead,
	                         const I2cTransPrepT PrepRestart = I2C_PREP_NONE, const I2cTransForceT ForceRestart = I2C_FORCE_NONE);
	Brg_StatusT StopReadI2C(uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeRead,
	                        const I2cTransForceT ForceRestart = I2C_FORCE_NONE);
	Brg_StatusT ReadNoWaitI2C(uint16_t Addr, uint16_t SizeInBytes,
	                          uint16_t *pSizeRead, uint16_t CmdTimeoutMs);
	Brg_StatusT ReadNoWaitI2C(uint16_t Addr, Brg_I2cAddrModeT AddrMode, uint16_t SizeInBytes,
	                          uint16_t *pSizeRead, uint16_t CmdTimeoutMs);
	Brg_StatusT GetReadDataI2C(uint8_t *pBuffer, uint16_t SizeInBytes);
	Brg_StatusT WriteReadI2C(uint8_t *pBufferR, const uint16_t Addr, const uint16_t ReadSizeInBytes, uint16_t *pSizeRead,
	                         const uint8_t *pBufferW,  const uint16_t WriteSizeInBytes);
	Brg_StatusT WriteReadI2C(uint8_t *pBufferR, const  uint16_t Addr, const Brg_I2cAddrModeT AddrMode,
	                         const uint16_t ReadSizeInBytes, uint16_t *pSizeRead,
	                         const uint8_t *pBufferW,  const uint16_t WriteSizeInBytes);
	Brg_StatusT WriteI2C(const uint8_t *pBuffer, uint16_t Addr,
	                     uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT WriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                     uint16_t SizeInBytes, uint16_t *pSizeWritten);
	Brg_StatusT StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr,
	                          uint16_t SizeInBytes, uint16_t *pSizeWritten, const I2cTransPrepT PrepRestart = I2C_PREP_NONE);
	Brg_StatusT StartWriteI2C(const uint8_t *pBuffer, uint16_t Addr, Brg_I2cAddrModeT AddrMode,
	                          uint16_t SizeInBytes, uint16_t *pSizeWritten, const I2cTransPrepT PrepRestart = I2C_PREP_NONE);
	Brg_StatusT ContWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten,
	                         const I2cTransPrepT PrepRestart = I2C_PREP_NONE, const I2cTransForceT ForceRestart = I2C_FORCE_NONE);
	Brg_StatusT StopWriteI2C(const uint8_t *pBuffer, uint16_t SizeInBytes, uint16_t *pSizeWritten,
	                         const I2cTransForceT ForceRestart = I2C_FORCE_NONE);

	Brg_StatusT InitCAN(const Brg_CanInitT *pInitParams, Brg_InitTypeT InitType);
	Brg_StatusT GetCANbaudratePrescal(const Brg_CanBitTimeConfT *pBitTimeConf, uint32_t ReqBaudrate,
	                                  uint32_t *pPrescal, uint32_t *pFinalBaudrate);
	Brg_StatusT InitFilterCAN(const Brg_CanFilterConfT *pInitParams);
	Brg_StatusT StartMsgReceptionCAN(void);
	Brg_StatusT StopMsgReceptionCAN(void);
	Brg_StatusT GetRxMsgNbCAN(uint16_t *pMsgNb);
	Brg_StatusT GetRxMsgCAN(Brg_CanRxMsgT *pCanMsg, uint16_t MsgNb, uint8_t *pBuffer,
	                        uint16_t BufSizeInBytes, uint16_t *pDataSizeInBytes);
	Brg_StatusT WriteMsgCAN(const Brg_CanTxMsgT *pCanMsg, const uint8_t *pBuffer, uint8_t SizeInBytes);

	Brg_StatusT InitFDCAN(const Brg_FdcanInitT* pInitParams, Brg_InitTypeT InitType, bool bStartBus=true);
	Brg_StatusT StartFDCAN(void);
	Brg_StatusT StopFDCAN(void);
	Brg_StatusT GetFDCANbaudratePrescal(const Brg_CanBitTimeConfT* pBitTimeConf, uint32_t ReqBaudrate,
	                                uint32_t* pPrescal, uint32_t* pFinalBaudrate, const Brg_FdcanFrameModeT CanMode, const bool bIsNomBitTime);
	Brg_StatusT InitFilterFDCAN(const Brg_FdcanFilterConfT* pInitParams);
	Brg_StatusT StartMsgReceptionFDCAN(void);
	Brg_StatusT StopMsgReceptionFDCAN(void);
	Brg_StatusT GetRxMsgNbFDCAN(uint16_t* pMsgNb, const Brg_CanRxFifoT FifoNb = CAN_MSG_RX_FIFO0);
	Brg_StatusT GetRxMsgFDCAN(Brg_FdcanRxMsgT* pFdcanMsg, uint16_t MsgNb, uint8_t* pBuffer,
	                          uint16_t BufSizeInBytes, uint16_t* pDataSizeInBytes,
	                          const Brg_CanRxFifoT FifoNb = CAN_MSG_RX_FIFO0);
	Brg_StatusT WriteMsgFDCAN(const Brg_FdcanMsgT* pFdcanMsg, const uint8_t* pBuffer, uint8_t SizeInBytes);

	Brg_StatusT InitGPIO(const Brg_GpioInitT *pInitParams);
	Brg_StatusT ReadGPIO(uint8_t GpioMask, Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask);
	Brg_StatusT SetResetGPIO(uint8_t GpioMask, const Brg_GpioValT *pGpioVal, uint8_t *pGpioErrorMask);

	static Brg_StatusT ConvSTLinkIfToBrgStatus(STLinkIf_StatusT IfStat);

	bool IsCanSupport(void) const;
	bool IsReadNoWaitI2CSupport(void) const;
	bool IsRestartI2CSupport(void) const;
	bool IsOldBrgFwVersion(void) const;
	bool IsCanFilter16Support(void) const;
	bool IsFdcanSupport(void) const;

private:

	Brg_StatusT AnalyzeStatus(const uint16_t *pStatus);

	Brg_StatusT SendRequestAndAnalyzeStatus(STLink_DeviceRequestT *pDevReq,
	                                        const uint16_t *pStatus,
	                                        const uint16_t UsbTimeoutMs=0);


	Brg_StatusT WriteI2Ccmd(const uint8_t *pBuffer, uint16_t Addr, uint16_t Size,
	                        Brg_I2cRWTransfer RwTransType, uint16_t *pSizeWritten, uint32_t *pErrorInfo,
	                        const I2cTransPrepT PrepRestart, const I2cTransForceT ForceRestart);
	Brg_StatusT ReadI2Ccmd(uint8_t *pBuffer, const uint16_t Addr, const uint16_t SizeInBytes,
	                       const Brg_I2cRWTransfer RwTransType, uint16_t *pSizeRead, uint32_t *pErrorInfo,
	                       const uint8_t *pBufferW,  const uint16_t WriteSizeInBytes,
						   const I2cTransPrepT PrepRestart, const I2cTransForceT ForceRestart);

	uint8_t GpioConfField(Brg_GpioConfT GpioConfParam);

	// Global to manage I2C partial transaction (START, STOP, CONT)
	uint16_t m_slaveAddrPartialI2cTrans;

	Brg_StatusT CalculateI2cTimingReg(I2cModeT I2CSpeedMode, int SpeedFrequency, double ClockSource,
	                                  int DNFn, int RiseTime, int FallTime, bool bAF, uint32_t *pTimingReg);
	Brg_StatusT FormatFilter32bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf);
	Brg_StatusT FormatFilter16bitCAN(const Brg_FilterBitsT *pInConf, uint8_t *pOutConf);
	Brg_StatusT CheckBitTimeClassicCAN(const Brg_CanBitTimeConfT* pBitTimeConf);
	Brg_StatusT CheckBitTimeFDCAN(const Brg_CanBitTimeConfT* pBitTimeConf, const Brg_FdcanFrameModeT CanMode, const bool bIsNomBitTime);
	Brg_StatusT InitBitTimeFDCAN(const Brg_CanBitTimeConfT* pBitTimeConf, const uint32_t Prescaler, const Brg_FdcanFrameModeT CanMode, const bool bIsNomBitTime);
};

#endif //_BRIDGE_H
/** @} */
/**********************************END OF FILE*********************************/