 
// WARNING: Instantiation, initialization steps are missing from the examples


// EXAMPLE FOR I2C input CLK, Brg::GetClk()</B>\n

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint32_t currFreqKHz=0;
	uint8_t com = COM_I2C;
	uint32_t StlHClkKHz, comInputClkKHz;
	// Get the current bridge input Clk
	brgStat = m_pBrg->GetClk(com, &comInputClkKHz, &StlHClkKHz);
	printf( "I2C input CLK: %d KHz, STLink HCLK: %d KHz \n", (int)comInputClkKHz, (int)StlHClkKHz);  

 
// EXAMPLE FOR I2C Initialization, Brg::InitI2C()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	// the STLink Bridge configuration is I2C master
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_I2cInitT i2cParam;
	int freqKHz = 400; //400KHz
	uint32_t timingReg; 
	int riseTimeNs, fallTimeNs, DNF;
	bool analogFilter;

	// I2C_FAST freqKHz: 1-400KHz I2C_FAST_PLUS: 1-1000KHz
	riseTimeNs = 0; //0-300ns
	fallTimeNs = 0; //0-300ns
	DNF = 0; // digital filter OFF
	analogFilter = true;

	brgStat = m_pBrg->GetI2cTiming(I2C_STANDARD, freqKHz, DNF, riseTimeNs, fallTimeNs, analogFilter, &timingReg);
	// example I2C_STANDARD, I2C input CLK= 192MHz, rise/fall time (ns) = 0, analog filter on, dnf=0
	// I2C freq = 400KHz timingReg = 0x20602274
	if( brgStat == BRG_NO_ERR ) {
		i2cParam.timingReg = timingReg;
		i2cParam.ownAddr = 0;// 0  unused in I2C master mode
		i2cParam.addrMode = I2C_ADDR_7BIT;
		i2cParam.anFilterEn = I2C_FILTER_ENABLE;
		i2cParam.digitalFilterEn = I2C_FILTER_DISABLE;
		i2cParam.dnf = (uint8_t)DNF; //0
		brgStat = m_pBrg->InitI2C(&i2cParam);
	} else {
			printf("I2C timing error, timing reg: 0x%08lx\n", timingReg);
	}


// EXAMPLE FOR I2C read/write loopback, Brg::WriteI2C() Brg::ReadI2C()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	// I2C LOOPBACK
	// following protocol:
	// I2C configured in master mode, 
	// 1- send 4 bytes = data size,
	// 2- wait to receive it back
	// 3- send data size bytes
	// 4- wait to receive same data size bytes back.
	// 5- restart to 1
	// Note: I2C target must be configured in slave mode to answer this protocol
	// I2C target slave configuration: I2C_ADDR_7BIT, slave address: 0x39 (57)
	Brg_StatusT brgStat = BRG_NO_ERR;
	uint8_t dataRx[3072], dataTx[3072]; //max size must be aligned with target buffer
	uint16_t size;
	InitTestBuffer(BUFF_0X00, dataRx, 3072);

	size = 8; //can be 1 - up to buffer size bytes
	InitTestBuffer(BUFF_PATTERN1, dataTx, size);
	brgStat = BrgRxTxVerifData(pTestParam, dataRx, dataTx, size);
	
	//BrgRxTxVerifData function
	Brg_StatusT BrgTest::BrgRxTxVerifData(Integ_TestParamT *pTestParam, uint8_t*pRxBuff, uint8_t*pTxBuff, uint16_t size)
	{
		Brg_StatusT brgStat = BRG_NO_ERR;
		int i, nb;
		uint16_t sizeWithoutErr =0;
		uint32_t txSize,rxSize=0;
		Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
		uint16_t i2cSlaveAddr = 0x39;
	
		txSize = (uint32_t) size;
	
		// 1- send 4 bytes = data size,
		brgStat = m_pBrg->WriteI2C((uint8_t*)&txSize, i2cSlaveAddr, addrMode, 4, &sizeWithoutErr);
		// or brgStat = m_pBrg->WriteI2C((uint8_t*)&txSize, I2C_7B_ADDR(i2cSlaveAddr), 4, &sizeWithoutErr);
		if( brgStat != BRG_NO_ERR ) {
			printf("BRG Write data size error (sent %d instead of 4)\n", (int)sizeWithoutErr);
		}
		// 2- wait to receive it back
		if( brgStat == BRG_NO_ERR ) {
			sizeWithoutErr = 0;
			brgStat = m_pBrg->ReadI2C((uint8_t*)&rxSize, i2cSlaveAddr, addrMode, 4, &sizeWithoutErr);
			// or brgStat = m_pBrg->ReadI2C((uint8_t*)&rxSize, I2C_7B_ADDR(i2cSlaveAddr), 4, &sizeWithoutErr);
			if( brgStat != BRG_NO_ERR ) {
				printf("BRG Read back data size error (read %d instead of 4)\n", (int)sizeWithoutErr);
			} else {
				if( rxSize != txSize ) {
					brgStat = BRG_VERIF_ERR;
					printf("BRG Read back RxSize = %d different from TxSize = %d \n", (int)rxSize, (int)txSize);
				}
			}
		}
		// 3- send data size bytes
		if( brgStat == BRG_NO_ERR ) {
			sizeWithoutErr = 0;
			brgStat = m_pBrg->WriteI2C(pTxBuff, i2cSlaveAddr, addrMode, size, &sizeWithoutErr);
			if( brgStat != BRG_NO_ERR ) {
				printf("BRG Write data error (sent %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
			}
		}
		// 4- wait to receive same data size bytes back.
		if( brgStat == BRG_NO_ERR ) {
			sizeWithoutErr = 0;
			brgStat = m_pBrg->ReadI2C(pRxBuff, i2cSlaveAddr, addrMode, size, &sizeWithoutErr);
			if( brgStat != BRG_NO_ERR ) {
				printf("BRG Read back data error (read %d instead of %d)\n", (int)sizeWithoutErr, (int)size);
			} else {
				if( memcmp(pRxBuff, pTxBuff, size) !=0 ) {
					brgStat = BRG_VERIF_ERR;
					printf("BRG ERROR Read/Write verification error(s)\n");
				}
			}
		}	
		return brgStat;
	}


// EXAMPLE FOR I2C low level Read transaction, Brg::StartReadI2C() Brg::ContReadI2C() Brg::StopReadI2C()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
    uint8_t cmdTabSize;
    uint8_t pCmd[12]={0,0,0,0,0,0,0,0,0,0,0,0};
    uint8_t cmdNb, version;
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
	uint16_t i2cSlaveAddr = 0x39; //= I2C_7B_ADDR(i2cSlaveAddr)
	
	// 1 I2C transaction = uint8_t1: cmd number, uint8_t2: version, uint8_t3 to 3+uint8_t1: cmd val
	
	// read byte 1
	brgStat = m_pBrg->StartReadI2C(&cmdNb, I2C_7B_ADDR(i2cSlaveAddr), 1, NULL);
	// or brgStat = m_pBrg->StartReadI2C(&cmdNb, i2cSlaveAddr, addrMode, 1, NULL);
	if( (brgStat != BRG_NO_ERR) || (cmdNb>12) ) {
		// Com error
		return brgStat;
	}
	// read byte 2
	brgStat = m_pBrg->ContReadI2C(&version, 1, NULL); 
	// read cmdNb bytes: byte 3 to 3+cmdNb
	if( brgStat == BRG_NO_ERR ) {
		brgStat = m_pBrg->StopReadI2C(pCmd, cmdNb, NULL);
	}


// EXAMPLE FOR I2C Write-Read Repeated start with full transaction, Brg::WriteReadI2C(),
// to perform this kind of I2C transaction: Start-addr-writedata-restart-addr-readdata-stop

	Brg *m_pBrg;
	**********[ Missing init steps ]**********
	#define TEST_BUFF_SIZE 3072
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
	uint16_t i2cSlaveAddr = 0x01; //= I2C_7B_ADDR(i2cSlaveAddr)
	uint8_t dataRx[TEST_BUFF_SIZE], dataTx[TEST_BUFF_SIZE];
	uint16_t size;

	memset(dataRx, 0, TEST_BUFF_SIZE);
	memset(dataTx, 0, TEST_BUFF_SIZE);

	// WriteRead Write 1byte then Read 1byte: 
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 7; // "cmd" (write data)
		brgStat = m_pBrg->WriteReadI2C(&dataRx[0], I2C_7B_ADDR(i2cSlaveAddr), 1, NULL, &dataTx[0], 1);
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

	// WriteRead Write 4bytes then Read 100 bytes: 
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 18;   // "cmd" (write data byte1)
		dataTx[1] = 0x0a; // (write data byte2)
		dataTx[2] = 0x0b; // (write data byte3)
		dataTx[3] = 0x0c; // (write data byte4)

		brgStat = m_pBrg->WriteReadI2C(&dataRx[0], I2C_7B_ADDR(i2cSlaveAddr), 100, NULL, &dataTx[0], 4);
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

// EXAMPLE FOR I2C Write-Read Repeated start with low level transaction,
// Brg::ContReadI2C() Brg::StopReadI2C(), Brg::StartWriteI2C() Brg::ContWriteI2C()
// to perform this kind of I2C transaction: Start-addr-writedata-restart-addr-readdata-stop
    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	#define TEST_BUFF_SIZE 3072
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
	uint16_t i2cSlaveAddr = 0x01; //= I2C_7B_ADDR(i2cSlaveAddr)
	uint8_t dataRx[TEST_BUFF_SIZE], dataTx[TEST_BUFF_SIZE];
	uint16_t size;

	memset(dataRx, 0, TEST_BUFF_SIZE);
	memset(dataTx, 0, TEST_BUFF_SIZE);

	// Split WriteRead, Write 1byte then Read 1byte: 
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 7; // "cmd" (write data)
		// send "cmd" and prepare restart
		brgStat = m_pBrg->StartWriteI2C(&dataTx[0], I2C_7B_ADDR(i2cSlaveAddr), addrMode, 1, NULL, I2C_PREP_RESTART);
		if (brgStat == BRG_NO_ERR) { // generate restart and read 1 byte
			brgStat = m_pBrg->StopReadI2C(&dataRx[0], 1, NULL, I2C_FORCE_RESTART);
			// or brgStat = m_pBrg->StopReadI2C(&dataRx[0], 1, NULL, I2C_FORCE_NONE);
			// I2C_FORCE_RESTART not mandatory because of direction change
		}
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

	// Split WriteRead, Write 4bytes then Read 100 bytes: 
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 18;   // "cmd" (write data byte1)
		dataTx[1] = 0x0a; // (write data byte2)
		dataTx[2] = 0x0b; // (write data byte3)
		dataTx[3] = 0x0c; // (write data byte4)
		// send byte1 and prepare restart
		brgStat = m_pBrg->StartWriteI2C(&dataTx[0], I2C_7B_ADDR(i2cSlaveAddr), 4, NULL, I2C_PREP_RESTART);
		if (brgStat == BRG_NO_ERR) {
			brgStat = m_pBrg->StopReadI2C(&dataRx[0], 100, NULL, I2C_FORCE_RESTART);
			// or brgStat = m_pBrg->StopReadI2C(&dataRx[0], 100, NULL, I2C_FORCE_NONE);
			// I2C_FORCE_RESTART not mandatory because of direction change
		}
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

	// Split WriteRead, Write 6bytes then Read 100 bytes using Cont(): 
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 19;   // "cmd" (write data byte1)
		dataTx[1] = 0x0a; // (write data byte2)
		dataTx[2] = 0x0b; // (write data byte3)
		dataTx[3] = 0x0c; // (write data byte4)
		dataTx[4] = 0xaa; // (write data byte5)
		dataTx[5] = 0xbb; // (write data byte6)
		// send byte1-2
		brgStat = m_pBrg->StartWriteI2C(&dataTx[0], I2C_7B_ADDR(i2cSlaveAddr), 2, NULL, I2C_PREP_NONE);
		if( brgStat == BRG_NO_ERR ) { // send byte3-6 and prepare restart
			brgStat = m_pBrg->ContWriteI2C(&dataTx[2], 4, NULL, I2C_PREP_RESTART, I2C_FORCE_NONE);
		}
		if( brgStat == BRG_NO_ERR ) { // generate restart and read 1 byte
			brgStat = m_pBrg->ContReadI2C(&dataRx[0], 1, NULL, I2C_PREP_NONE, I2C_FORCE_RESTART);
			// or brgStat = m_pBrg->ContReadI2C(&dataRx[0], 100, NULL, I2C_PREP_NONE, I2C_FORCE_NONE);
			// I2C_FORCE_RESTART not mandatory because of direction change
		}
		if( brgStat == BRG_NO_ERR ) { // read remaing data
			brgStat = m_pBrg->StopReadI2C(&dataRx[1], 99, NULL, I2C_FORCE_NONE);
		}
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

// EXAMPLE FOR I2C Read-Read Repeated start with low level transaction,
// Brg::StartWriteI2C() Brg::ContWriteI2C() Brg::ContReadI2C() Brg::StopReadI2C()
// to perform this kind of I2C transaction: Start-addr-writedata-restart-addr-readdata1-restart-addr-readdata2-stop

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	#define TEST_BUFF_SIZE 3072
	Brg_StatusT brgStat = BRG_NO_ERR;
	Brg_I2cAddrModeT addrMode = I2C_ADDR_7BIT;
	uint16_t i2cSlaveAddr = 0x01; //= I2C_7B_ADDR(i2cSlaveAddr)
	uint8_t dataRx[TEST_BUFF_SIZE], dataTx[TEST_BUFF_SIZE];
	uint16_t size;

	memset(dataRx, 0, TEST_BUFF_SIZE);
	memset(dataTx, 0, TEST_BUFF_SIZE);

	// Split WriteReadRead, Write 4bytes, Read 2 bytes then restart and read 98 bytes:
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 18;   // "cmd" (write data byte1)
		dataTx[1] = 0x0a; // (write data byte2)
		dataTx[2] = 0x0b; // (write data byte3)
		dataTx[3] = 0x0c; // (write data byte4)
		// send 4 bytes and prepare restart
		if( brgStat == BRG_NO_ERR ) {
			brgStat = m_pBrg->StartWriteI2C(&dataTx[0], I2C_7B_ADDR(i2cSlaveAddr), 4, NULL, I2C_PREP_RESTART);
		}
		if( brgStat == BRG_NO_ERR ) { // generate restart and read 2 bytes and prepare restart
			brgStat = m_pBrg->ContReadI2C(&dataRx[0], 2, NULL, I2C_PREP_RESTART, I2C_FORCE_RESTART);
			// or brgStat = m_pBrg->ContReadI2C(&dataRx[0], 2, NULL, I2C_PREP_NONE, I2C_FORCE_NONE);
			// I2C_FORCE_RESTART not mandatory because of direction change
		}
		if( brgStat == BRG_NO_ERR ) { // generate restart and read remaining data
			brgStat = m_pBrg->StopReadI2C(&dataRx[2], 98, NULL, I2C_FORCE_RESTART);
		}
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}
	// Split WriteReadRead, Write 4bytes, Read 2 bytes then restart and read 98 bytes using Cont():
	if (brgStat == BRG_NO_ERR) {
		dataTx[0] = 18;   // "cmd" (write data byte1)
		dataTx[1] = 0x0a; // (write data byte2)
		dataTx[2] = 0x0b; // (write data byte3)
		dataTx[3] = 0x0c; // (write data byte4)
		// send 4 bytes and prepare restart
		if( brgStat == BRG_NO_ERR ) {
			brgStat = m_pBrg->StartWriteI2C(&dataTx[0], I2C_7B_ADDR(i2cSlaveAddr), 4, NULL, I2C_PREP_RESTART);
		}
		if( brgStat == BRG_NO_ERR ) { // generate restart and read 1 byte
			brgStat = m_pBrg->ContReadI2C(&dataRx[0], 1, NULL, I2C_PREP_NONE, I2C_FORCE_RESTART);
			// or brgStat = m_pBrg->ContReadI2C(&dataRx[0], 1, NULL, I2C_PREP_NONE, I2C_FORCE_NONE);
			// I2C_FORCE_RESTART not mandatory because of direction change
		}
		if( brgStat == BRG_NO_ERR ) { // continue read 1 byte and prepare restart
			brgStat = m_pBrg->ContReadI2C(&dataRx[1], 1, NULL, I2C_PREP_RESTART, I2C_FORCE_NONE);
		}
		if( brgStat == BRG_NO_ERR ) { // force restart and read 1 byte
			brgStat = m_pBrg->ContReadI2C(&dataRx[2], 1, NULL, I2C_PREP_NONE, I2C_FORCE_RESTART);
		}
		if( brgStat == BRG_NO_ERR ) { // read remaining data
			brgStat = m_pBrg->StopReadI2C(&dataRx[3], 97, NULL, I2C_FORCE_NONE);
		}
		if (brgStat == BRG_NO_ERR) {
			// display read data from dataRx[]
		}
	}

// EXAMPLE FOR I2C Read without waiting, Brg::ReadNoWaitI2C() Brg::GetReadDataI2C() Brg::GetLastReadWriteStatus()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
    // case for slave not ending the low level transaction within less than 200ms (line streching)
    uint16_t cmdTimeoutMs = 15000; // 15s
	// receive 1 byte but do not wait for the data
	brgStat = m_pBrg->ReadNoWaitI2C(i2cSlaveAddr, 1, NULL, cmdTimeoutMs);
	// wait until command is complete at target side (BUSY state can be answered by the STLink during cmdTimeoutMs)
	// no other command than GetLastReadWriteStatus() can been sent while BUSY is answered
	while( brgStat == BRG_CMD_BUSY ) {
		Sleep(1000);
		brgStat = m_pBrg->GetLastReadWriteStatus(NULL, NULL);
	}
	// get the data once ready.
	if( brgStat == BRG_NO_ERR ) {
		brgStat = m_pBrg->GetReadDataI2C(&stm32Ack, 1);
	}


// EXAMPLE FOR I2C close, Brg::CloseBridge()

    Brg *m_pBrg;
	**********[ Missing init steps ]**********
	Brg_StatusT brgStat = BRG_NO_ERR;

	brgStat = m_pBrg->CloseBridge(COM_I2C);
