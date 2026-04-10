 
// WARNING: Instantiation, initialization steps are missing from the examples

// EXAMPLE FOR Bridge devices detection and enumeration: STLinkInterface() STLinkInterface::LoadStlinkLibrary(), STLinkInterface::EnumDevices() STLinkInterface::GetDeviceInfo2()
	cBrgExample brgTest; //see main_examples.cpp for details
	Brg_StatusT brgStat = BRG_NO_ERR;
	STLinkIf_StatusT ifStat = STLINKIF_NO_ERR;
	char path[MAX_PATH];
#ifdef WIN32 //Defined for applications for Win32 and Win64.
	char *pEndOfPrefix;
#endif
	int firstDevNotInUse=-1;
	Brg* pBrg = NULL;
	STLinkInterface *m_pStlinkIf = NULL;

	// Note: cErrLog g_ErrLog; to be instanciated and initialized if used with USING_ERRORLOG

	// In case previously used, close the previous connection (not the case here)

	// USB interface initialization and device detection done using STLinkInterface

	// Create USB BRIDGE interface
	m_pStlinkIf = new STLinkInterface(STLINK_BRIDGE);
	#ifdef USING_ERRORLOG
	m_pStlinkIf->BindErrLog(&g_ErrLog);
	#endif

	// Init path to STLinkUSBdriver library path
	strcpy(path, "STLinkUSBdriver library path");// to be updated

	// Load STLinkUSBDriver library
	// In this example STLinkUSBdriver (dll on windows) must be copied near test executable
	ifStat = m_pStlinkIf->LoadStlinkLibrary(path);
	if( ifStat!=STLINKIF_NO_ERR ) {
		printf("STLinkUSBDriver library (dll) issue \n");
	}

	// Enumerate the STLink Bridge instance, and choose the first one in the list
	brgStat = brgTest.SelectSTLink(m_pStlinkIf, &firstDevNotInUse);

	// function brgTest.SelectSTLink
	Brg_StatusT cBrgExample::SelectSTLink(STLinkInterface* pStlinkIf, int* pFirstDevNotInUse)
	{
		uint32_t i, numDevices;
		TDeviceInfo2 devInfo2;
		STLinkIf_StatusT ifStatus = STLINKIF_NO_ERR;
		STLink_EnumStlinkInterfaceT stlinkIfId;

		if ((pStlinkIf == NULL) || (pFirstDevNotInUse == NULL)) {
			printf("Internal parameter error in SelectSTLink\n");
			return BRG_PARAM_ERR;
		}
		stlinkIfId = pStlinkIf->GetIfId();
		if (stlinkIfId != STLINK_BRIDGE) {
			printf("Wrong interface in SelectSTLink\n");
			return BRG_PARAM_ERR;
		}

		ifStatus = pStlinkIf->EnumDevices(&numDevices, FALSE);
		// Choose the first STLink Bridge available
		if ((ifStatus == STLINKIF_NO_ERR) || (ifStatus == STLINKIF_PERMISSION_ERR)) {
			printf("%d BRIDGE device found\n", (int)numDevices);

			for( i=0; i<numDevices; i++ ) {
				ifStatus = pStlinkIf->GetDeviceInfo2(i, &devInfo2, sizeof(devInfo2));
				printf("Bridge %d PID: 0X%04hx SN:%s\n", (int)i, (unsigned short)devInfo2.ProductId, devInfo2.EnumUniqueId);

				if( (*pFirstDevNotInUse==-1) && (devInfo2.DeviceUsed == false) ) {
					*pFirstDevNotInUse = i;
					memcpy(m_serialNumber, &devInfo2.EnumUniqueId, SERIAL_NUM_STR_MAX_LEN);
					printf(" SELECTED BRIDGE Stlink SN:%s\n", m_serialNumber);
				}
			}
		} else if (ifStatus == STLINKIF_CONNECT_ERR) {
			printf("No STLink BRIDGE device detected\n");
		} else {
			printf("Enum error (status = %d)\n", ifStatus);
			if (ifStatus == STLINKIF_NO_STLINK) {
				printf("No BRIDGE STLink available\n");
			}
		}

		return Brg::ConvSTLinkIfToBrgStatus(ifStatus);
	}

// EXAMPLE FOR Bridge STLink connection: Brg() StlinkDevice::SetOpenModeExclusive() Brg::OpenStlink() Brg::GetTargetVoltage()

	STLinkInterface *m_pStlinkIf;
	STLinkIf_StatusT ifStat
	**********[ Missing init steps ]**********
	Brg_StatusT brgStat = BRG_NO_ERR;
	int firstDevNotInUse=0;
	Brg* pBrg = NULL;

	// Note: cErrLog g_ErrLog; to be instanciated and initialized if used with USING_ERRORLOG

	// See previous example for enumeration STLink index selection...
	// USB interface: m_pStlinkIf must have been initialized, library loaded (see previous example)
	// firstDevNotInUse point on a connected STLink having Bridge interface (see previous example)
		// Enumerate the STLink Bridge instance, and choose the first one in the list
	brgStat = brgTest.SelectSTLink(m_pStlinkIf, &firstDevNotInUse);

	// USB Connection to a given device done with Brg
	if (brgStat == BRG_NO_ERR) {
		pBrg = new Brg(*m_pStlinkIf);
	#ifdef USING_ERRORLOG
		m_pBrg->BindErrLog(&g_ErrLog);
	#endif
	}
	// Connect to the selected STLink
	if (brgStat == BRG_NO_ERR) {
		brgStat = brgTest.Connect(pBrg, firstDevNotInUse);
	}

	// function brgTest.Connect()
	Brg_StatusT cBrgExample::Connect(Brg* pBrg, int deviceNb)
	{
		// The firmware may not be the very last one, but it may be OK like that (just inform)
		bool bOldFirmwareWarning=false;
		Brg_StatusT brgStat = BRG_NO_ERR;
		if (pBrg == NULL) {
			return BRG_CONNECT_ERR;
		}
		m_pBrg = pBrg;
		// Open the STLink connection
		if (brgStat == BRG_NO_ERR) {
			m_pBrg->SetOpenModeExclusive(true);

			brgStat = m_pBrg->OpenStlink(deviceNb);

			if (brgStat == BRG_NOT_SUPPORTED) {
				printf("BRIDGE not supported SN:%s\n", m_serialNumber);
			}
			if (brgStat == BRG_OLD_FIRMWARE_WARNING) {
				// Status to restore at the end if all is OK
				bOldFirmwareWarning = true;
				brgStat = BRG_NO_ERR;
			}
		}

		// Test Voltage command
		if (brgStat == BRG_NO_ERR) {
			float voltage = 0;
			// T_VCC pin must be connected to target voltage on bridge or debug connector
			// T_VCC input is mandatory for STLink using levelshifter (STLINK-V3PWR or STLINK-V3SET+B-STLINK-VOLT/ISOL),
			// else bridge signals are all 0
			brgStat = m_pBrg->GetTargetVoltage(&voltage);
			if (brgStat != BRG_NO_ERR) {
				printf("BRIDGE get voltage error \n");
			} else if (voltage < 1) {
				printf("BRIDGE get voltage WARNING: %fV < 1V, check if T_VCC pin is connected to target voltage on bridge (or debug) connector \n", (double)voltage);
			} else {
				printf("BRIDGE get voltage: %f V \n", (double)voltage);
			}
		}

		if ((brgStat == BRG_NO_ERR) && (bOldFirmwareWarning == true)) {
			// brgStat = BRG_OLD_FIRMWARE_WARNING;
			printf("BRG_OLD_FIRMWARE_WARNING: v%d B%d \n",(int)m_pBrg->m_Version.Major_Ver, (int)m_pBrg->m_Version.Bridge_Ver);
		}

		return brgStat;

	}


// EXAMPLE FOR Bridge STLink Deconnection: Brg::CloseBridge() Brg::CloseStlink()

	Brg *pBrg;
	STLinkInterface *m_pStlinkIf;
	**********[ Missing init steps ]**********

	if( pBrg!=NULL ) {
		pBrg->CloseBridge(COM_UNDEF_ALL);
		pBrg->CloseStlink();
		delete pBrg;
		pBrg = NULL;
	}
	// unload STLinkUSBdriver library
	if( m_pStlinkIf!=NULL ) {
		// always delete STLinkInterface after Brg (because Brg uses STLinkInterface)
		delete m_pStlinkIf;
		m_pStlinkIf = NULL;
	}
