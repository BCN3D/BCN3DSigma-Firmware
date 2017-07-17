#include "Marlin.h"
#include "cardreader.h"
#include "ultralcd.h"
#include "stepper.h"
#include "temperature.h"
#include "language.h"
#include "Touch_Screen_Definitions.h"
#include "SD_ListFiles.h"
#ifdef SDSUPPORT



CardReader::CardReader()
{
	filesize = 0;
	sdpos = 0;
	sdprinting = false;
	sdispaused = false;
	cardOK = false;
	saving = false;
	logging = false;
	autostart_atmillis=0;
	workDirDepth = 0;
	file_subcall_ctr=0;
	memset(workDirParents, 0, sizeof(workDirParents));

	autostart_stilltocheck=true; //the SD start is delayed, because otherwise the serial cannot answer fast enough to make contact with the host software.
	lastnr=0;
	//power to SD reader
	#if SDPOWER > -1
	SET_OUTPUT(SDPOWER);
	WRITE(SDPOWER,HIGH);
	#endif //SDPOWER
	
	autostart_atmillis=millis()+5000;
}

char *createFilename(char *buffer,const dir_t &p) //buffer>12characters
{
	char *pos=buffer;
	for (uint8_t i = 0; i < 11; i++)
	{
		if (p.name[i] == ' ')continue;
		if (i == 8)
		{
			*pos++='.';
		}
		*pos++=p.name[i];
	}
	*pos++=0;
	return buffer;
}


void CardReader::lsDive(const char *prepend, SdFile parent, const char * const match/*=NULL*/) {
	dir_t p;
	uint8_t cnt = 0;

	// Read the next entry from a directory
	while (parent.readDir(p, longFilename) > 0) {

		// If the entry is a directory and the action is LS_SerialPrint
		if (DIR_IS_SUBDIR(&p) && lsAction != LS_Count && lsAction != LS_GetFilename) {

			// Get the short name for the item, which we know is a folder
			char lfilename[13];
			createFilename(lfilename, p);

			// Allocate enough stack space for the full path to a folder, trailing slash, and nul
			boolean prepend_is_empty = (prepend[0] == '\0');
			int len = (prepend_is_empty ? 1 : strlen(prepend)) + strlen(lfilename) + 1 + 1;
			char path[len];

			// Append the FOLDERNAME12/ to the passed string.
			// It contains the full path to the "parent" argument.
			// We now have the full path to the item in this folder.
			strcpy(path, prepend_is_empty ? "/" : prepend); // root slash if prepend is empty
			strcat(path, lfilename); // FILENAME_LENGTH-1 characters maximum
			strcat(path, "/");       // 1 character

			// Serial.print(path);

			// Get a new directory object using the full path
			// and dive recursively into it.
			SdFile dir;
			if (!dir.open(parent, lfilename, O_READ)) {
				if (lsAction == LS_SerialPrint) {
					SERIAL_ECHO_START;
					SERIAL_ECHOLN(MSG_SD_CANT_OPEN_SUBDIR);
					SERIAL_ECHOLN(lfilename);
				}
			}
			lsDive(path, dir);
			// close() is done automatically by destructor of SdFile
		}
		else {
			uint8_t pn0 = p.name[0];
			if (pn0 == DIR_NAME_FREE) break;
			if (pn0 == DIR_NAME_DELETED || pn0 == '.') continue;
			if (longFilename[0] == '.') continue;

			if (!DIR_IS_FILE_OR_SUBDIR(&p)) continue;

			filenameIsDir = DIR_IS_SUBDIR(&p);

			if (!filenameIsDir && (p.name[8] != 'G' || p.name[9] == '~')) continue;

			switch (lsAction) {
				case LS_Count:
				nrFiles++;
				break;
				case LS_SerialPrint:
				createFilename(filename, p);
				SERIAL_PROTOCOL(prepend);
				SERIAL_PROTOCOLLN(filename);
				break;
				case LS_GetFilename:
				createFilename(filename, p);
				if (match != NULL) {
					if (strcasecmp(match, filename) == 0) return;
				}
				else if (cnt == nrFiles) return;
				cnt++;
				break;
			}

		}
	} // while readDir
}

void CardReader::ls()
{
	lsAction=LS_SerialPrint;
	if(lsAction==LS_Count)
	nrFiles=0;

	root.rewind();
	lsDive("",root);
}


void CardReader::initsd()
{
	cardOK = false;
	if(root.isOpen())
	root.close();
	#ifdef SDSLOW
	if (!card.init(SPI_HALF_SPEED,SDSS))
	#else
	if (!card.init(SPI_FULL_SPEED,SDSS))
	#endif
	{
		//if (!card.init(SPI_HALF_SPEED,SDSS))
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM(MSG_SD_INIT_FAIL);
	}
	else if (!volume.init(&card))
	{
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_SD_VOL_INIT_FAIL);
	}
	else if (!root.openRoot(&volume))
	{
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_SD_OPENROOT_FAIL);
	}
	else
	{
		cardOK = true;
		SERIAL_ECHO_START;
		SERIAL_ECHOLNPGM(MSG_SD_CARD_OK);
	}
	workDir=root;
	curDir=&root;
	/*
	if(!workDir.openRoot(&volume))
	{
	SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
	}
	*/
	
}

void CardReader::setroot()
{
	/*if(!workDir.openRoot(&volume))
	{
	SERIAL_ECHOLNPGM(MSG_SD_WORKDIR_FAIL);
	}*/
	workDir=root;
	
	curDir=&workDir;
}
void CardReader::release()
{
	sdprinting = false;
	cardOK = false;
}

void CardReader::startFileprint()
{
	if(cardOK)
	{
		SERIAL_PROTOCOLPGM("zprobe: ");
		Serial.println(zprobe_zoffset);
		sdprinting = true;
		//Rapduch
		sdispaused = false;
	}
}

void CardReader::pauseSDPrint()
{
	if(sdprinting)
	{
		sdprinting = false;
		//Rapduch
		sdispaused = true;
	}
}


void CardReader::openLogFile(char* name)
{
	logging = true;
	openFile(name, false);
}

void CardReader::getAbsFilename(char *t){
  uint8_t cnt = 0;
  *t = '/'; t++; cnt++;
  for (uint8_t i = 0; i < workDirDepth; i++) {
    workDirParents[i].getFilename(t); //SDBaseFile.getfilename!
    while (*t && cnt < MAXPATHNAMELENGTH) { t++; cnt++; } //crawl counter forward.
  }
  if (cnt < MAXPATHNAMELENGTH - (13))
    file.getFilename(t);
  else
    t[0] = 0;
}

void CardReader::openFile(char* name,bool read, bool replace_current/*=true*/)
{
	if(!cardOK)
	return;
	if(file.isOpen())  //replacing current file by new file, or subfile call
	{
		if(!replace_current)
		{
			if((int)file_subcall_ctr>(int)SD_PROCEDURE_DEPTH-1)
			{
				SERIAL_ERROR_START;
				SERIAL_ERRORPGM("trying to call sub-gcode files with too many levels. MAX level is:");
				SERIAL_ERRORLN(SD_PROCEDURE_DEPTH);
				kill();
				return;
			}
			
			SERIAL_ECHO_START;
			SERIAL_ECHOPGM("SUBROUTINE CALL target:\"");
			SERIAL_ECHO(name);
			SERIAL_ECHOPGM("\" parent:\"");
			
			//store current filename and position
			getAbsFilename(filenames[file_subcall_ctr]);
			
			SERIAL_ECHO(filenames[file_subcall_ctr]);
			SERIAL_ECHOPGM("\" pos");
			SERIAL_ECHOLN(sdpos);
			filespos[file_subcall_ctr]=sdpos;
			file_subcall_ctr++;
		}
		else
		{
			SERIAL_ECHO_START;
			SERIAL_ECHOPGM("Now doing file: ");
			SERIAL_ECHOLN(name);
		}
		file.close();
	}
	else //opening fresh file
	{
		file_subcall_ctr=0; //resetting procedure depth in case user cancels print while in procedure
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM("Now fresh file: ");
		SERIAL_ECHOLN(name);
	}
	sdprinting = false;
	
	
	SdFile myDir;
	curDir=&root;
	char *fname=name;
	
	char *dirname_start,*dirname_end;
	if(name[0]=='/')
	{
		dirname_start=strchr(name,'/')+1;
		while(dirname_start>0)
		{
			dirname_end=strchr(dirname_start,'/');
			//SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
			//SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
			if(dirname_end>0 && dirname_end>dirname_start)
			{
				char subdirname[13];
				strncpy(subdirname, dirname_start, dirname_end-dirname_start);
				subdirname[dirname_end-dirname_start]=0;
				SERIAL_ECHOLN(subdirname);
				if(!myDir.open(curDir,subdirname,O_READ))
				{
					SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
					SERIAL_PROTOCOL(subdirname);
					SERIAL_PROTOCOLLNPGM(".");
					return;
				}
				else
				{
					//SERIAL_ECHOLN("dive ok");
				}
				
				curDir=&myDir;
				dirname_start=dirname_end+1;
				if(!screen_sdcard){
					char cmd[30];
					char* c;
					
					lsAction=LS_Count;
					nrFiles=0;
					curDir->rewind();
					lsDive("",myDir);
					int16_t num_files;
					num_files = nrFiles;
					
					memset(cmd, '\0', sizeof(cmd) );
					sprintf(cmd, filename);
					for(c = &cmd[0]; *c; c++)
					{
						*c = tolower(*c);
					}
					int i=0;
					while(strcmp(cmd, dirname_start )!=0 && i< num_files){
						lsAction=LS_GetFilename;
						nrFiles=i;
						curDir->rewind();
						lsDive("",myDir);
						memset(cmd, '\0', sizeof(cmd) );
						sprintf(cmd, filename);
						for(c = &cmd[0]; *c; c++)
						{
							*c = tolower(*c);
						}
						i++;
					}
					nrFiles = 0;
				}
			}
			else // the reminder after all /fsa/fdsa/ is the filename
			{
				fname=dirname_start;
				//SERIAL_ECHOLN("remaider");
				//SERIAL_ECHOLN(fname);
				break;
			}
			
		}
	}
	else //relative path
	{
		curDir=&workDir;
		
		if(!screen_sdcard){
			char cmd[30];
			char* c;
			int16_t num_files;
			num_files = getnrfilenames();
			memset(cmd, '\0', sizeof(cmd) );
			sprintf(cmd, filename);
			for(c = &cmd[0]; *c; c++)
			{
				*c = tolower(*c);
			}
			int i=0;
			while(strcmp(cmd, fname)!=0 && i< num_files){
				getfilename(i);
				memset(cmd, '\0', sizeof(cmd) );
				sprintf(cmd, filename);
				for(c = &cmd[0]; *c; c++)
				{
					*c = tolower(*c);
				}
				i++;
			}
			nrFiles = 0;
		}
	}
	if(read)
	{
		if (file.open(curDir, fname, O_READ))
		{
			filesize = file.fileSize();
			SERIAL_PROTOCOLPGM(MSG_SD_FILE_OPENED);
			SERIAL_PROTOCOL(fname);
			SERIAL_PROTOCOLPGM(MSG_SD_SIZE);
			SERIAL_PROTOCOLLN(filesize);
			sdpos = 0;
			
			
			SERIAL_PROTOCOLLNPGM(MSG_SD_FILE_SELECTED);
			lcd_setstatus(fname);
			
		}
		else
		{
			SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
			SERIAL_PROTOCOL(fname);
			SERIAL_PROTOCOLLNPGM(".");
		}
	}
	else
	{ //write
		if (!file.open(curDir, fname, O_CREAT | O_APPEND | O_WRITE | O_TRUNC))
		{
			SERIAL_PROTOCOLPGM(MSG_SD_OPEN_FILE_FAIL);
			SERIAL_PROTOCOL(fname);
			SERIAL_PROTOCOLLNPGM(".");
		}
		else
		{
			saving = true;
			SERIAL_PROTOCOLPGM(MSG_SD_WRITE_TO_FILE);
			SERIAL_PROTOCOLLN(name);
			lcd_setstatus(fname);
		}
	}
	
}

void CardReader::removeFile(char* name)
{
	if(!cardOK)
	return;
	file.close();
	sdprinting = false;
	
	
	SdFile myDir;
	curDir=&root;
	char *fname=name;
	
	char *dirname_start,*dirname_end;
	if(name[0]=='/')
	{
		dirname_start=strchr(name,'/')+1;
		while(dirname_start>0)
		{
			dirname_end=strchr(dirname_start,'/');
			//SERIAL_ECHO("start:");SERIAL_ECHOLN((int)(dirname_start-name));
			//SERIAL_ECHO("end  :");SERIAL_ECHOLN((int)(dirname_end-name));
			if(dirname_end>0 && dirname_end>dirname_start)
			{
				char subdirname[13];
				strncpy(subdirname, dirname_start, dirname_end-dirname_start);
				subdirname[dirname_end-dirname_start]=0;
				SERIAL_ECHOLN(subdirname);
				if(!myDir.open(curDir,subdirname,O_READ))
				{
					SERIAL_PROTOCOLPGM("open failed, File: ");
					SERIAL_PROTOCOL(subdirname);
					SERIAL_PROTOCOLLNPGM(".");
					return;
				}
				else
				{
					//SERIAL_ECHOLN("dive ok");
				}
				
				curDir=&myDir;
				dirname_start=dirname_end+1;
			}
			else // the reminder after all /fsa/fdsa/ is the filename
			{
				fname=dirname_start;
				//SERIAL_ECHOLN("remaider");
				//SERIAL_ECHOLN(fname);
				break;
			}
			
		}
	}
	else //relative path
	{
		curDir=&workDir;
	}
	if (file.remove(curDir, fname))
	{
		SERIAL_PROTOCOLPGM("File deleted:");
		SERIAL_PROTOCOLLN(fname);
		sdpos = 0;
	}
	else
	{
		SERIAL_PROTOCOLPGM("Deletion failed, File: ");
		SERIAL_PROTOCOL(fname);
		SERIAL_PROTOCOLLNPGM(".");
	}
	
}

void CardReader::getStatus()
{
	if(cardOK){
		SERIAL_PROTOCOLPGM(MSG_SD_PRINTING_BYTE);
		SERIAL_PROTOCOL(sdpos);
		SERIAL_PROTOCOLPGM("/");
		SERIAL_PROTOCOLLN(filesize);
	}
	else{
		SERIAL_PROTOCOLLNPGM(MSG_SD_NOT_PRINTING);
	}
}
void CardReader::write_command(char *buf)
{
	char* begin = buf;
	char* npos = 0;
	char* end = buf + strlen(buf) - 1;

	file.writeError = false;
	if((npos = strchr(buf, 'N')) != NULL)
	{
		begin = strchr(npos, ' ') + 1;
		end = strchr(npos, '*') - 1;
	}
	end[1] = '\r';
	end[2] = '\n';
	end[3] = '\0';
	file.write(begin);
	if (file.writeError)
	{
		SERIAL_ERROR_START;
		SERIAL_ERRORLNPGM(MSG_SD_ERR_WRITE_TO_FILE);
	}
}


void CardReader::checkautostart(bool force)
{
	if(!force)
	{
		if(!autostart_stilltocheck)
		return;
		if(autostart_atmillis<millis())
		return;
	}
	autostart_stilltocheck=false;
	if(!cardOK)
	{
		initsd();
		if(!cardOK) //fail
		return;
	}
	
	char autoname[30];
	sprintf_P(autoname, PSTR("auto%i.g"), lastnr);
	for(int8_t i=0;i<(int8_t)strlen(autoname);i++)
	autoname[i]=tolower(autoname[i]);
	dir_t p;

	root.rewind();
	
	bool found=false;
	while (root.readDir(p, NULL) > 0)
	{
		for(int8_t i=0;i<(int8_t)strlen((char*)p.name);i++)
		p.name[i]=tolower(p.name[i]);
		//Serial.print((char*)p.name);
		//Serial.print(" ");
		//Serial.println(autoname);
		if(p.name[9]!='~') //skip safety copies
		if(strncmp((char*)p.name,autoname,5)==0)
		{
			char cmd[30];

			sprintf_P(cmd, PSTR("M23 %s"), autoname);
			enquecommand(cmd);
			enquecommand_P(PSTR("M24"));
			found=true;
		}
	}
	if(!found)
	lastnr=-1;
	else
	lastnr++;
}

void CardReader::closefile(bool store_location)
{
	file.sync();
	file.close();
	saving = false;
	logging = false;
	
	if(store_location)
	{
		
		
		#ifdef RECOVERY_PRINT

		saved_x_position =  current_position[X_AXIS];
		saved_y_position =  current_position[Y_AXIS];
		saved_z_position =  current_position[Z_AXIS];
		saved_tool_active = active_extruder;
		saved_e_position =  current_position[E_AXIS];
		saved_fileposition = file.curPosition();
		saved_temp0 = target_temperature[0];
		saved_temp1 = target_temperature[1];
		saved_tempbed  = target_temperature_bed;
		saved_feedmulti = feedmultiply;
		saved_fanlayer = fanSpeed;
		saved_Flag_fanSpeed_mirror = Flag_fanSpeed_mirror;
		saved_workDir_vector_lenght = workDir_vector_lenght;
		saved_workDir_vector[0] = workDir_vector[0];
		saved_workDir_vector[1] = workDir_vector[1];
		saved_workDir_vector[2] = workDir_vector[2];
		saved_workDir_vector[3] = workDir_vector[3];
		saved_workDir_vector[4] = workDir_vector[4];
		saved_workDir_vector[5] = workDir_vector[5];
		saved_workDir_vector[6] = workDir_vector[6];
		saved_workDir_vector[7] = workDir_vector[7];
		saved_workDir_vector[8] = workDir_vector[8];
		saved_workDir_vector[9] = workDir_vector[9];
		saved_dual_x_carriage_mode = get_dual_x_carriage_mode();
		saved_duplicate_extruder_x_offset = get_duplicate_extruder_x_offset();
		
		SERIAL_PROTOCOLLNPGM("SAVED PRINT");
		#endif
		
		
		//future: store printer state, filename and position for continuing a stopped print
		// so one can unplug the printer and continue printing the next day.
		
	}

	
}

void CardReader::getfilename(uint16_t nr, const char * const match/*=NULL*/)
{
	curDir=&workDir;
	lsAction=LS_GetFilename;
	nrFiles=nr;
	curDir->rewind();
	lsDive("", *curDir, match);
	
}

uint16_t CardReader::getnrfilenames()
{
	curDir=&workDir;
	lsAction=LS_Count;
	nrFiles=0;
	curDir->rewind();
	lsDive("",*curDir);
	//SERIAL_ECHOLN(nrFiles);
	return nrFiles;
}

int CardReader::chdir(const char * relpath)
{
	SdFile newfile;
	SdFile *parent=&root;
	//char Workdir[256];
	//memset(Workdir, '\0', sizeof(Workdir));
	//strcat(Workdir,"/");
	
	
	if(workDir.isOpen())
	parent=&workDir;
	
	/*for (int d = workDirDepth-1; 0 != d; d--){
	strcat(Workdir,getworkDirParentsName(d));
	strcat(Workdir,"/");
	}
	strcat(Workdir, relpath);*/
	if(!newfile.open(*parent,relpath, O_READ))
	{
		SERIAL_ECHO_START;
		SERIAL_ECHOPGM(MSG_SD_CANT_ENTER_SUBDIR);
		SERIAL_ECHOLN(relpath);
		return -1;
	}
	else
	{
		if (workDirDepth < MAX_DIR_DEPTH) {
			for (int d = ++workDirDepth; d--;)
			workDirParents[d+1] = workDirParents[d];
			workDirParents[0]=*parent;
		}
		workDir=newfile;
	}
}

int CardReader::updir()
{
	if(workDirDepth > 0)
	{
		--workDirDepth;
		workDir = workDirParents[0];
		int d;
		for (int d = 0; d < workDirDepth; d++)
		workDirParents[d] = workDirParents[d+1];
		
		if(workDirDepth == 0){
			return 1;
		}
		return 0;
	}
	else{
		return -1;
	}
}


void CardReader::printingHasFinished()
{
	doblocking=false;
	log_prints_finished++;
	Config_StoreSettings();
	acceleration = acceleration_old;
	enquecommand_P(PSTR("M107"));
	st_synchronize();
	if(file_subcall_ctr>0) //heading up to a parent file that called current as a procedure.
	{
		file.close();
		file_subcall_ctr--;
		openFile(filenames[file_subcall_ctr],true,true);
		setIndex(filespos[file_subcall_ctr]);
		startFileprint();
	}
	else
	{
		quickStop();
		file.close();
		sdprinting = false;
		//Rapduch
		#ifdef SIGMA_TOUCH_SCREEN
		//also we need to put the platform down and do an autohome to prevent blocking
		genie.WriteObject(GENIE_OBJ_FORM,FORM_MAIN,0);
		enquecommand_P(PSTR("T0"));
		st_synchronize();
		enquecommand_P(PSTR("M107"));
		set_dual_x_carriage_mode(DEFAULT_DUAL_X_CARRIAGE_MODE);
		setTargetHotend0(0);
		setTargetHotend1(0);
		setTargetBed(0);
		Flag_fanSpeed_mirror=0;
		saved_print_smartpurge_flag = false;
		screen_sdcard = false;
		surfing_utilities=false;
		surfing_temps = false;
		log_hours_lastprint = (int)(log_min_print/60);
		log_minutes_lastprint = (int)(log_min_print%60);
		log_X0_mmdone += x0mmdone/axis_steps_per_unit[X_AXIS];
		log_X1_mmdone += x1mmdone/axis_steps_per_unit[X_AXIS];
		log_Y_mmdone += ymmdone/axis_steps_per_unit[Y_AXIS];
		log_E0_mmdone += e0mmdone/axis_steps_per_unit[E_AXIS];
		log_E1_mmdone += e1mmdone/axis_steps_per_unit[E_AXIS];
		x0mmdone = 0;
		x1mmdone = 0;
		ymmdone = 0;
		e0mmdone = 0;
		e1mmdone = 0;
		Config_StoreSettings();
		//The default states is Left Extruder active
		#endif
		if(SD_FINISHED_STEPPERRELEASE)
		{
			//finishAndDisableSteppers();
			enquecommand_P(PSTR(SD_FINISHED_RELEASECOMMAND));
		}
		autotempShutdown();
	}
}

uint32_t CardReader::getFileSize()
{
	return filesize;
}


uint32_t CardReader::getSdPosition()
{
	return sdpos;
}


#endif //SDSUPPORT
