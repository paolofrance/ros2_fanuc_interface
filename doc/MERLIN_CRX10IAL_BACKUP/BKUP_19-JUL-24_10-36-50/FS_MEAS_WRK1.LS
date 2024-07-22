/PROG  FS_MEAS_WRK1
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "FS MEAS WRK1";
PROG_SIZE	= 588;
CREATE		= DATE 21-01-20  TIME 14:09:32;
MODIFIED	= DATE 21-01-21  TIME 16:21:54;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 8;
MEMORY_SIZE	= 944;
PROTECT		= READ;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
LOCAL_REGISTERS	= 0,0,0;
/APPL
/MN
   1:  ! Init Work Grv ;
   2:L P[1] 200mm/sec CNT30 TB    .20sec,CALL FSGRVINIT    ;
   3:   ;
   4:  ! Load Workpiece here ;
   5:   ;
   6:   ;
   7:  ! Get Work Grv and output to Reg ;
   8:L P[2] 200mm/sec CNT30 TB    .20sec,CALL FSGRVMEAS(Constant)    ;
/POS
P[1]{
   GP1:
	UF : F, UT : F,		CONFIG : 'N D B, 0, 0, 0',
	X = ********  mm,	Y = ********  mm,	Z = ********  mm,
	W = ******** deg,	P = ******** deg,	R = ******** deg
};
P[2]{
   GP1:
	UF : F, UT : F,		CONFIG : 'N D B, 0, 0, 0',
	X = ********  mm,	Y = ********  mm,	Z = ********  mm,
	W = ******** deg,	P = ******** deg,	R = ******** deg
};
/END
