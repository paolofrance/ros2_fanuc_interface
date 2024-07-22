/PROG  FS_MEAS_WRK2
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "FS MEAS WRK2";
PROG_SIZE	= 638;
CREATE		= DATE 21-01-20  TIME 14:09:30;
MODIFIED	= DATE 21-01-21  TIME 16:22:32;
FILE_NAME	= FS_MEAS_;
VERSION		= 0;
LINE_COUNT	= 10;
MEMORY_SIZE	= 986;
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
/APPL
/MN
   1:L P[1] 200mm/sec FINE    ;
   2:  ! Init Work Grv ;
   3:  CALL FSGRVINIT    ;
   4:   ;
   5:  ! Load Workpiece here ;
   6:   ;
   7:   ;
   8:L P[2] 200mm/sec FINE    ;
   9:  ! Get Work Grv and output to Reg ;
  10:  CALL FSGRVMEAS(Constant) ;
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
