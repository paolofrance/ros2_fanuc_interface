/PROG  FS_CLB_HND
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "FS CLB HAND";
PROG_SIZE	= 847;
CREATE		= DATE 21-01-20  TIME 14:09:32;
MODIFIED	= DATE 21-01-21  TIME 16:12:40;
FILE_NAME	= FS_DO_CA;
VERSION		= 0;
LINE_COUNT	= 22;
MEMORY_SIZE	= 1179;
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
   1:  ! Set hand mass only,[N] ;
   2:  CALL FSSETHAND(Constant) ;
   3:   ;
   4:J P[1] 10% FINE    ;
   5:  WAIT   5.00(sec) ;
   6:  CALL FS_GET_F    ;
   7:   ;
   8:  !J4_5deg ;
   9:  CALL FS_MOV_5DEG    ;
  10:   ;
  11:  !J4_15deg ;
  12:  CALL FS_MOV_15DEG    ;
  13:   ;
  14:  !J4_30deg ;
  15:  CALL FS_MOV_30DEG    ;
  16:   ;
  17:  !J4_45deg ;
  18:  CALL FS_MOV_45DEG    ;
  19:   ;
  20:J P[1] 10% FINE    ;
  21:  WAIT   5.00(sec) ;
  22:  CALL FS_GET_F    ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=     0.000000 deg,	J5=   -90.000000 deg,	J6=   180.000000 deg
};
/END
