/PROG  FS_MOV_45DEG
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "FS MOVE 45DEG";
PROG_SIZE	= 1696;
CREATE		= DATE 21-01-20  TIME 14:09:32;
MODIFIED	= DATE 21-01-21  TIME 10:26:08;
FILE_NAME	= FS_DO_CA;
VERSION		= 0;
LINE_COUNT	= 34;
MEMORY_SIZE	= 2068;
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
   1:  !J4_45DEG ;
   2:J P[1] 10% FINE    ;
   3:  WAIT   5.00(sec) ;
   4:  CALL FS_GET_F    ;
   5:J P[2] 10% FINE    ;
   6:  CALL FS_GET_F    ;
   7:J P[3] 10% FINE    ;
   8:  CALL FS_GET_F    ;
   9:J P[4] 10% FINE    ;
  10:  CALL FS_GET_F    ;
  11:J P[5] 10% FINE    ;
  12:  CALL FS_GET_F    ;
  13:J P[6] 10% FINE    ;
  14:  CALL FS_GET_F    ;
  15:J P[7] 10% FINE    ;
  16:  CALL FS_GET_F    ;
  17:J P[8] 10% FINE    ;
  18:  CALL FS_GET_F    ;
  19:J P[9] 10% FINE    ;
  20:  CALL FS_GET_F    ;
  21:J P[10] 10% FINE    ;
  22:  CALL FS_GET_F    ;
  23:J P[11] 10% FINE    ;
  24:  CALL FS_GET_F    ;
  25:J P[12] 10% FINE    ;
  26:  CALL FS_GET_F    ;
  27:J P[13] 10% FINE    ;
  28:  CALL FS_GET_F    ;
  29:J P[14] 10% FINE    ;
  30:  CALL FS_GET_F    ;
  31:J P[15] 10% FINE    ;
  32:  CALL FS_GET_F    ;
  33:J P[16] 10% FINE    ;
  34:  CALL FS_GET_F    ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=  -157.500000 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=  -135.000000 deg
};
P[3]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=  -112.500000 deg
};
P[4]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   -90.000000 deg
};
P[5]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   -67.500000 deg
};
P[6]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   -45.000000 deg
};
P[7]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   -22.500000 deg
};
P[8]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=     0.000000 deg
};
P[9]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=    22.500000 deg
};
P[10]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=    45.000000 deg
};
P[11]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=    67.500000 deg
};
P[12]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=    90.000000 deg
};
P[13]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   112.500000 deg
};
P[14]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   135.000000 deg
};
P[15]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   157.500000 deg
};
P[16]{
   GP1:
	UF : 0, UT : 1,	
	J1=     0.000000 deg,	J2=     0.000000 deg,	J3=     0.000000 deg,
	J4=    45.000000 deg,	J5=   -90.000000 deg,	J6=   180.000000 deg
};
/END
