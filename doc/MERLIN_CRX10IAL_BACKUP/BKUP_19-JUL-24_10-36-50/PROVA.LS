/PROG  PROVA
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 730;
CREATE		= DATE 24-03-26  TIME 12:35:16;
MODIFIED	= DATE 24-03-27  TIME 11:46:40;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 6;
MEMORY_SIZE	= 1078;
PROTECT		= READ_WRITE;
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

AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : FALSE;

PLTZ_MODE_HEADER;
 PLTZ_MODE_ENABLE   : FALSE;
 J4TURN  : ZERORAD;
 ORIENT  : DOWNWARDS;
/MN
   1:  FOR R[1]=1 TO 5 ;
   2:L P[1] 500mm/sec FINE    ;
   3:L P[2] 500mm/sec FINE    ;
   4:L P[3] 500mm/sec FINE    ;
   5:  ENDFOR ;
   6:L P[4] 500mm/sec FINE    ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -208.826981  mm,	Y =   371.101685  mm,	Z =   453.817596  mm,
	W =   178.323120 deg,	P =    -1.130688 deg,	R =    93.158020 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -210.158646  mm,	Y =   792.454834  mm,	Z =   477.926086  mm,
	W =   178.323120 deg,	P =    -1.130708 deg,	R =    93.158020 deg
};
P[3]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   113.338654  mm,	Y =   591.013855  mm,	Z =   461.960632  mm,
	W =   178.323120 deg,	P =    -1.130707 deg,	R =    93.158020 deg
};
P[4]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -209.236191  mm,	Y =   500.572601  mm,	Z =   461.225525  mm,
	W =   178.323120 deg,	P =    -1.130690 deg,	R =    93.158020 deg
};
/END
