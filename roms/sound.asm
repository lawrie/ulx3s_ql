SOUND    EQU $18009

RDRF    EQU 0
TDRE    EQU 1

CR      EQU 13
LF      EQU 10
	ORG	$0000

	DC.L	$30000		; Set stack to top of RAM
	DC.L    START		; Set PC to start

START	
	MOVE.B	#$90,SOUND.L	; Set full volume on channel 0
LOOP	
	MOVE.B	#$3F,D0
	BSR.S	TONE
	MOVE.B	#$2F,D0
	BSR.S	TONE
	MOVE.B	#$1F,D0
	BSR.S	TONE
	MOVE.B	#$0F,D0
	BSR.S	TONE

	BRA.S	LOOP

STOP	STOP    #$2700

DELAY	MOVE.W	#$FFFF,D1
	DBRA	D1,$
	RTS

TONE    MOVE.B	#$80,SOUND.L	; Set frequency for channel 0
	MOVE.B	D0,SOUND.L
	MOVE.W  #$0010,D2
DELAY1
	BSR.S	DELAY
	DBRA	D2,DELAY1
	RTS

