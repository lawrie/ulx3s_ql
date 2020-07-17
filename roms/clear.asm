DISPLAY EQU     $18063
	
	ORG	$0000

	DC.L	$30000		; Set stack to top of RAM
	DC.L    start		; Set PC to start

start:	MOVE.B	#$08,DISPLAY.L	; Set 256x256 mode
	MOVE.L	#$20000,A0	; VRAM address
	MOVE.W  #$3FFF,D0	; Full screen in words
loop:
	MOVE.W  #0, (A0)+	; Write to VRAM
	DBRA    D0, loop	; Loop until done
	STOP    #$2700



