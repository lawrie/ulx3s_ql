DISPLAY EQU	$18063
	ORG	$0000

	DC.L	$30000		; Set stack to top of RAM
	DC.L    start		; Set PC to start

start:
	MOVE.B	#$00,DISPLAY.L	; Set 512x512 mode
	MOVE.L	#$20000,A0	; VRAM address
	MOVE.W  #$3FFF,D0	; Full screen in words
loop:
	MOVE.W  #$E4E4, (A0)+	; Write to VRAM WGRB
	DBRA    D0, loop	; Loop until done
	STOP    #$2700



