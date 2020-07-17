DISPLAY	EQU	$18063
KEYROW1	EQU	$1808B
SOUND	EQU	$18009

	ORG	$0000

	DC.L	$30000		; Set stack to top of RAM
	DC.L    START		; Set PC to start

START
; Clear the screen
	MOVE.B	#$08,DISPLAY.L	; Set 8-color mode
	LEA	$20000,A0	; VRAM address of top half
	MOVE.W  #$1FFF,D0	; Clear too half of  screen to cyan
CLEAR	MOVE.W  #$AA55, (A0)+	; Write to VRAM
        DBRA    D0, CLEAR	; Loop until done
        LEA     $24000,A0       ; VRAM address of bottom half
	MOVE.W  #$1FFF,D0	; Clear bottom half of screen to yellow
CLEAR1	MOVE.W  #$AAAA, (A0)+	; Write to VRAM
        DBRA    D0, CLEAR1	; Loop until done
; Draw sprites
	MOVE.W	#61, D3		; Draw it 62 times
DRAW
	LEA	$23800,A0	; 16 pixels above bottom half
        MOVE.B  KEYROW1.L,D4    ; Get Keyrow 1
        BTST.L  #2,D4
	BEQ.S	NOJMP
	SUBA.L	#2048,A0
NOJMP	
	MOVE.W	#61,D4		; Get positive index 
	SUB.W	D3,D4
	LSL.W	#1,D4		; Double it
	ADDA.W	D4,A0		; Add it to screen address
	BSR	PLAY
	MOVE.L	A0,-(SP)	; Save address
	LEA	BACK,A1		; Read existing screen contents
	BSR	READVRAM
	MOVE.L	(SP),A0		; Restore screen address
	LEA	SPRITE1,A1	; Get address of sprite
	BSR	SPRITE		; Draw the sprite
	MOVE.W	#$FFFF,D2	; Delay
	DBRA	D2,$		
	MOVE.L	(SP)+,A0	; Restore screen address
	LEA	BACK,A1		; Restore background
	BSR	SPRITE16
	DBRA	D3,DRAW		; Continue to next position
	BSR	SOUNDOFF
	MOVE.W	#61, D3		; Draw it 62 times
DRAWBACK
	LEA	$2387C,A0	; 16 pixels above bottom half
        MOVE.B  KEYROW1.L,D4    ; Get Keyrow 1
        BTST.L  #2,D4
	BEQ.S	NOJMP1
	SUBA.L	#2048,A0
NOJMP1
	MOVE.W	#61,D4		; Get positive index 
	SUB.W	D3,D4
	LSL.W	#1,D4		; Double it		
	SUB.W	D4,A0		; Subtract it from screen address
	BSR	PLAY
	MOVE.L	A0,-(SP)	; Save address
	LEA	BACK,A1		; Read existing screen contents
	BSR	READVRAM
	MOVE.L	(SP),A0		; Restore screen address
	LEA	SPRITE1,A1	; Get address of sprite
	BSR.S	SPRFLIP		; Draw the sprite
	MOVE.W	#$FFFF,D2	; Delay
	DBRA	D2,$		
	MOVE.L	(SP)+,A0	; Restore screen address
	LEA	BACK,A1		; Restore background
	BSR	SPRITE16
	DBRA	D3,DRAWBACK	; Continue to next position
	LEA	$23800,A0
	LEA	SPRITE1,A1
	BSR.S	SPRITE
	BSR	SOUNDOFF
STOP
	STOP    #$2700

; A0 = screen address, A1 = sprite
; 8x8 sprite expanded to 8x16 on screen
; Byte-aligned only
SPRITE
	MOVE.W	#7,D0		; 8 rows in a sprite
LINE	MOVE.W	#3,D1		; 4 bytes, 8 pixels on a row
PIXEL2	MOVE.B	(A1)+,(A0)+     ; Write 2 pixels to screen
	DBRA	D1,PIXEL2       ; Loop until 8 pixels written
	SUBQ.L	#4, A1		; Go back to start of sprite row
	ADDA.L	#124,A0		; Move to next screen row
        MOVE.W	#3,D1		; Write second copy of row
PIX2A   MOVE.B  (A1)+,(A0)+	; For 8x16 sprite
	DBRA	D1,PIX2A
	ADDA.L	#124,A0		; Move to next line
	DBRA	D0,LINE
	RTS

; A0 = screen address, A1 = sprite
; 8x8 sprite expanded to 8x16 on screen
; Byte-aligned only
SPRFLIP
	MOVE.W	#7,D0		; 8 rows in a sprite
LINEF	BSR.S	FLIPLINE
	ADDA.L	#128,A0		; Move to next line
	BSR.S	FLIPLINE
	ADDA.L	#4,A1
	ADDA.L	#128,A0
	DBRA	D0,LINEF
	RTS

FLIPLINE
	MOVE.B  2(A1),D4
	BSR.S	FLIPPIX
	MOVE.B	D4,(A0)
	MOVE.B  3(A1), D4
	BSR.S	FLIPPIX
	MOVE.B  D4,1(A0)
	MOVE.B  (A1),D4
	BSR.S	FLIPPIX
	MOVE.B	D4,2(A0)
	MOVE.B  1(A1),D4
	BSR.S	FLIPPIX
	MOVE.B	D4,3(A0)
	RTS

FLIPPIX
	MOVE.B	D4,D5           ; Duplicate it
	MOVE.B  D4,D6
        MOVE.B  D4,D7
	AND.B   #$03,D4
	AND.B	#$0C,D5
	AND.B	#$30,D6
	AND.B	#$C0,D7
        LSL.B   #6,D4
	LSL.B	#2,D5
	LSR.B	#2,D6
	LSR.B	#6,D7
	OR.B	D5,D4
	OR.B	D6,D4
	OR.B	D7,D4
	RTS

; A0 = screen address, A1 = sprite
; 8x16 sprite 
; Byte-aligned only
SPRITE16
	MOVE.W	#15,D0		; 16 rows in a sprite
LINE16	MOVE.W	#3,D1		; 4 bytes, 8 pixels on a row
PIX16	MOVE.B	(A1)+,(A0)+     ; Write 2 pixels to screen
	DBRA	D1,PIX16	; Loop until 8 pixels written
	ADDA.L	#124,A0		; Move to next line
	DBRA	D0,LINE16
	RTS

; Read screen data for sprite
; A0 = screen address, A1 = storage area
READVRAM
	MOVE.W	#7,D0		; 8 rows in a sprite
RLINE	MOVE.W	#3,D1		; 4 bytes, 8-pixels on a row
RPIXEL2	MOVE.B	(A0)+,(A1)+     ; Read 2 pixels from screen
	DBRA	D1,RPIXEL2	; Loop until 8-pixels read
	ADDA.L	#124,A0		; Move to next screen row
        MOVE.W	#3,D1		; Read second copy of row
RPIX2A	MOVE.B  (A0)+,(A1)+	; For 8x16 sprite
	DBRA	D1,RPIX2A
	ADDA.L	#124,A0		; Move to next line
	DBRA	D0,RLINE
	RTS

PLAY	LSR	#1,D4
	AND.W	#$1F,D4
	CMP	#13,D4
	BEQ.S	NOSOUND
	CMP	#17,D4
	BEQ.S	NOSOUND
	CMP	#25,D4
	BEQ.S	NOSOUND
	BTST	#0,D4
	BEQ.S	NOSOUND
	LSR.W	#1,D4
	AND.B	#$F,D4
	CMP	#2,D4
	BCS.S	NOTE_E
	CMP	#3,D4
	BEQ.S	NOTE_E
	CMP	#5,D4
	BEQ.S	NOTE_C
	CMP	#8,D4
	BCS.S	NOTE_E
	CMP	#10,D4
	BCS.S	NOTE_G
	CMP	#12,D4
	BEQ.S	LOW_G
	CMP	#13,D4
	BEQ.S	LOW_G
NOSOUND
	BSR.S	SOUNDOFF
	RTS
NOTE_G
	MOVE.W	#62,D4
	BSR.S	TONE
	RTS
NOTE_C
	MOVE.W	#88,D4
	BSR.S	TONE
	RTS
NOTE_E
	MOVE.W	#72,D4
	BSR.S	TONE
	RTS
LOW_G
	MOVE.W	#124,D4
	BSR.S	TONE
	RTS

SOUNDOFF
        MOVE.B  #$9F,SOUND.L    ; Channel 1 off
	RTS
	
SOUNDON
        MOVE.B  #$90,SOUND.L    ; Channel 1 on full
	RTS
	
TONE    BSR	SOUNDON
	MOVE.B	D4,D5
	AND.B	#$F,D5
	OR.B	#$80,D5
	LSR.W	#4,D4
	AND.B	#$3F,D4
	MOVE.B  D5,SOUND.L    ; Set frequency for channel 0
        MOVE.B  D4,SOUND.L
	RTS

SPRITE1	
	DC.L	$800A0A05
	DC.L	$802AA0FA
	DC.L	$02020221
	DC.L	$02020221
	DC.L	$2020AAA5
	DC.L	$00960281
	DC.L	$80552A55
	DC.L	$0A050A05

	ORG	$28000		; RAM

BACK	DS.L	64

