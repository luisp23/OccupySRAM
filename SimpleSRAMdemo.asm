; This program uses the SLOW_SRAM device to read consecutive
;  values from the DE2 external SRAM chip.  Values are read
;  and displayed to the LEDs 10 times per second.
; SRAM contains random data after a cold start (complete
;  power down and power up), so you should see random LEDs
;  turning on.
; This program also includes...
; - Several subroutines (ATAN2, Neg, Abs, mult, div).
; - Some useful constants (masks, numbers, robot stuff, etc.)

ORG 0

;***************************************************************
;* Main code
;***************************************************************
Main:
	; Initialize values
	LOADI  0      ; New instruction, LOADI, "load immediate"
	; Set the starting address to 0.  Note that these are
	; variables WITHIN SCOMP's MEMORY, not the SRAM address
	; within the I/O peripheral.
	STORE  SRAM_addr_low
	STORE  SRAM_addr_high
	; Set the SRAM controller to a safe state
	LOADI   &B11
	OUT		SRAM_CTRL   ; 11 = no write, output disabled
	
	;Added code
	LOAD	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	1
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	2
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	3
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	4
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	5
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	6
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	7
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	8
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	9
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	10
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	11
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	12
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	13
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	14
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	15
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	16
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	17
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	18
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	19
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOAD	SRAM_addr_low
	ADDI	1
	STORE	SRAM_addr_low
	OUT		SRAM_ADLOW
	LOAD   	SRAM_addr_high
	OUT    	SRAM_ADHI
	LOADI   &B01
	OUT		SRAM_CTRL
	LOADI	20
	OUT		SRAM_DATA
	LOADI	&B11
	OUT		SRAM_CTRL
	
	LOADI  0
	STORE  SRAM_addr_low
	STORE  SRAM_addr_high
ReadLoop:
	; Send the desired address to the peripheral
	LOAD   SRAM_addr_low
	OUT    SRAM_ADLOW
	OUT    SSEG2        ; debugging / user feedback
	LOAD   SRAM_addr_high
	OUT    SRAM_ADHI
	OUT    SSEG1        ; debugging / user feedback
	; Configure the control signals for a read
	LOADI   &B10
	OUT		SRAM_CTRL   ; 01 = no write, output enabled
	; Get the data from the I/O peripheral
	IN      SRAM_DATA
	; Disable SRAM output to be extremely safe
	LOADI   &B11
	OUT		SRAM_CTRL   ; 01 = no write, output disabled

	; Get the data from the SRAM
	IN      SRAM_DATA
	
	; Display the data that was read
	OUT     LEDS
	
	; Use the timer to delay for 5/10 second
	OUT     TIMER
PauseLoop:
	IN      TIMER
	ADDI    -5
	JNEG    PauseLoop
	
	; Increment the address
	LOAD    SRAM_addr_low
	ADDI    1
	STORE   SRAM_addr_low
	JZERO   IncAddrHigh  ; Check for low address overflow
	JUMP    ReadLoop
IncAddrHigh:
	LOAD    SRAM_addr_high
	ADDI    1
	STORE   SRAM_addr_high
	JUMP    ReadLoop


; Variables for holding information related to the SRAM control
SRAM_addr_low: DW 0
SRAM_addr_high: DW 0


;*******************************************************************************
; Mod360: modulo 360
; Returns AC%360 in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Mod360:
	; easy modulo: subtract 360 until negative then add 360 until not negative
	JNEG   M360N
	ADDI   -360
	JUMP   Mod360
M360N:
	ADDI   360
	JNEG   M360N
	RETURN

;*******************************************************************************
; Abs: 2's complement absolute value
; Returns abs(AC) in AC
; Neg: 2's complement negation
; Returns -AC in AC
; Written by Kevin Johnson.  No licence or copyright applied.
;*******************************************************************************
Abs:
	JPOS   Abs_r
Neg:
	XOR    NegOne       ; Flip all bits
	ADDI   1            ; Add one (i.e. negate number)
Abs_r:
	RETURN

;******************************************************************************;
; Atan2: 4-quadrant arctangent calculation                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Original code by Team AKKA, Spring 2015.                                     ;
; Based on methods by Richard Lyons                                            ;
; Code updated by Kevin Johnson to use software mult and div                   ;
; No license or copyright applied.                                             ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; To use: store dX and dY in global variables AtanX and AtanY.                 ;
; Call Atan2                                                                   ;
; Result (angle [0,359]) is returned in AC                                     ;
; ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ ;
; Requires additional subroutines:                                             ;
; - Mult16s: 16x16->32bit signed multiplication                                ;
; - Div16s: 16/16->16R16 signed division                                       ;
; - Abs: Absolute value                                                        ;
; Requires additional constants:                                               ;
; - One:     DW 1                                                              ;
; - NegOne:  DW 0                                                              ;
; - LowByte: DW &HFF                                                           ;
;******************************************************************************;
Atan2:
	LOAD   AtanY
	CALL   Abs          ; abs(y)
	STORE  AtanT
	LOAD   AtanX        ; abs(x)
	CALL   Abs
	SUB    AtanT        ; abs(x) - abs(y)
	JNEG   A2_sw        ; if abs(y) > abs(x), switch arguments.
	LOAD   AtanX        ; Octants 1, 4, 5, 8
	JNEG   A2_R3
	CALL   A2_calc      ; Octants 1, 8
	JNEG   A2_R1n
	RETURN              ; Return raw value if in octant 1
A2_R1n: ; region 1 negative
	ADDI   360          ; Add 360 if we are in octant 8
	RETURN
A2_R3: ; region 3
	CALL   A2_calc      ; Octants 4, 5            
	ADDI   180          ; theta' = theta + 180
	RETURN
A2_sw: ; switch arguments; octants 2, 3, 6, 7 
	LOAD   AtanY        ; Swap input arguments
	STORE  AtanT
	LOAD   AtanX
	STORE  AtanY
	LOAD   AtanT
	STORE  AtanX
	JPOS   A2_R2        ; If Y positive, octants 2,3
	CALL   A2_calc      ; else octants 6, 7
	CALL   Neg          ; Negatge the number
	ADDI   270          ; theta' = 270 - theta
	RETURN
A2_R2: ; region 2
	CALL   A2_calc      ; Octants 2, 3
	CALL   Neg          ; negate the angle
	ADDI   90           ; theta' = 90 - theta
	RETURN
A2_calc:
	; calculates R/(1 + 0.28125*R^2)
	LOAD   AtanY
	STORE  d16sN        ; Y in numerator
	LOAD   AtanX
	STORE  d16sD        ; X in denominator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  AtanRatio
	STORE  m16sA
	STORE  m16sB
	CALL   A2_mult      ; X^2
	STORE  m16sA
	LOAD   A2c
	STORE  m16sB
	CALL   A2_mult
	ADDI   256          ; 256/256+0.28125X^2
	STORE  d16sD
	LOAD   AtanRatio
	STORE  d16sN        ; Ratio in numerator
	CALL   A2_div       ; divide
	LOAD   dres16sQ     ; get the quotient (remainder ignored)
	STORE  m16sA        ; <= result in radians
	LOAD   A2cd         ; degree conversion factor
	STORE  m16sB
	CALL   A2_mult      ; convert to degrees
	STORE  AtanT
	SHIFT  -7           ; check 7th bit
	AND    One
	JZERO  A2_rdwn      ; round down
	LOAD   AtanT
	SHIFT  -8
	ADDI   1            ; round up
	RETURN
A2_rdwn:
	LOAD   AtanT
	SHIFT  -8           ; round down
	RETURN
A2_mult: ; multiply, and return bits 23..8 of result
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8            ; move high word of result up 8 bits
	STORE  mres16sH
	LOAD   mres16sL
	SHIFT  -8           ; move low word of result down 8 bits
	AND    LowByte
	OR     mres16sH     ; combine high and low words of result
	RETURN
A2_div: ; 16-bit division scaled by 256, minimizing error
	LOADI  9            ; loop 8 times (256 = 2^8)
	STORE  AtanT
A2_DL:
	LOAD   AtanT
	ADDI   -1
	JPOS   A2_DN        ; not done; continue shifting
	CALL   Div16s       ; do the standard division
	RETURN
A2_DN:
	STORE  AtanT
	LOAD   d16sN        ; start by trying to scale the numerator
	SHIFT  1
	XOR    d16sN        ; if the sign changed,
	JNEG   A2_DD        ; switch to scaling the denominator
	XOR    d16sN        ; get back shifted version
	STORE  d16sN
	JUMP   A2_DL
A2_DD:
	LOAD   d16sD
	SHIFT  -1           ; have to scale denominator
	STORE  d16sD
	JUMP   A2_DL
AtanX:      DW 0
AtanY:      DW 0
AtanRatio:  DW 0        ; =y/x
AtanT:      DW 0        ; temporary value
A2c:        DW 72       ; 72/256=0.28125, with 8 fractional bits
A2cd:       DW 14668    ; = 180/pi with 8 fractional bits

;*******************************************************************************
; Mult16s:  16x16 -> 32-bit signed multiplication
; Based on Booth's algorithm.
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: does not work with factor B = -32768 (most-negative number).
; To use:
; - Store factors in m16sA and m16sB.
; - Call Mult16s
; - Result is stored in mres16sH and mres16sL (high and low words).
;*******************************************************************************
Mult16s:
	LOADI  0
	STORE  m16sc        ; clear carry
	STORE  mres16sH     ; clear result
	LOADI  16           ; load 16 to counter
Mult16s_loop:
	STORE  mcnt16s      
	LOAD   m16sc        ; check the carry (from previous iteration)
	JZERO  Mult16s_noc  ; if no carry, move on
	LOAD   mres16sH     ; if a carry, 
	ADD    m16sA        ;  add multiplicand to result H
	STORE  mres16sH
Mult16s_noc: ; no carry
	LOAD   m16sB
	AND    One          ; check bit 0 of multiplier
	STORE  m16sc        ; save as next carry
	JZERO  Mult16s_sh   ; if no carry, move on to shift
	LOAD   mres16sH     ; if bit 0 set,
	SUB    m16sA        ;  subtract multiplicand from result H
	STORE  mres16sH
Mult16s_sh:
	LOAD   m16sB
	SHIFT  -1           ; shift result L >>1
	AND    c7FFF        ; clear msb
	STORE  m16sB
	LOAD   mres16sH     ; load result H
	SHIFT  15           ; move lsb to msb
	OR     m16sB
	STORE  m16sB        ; result L now includes carry out from H
	LOAD   mres16sH
	SHIFT  -1
	STORE  mres16sH     ; shift result H >>1
	LOAD   mcnt16s
	ADDI   -1           ; check counter
	JPOS   Mult16s_loop ; need to iterate 16 times
	LOAD   m16sB
	STORE  mres16sL     ; multiplier and result L shared a word
	RETURN              ; Done
c7FFF: DW &H7FFF
m16sA: DW 0 ; multiplicand
m16sB: DW 0 ; multipler
m16sc: DW 0 ; carry
mcnt16s: DW 0 ; counter
mres16sL: DW 0 ; result low
mres16sH: DW 0 ; result high

;*******************************************************************************
; Div16s:  16/16 -> 16 R16 signed division
; Written by Kevin Johnson.  No licence or copyright applied.
; Warning: results undefined if denominator = 0.
; To use:
; - Store numerator in d16sN and denominator in d16sD.
; - Call Div16s
; - Result is stored in dres16sQ and dres16sR (quotient and remainder).
; Requires Abs subroutine
;*******************************************************************************
Div16s:
	LOADI  0
	STORE  dres16sR     ; clear remainder result
	STORE  d16sC1       ; clear carry
	LOAD   d16sN
	XOR    d16sD
	STORE  d16sS        ; sign determination = N XOR D
	LOADI  17
	STORE  d16sT        ; preload counter with 17 (16+1)
	LOAD   d16sD
	CALL   Abs          ; take absolute value of denominator
	STORE  d16sD
	LOAD   d16sN
	CALL   Abs          ; take absolute value of numerator
	STORE  d16sN
Div16s_loop:
	LOAD   d16sN
	SHIFT  -15          ; get msb
	AND    One          ; only msb (because shift is arithmetic)
	STORE  d16sC2       ; store as carry
	LOAD   d16sN
	SHIFT  1            ; shift <<1
	OR     d16sC1       ; with carry
	STORE  d16sN
	LOAD   d16sT
	ADDI   -1           ; decrement counter
	JZERO  Div16s_sign  ; if finished looping, finalize result
	STORE  d16sT
	LOAD   dres16sR
	SHIFT  1            ; shift remainder
	OR     d16sC2       ; with carry from other shift
	SUB    d16sD        ; subtract denominator from remainder
	JNEG   Div16s_add   ; if negative, need to add it back
	STORE  dres16sR
	LOADI  1
	STORE  d16sC1       ; set carry
	JUMP   Div16s_loop
Div16s_add:
	ADD    d16sD        ; add denominator back in
	STORE  dres16sR
	LOADI  0
	STORE  d16sC1       ; clear carry
	JUMP   Div16s_loop
Div16s_sign:
	LOAD   d16sN
	STORE  dres16sQ     ; numerator was used to hold quotient result
	LOAD   d16sS        ; check the sign indicator
	JNEG   Div16s_neg
	RETURN
Div16s_neg:
	LOAD   dres16sQ     ; need to negate the result
	CALL   Neg
	STORE  dres16sQ
	RETURN	
d16sN: DW 0 ; numerator
d16sD: DW 0 ; denominator
d16sS: DW 0 ; sign value
d16sT: DW 0 ; temp counter
d16sC1: DW 0 ; carry value
d16sC2: DW 0 ; carry value
dres16sQ: DW 0 ; quotient result
dres16sR: DW 0 ; remainder result

;*******************************************************************************
; L2Estimate:  Pythagorean distance estimation
; Written by Kevin Johnson.  No license or copyright applied.
; Warning: this is *not* an exact function.  I think it's most wrong
; on the axes, and maybe at 45 degrees.
; To use:
; - Store X and Y offset in L2X and L2Y.
; - Call L2Estimate
; - Result is returned in AC.
; Result will be in same units as inputs.
; Requires Abs and Mult16s subroutines.
;*******************************************************************************
L2Estimate:
	; take abs() of each value, and find the largest one
	LOAD   L2X
	CALL   Abs
	STORE  L2T1
	LOAD   L2Y
	CALL   Abs
	SUB    L2T1
	JNEG   GDSwap    ; swap if needed to get largest value in X
	ADD    L2T1
CalcDist:
	; Calculation is max(X,Y)*0.961+min(X,Y)*0.406
	STORE  m16sa
	LOADI  246       ; max * 246
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	STORE  L2T3
	LOAD   L2T1
	STORE  m16sa
	LOADI  104       ; min * 104
	STORE  m16sB
	CALL   Mult16s
	LOAD   mres16sH
	SHIFT  8
	STORE  L2T2
	LOAD   mres16sL
	SHIFT  -8        ; / 256
	AND    LowByte
	OR     L2T2
	ADD    L2T3     ; sum
	RETURN
GDSwap: ; swaps the incoming X and Y
	ADD    L2T1
	STORE  L2T2
	LOAD   L2T1
	STORE  L2T3
	LOAD   L2T2
	STORE  L2T1
	LOAD   L2T3
	JUMP   CalcDist
L2X:  DW 0
L2Y:  DW 0
L2T1: DW 0
L2T2: DW 0
L2T3: DW 0


; Subroutine to wait (block) for 1 second
Wait1:
	OUT    TIMER
Wloop:
	IN     TIMER
	OUT    XLEDS       ; User-feedback that a pause is occurring.
	ADDI   -10         ; 1 second at 10Hz.
	JNEG   Wloop
	RETURN

;***************************************************************
;* Variables
;***************************************************************
Temp:     DW 0 ; "Temp" is not a great name, but can be useful

;***************************************************************
;* Constants
;* (though there is nothing stopping you from writing to these)
;***************************************************************
NegOne:   DW -1
Zero:     DW 0
One:      DW 1
Two:      DW 2
Three:    DW 3
Four:     DW 4
Five:     DW 5
Six:      DW 6
Seven:    DW 7
Eight:    DW 8
Nine:     DW 9
Ten:      DW 10

; Some bit masks.
; Masks of multiple bits can be constructed by ORing these
; 1-bit masks together.
Mask0:    DW &B00000001
Mask1:    DW &B00000010
Mask2:    DW &B00000100
Mask3:    DW &B00001000
Mask4:    DW &B00010000
Mask5:    DW &B00100000
Mask6:    DW &B01000000
Mask7:    DW &B10000000
LowByte:  DW &HFF      ; binary 00000000 1111111
LowNibl:  DW &HF       ; 0000 0000 0000 1111

; some useful movement values
OneMeter: DW 961       ; ~1m in 1.04mm units
HalfMeter: DW 481      ; ~0.5m in 1.04mm units
Ft2:      DW 586       ; ~2ft in 1.04mm units
Ft3:      DW 879
Ft4:      DW 1172
Deg90:    DW 90        ; 90 degrees in odometer units
Deg180:   DW 180       ; 180
Deg270:   DW 270       ; 270
Deg360:   DW 360       ; can never actually happen; for math only
FSlow:    DW 100       ; 100 is about the lowest velocity value that will move
RSlow:    DW -100
FMid:     DW 350       ; 350 is a medium speed
RMid:     DW -350
FFast:    DW 500       ; 500 is almost max speed (511 is max)
RFast:    DW -500

MinBatt:  DW 140       ; 14.0V - minimum safe battery voltage
I2CWCmd:  DW &H1190    ; write one i2c byte, read one byte, addr 0x90
I2CRCmd:  DW &H0190    ; write nothing, read one byte, addr 0x90

DataArray:
	DW 0
;***************************************************************
;* IO address space map
;***************************************************************
SWITCHES: EQU &H00  ; slide switches
LEDS:     EQU &H01  ; red LEDs
TIMER:    EQU &H02  ; timer, usually running at 10 Hz
XIO:      EQU &H03  ; pushbuttons and some misc. inputs
SSEG1:    EQU &H04  ; seven-segment display (4-digits only)
SSEG2:    EQU &H05  ; seven-segment display (4-digits only)
LCD:      EQU &H06  ; primitive 4-digit LCD display
XLEDS:    EQU &H07  ; Green LEDs (and Red LED16+17)
BEEP:     EQU &H0A  ; Control the beep
CTIMER:   EQU &H0C  ; Configurable timer for interrupts
SRAM_CTRL: EQU &H10 ; write the two bits controlling SRAM function (bit 1 is write, bit 0 is output enable)
SRAM_DATA: EQU &H11 ; write the data to go to SRAM (before setting control bits) or read the data from SRAM (after setting bits)
SRAM_ADLOW: EQU &H12 ; write the lower 16 bits of the SRAM address (before setting control bits)
SRAM_ADHI: EQU &H13  ; write the upper 2 bits of the SRAM address (before setting control bits)
LPOS:     EQU &H80  ; left wheel encoder position (read only)
LVEL:     EQU &H82  ; current left wheel velocity (read only)
LVELCMD:  EQU &H83  ; left wheel velocity command (write only)
RPOS:     EQU &H88  ; same values for right wheel...
RVEL:     EQU &H8A  ; ...
RVELCMD:  EQU &H8B  ; ...
I2C_CMD:  EQU &H90  ; I2C module's CMD register,
I2C_DATA: EQU &H91  ; ... DATA register,
I2C_RDY:  EQU &H92  ; ... and BUSY register
UART_DAT: EQU &H98  ; UART data
UART_RDY: EQU &H99  ; UART status
SONAR:    EQU &HA0  ; base address for more than 16 registers....
DIST0:    EQU &HA8  ; the eight sonar distance readings
DIST1:    EQU &HA9  ; ...
DIST2:    EQU &HAA  ; ...
DIST3:    EQU &HAB  ; ...
DIST4:    EQU &HAC  ; ...
DIST5:    EQU &HAD  ; ...
DIST6:    EQU &HAE  ; ...
DIST7:    EQU &HAF  ; ...
SONALARM: EQU &HB0  ; Write alarm distance; read alarm register
SONARINT: EQU &HB1  ; Write mask for sonar interrupts
SONAREN:  EQU &HB2  ; register to control which sonars are enabled
XPOS:     EQU &HC0  ; Current X-position (read only)
YPOS:     EQU &HC1  ; Y-position
THETA:    EQU &HC2  ; Current rotational position of robot (0-359)
RESETPOS: EQU &HC3  ; write anything here to reset odometry to 0
RIN:      EQU &HC8
LIN:      EQU &HC9
IR_HI:    EQU &HD0  ; read the high word of the IR receiver (OUT will clear both words)
IR_LO:    EQU &HD1  ; read the low word of the IR receiver (OUT will clear both words)
