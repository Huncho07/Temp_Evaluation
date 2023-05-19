; macro per settare il valore di un registro (movlw + banksel + movwf)
setRegK			macro Reg, Const
		movlw Const
		banksel Reg
		movwf Reg
				endm

				
; macro per azzerare un registro (banksel + clrf)
setReg0			macro Reg
		banksel Reg
		clrf Reg
				endm
