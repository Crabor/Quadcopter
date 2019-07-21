			EXTERN		OSRunning
			EXTERN		OSPrioCur
			EXTERN		OSPrioHighRdy
			EXTERN		OSTCBCur
			EXTERN		OSTCBHighRdy
			EXTERN		OSIntNesting
			EXTERN		OSIntExit
			EXTERN		OSTaskSwHook

			EXPORT		OS_CPU_SR_Save 
			EXPORT		OS_CPU_SR_Restore
			EXPORT		OSStartHighRdy
			EXPORT		OSCtxSw
			EXPORT		OSIntCtxSw
			EXPORT		OS_CPU_PendSVHandler
		
NVIC_INT_CTRL		EQU		0xE000ED04
NVIC_SYSPRI14		EQU		0xE000ED22 
NVIC_PENDSV_PRI		EQU		0xFF 
NVIC_PENDSVSET		EQU		0x10000000

			AREA |.text|,CODE,READONLY,ALIGN=2
			THUMB
			REQUIRE8
			PRESERVE8
			
OS_CPU_SR_Save
			MRS 	R0,PRIMASK
			CPSID	I
			BX		LR
	
OS_CPU_SR_Restore
			MSR		PRIMASK,R0
			BX		LR
			
OSStartHighRdy
			;set PendSV prio
			LDR		R0,=NVIC_SYSPRI14	
			LDR 	R1,=NVIC_PENDSV_PRI
			STR		R1,[R0]
			
			;clear PSP
			MOV		R0,#0
			MSR		PSP,R0
			
			;OSRunning = true
			LDR		R0,=OSRunning
			MOV		R1,#1
			STR		R1,[R0]
			
			;pend(enable) PendSV interrupt
			LDR		R0,=NVIC_INT_CTRL
			LDR		R1,=NVIC_PENDSVSET
			STR		R1,[R0]
			
			;open interrupt
			CPSIE	I
	
OSCtxSw		;only need to enable PendSV interrupt
			LDR		R0,=NVIC_INT_CTRL
			LDR		R1,=NVIC_PENDSVSET
			STR		R1,[R0]
			
			BX		LR
	
OSIntCtxSw	;the same with OSCtxSw
			LDR		R0,=NVIC_INT_CTRL
			LDR		R1,=NVIC_PENDSVSET
			STR		R1,[R0]
			
			BX		LR
	
OS_CPU_PendSVHandler
			;close interrupt
			CPSID	I
			
			;judge if the PSP equal 0
			MRS		R0,PSP
			CBZ		R0,OSPendSV_nosave
			
			;save context
			STMFD	R0!,{R4-R11}
			;SUB	R0,R0,#0x20
			;STM	R0,{R4-R11}
			
			;OSTCBCur->OSTCBStkPtr = SP
			LDR		R1,=OSTCBCur
			LDR		R1,[R1]
			STR		R0,[R1]
	
OSPendSV_nosave
			;OSTaskSwHook()
			PUSH	{LR}
			LDR		R0,=OSTaskSwHook
			BLX		R0
			POP		{LR}
			
			;OSTCBCur=OSTCBHighRdy
			LDR		R0,=OSTCBHighRdy
			LDR		R1,=OSTCBCur
			LDR		R0,[R0]
			STR		R0,[R1]
			
			;OSPrioCur=OSPrioHighRdy
			LDR		R0,=OSPrioHighRdy
			LDR		R1,=OSPrioCur
			LDRB	R0,[R0]
			STRB	R0,[R1]
			
			;OSTCBHighRdy->OSTCBStkPtr
			LDR		R0,=OSTCBHighRdy
			LDR		R0,[R0]
			LDR		R1,[R0]
			
			;restore new task context
			LDMFD	R1!,{R4-R11}
			MSR		PSP,R1
			
			;ensure exception return uses process stack
			ORR		LR,LR,#0x04
			
			;open interrupt
			CPSIE	I
			
			;return
			BX		LR
			
			NOP
			END
