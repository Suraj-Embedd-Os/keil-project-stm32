POINTER 	RN 		R2 
ARRAY1		RN		R1


			AREA mycode,CODE,READONLY
			ENTRY
			EXPORT		__main


__main	
				LDR		ARRAY1,=isDATA
				LDRB	R4,[ARRAY1]
				
				MOV		POINTER,#1
				
				LDRB	R5,[ARRAY1,POINTER]
				
				MOV		POINTER,#2
				
				LDRB	R6,[ARRAY1,POINTER]
				
				MOV		POINTER,#3
				
				LDRB	R7,[ARRAY1,POINTER]
				

stop			B			stop


			
			
				
				AREA myData,DATA,READONLY
isDATA			DCB		0x45,0x24,0x18,0x63
				
				ALIGN
				END