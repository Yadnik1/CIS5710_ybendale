
uppercase.bin:     file format elf32-littleriscv


Disassembly of section .text:

00010074 <_start>:
   10074:	ffff2517          	auipc	a0,0xffff2
   10078:	f8c50513          	addi	a0,a0,-116 # 2000 <__DATA_BEGIN__>
   1007c:	06100313          	li	t1,97
   10080:	07a00393          	li	t2,122

00010084 <convert_loop>:
   10084:	00050283          	lb	t0,0(a0)
   10088:	02028063          	beqz	t0,100a8 <end_program>
   1008c:	0062ca63          	blt	t0,t1,100a0 <skip>
   10090:	0053c863          	blt	t2,t0,100a0 <skip>
   10094:	02000e13          	li	t3,32
   10098:	41c282b3          	sub	t0,t0,t3
   1009c:	00550023          	sb	t0,0(a0)

000100a0 <skip>:
   100a0:	00150513          	addi	a0,a0,1
   100a4:	fe1ff06f          	j	10084 <convert_loop>

000100a8 <end_program>:
   100a8:	0000006f          	j	100a8 <end_program>
