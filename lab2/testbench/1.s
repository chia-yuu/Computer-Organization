        .data   0x10008000      # start of Dynamic Data (pointed by $gp)
hun:    .word   0x00114514      # 0($gp)
hah:    .word   0xf1919810
        .word   0x1
        .word   0x2
        .word   0x3             # 16($gp)

        .text   0x00400000

main:   add $t0, $gp, $gp       # $t0 = 2 * $gp
        sub $t1, $t0, $gp       # $t1 = $gp
        and $t2, $t0, $t1       # $t2 = $t0 & $t1
        or  $t3, $t0, $t1       # $t3 = $t0 | $t1
        slt $t4, $t1, $t0       # $t4 = $t1 < $t0 ? 1 : 0; $t4 = 1
        beq $t0, $t1, Label3    # should not execute
        bne $t4, $zero, Label2  # should branch

Label1: lw $t6, hun             # test LW
        sw $t6, 4($gp)          # test SW
        j  Label3               # test jump

Label2: j  Label1               # test jump

Label3: li $t6, 100              # $t6 = 100
