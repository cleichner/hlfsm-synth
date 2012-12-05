INPUTS : a b c d e f g h
OUTPUTS : out1 out2
REGS : r1 r2 r3 r4 r5 r6

r1 = a + b
r2 = c + d
do {
    r3 = d + e
    r4 = d + d
    r5 = f + h
} while ( r4 )
r6 = r3 + r4
out1 = r6
out2 = r5

