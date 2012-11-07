// should be able to handle comments
INPUTS  : a b c
OUTPUTS : z x
REGS    : d e f g xrin zrin

// more comments
d = a + b
e = a + c
g = d > e
zrin = g ? d : e // even more comments
f = a * c
xrin = f - d  
x = xrin
z = zrin
