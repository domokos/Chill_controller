v 20121123 2
C 33300 62100 1 0 0 AT89C4051-1.sym
{
T 35800 66100 5 10 1 1 0 6 1
refdes=U1
T 33295 62095 5 10 0 1 0 0 1
footprint=DIP20
T 33295 62095 5 10 0 1 0 0 1
device=AT89C4051
}
C 31800 66000 1 180 0 74595-1.sym
{
T 31500 63160 5 10 0 0 180 0 1
device=74595
T 30100 63300 5 10 1 1 180 6 1
refdes=U2
T 31500 62950 5 10 0 0 180 0 1
footprint=DIP16
}
N 35400 61100 35400 62100 4
N 34600 61100 34600 62100 4
N 34600 61400 34700 61400 4
C 34900 59000 1 0 0 gnd-1.sym
N 34600 60200 35400 60200 4
N 35000 60200 35000 59300 4
N 34000 62100 34000 59700 4
C 34700 61300 1 0 0 CRYSTAL.sym
{
T 34900 61800 5 10 0 0 0 0 1
device=CRYSTAL
T 34900 62200 5 10 0 0 0 0 1
footprint=CRYSTAL 300
T 34900 61600 5 10 1 1 0 0 1
refdes=CR1
}
C 34800 60200 1 90 0 SMALLCERCAP.sym
{
T 34100 60400 5 10 0 0 90 0 1
device=SMALLCERCAP
T 33700 60400 5 10 0 0 90 0 1
footprint=ACY150
T 34500 60900 5 10 1 1 180 0 1
refdes=C1
T 34100 60400 5 10 1 1 0 0 1
value=33pF
}
C 35600 60200 1 90 0 SMALLCERCAP.sym
{
T 34900 60400 5 10 0 0 90 0 1
device=SMALLCERCAP
T 34500 60400 5 10 0 0 90 0 1
footprint=ACY150
T 35300 60900 5 10 1 1 180 0 1
refdes=C2
T 34900 60400 5 10 1 1 0 0 1
value=33pF
}
N 34000 59700 35000 59700 4
C 25900 66200 1 0 0 BS170.sym
{
T 26800 66600 5 10 0 0 0 0 1
device=BS170
T 26800 66400 5 10 0 0 0 0 1
footprint=TO92
T 25800 67000 5 10 1 1 0 0 1
refdes=Q1
}
C 37900 60600 1 0 0 BS170.sym
{
T 38800 61000 5 10 0 0 0 0 1
device=BS170
T 38800 60800 5 10 0 0 0 0 1
footprint=TO92
T 38800 61100 5 10 1 1 0 0 1
refdes=Q4
}
N 38500 59900 38500 60600 4
N 38500 61600 38500 61800 4
N 38500 61800 38900 61800 4
C 38900 62400 1 90 0 SMALLRES.sym
{
T 38250 62500 5 10 0 0 90 0 1
device=SMALLRES
T 38400 62500 5 10 0 0 90 0 1
footprint=ACY300
T 38600 63000 5 10 1 1 180 0 1
refdes=R1
T 38900 62700 5 10 1 1 90 0 1
value=1k
}
N 38900 62300 38800 62300 4
N 38800 62300 38800 62400 4
N 38800 63300 38800 63800 4
C 38700 61400 1 0 0 TLP3063.sym
{
T 39395 62800 5 10 0 0 0 0 1
device=TLP3063
T 39395 63000 5 10 0 0 0 0 1
footprint=DIL_6_300
T 39900 62600 5 10 1 1 0 0 1
refdes=U4
}
C 42400 61500 1 90 0 triac-1.sym
{
T 41500 61800 5 10 0 0 90 0 1
device=TRIAC
T 42400 61500 5 10 0 1 0 0 1
footprint=TO220_TRIAC
T 42000 62400 5 10 1 1 180 0 1
refdes=D1
}
N 41700 62200 41400 62200 4
N 41400 62200 41400 61700 4
N 41400 61700 40500 61700 4
C 42900 65400 1 0 0 flatio.sym
{
T 43500 66100 5 10 0 1 0 0 1
device=IO
T 42900 65400 5 10 0 1 0 0 1
footprint=FLATCONN
T 43200 65700 5 10 1 1 0 0 1
refdes=230V_L
}
N 40500 62400 40600 62400 4
N 40600 62400 40600 63000 4
N 40600 63000 40700 63000 4
N 41600 63000 42100 63000 4
C 41600 63100 1 180 0 SMALLRES.sym
{
T 41500 62450 5 10 0 0 180 0 1
device=SMALLRES
T 41500 62600 5 10 0 0 180 0 1
footprint=ACY300
T 41000 63200 5 10 1 1 0 0 1
refdes=R2
T 41300 63100 5 10 1 1 180 0 1
value=100
}
N 42900 64000 42700 64000 4
N 42700 64000 42700 61300 4
N 42700 61300 42100 61300 4
N 42100 61300 42100 61500 4
N 33300 65600 32500 65600 4
N 32500 65600 32500 64900 4
N 32500 64900 31800 64900 4
N 33300 63600 32500 63600 4
N 32500 63600 32500 63700 4
N 32500 63700 31800 63700 4
N 33300 64100 32500 64100 4
N 32500 64100 32500 64000 4
N 32500 64000 31800 64000 4
N 31800 64600 32700 64600 4
N 32700 64600 32700 65100 4
N 32700 65100 33300 65100 4
N 31800 64300 32900 64300 4
N 32900 64300 32900 64600 4
N 32900 64600 33300 64600 4
C 27100 66400 1 0 0 BS170.sym
{
T 28000 66800 5 10 0 0 0 0 1
device=BS170
T 28000 66600 5 10 0 0 0 0 1
footprint=TO92
T 27200 67200 5 10 1 1 0 0 1
refdes=Q2
}
C 28300 66800 1 0 0 BS170.sym
{
T 29200 67200 5 10 0 0 0 0 1
device=BS170
T 29200 67000 5 10 0 0 0 0 1
footprint=TO92
T 28300 67600 5 10 1 1 0 0 1
refdes=Q3
}
C 25100 64100 1 0 0 BA56-12.sym
{
T 25500 64200 5 10 1 1 0 0 1
refdes=U3
T 26100 65400 5 10 0 1 0 0 1
device=BA56_12
T 25700 65300 5 10 0 1 0 0 1
footprint=BA56_12
}
N 29800 64900 28800 64900 4
N 28800 64900 28800 63000 4
N 28800 63000 25900 63000 4
N 25900 63000 25900 64100 4
N 29100 63100 26200 63100 4
N 26200 63100 26200 64100 4
N 29100 63100 29100 64600 4
N 29100 64600 29800 64600 4
N 29800 65800 28100 65800 4
N 28100 65800 28100 63400 4
N 28100 63400 26500 63400 4
N 26500 63400 26500 64100 4
N 26800 64100 26800 63600 4
N 26800 63600 28500 63600 4
N 28500 63600 28500 64300 4
N 28500 64300 29800 64300 4
N 27100 64100 27100 63800 4
N 27100 63800 28300 63800 4
N 28300 63800 28300 65500 4
N 28300 65500 29800 65500 4
N 29800 64000 27700 64000 4
N 27700 64000 27700 65200 4
N 27700 65200 27400 65200 4
N 27400 65200 27400 65100 4
N 26500 65100 26500 65300 4
N 26500 65300 28500 65300 4
N 29800 65200 28500 65200 4
N 28500 65200 28500 65300 4
N 29800 63700 29400 63700 4
N 29400 63700 29400 65400 4
N 29400 65400 26200 65400 4
N 26200 65400 26200 65100 4
N 25900 65100 25900 65800 4
N 25900 65800 26500 65800 4
N 26500 65800 26500 66200 4
N 26800 65100 26800 65900 4
N 26800 65900 27700 65900 4
N 27700 65900 27700 66400 4
N 28900 66800 28900 66100 4
N 28900 66100 27900 66100 4
N 27900 66100 27900 65600 4
N 27900 65600 27100 65600 4
N 27100 65600 27100 65100 4
C 27500 68500 1 0 0 vcc-1.sym
{
T 27600 69000 5 10 0 0 0 0 1
netname=Vcc
}
N 26500 68000 26500 67200 4
N 28900 67800 28900 68000 4
N 27700 68500 27700 67400 4
N 26500 68000 27700 68000 4
N 27700 68000 28900 68000 4
N 33300 63200 32500 63200 4
N 32500 63200 32500 62200 4
N 32500 62200 24800 62200 4
N 24800 62200 24800 66700 4
N 24800 66700 25900 66700 4
N 33300 62800 33000 62800 4
N 33000 62800 33000 61800 4
N 33000 61800 24500 61800 4
N 24500 61800 24500 67400 4
N 27100 66900 27000 66900 4
N 27000 66900 27000 67400 4
N 27000 67400 24500 67400 4
N 36100 63600 36800 63600 4
N 36800 63600 36800 68200 4
N 36800 68200 28100 68200 4
N 28100 68200 28100 67300 4
N 28100 67300 28300 67300 4
C 42900 63900 1 0 0 flatio.sym
{
T 43500 64600 5 10 0 1 0 0 1
device=IO
T 42900 63900 5 10 0 1 0 0 1
footprint=FLATCONN
T 43100 64200 5 10 1 1 0 0 1
refdes=MOTOR_L
}
N 42100 62400 42100 65500 4
N 42100 65500 42900 65500 4
C 42900 66900 1 0 0 flatio.sym
{
T 43500 67600 5 10 0 1 0 0 1
device=IO
T 42900 66900 5 10 0 1 0 0 1
footprint=FLATCONN
T 43200 67100 5 10 1 1 0 0 1
refdes=INPUT_PLUS
}
N 36100 64400 41200 64400 4
N 41200 64400 41200 67000 4
N 41200 67000 42900 67000 4
C 42900 67800 1 0 0 flatio.sym
{
T 43500 68500 5 10 0 1 0 0 1
device=IO
T 42900 67800 5 10 0 1 0 0 1
footprint=FLATCONN
T 43200 68100 5 10 1 1 0 0 1
refdes=INPUT_MINUS
}
N 36100 64000 38300 64000 4
N 42900 67900 38300 67900 4
N 38300 67900 38300 64000 4
C 42900 68900 1 0 0 flatio.sym
{
T 43500 69600 5 10 0 1 0 0 1
device=IO
T 42900 68900 5 10 0 1 0 0 1
footprint=FLATCONN
T 43200 69100 5 10 1 1 0 0 1
refdes=INPUT_GND
}
C 42200 68300 1 0 0 gnd-1.sym
N 42300 68600 42300 69900 4
N 42300 69000 42900 69000 4
C 42900 69800 1 0 0 flatio.sym
{
T 43500 70500 5 10 0 1 0 0 1
device=IO
T 42900 69800 5 10 0 1 0 0 1
footprint=FLATCONN
T 43200 70100 5 10 1 1 0 0 1
refdes=OW_GND
}
N 42900 69900 42300 69900 4
C 40100 69800 1 0 0 flatio.sym
{
T 40700 70500 5 10 0 1 0 0 1
device=IO
T 40100 69800 5 10 0 1 0 0 1
footprint=FLATCONN
T 40300 70000 5 10 1 1 0 0 1
refdes=GND_IN
}
C 40100 68900 1 0 0 flatio.sym
{
T 40700 69600 5 10 0 1 0 0 1
device=IO
T 40100 68900 5 10 0 1 0 0 1
footprint=FLATCONN
T 40500 69100 5 10 1 1 0 0 1
refdes=Vcc_IN
}
N 40100 69900 39500 69900 4
N 39500 69900 39500 69400 4
N 39500 69400 42300 69400 4
C 38700 69000 1 0 0 vcc-1.sym
{
T 38800 69500 5 10 0 0 0 0 1
netname=Vcc
}
N 40100 69000 38900 69000 4
C 40100 68300 1 0 0 flatio.sym
{
T 40700 69000 5 10 0 1 0 0 1
device=IO
T 40100 68300 5 10 0 1 0 0 1
footprint=FLATCONN
T 40500 68500 5 10 1 1 0 0 1
refdes=OW_Vcc
}
N 40100 68400 39500 68400 4
N 39500 68400 39500 69000 4
C 39000 66800 1 0 0 flatio.sym
{
T 39600 67500 5 10 0 1 0 0 1
device=IO
T 39000 66800 5 10 0 1 0 0 1
footprint=FLATCONN
T 39300 67000 5 10 1 1 0 0 1
refdes=RADIATOR_OW
}
C 39000 65900 1 0 0 flatio.sym
{
T 39600 66600 5 10 0 1 0 0 1
device=IO
T 39000 65900 5 10 0 1 0 0 1
footprint=FLATCONN
T 39200 66100 5 10 1 1 0 0 1
refdes=ROOM_OW
}
N 39000 66900 37400 66900 4
N 37400 66900 37400 63200 4
N 37400 63200 36100 63200 4
N 39000 66000 37800 66000 4
N 37800 66000 37800 62800 4
N 37800 62800 36100 62800 4
C 39000 65000 1 0 0 flatio.sym
{
T 39600 65700 5 10 0 1 0 0 1
device=IO
T 39000 65000 5 10 0 1 0 0 1
footprint=FLATCONN
T 39200 65200 5 10 1 1 0 0 1
refdes=MOTOR_OUT
}
N 39000 65100 36500 65100 4
N 36500 65100 36500 64800 4
N 36500 64800 36100 64800 4
C 37000 61800 1 0 0 flatio.sym
{
T 37600 62500 5 10 0 1 0 0 1
device=IO
T 37000 61800 5 10 0 1 0 0 1
footprint=FLATCONN
T 37100 62100 5 10 1 1 0 0 1
refdes=MOTOR_IN
}
N 36400 61100 36400 61900 4
N 36400 61900 37000 61900 4
N 37900 61100 36400 61100 4
C 39100 63700 1 0 0 flatio.sym
{
T 39700 64400 5 10 0 1 0 0 1
device=IO
T 39100 63700 5 10 0 1 0 0 1
footprint=FLATCONN
T 39200 64000 5 10 1 1 0 0 1
refdes=Vcc_IN_SW
}
N 38800 63800 39100 63800 4
C 38900 59800 1 0 0 flatio.sym
{
T 39500 60500 5 10 0 1 0 0 1
device=IO
T 38900 59800 5 10 0 1 0 0 1
footprint=FLATCONN
T 39000 60100 5 10 1 1 0 0 1
refdes=GND_IN_SW
}
N 38900 59900 38500 59900 4
