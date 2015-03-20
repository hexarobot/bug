Based on http://www.lamja.com/?p=504

Design scaled from http://www.thingiverse.com/thing:56698 for using with 9g servos


Legs:

tibia to leg end: 90mm
tibia to femur: 53mm

Coxa positions TOP:

LEG1 = (-39.4, 78.2)\n
LEG2 = (-42.2, 0)
LEG3 = (-39.4, -78.2)
LEG4 = (39.4, 78.2)
LEG5 = (42.2, 0)
LEG6 = (39.4, -78.2)

Servo angles: 
α - coxa
β - femur
γ - tibia

Servo "zero angle" positions vectors:
coxa (-1,1), (-1,0), (-1,-1), (1,1), (1,0), (1,-1) (NW, W, SW, NE, E, SE) TOP VIEW
femur 3x (-1,0), 3x (1,0) (W, E) FRONT VIEW
tibia 6x (0,-1) (S) FRONT VIEW

Calculating tibia tip relative to body center, top view:

LEG1 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))
LEG2 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))
LEG3 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))
LEG4 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))
LEG5 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))
LEG6 = ((53*cos(β) + 90*cos(β+γ) + 12.8) * cos(α+45), (53*cos(β) + 90*cos(β+γ) + 12.8) * sin(α+45))