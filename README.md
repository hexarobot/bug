### **Based on** http://www.lamja.com/?p=504

### Design scaled from http://www.thingiverse.com/thing:5669, for using with 9, servos

### Legs:

*   Tibia to leg end: 90mm
*   Tibia to femur: 53mm

### Coxa positions TOP

*   LEG1 (-39.4, 78.2)
*   LEG2 (-42.2, 0)
*   LEG3 (-39.4, -78.2)
*   LEG4 (39.4, 78.2)
*   LEG5 (42.2, 0)
*   LEG6 (39.4, -78.2)

### Servo angles&nbsp;

*   α - coxa
*   β - femur
*   γ - tibia

### Servo "zero angle" position vectors

*   Coxa (-1,1), (-1,0), (-1,-1), (1,1), (1,0), (1,-1) (NW, W, SW, NE, E, SE) **TOP VIEW**
*   Femur 3x (-1,0), 3x (1,0) (W, E) **FRONT VIEW**
*   Tibia 6x (0,-1) (S) **FRONT VIEW**

### Calculating tibia tip relative to body center, top view

*   LEG1 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))
*   LEG2 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))
*   LEG3 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))
*   LEG4 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))
*   LEG5 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))
*   LEG6 = ((53_cos(β) + 90_cos(β+γ) + 12.8) * cos(α+45), (53_cos(β) + 90_cos(β+γ) + 12.8) * sin(α+45))