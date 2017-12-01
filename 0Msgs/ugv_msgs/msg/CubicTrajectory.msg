#cubic bezier curve first control point, P0, current vehicle position
#=======================================================================================================
#P0 x, always 0.0, relative to current vehicle pose
float32 fX0
#P0 y, always 0.0, relative to current vehicle pose
float32 fY0

#cubic bezier curve second control point, P1
#=======================================================================================================
#P1 x, always fDistP0P1, relative to current vehicle pose
float32 fX1
#P1 y, always 0.0, relative to current vehicle pose
float32 fY1

#cubic bezier curve third control point, P2
#=======================================================================================================
#P2 x, relative to current vehicle pose
float32 fX2
#P2 y, relative to current vehicle pose
float32 fY2

#cubic bezier curve fourth control point, P3, sample target position
#=======================================================================================================
#P3 x, relative to current vehicle pose
float32 fX3
#P3 y, relative to current vehicle pose
float32 fY3

#distance from P0 to P1
float32 fDistP0P1

#distance from P2 to P3
float32 fDistP2P3

#target sample lateral offset relative to reference sample point
float32 fLateralOffset

#target sample heading offset relative to reference sample point
float32 fHeadingOffset

#target sample heading relative to current vehicle pose
float32 fHeading
