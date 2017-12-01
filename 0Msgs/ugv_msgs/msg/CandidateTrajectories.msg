Header header
float32 fMaxHeadingOffset
float32 fMaxLateralOffset

CubicTrajectory[] candidateTrajectory

#======================================================================
#current target info;
uint16 nCurrentTargetID
uint8 nCurrentTargetType
float64 fCurrentTargetLatitude
float64 fCurrentTargetLongitude

#======================================================================
#next target info;
uint16 nNextTargetID
uint8 nNextTargetType
float64 fNextTargetLatitude
float64 fNextTargetLongitude

#======================================================================
#next next target info;
uint16 nNextNextTargetID
uint8 nNextNextTargetType
float64 fNextNextTargetLatitude
float64 fNextNextTargetLongitude

#======================================================================
#candidate trajectory look ahead distance
float32 fLookAheadDist

#=========================================================================
#vehicle status when generating the candidate trajectories;
float32 accelerationX
float32 accelerationY
float32 accelerationZ

float32 angularRateX
float32 angularRateY
float32 angularRateZ

float64 latitude
float64 longitude
float32 altitude

float32 velocityX
float32 velocityY
float32 velocityZ

float32 heading
float32 pitch
float32 roll

float32 x
float32 y
float32 z
#=========================================================================
