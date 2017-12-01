#include "arbiter.h"



void GetExecutableTrajector()
{
    CandidateToArbitor();
    if(GetOptimalTrajectory(&index))
    {
        OptimalTrajectoryInterpolation(index);
        PublishCurrentCurvature(index);
    }
    else
    {
        m_msgExecutionTrajector.poses.clear();
        m_msgVehicleShape.poses.clear();
    }
}

void EvaluateTrajectory(int nIndex)
{
    m_nCellNum = 1;
    m_nTraversability = 0;
    m_nNullCellNum = 0;
    m_nGrassCellNum = 0;

    m_pTrajectoryEvaulator[nIndex].fMaxCurvature = 0.0;
    m_pTrajectoryEvaulator[nIndex].fMinCurvature = 0.0;
    m_pTrajectoryEvaulator[nIndex].fTrajDist = 0.0;
    m_fLastPointX = 0.0;
    m_fLastPointY = 0.0;

    /** interpolate trajectory to get obstacle*/
   /// float fTInterval = 1.0/(m_fLookAheadDist/(5.0*m_fMapResolution));
    float fTInterval = 0.05;
    float ft0 = 0.0;
    float ft = ft0;
    bool bOstacle = false;
    float fCurvature = 0.0;

    m_fHalfVehicleWidth = param_.vehicle_width/2.0;

    while(ft<=1.0)
    {
      //  m_fHalfVehicleWidth = m_fHalfVehicleWidth+0.3*(ft-ft0);
        if(ft>=1.0)
        {
            ft = 1.0;
            EvaluatePoint(nIndex, ft, &bOstacle, &fCurvature);
            ft = 1.2;
        }
        else
        {
            EvaluatePoint(nIndex, ft, &bOstacle, &fCurvature);
        }

        if(bOstacle)
        {
            m_pTrajectoryEvaulator[nIndex].bObstacle = true;
            return;
        }

        if(fCurvature>m_pTrajectoryEvaulator[nIndex].fMaxCurvature)
        {
            m_pTrajectoryEvaulator[nIndex].fMaxCurvature = fCurvature;
        }
        if(fCurvature<m_pTrajectoryEvaulator[nIndex].fMinCurvature)
        {
            m_pTrajectoryEvaulator[nIndex].fMinCurvature = fCurvature;
        }

        if(fabs(fCurvature)>0.25)
        {
            m_pTrajectoryEvaulator[nIndex].bFeasible = false;
            return;
        }
        ft = ft+fTInterval;
    }
    m_pTrajectoryEvaulator[nIndex].nCellNum = m_nCellNum;
    m_pTrajectoryEvaulator[nIndex].nGrassCellNum = m_nGrassCellNum;
    m_pTrajectoryEvaulator[nIndex].nNullCellNum = m_nNullCellNum;
    m_pTrajectoryEvaulator[nIndex].nTraversability = m_nTraversability;
}

void CB_Planner::OptimalTrajectoryInterpolation(int index)
{
    /** interpolate trajectory to get obstacle*/
    double fTInterval = 0.05;
    double ft = 0.0;

    double fP0X = m_pTrajectoryEvaulator[index].trajectory.fX0;
    double fP0Y = m_pTrajectoryEvaulator[index].trajectory.fY0;

    double fP1X = m_pTrajectoryEvaulator[index].trajectory.fX1;
    double fP1Y = m_pTrajectoryEvaulator[index].trajectory.fY1;

    double fP2X = m_pTrajectoryEvaulator[index].trajectory.fX2;
    double fP2Y = m_pTrajectoryEvaulator[index].trajectory.fY2;

    double fP3X = m_pTrajectoryEvaulator[index].trajectory.fX3;
    double fP3Y = m_pTrajectoryEvaulator[index].trajectory.fY3;


    double fPX = 0.0;
    double fPY = 0.0;
    double fDiffX = 0.0;
    double fDiffY = 0.0;
    double fLPX = 0.0;
    double fLPY = 0.0;
    double fRPX = 0.0;
    double fRPY = 0.0;
    double fRelativeHeading;
    int nGridsNum = ceil(m_fHalfVehicleWidth/m_fMapResolution);

    m_msgExecutionTrajector.poses.clear();
    nav_msgs::Path LeftExecutionTrajectory;
    nav_msgs::Path RightExecutionTrajectory;
    geometry_msgs::PoseStamped pose;
    while(ft<=1.0)
    {
        if(ft>=1.0)
        {
            ft = 1.0;
            /** get point coordinate in vehicle frame*/
            fPX = (1-ft)*(1-ft)*(1-ft)*fP0X+3*(1-ft)*(1-ft)*ft*fP1X+3*(1-ft)*ft*ft*fP2X+ft*ft*ft*fP3X;
            fPY = (1-ft)*(1-ft)*(1-ft)*fP0Y+3*(1-ft)*(1-ft)*ft*fP1Y+3*(1-ft)*ft*ft*fP2Y+ft*ft*ft*fP3Y;
            ft = 1.2;
        }
        else
        {
            /** get point coordinate in vehicle frame*/
            fPX = (1-ft)*(1-ft)*(1-ft)*fP0X+3*(1-ft)*(1-ft)*ft*fP1X+3*(1-ft)*ft*ft*fP2X+ft*ft*ft*fP3X;
            fPY = (1-ft)*(1-ft)*(1-ft)*fP0Y+3*(1-ft)*(1-ft)*ft*fP1Y+3*(1-ft)*ft*ft*fP2Y+ft*ft*ft*fP3Y;
            fDiffX = 3*(1-ft)*(1-ft)*(fP1X-fP0X)+6*(1-ft)*ft*(fP2X-fP1X)+3*ft*ft*(fP3X-fP2X);
            fDiffY = 3*(1-ft)*(1-ft)*(fP1Y-fP0Y)+6*(1-ft)*ft*(fP2Y-fP1Y)+3*ft*ft*(fP3Y-fP2Y);
            fRelativeHeading = atan2(fDiffY, fDiffX);
            fLPX = fPX+nGridsNum*m_fMapResolution*sin(fRelativeHeading);
            fLPY = fPY-nGridsNum*m_fMapResolution*cos(fRelativeHeading);
            fRPX = fPX-nGridsNum*m_fMapResolution*sin(fRelativeHeading);
            fRPY = fPY+nGridsNum*m_fMapResolution*cos(fRelativeHeading);


        }

        pose.pose.position.x = fPX;
        pose.pose.position.y = fPY;
        m_msgExecutionTrajector.poses.push_back(pose);
        pose.pose.position.x = fLPX;
        pose.pose.position.y = fLPY;
        LeftExecutionTrajectory.poses.push_back(pose);
        pose.pose.position.x = fRPX;
        pose.pose.position.y = fRPY;
        RightExecutionTrajectory.poses.push_back(pose);
        ft = ft+fTInterval;
    }

    m_msgVehicleShape.poses = LeftExecutionTrajectory.poses;
    for(int i=RightExecutionTrajectory.poses.size()-1; i>=0; i--)
    {
        pose = RightExecutionTrajectory.poses[i];
        m_msgVehicleShape.poses.push_back(pose);
    }

}

