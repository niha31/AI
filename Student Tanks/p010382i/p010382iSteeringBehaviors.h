#ifndef STEERINGBEHAVIORS_H
#define STEERINGBEHAVIORS_H
#pragma warning (disable:4786)

#include <vector>
#include <windows.h>
#include <string>
#include <list>
#include <Windowsx.h>

#include "../../Commons.h"
#include "../../Constants.h"
#include "../../C2DMatrix.h"
#include "../../ObstacleManager.h"
#include "../../TankManager.h"



class p010382iTank;
class ObstacleManager;
class BaseTank;


//--------------------------- Constants ----------------------------------

//the radius of the constraining circle for the wander behavior
const double WanderRad = 1.2;
//distance the wander circle is projected in front of the agent
const double WanderDist = 2.0;
//the maximum amount of displacement along the circle each frame
const double WanderJitterPerSec = 80.0;

//used in path following
const double WaypointSeekDist = 20;
//------------------------------------------------------------------------


class SteeringBehavior
{
public:

    enum summing_method { weighted_average, prioritized, dithered };

private:
    enum behavior_type
    {
        none = 0x00000,
        seek = 0x00002,
        flee = 0x00004,
        arrive = 0x00008,
        wander = 0x00010,
        fleeFromClick = 0x00020,
        a_star = 0x00040,
        allignment = 0x00080,
        obstacle_avoidance = 0x00100,
        wall_avoidance = 0x00200,
        follow_path = 0x00400,
        pursuit = 0x00800,
        evade = 0x01000,
        interpose = 0x02000,
        hide = 0x04000,
        flock = 0x08000,
        offset_pursuit = 0x10000,
    };
private:


    //a pointer to the owner of this instance
    p010382iTank* m_P010382iTank;


    //the steering force created by the combined effect of all
    //the selected behaviors
    Vector2D    m_vSteeringForce;

    //these can be used to keep track of friends, pursuers, or prey
    BaseTank* mpTargetAgent1;
    BaseTank* mpTargetAgent2;

    //binary flags to indicate whether or not a behavior should be active
    int           m_iFlags;

    //Arrive makes use of these to determine how quickly a vehicle
//should decelerate to its target
    enum Deceleration { slow = 3, normal = 2, fast = 1 };

    //default
    Deceleration m_Deceleration;

    //Where tank is ariving
    Vector2D m_arivePoint;
    Vector2D m_tempArivePoint;
    Vector2D m_targetPos;
    Vector2D m_fleeFrom;


    //this function tests if a specific bit of m_iFlags is set
    bool      On(behavior_type bt) { return (m_iFlags & bt) == bt; }

    //length of the 'detection box' utilized in obstacle avoidance
    double m_dDBoxLength;

    //a vertex buffer to contain the feelers rqd for wall avoidance  
    std::vector<Vector2D> m_Feelers;

    //the length of the 'feeler/s' used in wall detection
    double                 m_dWallDetectionFeelerLength;

    //the current position on the wander circle the agent is
    //attempting to steer towards
    Vector2D     m_vWanderTarget;

    //explained above
    double        m_dWanderJitter;
    double        m_dWanderRadius;
    double        m_dWanderDistance;

//the distance (squared) a vehicle has to be from a path waypoint before
//it starts seeking to the next waypoint
    double        m_dWaypointSeekDistSq;


    //what type of method is used to sum any active behavior
    summing_method  m_SummingMethod;


    //Steering Behaviors
    Vector2D Seek(Vector2D TargetPos);
    Vector2D Flee(vector<BaseTank*> TargetTanks);
    Vector2D FleeFromClick();
    Vector2D Wander();
    //this behavior is similar to seek but it attempts to arrive at the target position with a zero velocity
    Vector2D Arrive (Deceleration deceleration);
    //this behavior predicts where an agent will be in time T and seeks towards that point to intercept it.
    Vector2D Pursuit(vector<BaseTank*> agents);
    //this returns a steering force which will attempt to keep the agent away from any obstacles it may encounter
    Vector2D ObstacleAvoidance(const vector<GameObject*>& obstacles);
    //this returns a steering force which will keep the agent away from any walls it may encounter
    Vector2D WallAvoidance();

    //calculates and sums the steering forces from any active behaviors
    Vector2D CalculatePrioritized();

    bool SteeringBehavior::AccumulateForce(Vector2D& RunningTot, Vector2D ForceToAdd);

    void CreateFeelers();
    //returns a random double in the range -1 < n < 1
    inline double RandomClamped() { return RandFloat() - RandFloat(); }

    //returns a random double between zero and 1
    inline double RandFloat() { return ((rand()) / (RAND_MAX + 1.0)); }



public:
    SteeringBehavior(p010382iTank* agent);
    ~SteeringBehavior();

    Vector2D SetRandomTargetPosition();
    Vector2D MouseClicked();
    vector<Vector2D> AStar(Vector2D EndPoint);



    //calculates and sums the steering forces from any active behaviors
    Vector2D Calculate();

    void FleeOn() { m_iFlags |= flee; }
    void FleeFromClickOn() { m_iFlags |= fleeFromClick; }
    void SeekOn() { m_iFlags |= seek; }
    void ArriveOn() { m_iFlags |= arrive; }
    void WanderOn() { m_iFlags |= wander; }
    void PursuitOn() { m_iFlags |= pursuit;}
    void ObstacleAvoidanceOn() { m_iFlags |= obstacle_avoidance; }
    void WallAvoidanceOn() { m_iFlags |= wall_avoidance; }
    void AStarOn() { m_iFlags |= a_star; }


    void FleeOff() { if (On(flee))   m_iFlags ^= flee; }
    void FleeFromClickOff() { if (On(fleeFromClick)) m_iFlags ^= fleeFromClick; }
    void SeekOff() { if (On(seek))   m_iFlags ^= seek; }
    void ArriveOff() { if (On(arrive)) m_iFlags ^= arrive; }
    void WanderOff() { if (On(wander)) m_iFlags ^= wander; }
    void PursuitOff() { if (On(pursuit)) m_iFlags ^= pursuit; }
    void ObstacleAvoidanceOff() { if (On(obstacle_avoidance)) m_iFlags ^= obstacle_avoidance; }
    void WallAvoidanceOff() { if (On(wall_avoidance)) m_iFlags ^= wall_avoidance; }
    void AStarOff() { if (On(a_star)) m_iFlags ^= a_star; }


    bool isFleeOn() { return On(flee); }
    bool isFleeFromClickOn() { return On(fleeFromClick); }
    bool isSeekOn() { return On(seek); }
    bool isArriveOn() { return On(arrive); }
    bool isWanderOn() { return On(wander); }
    bool isPursuitOn() { return On(pursuit); }
    bool isObstacleAvoidanceOn() { return On(obstacle_avoidance); }
    bool isWallAvoidanceOn() { return On(wall_avoidance); }
    bool isAStarOn() { return On(a_star); }




    struct node
    {
        double f = DBL_MAX;
        double g = 0; 
        double h = LDBL_MAX;
        int parent = -1;
    };



    //--------------------- PointToLocalSpace --------------------------------
    //
    //------------------------------------------------------------------------
inline Vector2D PointToLocalSpace(const Vector2D& point, Vector2D& AgentHeading, Vector2D& AgentSide, Vector2D& AgentPosition)
{

    //make a copy of the point
    Vector2D TransPoint = point;

    //create a transformation matrix
    C2DMatrix matTransform;

    double Tx = -AgentPosition.Dot(AgentHeading);
    double Ty = -AgentPosition.Dot(AgentSide);

    //create the transformation matrix
    matTransform._11(AgentHeading.x); matTransform._12(AgentSide.x);
    matTransform._21(AgentHeading.y); matTransform._22(AgentSide.y);
    matTransform._31(Tx);           matTransform._32(Ty);

    //now transform the vertices
    matTransform.TransformVector2Ds(TransPoint);

    return TransPoint;
}

//--------------------- VectorToWorldSpace --------------------------------
//
//  Transforms a vector from the agent's local space into world space
//------------------------------------------------------------------------
inline Vector2D VectorToWorldSpace(const Vector2D& vec, const Vector2D& AgentHeading, const Vector2D& AgentSide)
{
    //make a copy of the point
    Vector2D TransVec = vec;

    //create a transformation matrix
    C2DMatrix matTransform;

    //rotate
    matTransform.Rotate(AgentHeading, AgentSide);

    //now transform the vertices
    matTransform.TransformVector2Ds(TransVec);

    return TransVec;
}

//--------------------- PointToWorldSpace --------------------------------
//
//  Transforms a point from the agent's local space into world space
//------------------------------------------------------------------------
    inline Vector2D PointToWorldSpace(const Vector2D& point, const Vector2D& AgentHeading, const Vector2D& AgentSide, const Vector2D& AgentPosition)
    {
        //make a copy of the point
        Vector2D TransPoint = point;

        //create a transformation matrix
        C2DMatrix matTransform;

        //rotate
        matTransform.Rotate(AgentHeading, AgentSide);

        //and translate
        matTransform.Translate(AgentPosition.x, AgentPosition.y);

        //now transform the vertices
        matTransform.TransformVector2Ds(TransPoint);

        return TransPoint;
    }


//-------------------- LineIntersection2D-------------------------
//
//	Given 2 lines in 2D space AB, CD this returns true if an 
//	intersection occurs and sets dist to the distance the intersection
//  occurs along AB. Also sets the 2d vector point to the point of
//  intersection
//----------------------------------------------------------------- 
    inline bool LineIntersection2D(Vector2D   A,
        Vector2D   B,
        Vector2D   C,
        Vector2D   D,
        double& dist,
        Vector2D& point)
    {

        double rTop = (A.y - C.y) * (D.x - C.x) - (A.x - C.x) * (D.y - C.y);
        double rBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

        double sTop = (A.y - C.y) * (B.x - A.x) - (A.x - C.x) * (B.y - A.y);
        double sBot = (B.x - A.x) * (D.y - C.y) - (B.y - A.y) * (D.x - C.x);

        if ((rBot == 0) || (sBot == 0))
        {
            //lines are parallel
            return false;
        }

        double r = rTop / rBot;
        double s = sTop / sBot;

        if ((r > 0) && (r < 1) && (s > 0) && (s < 1))
        {
            dist = Vec2DDistance(A, B) * r;

            point = A + r * (B - A);

            return true;
        }

        else
        {
            dist = 0;

            return false;
        }
    }


    //-------------------------- Vec2DRotateAroundOrigin --------------------------
//
//  rotates a vector ang rads around the origin
//-----------------------------------------------------------------------------
    inline void Vec2DRotateAroundOrigin(Vector2D& v, double ang)
    {
        //create a transformation matrix
        C2DMatrix mat;

        //rotate
        mat.Rotate(ang);

        //now transform the object's vertices
        mat.TransformVector2Ds(v);
    }



    bool AABBRectCollision(SDL_Rect& a, SDL_Rect& b)
    {
        if (a.x < b.x + b.w && a.x + a.w > b.x&& a.y < b.y + b.h && a.y + a.h > b.y)
        {
            return true;
        }
        return false;
    }

};
#include "p010382iTank.h"
#include "../../BaseTank.h"
#include "../../ObstacleManager.h"

#endif // !#ifndef STEERINGBEHAVIORS_H
