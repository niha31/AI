#include "p010382iSteeringBehaviors.h"
#include "../../TankManager.h"

#include <ctime>

SteeringBehavior::SteeringBehavior(p010382iTank* agent):

    m_dWanderDistance(WanderDist),
    m_dWanderJitter(WanderJitterPerSec),
    m_dWanderRadius(WanderRad),
    m_dWaypointSeekDistSq(WaypointSeekDist * WaypointSeekDist),
    m_SummingMethod(prioritized),
    m_Deceleration(normal)
{
    m_P010382iTank = agent;
    m_dWallDetectionFeelerLength = 100.0;
    
    m_arivePoint = Vector2D(400, 200);//SetRandomTargetPosition();
    m_targetPos = Vector2D(400,200);
    m_fleeFrom = Vector2D(400, 200);


    //stuff for the wander behavior
    srand(time(NULL));
    double theta = RandFloat() * TwoPi;

    //create a vector to a target position on the wander circle
    m_vWanderTarget = Vector2D(m_dWanderRadius * cos(theta), m_dWanderRadius * sin(theta));
}

SteeringBehavior::~SteeringBehavior()
{}

//--------------------------- WallAvoidance --------------------------------
//
//  This returns a steering force that will keep the agent away from any
//  walls it may encounter
//------------------------------------------------------------------------
Vector2D SteeringBehavior::WallAvoidance()
{
    //the feelers are contained in a std::vector, m_Feelers
    CreateFeelers();

    double DistToThisIP = 0.0;
    double DistToClosestIP = MaxDouble;

    //this will hold an index into the vector of walls
    int ClosestWall = -1;

    Vector2D SteeringForce,
        point,         //used for storing temporary info
        ClosestPoint;  //holds the closest intersection point

    //examine each feeler in turn
    for (unsigned int flr = 0; flr < m_Feelers.size(); ++flr)
    {
        //run through each wall checking for any intersection points
        if (LineIntersection2D(m_P010382iTank->GetCentralPosition(),m_Feelers[flr], Vector2D(0.0, 0.0) , Vector2D(kScreenWidth,0.0f), DistToThisIP, point))
        {
            //is this the closest found so far? If so keep a record
            if (DistToThisIP < DistToClosestIP)
            {
                DistToClosestIP = DistToThisIP;
                ClosestWall = 1;
                ClosestPoint = point;
            }
        }//next wall
        if (LineIntersection2D(m_P010382iTank->GetCentralPosition(), m_Feelers[flr], Vector2D(kScreenWidth, 0.0), Vector2D(kScreenWidth, kScreenHeight), DistToThisIP, point))
        {
            //is this the closest found so far? If so keep a record
            if (DistToThisIP < DistToClosestIP)
            {
                DistToClosestIP = DistToThisIP;
                ClosestWall = 2;
                ClosestPoint = point;
            }
        }//next wall
        if (LineIntersection2D(m_P010382iTank->GetCentralPosition(), m_Feelers[flr], Vector2D(0.0, kScreenHeight), Vector2D(kScreenWidth, kScreenHeight), DistToThisIP, point))
        {
            //is this the closest found so far? If so keep a record
            if (DistToThisIP < DistToClosestIP)
            {
                DistToClosestIP = DistToThisIP;
                ClosestWall = 3;
                ClosestPoint = point;
            }
        }//next wall
        if (LineIntersection2D(m_P010382iTank->GetCentralPosition(), m_Feelers[flr], Vector2D(0.0, 0.0), Vector2D(0.0, kScreenHeight), DistToThisIP, point))
        {
            //is this the closest found so far? If so keep a record
            if (DistToThisIP < DistToClosestIP)
            {
                DistToClosestIP = DistToThisIP;
                ClosestWall = 4;
                ClosestPoint = point;
            }
        }



        //if an intersection point has been detected, calculate a force  
        //that will direct the agent away
        if (ClosestWall > 0 && DistToClosestIP < 10)
        {
            m_P010382iTank->SetWallSeen(1);
            //calculate by what distance the projected position of the agent
            //will overshoot the wall
            Vector2D OverShoot = m_Feelers[flr] - ClosestPoint;
            Vector2D m_vN;

            if (ClosestWall == 1)
            {
                Vector2D temp = Vec2DNormalize(Vector2D(kScreenWidth, 0.0f) - Vector2D(0.0, 0.0));

                m_vN.x = -temp.y;
                m_vN.y = temp.x;

                //create a force in the direction of the wall normal, with a 
                //magnitude of the overshoot
                SteeringForce = m_vN * OverShoot.Length();
            }
            else if (ClosestWall == 2)
            {
                Vector2D temp = Vec2DNormalize(Vector2D(kScreenWidth, kScreenHeight) - Vector2D(kScreenWidth, 0.0));

                m_vN.x = -temp.y;
                m_vN.y = temp.x;

                //create a force in the direction of the wall normal, with a 
                //magnitude of the overshoot
                SteeringForce = m_vN * OverShoot.Length();
            }
            else if (ClosestWall == 3)
            {
                Vector2D temp = Vec2DNormalize(Vector2D(kScreenWidth, kScreenHeight) - Vector2D(0.0, kScreenHeight));

                m_vN.x = -temp.y;
                m_vN.y = temp.x;

                //create a force in the direction of the wall normal, with a 
                //magnitude of the overshoot
                SteeringForce = m_vN * OverShoot.Length();
            }
            else if (ClosestWall == 4)
            {
                Vector2D temp = Vec2DNormalize(Vector2D(0.0, kScreenHeight) - Vector2D(0.0, 0.0));

                m_vN.x = -temp.y;
                m_vN.y = temp.x;

                //create a force in the direction of the wall normal, with a 
                //magnitude of the overshoot
                SteeringForce = m_vN * OverShoot.Length();
            }
        }
        else
        {
            m_P010382iTank->SetWallSeen(0);
        }
        
    }//next feeler
    
    return SteeringForce;
}


//------------------------------- Seek -----------------------------------
//
//  Given a target, this behavior returns a steering force which will
//  direct the agent towards the target
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Seek(Vector2D TargetPos)
{
    if ((GetKeyState(VK_LBUTTON) & 0x8000))
    {
        m_targetPos = MouseClicked();
    }

    Vector2D DesiredVelocity = Vec2DNormalize(TargetPos - m_P010382iTank->GetCentralPosition()) * m_P010382iTank->GetMaxSpeed();


    return (DesiredVelocity - m_P010382iTank->GetVelocity());
}


//----------------------------- Flee -------------------------------------
//
//  Does the opposite of Seek
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Flee(vector<BaseTank*> TargetTanks)
{
    //only flee if the target is within 'panic distance'. Work in distance
    //squared space.
    const double PanicDistanceSq = 100.0f * 100.0;
    std::vector<BaseTank*>::const_iterator curOb = TargetTanks.begin();
    for (int i = 0; i < TargetTanks.size(); i++)
    {
        if (Vec2DDistanceSq(m_P010382iTank->GetPosition(), (*curOb)->GetPosition()) < PanicDistanceSq)
        {
            Vector2D DesiredVelocity = Vec2DNormalize(m_P010382iTank->GetPosition() - (*curOb)->GetPosition()) * m_P010382iTank->GetMaxSpeed();

            return (DesiredVelocity - m_P010382iTank->GetVelocity());
        }
    }

    return Vector2D(0, 0);
}

Vector2D SteeringBehavior::FleeFromClick()
{
    if ((GetKeyState(VK_LBUTTON) & 0x8000))
    {
        m_fleeFrom = MouseClicked();
    }


    //only flee if the target is within 'panic distance'. Work in distance
    //squared space.
    const double PanicDistanceSq = 100.0f * 100.0;
    
        if (Vec2DDistanceSq(m_P010382iTank->GetPosition(), m_fleeFrom) < PanicDistanceSq)
        {
            Vector2D DesiredVelocity = Vec2DNormalize(m_P010382iTank->GetPosition() - m_fleeFrom) * m_P010382iTank->GetMaxSpeed();

            return (DesiredVelocity - m_P010382iTank->GetVelocity());
        }

    return Vector2D(0, 0);
}


//--------------------------- Wander -------------------------------------
//
//  This behavior makes the agent wander about randomly
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Wander()
{
    //this behavior is dependent on the update rate, so this line must
    //be included when using time independent framerate.
    double JitterThisTimeSlice = m_dWanderJitter * m_P010382iTank->TimeElapsed();

    //first, add a small random vector to the target's position
    m_vWanderTarget += Vector2D(RandomClamped() * JitterThisTimeSlice,
        RandomClamped() * JitterThisTimeSlice);

    //reproject this new vector back on to a unit circle
    m_vWanderTarget.Normalize();

    //increase the length of the vector to the same as the radius
    //of the wander circle
    m_vWanderTarget *= m_dWanderRadius;

    //move the target into a position WanderDist in front of the agent
    Vector2D target = m_vWanderTarget + Vector2D(m_dWanderDistance, 0);

    //project the target into world space
    Vector2D Target = PointToWorldSpace(target,
        m_P010382iTank->GetHeading(),
        m_P010382iTank->GetSide(),
        m_P010382iTank->GetCentralPosition());

    //and steer towards it
    return Target - m_P010382iTank->GetCentralPosition();
}

//--------------------------- Arrive -------------------------------------
//
//  This behavior is similar to seek but it attempts to arrive at the
//  target with a zero velocity
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Arrive(Deceleration deceleration)
{

    if ((GetKeyState(VK_LBUTTON) & 0x8000))
    {
        m_arivePoint = MouseClicked();
    }

    Vector2D ToTarget = m_arivePoint - m_P010382iTank->GetCentralPosition();

    //calculate the distance to the target
    double dist = ToTarget.Length();

    this->m_P010382iTank->AddDebugCircle(m_arivePoint);
    //because Deceleration is enumerated as an int, this value is required
        //to provide fine tweaking of the deceleration..
    const double DecelerationTweaker = 0.3;
    //calculate the speed required to reach the target given the desired
        //deceleration
    double speed;
    //from here proceed just like Seek except we don't need to normalize 
        //the ToTarget vector because we have already gone to the trouble
        //of calculating its length: dist. 
    Vector2D DesiredVelocity;

     if (dist > 20.0)
     {
        speed = dist / ((double)deceleration * DecelerationTweaker);

        //make sure the velocity does not exceed the max
        speed = min(speed, m_P010382iTank->GetMaxSpeed());

        //from here proceed just like Seek except we don't need to normalize 
        //the ToTarget vector because we have already gone to the trouble
        //of calculating its length: dist. 
        DesiredVelocity = ToTarget * speed / dist;

        return (DesiredVelocity - m_P010382iTank->GetVelocity());
     }
     //else
     //{
     //    m_arivePoint = SetRandomTargetPosition();

     //    //calculate the distance to the target
     //    dist = ToTarget.Length();
     //    ToTarget = m_arivePoint - m_P010382iTank->GetCentralPosition();
     //    speed = dist / ((double)deceleration * DecelerationTweaker);
     //    DesiredVelocity = ToTarget * speed / dist;
     //    
     //    return (DesiredVelocity - m_P010382iTank->GetVelocity());
     //}

    return Vector2D(0, 0);
}



//------------------------------ Pursuit ---------------------------------
//
//  this behavior creates a force that steers the agent towards the 
//  evader
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Pursuit(vector<BaseTank*> agents)
{
    Vector2D ToTarget;
    double currentDist;
    double closestDist = 1000;
    double dist;
    int ClosestObject = -1;

    int size = agents.size();
    int i = 0;
    vector<BaseTank*>::const_iterator curob = agents.begin();
    for (i = 0; i < agents.size(); i++)
    {
        ToTarget = (*curob)->GetCentralPosition() - m_P010382iTank->GetCentralPosition();

        if (ToTarget.Length() < closestDist)
        {
            ClosestObject = i;
            closestDist = ToTarget.Length();
        }
    }

    if (ClosestObject == -1 )
    {
        return Seek(SetRandomTargetPosition());
    }

    Vector2D ToEvader = agents[ClosestObject]->GetPosition() - m_P010382iTank->GetPosition();

    //Not considered ahead so we predict where the evader will be.

    //the lookahead time is propotional to the distance between the evader
    //and the pursuer; and is inversely proportional to the sum of the
    //agent's velocities
    double LookAheadTime = ToEvader.Length() / (m_P010382iTank->GetMaxSpeed() + agents[ClosestObject]->GetCurrentSpeed());

    //now seek to the predicted future position of the evader
    Vector2D futurePos = agents[ClosestObject]->GetPosition() + agents[ClosestObject]->GetVelocity() * LookAheadTime;
    m_P010382iTank->SetTargetTank(futurePos);

    //fire at tank
    m_P010382iTank->FireWeapon(ClosestObject, agents);
    
    return Seek(futurePos);
}


//---------------------- ObstacleAvoidance -------------------------------
//
//  Given a vector of CObstacles, this method returns a steering force
//  that will prevent the agent colliding with the closest obstacle
//------------------------------------------------------------------------
Vector2D SteeringBehavior::ObstacleAvoidance(const vector<GameObject*>& obsticles)
{
    CreateFeelers();

    Vector2D SteeringForce,
        point,         //used for storing temporary info
        ClosestPoint;  //holds the closest intersection point
   
    for (unsigned int flr = 0; flr < m_Feelers.size(); ++flr)
    {
        double DistToThisIP = 0.0;
        double DistToClosestIP = MaxDouble;

        //this will hold an index into the vector of walls
        int ClosestObject = -1;
        int wallOfObject1;
        int wallOfObject2;

        double feelerLength = m_Feelers.at(flr).Distance(this->m_P010382iTank->GetCentralPosition());

        vector<GameObject*>::const_iterator curob = obsticles.begin();

        while (curob != obsticles.end())
        {
            int j = 0;
            if ((*curob)->GetGameObjectType() == GAMEOBJECT_OBSTACLE)
            {
                std::vector<Vector2D> points = (*curob)->GetAdjustedBoundingBox();

                for (int i = 0; i < points.size(); ++i)
                {
                    int index1 = i;
                    int index2;

                    if (i == 0)
                    {
                        index2 = points.size() - 1;
                    }
                    else
                    {
                        index2 = i - 1;
                    }

                    if (LineIntersection2D(m_P010382iTank->GetCentralPosition(), m_Feelers[flr], points[index1], points[index2], DistToThisIP, point))
                    {
                        //is this the closest found so far? If so keep a record
                        if (DistToThisIP < DistToClosestIP)
                        {
                            DistToClosestIP = DistToThisIP;

                            ClosestObject = j;
                            wallOfObject1 = index1;
                            wallOfObject2 = index2;

                            ClosestPoint = point;
                        }
                    }
                }
            }
            curob++;
            j++;
        } 

        if (ClosestObject >= 0 && DistToClosestIP < 20)//feelerLength)
        {
            m_P010382iTank->SetObsticleSeen(1);

            double distance = ClosestPoint.Distance(this->m_P010382iTank->GetCentralPosition());
            double modifier = 1.0 - (distance / feelerLength);

            this->m_P010382iTank->AddDebugCircle(ClosestPoint);

            std::vector<Vector2D> points = obsticles[ClosestObject]->GetAdjustedBoundingBox();

            Vector2D temp = Vec2DNormalize(points[wallOfObject2] - points[wallOfObject1]);

            Vector2D wallNormal = temp.Perp();

            this->m_P010382iTank->AddDebugLine(ClosestPoint, ClosestPoint + wallNormal * 30.0);

            //create a force in the direction of the wall normal, with a 
            //magnitude of the overshoot
            SteeringForce += wallNormal * modifier * 10.0f;

            this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), this->m_P010382iTank->GetCentralPosition() + SteeringForce);

        }
        else
        {
            m_P010382iTank->SetObsticleSeen(0);
        }

    }

    return SteeringForce;

}

vector<Vector2D> SteeringBehavior::AStar(Vector2D EndPoint)
{
   vector<Waypoint*> waypoints = WaypointManager::Instance()->GetAllWaypoints();
   int startPoint;
   int endPoint;
   int smallestDistanceFromTank = 1000;
   int smallestDistFromEndPoint = 1000;


   vector<Waypoint*>::const_iterator curob = waypoints.begin();
   for (int i = 0; i < waypoints.size(); i++)
   {
       Vector2D ToTank = (*curob)->GetPosition() - m_P010382iTank->GetCentralPosition();
       //calculate the distance to the target
       if (ToTank.Length() < smallestDistanceFromTank)
       {
           smallestDistanceFromTank = ToTank.Length();
           startPoint = i;
       }

       Vector2D ToEndPoint = (*curob)->GetPosition() - EndPoint;
       if (ToEndPoint.Length() < smallestDistFromEndPoint)
       {
           smallestDistFromEndPoint = ToEndPoint.Length();
           endPoint = i;
       }

       curob++;
   }

   this->m_P010382iTank->AddDebugCircle(waypoints.at(endPoint)->GetPosition());

   vector<int> OpenList;
   vector<bool> ClosedList = vector<bool>(waypoints.size());
   vector<node> path = vector<node>(waypoints.size());
   vector<Vector2D> finalPath;

   OpenList.push_back(startPoint);


   while (!OpenList.empty())
   {

       int ID = OpenList.at(0);

       OpenList.erase(OpenList.begin()); 
       ClosedList.at(ID) = true;

       double fNew, gNew, hNew;
       
       vector<int> connectedNodeID = waypoints.at(ID)->GetConnectedWaypointIDs();

       for (size_t i = 0; i < connectedNodeID.size(); i++)
       {
           int newNode = connectedNodeID.at(i);

           if (newNode == endPoint)
           {
               path.at(newNode).parent = ID;
               int thisNode = newNode;

               while (thisNode != -1)
               {
                   finalPath.push_back(waypoints.at(thisNode)->GetPosition());

                   thisNode = path.at(thisNode).parent;
               }
                
               return finalPath;
               
           }
           else if (ClosedList.at(newNode) == false)
           {
               double distToNewNode = waypoints.at(ID)->GetPosition().Distance(waypoints.at(newNode)->GetPosition());
               double distToEnd = waypoints.at(newNode)->GetPosition().Distance(waypoints.at(endPoint)->GetPosition());


               gNew = path.at(ID).g + distToNewNode;
               hNew = distToEnd;
               fNew = gNew + hNew;

               if (path.at(newNode).f == DBL_MAX || fNew < path.at(newNode).f)
               {
                   OpenList.push_back(newNode);

                   path.at(newNode).f = fNew;
                   path.at(newNode).g = gNew;
                   path.at(newNode).h = hNew;
                   path.at(newNode).parent = ID;
               }

           }
       }
   }

 
}




//------------------------------- CreateFeelers --------------------------
//
//  Creates the antenna utilized by WallAvoidance
//------------------------------------------------------------------------
void SteeringBehavior::CreateFeelers()
{
    m_Feelers.clear();

    //feeler pointing straight in front
    m_Feelers.push_back(m_P010382iTank->GetCentralPosition() + 70 * m_P010382iTank->GetHeading());

    this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), m_Feelers.back());

    //feeler to left
    Vector2D temp = m_P010382iTank->GetHeading();
    Vec2DRotateAroundOrigin(temp, HalfPi * -0.7);
    m_Feelers.push_back(m_P010382iTank->GetCentralPosition() + (m_dWallDetectionFeelerLength * 0.5) * temp);

    this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), m_Feelers.back());

    //feeler to left
    temp = m_P010382iTank->GetHeading();
    Vec2DRotateAroundOrigin(temp, HalfPi * -0.3);
    m_Feelers.push_back(m_P010382iTank->GetCentralPosition() + (m_dWallDetectionFeelerLength * 0.8) * temp);

    this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), m_Feelers.back());

    //feeler to right
    temp = m_P010382iTank->GetHeading();
    Vec2DRotateAroundOrigin(temp, HalfPi * 0.3);
    m_Feelers.push_back(m_P010382iTank->GetCentralPosition() + (m_dWallDetectionFeelerLength * 0.8) * temp);

    this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), m_Feelers.back());

    //feeler to right
    temp = m_P010382iTank->GetHeading();
    Vec2DRotateAroundOrigin(temp, HalfPi * 0.7);
    m_Feelers.push_back(m_P010382iTank->GetCentralPosition() + (m_dWallDetectionFeelerLength * 0.5) * temp);

    this->m_P010382iTank->AddDebugLine(this->m_P010382iTank->GetCentralPosition(), m_Feelers.back());
}


//----------------------- Calculate --------------------------------------
//
//  calculates the accumulated steering force according to the method set
//  in m_SummingMethod
//------------------------------------------------------------------------
Vector2D SteeringBehavior::Calculate()
{
    //reset the steering force
    m_vSteeringForce.Zero();

    switch (m_SummingMethod)
    {
    case prioritized:
        m_vSteeringForce = CalculatePrioritized(); break;

    default:
        m_vSteeringForce = Vector2D(0, 0);

    }//end switch

    return m_vSteeringForce;
}

//---------------------- CalculatePrioritized ----------------------------
//
//  this method calls each active steering behavior in order of priority
//  and acumulates their forces until the max steering force magnitude
//  is reached, at which time the function returns the steering force 
//  accumulated to that  point
//------------------------------------------------------------------------
Vector2D SteeringBehavior::CalculatePrioritized()
{
    Vector2D force;

    if (On(wall_avoidance))
    {
        force = WallAvoidance();

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(obstacle_avoidance))
    {
        force = ObstacleAvoidance(ObstacleManager::Instance()->GetObstacles());

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(seek))
    {         
        force = Seek(m_targetPos);

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(flee))
    {

        m_vSteeringForce += Flee(m_P010382iTank->GetTanksCanSee());
    }

    if (On(fleeFromClick))
    {
        m_vSteeringForce += FleeFromClick();
    }

    if (On(arrive))
    {         
        force = Arrive(m_Deceleration);

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(wander))
    {
        force = Wander();

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(pursuit))
    {
        force = Pursuit(m_P010382iTank->GetTanksCanSee()); //TankManager::Instance()->GetVisibleTanks(m_P010382iTank)

        if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    return m_vSteeringForce;
}


//--------------------- AccumulateForce ----------------------------------
//
//  This function calculates how much of its max steering force the 
//  vehicle has left to apply and then applies that amount of the
//  force to add.
//------------------------------------------------------------------------
bool SteeringBehavior::AccumulateForce(Vector2D& RunningTot, Vector2D ForceToAdd)
{

    //calculate how much steering force the vehicle has used so far
    double MagnitudeSoFar = RunningTot.Length();

    //calculate how much steering force remains to be used by this vehicle
    double MagnitudeRemaining = m_P010382iTank->GetMaxForce() - MagnitudeSoFar;

    //return false if there is no more force left to use
    if (MagnitudeRemaining <= 0.0) return false;

    //calculate the magnitude of the force we want to add
    double MagnitudeToAdd = ForceToAdd.Length();

    //if the magnitude of the sum of ForceToAdd and the running total
    //does not exceed the maximum force available to this vehicle, just
    //add together. Otherwise add as much of the ForceToAdd vector is
    //possible without going over the max.
    if (MagnitudeToAdd < MagnitudeRemaining)
    {
        RunningTot += ForceToAdd;
    }
    else
    {
        //add it to the steering force
        RunningTot += (Vec2DNormalize(ForceToAdd) * MagnitudeRemaining);
    }

    return true;
}


Vector2D SteeringBehavior::SetRandomTargetPosition()
{
    srand(time((NULL)));
    vector<SDL_Rect>* obsticleRect = m_P010382iTank->ReturnObsticleRect();

    while (1)
    {
        Vector2D temporaryTargetPosition = Vector2D(50 + (rand() % (kScreenWidth - 100)), 50 + (rand() % (kScreenHeight - 100)));
        SDL_Rect tempPosRect = { temporaryTargetPosition.x - 20, temporaryTargetPosition.y - 20, 40, 40 };

        bool collided = false;

        for (int i = 0; i < obsticleRect->size(); ++i)
        {
            if (AABBRectCollision(obsticleRect->at(i), tempPosRect))
            {
                collided = true;
                break;
            }
        }

        if (!collided)
        {
             return temporaryTargetPosition;            
        }
    }
}

Vector2D SteeringBehavior::MouseClicked()
{
    int xPos;
    int yPos;

    SDL_GetMouseState(&xPos, &yPos);


    return Vector2D(xPos, yPos);
}

