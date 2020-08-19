#include "p010382iTank.h"
#include "../../TankManager.h"
#include "../../C2DMatrix.h"

//--------------------------------------------------------------------------------------------------

p010382iTank::p010382iTank(SDL_Renderer* renderer, TankSetupDetails details) : BaseTank(renderer, details)
{
	mSteeringBehavior = new SteeringBehavior(this);
	mSteeringBehavior->WallAvoidanceOn();
	mSteeringBehavior->ObstacleAvoidanceOn();
	mSteeringBehavior->ArriveOff();
	mSteeringBehavior->WanderOff();
	mSteeringBehavior->PursuitOff();
	mSteeringBehavior->SeekOff();
	mSteeringBehavior->FleeOff();
	mSteeringBehavior->FleeFromClickOff();
	mSteeringBehavior->AStarOn();


	LoadObstacleRects();
}

//------------------------------------------------------------------------------------------------

p010382iTank::~p010382iTank()
{
}

//--------------------------------------------------------------------------------------------------

void p010382iTank::ChangeState(BASE_TANK_STATE newState)
{
	BaseTank::ChangeState(newState);
}

//--------------------------------------------------------------------------------------------------

void p010382iTank::Update(float deltaTime, SDL_Event e)
{
	debugLines.clear();
	debugCircles.clear();
	vector<GameObject*> pickups = PickUpManager::Instance()->GetAllPickUps();

	mdTimeElapsed = deltaTime;

	if ((GetKeyState(VK_LBUTTON) & 0x8000) && mSteeringBehavior->isAStarOn())
	{
		path = mSteeringBehavior->AStar(mSteeringBehavior->MouseClicked());
	}

	if (!path.empty())
	{
		mSteeringBehavior->WanderOff();
		GoToTargetWaypoint(deltaTime);
	}
	else if(mSteeringBehavior->isAStarOn() && !findCrates)
	{
		path = mSteeringBehavior->AStar(mSteeringBehavior->SetRandomTargetPosition());
	}
	else if (mSteeringBehavior->isAStarOn() && findCrates)
	{
		
		path = mSteeringBehavior->AStar(pickups.back()->GetCentralPosition());
		pickups.pop_back();
		
		if (pickups.empty())
		{
			findCrates = false;
		}
	}


		

	/*if (GetTanksCanSee().size() != 0)
	{	
		mSteeringBehavior->PursuitOn();
		mSteeringBehavior->ArriveOff();
		ChangeState(TANKSTATE_MANFIRE);

	}
	else
	{
		mSteeringBehavior->PursuitOff();
		mSteeringBehavior->ArriveOn();
		ChangeState(TANKSTATE_IDLE);

	}*/


	//calculate the combined force from each steering behavior in the 
	//vehicle's list
	steeringForce += mSteeringBehavior->Calculate();

	if (wallSeen)
	{
		mCurrentSpeed = 30.0f;
	}
	else
	{
		mCurrentSpeed = 50.0;
	}

	if (obsticlSeen)
	{
		mCurrentSpeed = 30.0f;
	}
	else
	{
		mCurrentSpeed = 50.0f;
	}


	//Call parent update.
	BaseTank::Update(deltaTime, e);
}

void p010382iTank::GoToTargetWaypoint(float deltaTime)
{
	Vector2D ToTarget = path.back() - GetCentralPosition();
	double dist = ToTarget.Length();

	if (dist < 10)
	{
		path.pop_back();
		return;
	}

	Vector2D DesiredVelocity = Vec2DNormalize(path.back() - GetCentralPosition()) * GetMaxSpeed();

	steeringForce = DesiredVelocity - GetVelocity();
}

//--------------------------------------------------------------------------------------------------

void p010382iTank::AddDebugLine(Vector2D a, Vector2D b)
{
	this->debugLines.push_back(std::make_pair(a,b));
}

void p010382iTank::AddDebugCircle(Vector2D a)
{
	this->debugCircles.push_back(a);
}

void p010382iTank::Render()
{
	BaseTank::Render();

	/*for (int i = 0; i < debugLines.size(); ++i)
	{
		DrawDebugLine(debugLines.at(i).first, debugLines.at(i).second, 255, 0, 255);
	}

	for (int i = 0; i < debugCircles.size(); ++i)
	{
		DrawDebugCircle(debugCircles.at(i), 10.0, 255, 0, 255);
	}*/

	/*for (int i = 0; i < obstaclesRectangles.size(); i++)
	{
		SDL_SetRenderDrawColor(mRenderer, 255, 255, 255, 255);
		SDL_RenderFillRect(mRenderer, &obstaclesRectangles.at(i));
	}*/
}

void p010382iTank::MoveInHeadingDirection(float deltaTime)
{
	Vector2D force = (steeringForce * 75.0) - mVelocity;//mCurrentSpeed

	//Acceleration = Force/Mass
	Vector2D acceleration = force / GetMass();

	//Update velocity.
	mVelocity += acceleration * deltaTime;
	
	//Don't allow the tank does not go faster than max speed.
	mVelocity.Truncate(GetMaxSpeed()); //TODO: Add Penalty for going faster than MAX Speed.

	//Finally, update the position.
	Vector2D newPosition = GetPosition();
	newPosition += mVelocity * deltaTime;
	SetPosition(newPosition);

	//update the heading if the vehicle has a non zero velocity
	if (!mVelocity.isZero())
	{
		mHeading = mVelocity;
		mHeading.Normalize();

		mSide = mHeading.Perp();

		mRotationAngle = 90.0 - (atan2(mHeading.y, -mHeading.x) * 180.0) / Pi;
	}
}

//--------------------------------------------------------------------------------------------------

vector<BaseTank*> p010382iTank::GetTanksCanHear()
{
	return this->mTanksICanHear;
}

vector<BaseTank*> p010382iTank::GetTanksCanSee()
{
	return this->mTanksICanSee;
}

vector<BaseTank*> p010382iTank::GetTanksCanSeeAndHear()
{
	vector<BaseTank*> allTanks = GetTanksCanSee();
	vector<BaseTank*> tankCanHear = GetTanksCanHear();
	for (int i = 0; i < tankCanHear.size(); i++)
	{
		allTanks.push_back(tankCanHear.at(i));
	}

	return allTanks;
}

void p010382iTank::FireWeapon(int ClosestObject, vector<BaseTank*> agents)
{
	//fore at tank
	//Rotate man to face enemy tank.
	Vector2D toTarget = agents[ClosestObject]->GetCentralPosition() - GetCentralPosition();
	toTarget.Normalize();
	double dot = toTarget.Dot(mManFireDirection);
	if (dot < 0.95f)
		RotateManByRadian(kManTurnRate, -1, mdTimeElapsed);

}


void p010382iTank::LoadObstacleRects()
{
	vector<GameObject*> obstacles = ObstacleManager::Instance()->GetObstacles();

	for (int i = 0; i < obstacles.size(); i++)
	{
		vector<Vector2D> points = obstacles.at(i)->GetAdjustedBoundingBox();

		int xmin = INT_MAX, xmax = -INT_MAX, ymin = INT_MAX, ymax = -INT_MAX;

		for (int j = 0; j < points.size(); j++)
		{
			xmin = min(xmin, points[j].x);
			xmax = max(xmax, points[j].x);

			ymin = min(ymin, points[j].y);
			ymax = max(ymax, points[j].y);
		}
		this->obstaclesRectangles.push_back({ xmin, ymin, xmax - xmin, ymax - ymin });
	}
}



//--------------------------------------------------------------------------------------------------