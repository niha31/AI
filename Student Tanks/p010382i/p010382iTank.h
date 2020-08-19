//------------------------------------------------------------------------
//  Author: Paul Roberts (2019)
//------------------------------------------------------------------------
#ifndef P010382ITANK_H
#define P010382ITANK_H

#include "../../Commons.h"

#include <commctrl.h>
#include <SDL.h>

#include "../../BaseTank.h"
#include "p010382iSteeringBehaviors.h"
#include "../../WaypointManager.h"
#include "../../Waypoint.h"
#include "../../PickUpManager.h"


//---------------------------------------------------------------

class p010382iTank : public BaseTank
{
	//---------------------------------------------------------------
public:
	p010382iTank(SDL_Renderer* renderer, TankSetupDetails details);
	~p010382iTank();

	void ChangeState(BASE_TANK_STATE newState);

	void Update(float deltaTime, SDL_Event e);

	double TimeElapsed()const { return mdTimeElapsed; }

	vector<BaseTank*> GetTanksCanHear();
	vector<BaseTank*> GetTanksCanSee();
	vector<BaseTank*> GetTanksCanSeeAndHear();
	void LoadObstacleRects();

	void SetTargetTank(Vector2D tank) { targetTank = tank; }

	void SetObsticleSeen(bool seen) { obsticlSeen = seen; }
	void SetWallSeen(bool seen) { wallSeen = seen; }

	void AddDebugLine(Vector2D a, Vector2D b);
	void AddDebugCircle(Vector2D a);

	void FireWeapon(int ClosestObject, vector<BaseTank*> agents);

	vector<SDL_Rect>* ReturnObsticleRect() { return &obstaclesRectangles; }

	void Render();

	void GoToTargetWaypoint(float deltaTime);
	bool findCrates = true;;

	//---------------------------------------------------------------
protected:
	void	MoveInHeadingDirection(float deltaTime);

private:

	double                mdTimeElapsed;
	SteeringBehavior*     mSteeringBehavior;
	Vector2D steeringForce;
	bool obsticlSeen;
	bool wallSeen;
	vector<SDL_Rect> obstaclesRectangles;

	vector<Vector2D> path;

	Vector2D targetTank;

	std::vector<std::pair<Vector2D, Vector2D>> debugLines;
	std::vector<Vector2D> debugCircles;

};

//---------------------------------------------------------------

#endif //P010382ITANK_H