#ifndef STEERINGSYSTEM_H
#define STEERINGSYSTEM_H

#include <flecs.h>
#include "crossforge/Graphics/SceneGraph/SGNTransformation.h"
#include "AIComponent.h"
#include "SteeringComponent.h"
#include "crossforge/Graphics/SceneGraph/SGNGeometry.h"

namespace CForge {

    class SteeringSystem {
    public:
        static void addSteeringSystem(flecs::world& world);

        static void processEntity(float dt, AIComponent& ai, SGNTransformation& p, SteeringComponent& sc, SGNGeometry& geo, flecs::world& world);

        static bool obstacleIsInPath(SGNTransformation& p,Eigen::Vector3f& target, Eigen::Vector3f& obstaclePosition, float obstacleRadius, float robotRadius);

        static bool obstacleAvoidance(SGNTransformation& p, flecs::world& world, Eigen::Vector3f& target, Eigen::Vector3f& obstacle, float obstacleRadius, float roboterRadius, float securityDistance);

        static bool arrivedAtWayPoint(Eigen::Vector3f position, Eigen::Vector3f target);

        static void seekingBehavior(float dt, Eigen::Vector3f targetPosition, SGNTransformation& p);
    };

} // CForge

#endif // STEERINGSYSTEM_H
