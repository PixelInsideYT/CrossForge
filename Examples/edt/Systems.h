#ifndef CFORGESANDBOX_SYSTEMS_H
#define CFORGESANDBOX_SYSTEMS_H

#include <flecs.h>
#include <iostream>
#include "Components.h"

namespace CForge {

    class Systems {
    public:
        float waterDecreaseRate = 0.1;
        float waterIncreaseRate = 1.0;

        static void reduceWaterLevel(flecs::world& world) {
            world.query<PlantComponent>()
                .iter([&world](flecs::iter it, PlantComponent* p) {
                for (int i : it) {
                    if (p[i].waterLevel > 0) {
                        p[i].waterLevel -= waterDecreaseRate * it.delta_time();
                    }
                    else {
                        p[i].waterLevel = 0; // Ensure the water level doesn't go negative
                    }
                }
            });
        }


    };

} // CForge

#endif //CFORGESANDBOX_SYSTEMS_H