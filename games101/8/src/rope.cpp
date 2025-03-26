#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL
{

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        //        Comment-in this part when you implement the constructor
        //        for (auto &i : pinned_nodes) {
        //            masses[i]->pinned = true;
        //        }

        // Home Work Begin
        for (int i = 0; i < num_nodes; ++i)
        {
            // 转成double再做除法，否则就会截断成int
            Vector2D pos = start + (end - start) * (double)i / (num_nodes - 1.0);
            masses.push_back(new Mass(pos, node_mass, false)); // 放质点
        }
        for (int i = 0; i < num_nodes - 1; ++i)
            springs.push_back(new Spring(masses[i], masses[i + 1], k)); // 放绳子
        for (auto &i : pinned_nodes)
            masses[i]->pinned = true;
        // Home Work End
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            // Home Work Begin
            auto mod_ab = (s->m1->position - s->m2->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / mod_ab * (mod_ab - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / mod_ab * (mod_ab - s->rest_length);
            // Home Work End
        }

        // Home Work Begin
        // damping coefficient
        float damping_factor = 0.00005;
        // Home Work End

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position

                // Home Work Begin
                // auto a = m->forces / m->mass + gravity;
                auto a = m->forces / m->mass + gravity - damping_factor * m->velocity / m->mass;
                // Home Work End

                // TODO (Part 2): Add global damping

                // Home Work Begin
                // For explicit method
                // m->position += m->velocity * delta_t;
                // m->velocity += a * delta_t;
                // For semi-implicit method
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                // Home Work End
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)

            // Home Work Begin
            auto mod_ab = (s->m1->position - s->m2->position).norm();
            s->m1->forces += -s->k * (s->m1->position - s->m2->position) / mod_ab * (mod_ab - s->rest_length);
            s->m2->forces += -s->k * (s->m2->position - s->m1->position) / mod_ab * (mod_ab - s->rest_length);
            // Home Work End
        }

        // Home Work Begin
        // damping coefficient
        float damping_factor = 0.00005;
        // Home Work End

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass

                // Home Work Begin
                auto a = m->forces / m->mass + gravity;
                // Home Work End

                // TODO (Part 4): Add global Verlet damping

                // Home Work Begin
                // m->position = temp_position + (temp_position - m->last_position) + a * delta_t * delta_t;
                m->position = temp_position + (1 - damping_factor) * (temp_position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
                // Home Work End
            }

            // Home Work Begin
            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
            // Home Work End
        }
    }
}
