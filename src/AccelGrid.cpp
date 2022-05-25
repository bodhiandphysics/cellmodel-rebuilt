#include "sim.hpp"

namespace Sim {

	AcceleratorGrid::AcceleratorGrid(vec2 amin, vec2 amax, vec2 step) {
			current_step = 0;
			min_x = amin.x;
			min_y = amin.y;
			max_x = amax.x;
			max_y = amax.y;
			length_x = max_x - min_x;
			length_y = max_y - max_x;
			size_x = step.x;
			size_y = step.y;
			stride = floor(length_x/size_x);

			for (float y = min_y; y <= max_y; y += size_y)
				for (float x = min_x; x <= max_x; x += size_x)
					buckets.emplace_back(1);	
		}

	AcceleratorGrid::Bucket &AcceleratorGrid::get(vec2 pos) {

		size_t x_coord = floor((pos.x - min_x)/size_x);
		size_t y_coord = floor((pos.y - min_y/size_y));

		Bucket &result = buckets[stride*y_coord + x_coord];
		if (result.step != current_step) {
			result.step = current_step;
			result.walls.clear();
		}

		return result;
	}



	void AcceleratorGrid::add(LinConstraintList::iterator linconstraint) {

		vec2 pos1min = glm::min(linconstraint->pos1->position, linconstraint->pos1->predict);
		vec2 pos1max = glm::max(linconstraint->pos1->position, linconstraint->pos1->predict);
		vec2 pos2min = glm::min(linconstraint->pos2->position, linconstraint->pos2->predict);
		vec2 pos2max = glm::max(linconstraint->pos2->position, linconstraint->pos2->predict);
		vec2 boxmin = glm::min(pos1min, pos2min);
		vec2 boxmax = glm::max(pos1max, pos2max);
		for (float x = boxmin.x; x <= boxmax.x; x += size_x)
			for (float y = boxmin.y; y <= boxmax.y; y += size_y) {
				auto &bucket = get(vec2{x,y});
				bucket.walls.push_back(linconstraint);

			}
	}


	bool AcceleratorGrid::collide(PositionList::iterator position_i, float deltat) {

		bool did_collide = false;


		vec2 pcv = position_i->velocity;
		vec2 pbegin = position_i->position;
		vec2 pend = position_i->predict;

		vec2 pos_max = glm::max(pbegin, pend);
		vec2 pos_min = glm::min(pbegin, pend);

		float collision_time = 1000000.f;

		for (float x = pos_min.x; x <= pos_max.x; x += size_x)
			for (float y = pos_min.y; y <= pos_max.y; y += size_y) {

				const auto &bucket = get(vec2{x,y});

				for (auto &linconstraint: bucket.walls) {

					if (linconstraint->pos1 == position_i || linconstraint->pos2 == position_i) continue;		

					vec2 p1 = linconstraint->pos1->position;
					vec2 p2 = linconstraint->pos2->position;
					vec2 pc = position_i->position;
					vec2 pcv = (position_i->predict - pc)/deltat;
					vec2 p1v = (linconstraint->pos1->predict - linconstraint->pos1->position)/deltat;
					vec2 p2v = (linconstraint->pos2->predict - linconstraint->pos2->position)/deltat;

					vec2 wall = p2 - p1;
					vec2 wallv = p2v - p1v;
					vec2 cross = pc - p1;
					vec2 cross2 = pc - p2;
					vec2 crossv = pcv - p1v;

					float a, b, c; // quadratic equation

					a = crossZ(wallv, crossv);
					b = crossZ(wall, crossv) + crossZ(wallv, cross);
					c = crossZ(wall, cross);

					float descr = b*b - 4*a*c;
					if (descr < 0) continue; //no collision
					bool did_collide_this = false;
					float new_collision_time = 0.f;
					float tbig = (-b + sqrt(descr))/(2*a);

					if (tbig >= 0 && tbig < deltat && tbig <= collision_time)
						new_collision_time = tbig, did_collide_this = true;

					float tsmall = (-b - sqrt(descr))/(2*a);

					if (tsmall >= 0 && tsmall < tbig && tsmall < deltat && tsmall <= collision_time)
						new_collision_time = tsmall, did_collide_this = true;

					if (length(cross) < .001 || length(cross2) < .001)
						collision_time = 0, did_collide_this = true;


					if (did_collide_this) {

						vec2 collision_p = pc + pcv*new_collision_time;
						if (dot(collision_p - p1, wall) < 0  || length(collision_p -p1) > length(wall)) {
							std::cout << "did not collide\n";
							did_collide_this = false;
							continue;
						}
						
						std::cout << "collided\n";
						collision_time = new_collision_time;
						did_collide = true;
						
						linconstraint->pos1->predict = linconstraint->pos1->position + p1v * collision_time;
						linconstraint->pos2->predict = linconstraint->pos2->position + p2v * collision_time;


						vec2 walld = normalize(linconstraint->pos2->predict - linconstraint->pos1->predict);
						vec2 walln(walld.y, -walld.x);


						position_i->predict = dot((collision_p - linconstraint->pos1->predict), walld)*walld + linconstraint->pos1->predict + walln*.4f;

						if (dot(pcv, walln) < 0) {
							position_i->velocity = -dot(pcv, walln)*walln;
							linconstraint->pos1->velocity = -position_i->velocity;
							linconstraint->pos2->velocity = -position_i->velocity;
						}

						//position_i->velocity = pcv + deltav;
						// linconstraint->pos1->velocity = p1v - deltav;
						// linconstraint->pos2->velocity = p2v - deltav;

						
					}
					
				}
		}
		return did_collide;
	}	
}
