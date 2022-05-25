//#pragma once
#ifndef SIM_HPP
#define SIM_HPP
#include <glm/glm.hpp>
#include <vector>
#include <optional>
#include <iostream>

using namespace glm;

static const int NUMITERS = 300; //guessing?

inline float crossZ(vec2 a,vec2 b){
	return a.x*b.y - a.y*b.x;
}

struct Position {

	glm::vec2 position;
	glm::vec2 predict;
	glm::vec2 velocity = vec2(0,0);
	bool pinned = false; 


	float mass = .001f;

	Position(glm::vec2 pos): position(pos), predict(pos) {}
	Position(glm::vec2 pos, bool ispinned): position(pos), predict(pos), pinned(ispinned) {}
	
};

using PositionList = std::vector<Position>;

struct AngConstraint {

	PositionList::iterator pos1, pos2, posc;
	vec2 d1,d2,d3;

	float alpha;
	float lambda = 0;
	float theta0;

	AngConstraint(PositionList::iterator apos1, 
				  PositionList::iterator apos2, 
				  PositionList::iterator aposc, 
				  float analpha): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha) {

		vec2 s1 = apos1->position - aposc->position;
		vec2 s2 = apos2->position - aposc->position;
		float s1xs2 = crossZ(s1,s2);//s1.x*s2.y - s1.y*s2.x;
		float s1ds2 = dot(s1, s2);
		theta0 = 0.0;//atan2(s1xs2, s1ds2);
		std::cout<<atan2(s1xs2, s1ds2)<<std::endl;
	};

	AngConstraint(PositionList::iterator apos1, 
				  PositionList::iterator apos2, 
				  PositionList::iterator aposc, 
				  float analpha, float atheta0): pos1(apos1), pos2(apos2), posc(aposc), alpha(analpha), theta0(atheta0) {}

};

using AngConstraintList = std::vector<AngConstraint>;


struct LinConstraint {

	PositionList::iterator pos1, pos2;

	float alpha;
	float lambda = 0;
	float restlength;
	float original_length;
	std::optional<AngConstraintList::iterator> begin_ang = std::nullopt; 
	std::optional<AngConstraintList::iterator> end_ang = std::nullopt; 


	LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha): pos1(apos1), pos2(apos2), alpha(analpha) {

		restlength = glm::length(pos1->position - pos2->position);
		original_length = length(apos2->position - apos1->position);
	}

	LinConstraint(PositionList::iterator apos1, PositionList::iterator apos2, float analpha, float arestlength): pos1(apos1), pos2(apos2), alpha(analpha), restlength(arestlength) {
		original_length = length(apos2->position - apos1->position);
	}




	float currentlength() {return length(pos1->position - pos2->position);}
	

};

using LinConstraintList = std::vector<LinConstraint>;

struct AcceleratorGrid {

	unsigned long current_step;

	struct Bucket {

		std::vector<LinConstraintList::iterator> walls;
		unsigned long step;

		Bucket(unsigned long astep): step(astep) {}
	};

	std::vector<Bucket> buckets;
	float min_x, max_x, min_y, max_y, length_x, length_y, size_x, size_y;
	size_t stride;

	AcceleratorGrid(vec2 amin, vec2 amax, vec2 step); 
	Bucket &get(vec2 pos);


	void add(LinConstraintList::iterator linconstraint);


	bool collide(PositionList::iterator position_i, float deltat);
};


void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, AcceleratorGrid& grid, float deltat, bool gravity, bool use_bounds, bool use_collisions);

#endif