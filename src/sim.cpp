#include "sim.hpp"
#include <cstdlib>
#include <iostream>
#include <algorithm>

float originallength = 20.f;
float conv_thresh = 5;//1e-10;
float growth_mult = 0.1;

void sim_iteration(PositionList &positions, LinConstraintList &linconstraints, AngConstraintList &angconstraints, AcceleratorGrid& grid, float deltat, bool gravity, bool use_bounds, bool use_collisions) {



	//update prodicted positions

	for (auto &position: positions) {
		position.velocity*=0.95; //Damping
		position.predict = position.position + position.velocity * deltat;
		if (gravity) position.predict += vec2(0.f, -9.8f)*deltat*deltat; //gravity
	}

	grid.current_step++;
	for (auto i = linconstraints.begin(); i != linconstraints.end(); i++)
		grid.add(i);

	for (auto i = positions.begin(); i != positions.end(); i++)
		grid.collide(i, deltat);


		


	for (auto &linconstraint: linconstraints) linconstraint.lambda = 0; // zero the lambdas for every frame
	for (auto &angconstraint: angconstraints) angconstraint.lambda = 0; // note that colconstraints don't need lambda

	// for each constraint use interated solver to solve constraint

	bool done = false;
	float convergence_distance = 0;
	float convergence_distance_last = 0; 
	float convergence_max = 0;
	float allow_flag = true;
	int num_its = 0;

	//Blindly iterating for 20 steps works well in most cases
	while (!done && num_its<500) {
		num_its++;
		convergence_distance = 0; 
		convergence_max = 0;


	// first linear constraints
		float lin_const_weight = 1.0;
		for (auto &linconstraint: linconstraints) {

			float m1 = linconstraint.pos1->mass;
			float m2 = linconstraint.pos2->mass;
			vec2 pred1 = linconstraint.pos1->predict;
			vec2 pred2 = linconstraint.pos2->predict;
			vec2 diff = pred1 - pred2;

			float modalph = linconstraint.alpha/(deltat*deltat);

			// C(p)

			float distance =  length(diff);

			if (distance < 1e-20) 
				distance = 1e-20;

			float distance2 = distance * distance;
			auto constraintval = distance - linconstraint.restlength;
								
			// delC * 1/m * delCT					 

			float cinnerval = 1.f/(distance*m1) + 1.f/(distance*m2);

			if (isnan(cinnerval)) cinnerval = 0.f;

			float deltal = (-constraintval - modalph*linconstraint.lambda) / (cinnerval + modalph);

			vec2 delta1 = (1.f/(distance*m1)) *diff*deltal;
			vec2 delta2 = -delta1;


			linconstraint.lambda += deltal;

			if (linconstraint.pos1->pinned || isnan(delta1.x+delta1.y)) {
				delta1 *= 0;
			}
			if (linconstraint.pos2->pinned || isnan(delta2.x+delta2.y)) {
				delta2 *= 0;
			}


			linconstraint.pos1->predict += delta1*lin_const_weight;
			linconstraint.pos2->predict += delta2*lin_const_weight;

			//Modified convergence condition
			convergence_distance += length(delta1*deltal)+length(delta2*deltal);//deltax1*deltax1 + deltay1*deltay1 + deltax2*deltax2 + deltay2*deltay2;
			convergence_max = std::max({convergence_max, delta1.x, delta1.y, delta2.x, delta2.y});

		} 



		// Now do angular constraints

		float ang_const_weight = 0.005;
		for (auto &angconstraint: angconstraints) {

			float x1 = angconstraint.pos1->predict.x;
			float x2 = angconstraint.pos2->predict.x;
			float xc = angconstraint.posc->predict.x;
			float y1 = angconstraint.pos1->predict.y;
			float y2 = angconstraint.pos2->predict.y;
			float yc = angconstraint.posc->predict.y;
			float s1x = x1 - xc;
			float s1y = y1 - yc;
			float s2x = x2 - xc;
			float s2y = y2 - yc;
			float m1 = angconstraint.pos1->mass;
			float m2 = angconstraint.pos2->mass;
			float mc = angconstraint.posc->mass;
			vec2 pos1 = angconstraint.pos1->predict;
			vec2 pos2 = angconstraint.pos2->predict;
			vec2 posC = angconstraint.posc->predict;
			vec2 s1 = pos1 - posC;
			vec2 s2 = pos2 - posC;

			float modalph = angconstraint.alpha/(deltat*deltat);

			float s1ds2 = dot(s1,s2);//s1x*s2x + s1y*s2y;
			float s1xs2 = crossZ(s1,s2);//s1x*s2y - s1y*s2x;

			float constraintval = atan2(s1xs2,s1ds2) - angconstraint.theta0;
			vec2 dot_cross(s1ds2,s1xs2);

			float datan2 = 1.f / (dot(dot_cross,dot_cross));//1.f / (s1ds2*s1ds2 + s1xs2*s1xs2); 

			float delcx1 = datan2 * ((s1ds2*s2.y) - (s1xs2*s2.x));
			float delcx2 = datan2 * (-(s1ds2*s1.y) - (s1xs2*s1.x));

			float delcy1 = datan2 * (-(s1ds2*s2.x) - (s1xs2*s2.y));
			float delcy2 = datan2 * ((s1ds2*s1.x) - (s1xs2*s1.y));
			
			float delcxc = -delcx1-delcx2;
			float delcyc = -delcy1-delcy2;
//			vec2 delc1(crossZ(dot_cross,s2),-dot(dot_cross,s2));
//			vec2 delc2(-dot(dot_cross,s1),crossZ(dot_cross,s1));
//			vec2 delcc(delc1.x)
//			delc1*=datan2;
//			delc2*=datan2;

			float cinnerval = (1.f/m1)*(delcx1*delcx1 + delcy1*delcy1) +
							  (1.f/m2)*(delcx2*delcx2 + delcy2*delcy2) +
							  (1.f/mc)*(delcxc*delcxc + delcyc*delcyc);


			float deltal = (-constraintval - modalph*angconstraint.lambda) / (cinnerval + modalph);



			float deltax1 = (1.f/m1)*delcx1*deltal;
			float deltax2 = (1.f/m2)*delcx2*deltal;
			float deltay1 = (1.f/m1)*delcy1*deltal;
			float deltay2 = (1.f/m2)*delcy2*deltal;
			float deltaxc = (1.f/mc)*delcxc*deltal;
			float deltayc = (1.f/mc)*delcyc*deltal;



			angconstraint.lambda += deltal;
			angconstraint.d1 = vec2(deltax1,deltay1);
			angconstraint.d2 = vec2(deltaxc,deltayc);
			angconstraint.d3 = vec2(deltax2,deltay2);

			if (angconstraint.pos1->pinned) {
				deltax1 = 0.f;
				deltay1 = 0.f;
			}
			if (angconstraint.pos2->pinned) {
				deltax2 = 0.f;
				deltay2 = 0.f;
			}
			
			if (angconstraint.posc->pinned) {
				deltaxc = 0.f;
				deltayc = 0.f;
			}


				if(isnan(deltax1))
					deltax1 = 0;
				if(isnan(deltay1))
					deltay1 = 0;
				if(isnan(deltax2))
					deltax2 = 0;
				if(isnan(deltay2))
					deltay2 = 0;
				if(isnan(deltaxc))
					deltaxc = 0;
				if(isnan(deltayc))
					deltayc = 0;


			angconstraint.pos1->predict.x += deltax1*ang_const_weight;
			angconstraint.pos1->predict.y += deltay1*ang_const_weight;
			angconstraint.pos2->predict.x += deltax2*ang_const_weight;
			angconstraint.pos2->predict.y += deltay2*ang_const_weight;
			angconstraint.posc->predict.x += deltaxc*ang_const_weight;
			angconstraint.posc->predict.y += deltayc*ang_const_weight;


			convergence_distance += sqrt(deltax1*deltax1 + deltay1*deltay1) + sqrt(deltax2*deltax2 + deltay2*deltay2) + sqrt(deltaxc*deltaxc + deltayc*deltayc);
			convergence_max = std::max({convergence_max, deltax1, deltay1, deltax2, deltay2, deltaxc, deltayc});

		}

		grid.current_step++;
		for (auto i = linconstraints.begin(); i != linconstraints.end(); i++)
			grid.add(i);

		for (auto i = positions.begin(); i != positions.end(); i++)
			grid.collide(i, deltat);

			





//		condition ignored for the moment
		//if ((convergence_distance - convergence_distance_last) <= .01)
		//	done = true;
		//else {
		//	done = false;
		//	convergence_distance_last = convergence_distance;
		//}
		if(num_its%25==0)
			std::cout<<"Current convergence: "<<convergence_distance<<" and delta "<< convergence_distance - convergence_distance_last<<std::endl;
		convergence_distance_last = convergence_distance;
}

	//udate the accelerator gird
	
	grid.current_step++;
	for (auto i = linconstraints.begin(); i != linconstraints.end(); i++)
		grid.add(i);


	// update positions and velcities
	for (auto i = positions.begin(); i != positions.end(); i++) {

		bool collided = grid.collide(i, deltat);

		auto& position = *i;

		if (!position.pinned) {
			if (!collided) position.velocity.x = (position.predict.x - position.position.x) / deltat;
			if (!collided) position.velocity.y = (position.predict.y - position.position.y) / deltat;
			position.position.x = position.predict.x;
			position.position.y = position.predict.y;



			if (use_bounds) {
				if (position.position.x < -originallength/2) {
			
					// position.velocity.x = -position.velocity.x; 
					position.position.x = -originallength/2;
				}
	
				if (position.position.x > originallength/2) {
	
					// position.velocity.x = -position.velocity.x; 
					position.position.x = originallength/2;
				}
	
				if (position.position.y < -originallength/2) {
	
					// position.velocity.y = -position.velocity.y; 
					position.position.y = -originallength/2.f;// + (float)(rand()%1000)/10000.0f;
				}
	
				if (position.position.y > originallength/2) {
	
					// position.velocity.y = -position.velocity.y; 
					position.position.y = originallength/2;
				}
			}
		}
	}

	std::cout<<"Current error: "<< convergence_distance <<std::endl;


	float grate = growth_mult*(1.f/linconstraints.size());
    //std::cout<<"Growth delta:"<<deltat<<" Growth rate:"<<grate<<std::endl;

	for (auto &constraint: linconstraints){
         constraint.restlength += grate*deltat; // add some growth
       } 
}
