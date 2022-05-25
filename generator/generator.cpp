#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>



struct Position {

	float x,y;
	Position(float anx, float ay): x(anx), y(ay) {}
};

struct LinConstraint {

	size_t pos1, pos2;

	LinConstraint(size_t ap1, size_t ap2): pos1(ap1), pos2(ap2) {}
};

struct AngConstraint {

	size_t pos1, pos2, posc;
	float ang0;

	AngConstraint(size_t ap1, size_t ap2, size_t ap3, float a0): pos1(ap1), pos2(ap2), posc (ap3), ang0(a0) {}
};

int main(int argc, char const *argv[]) {
	if (argc != 3) {
		std::cout << "Needs a filename and an output name\n";
		exit(0);
	}

	std::ifstream input_file(argv[1], std::ifstream::in);

	float bounds;
	float linalpha;
	float angalpha;
	std::vector<Position> positions;
	std::vector<LinConstraint> linconstraints;
	std::vector<AngConstraint> angconstraints;


	input_file >> bounds;
	input_file >> linalpha;
	input_file >> angalpha;


	float xmin, ymin, xstart, ystart, dx1, dy1, dx2, dy2;
	int max_i, max_j;
	int num_verts;
	float vertx, verty;
	std::vector<Position> verts;
	std::vector<long> position_map; 
	std::vector<size_t> vert_map;
	std::vector<bool> already_in;
	std::vector<float> ang0s;
	size_t end_vert = 0;

	input_file >> xmin;
	input_file >> ymin;
	input_file >> xstart;
	input_file >> ystart;
	input_file >> max_i;
	input_file >> max_j;
	input_file >> dx1;
	input_file >> dy1;
	input_file >> dx2;
	input_file >> dy2;
	input_file >> num_verts;

	size_t stride = 0;

	for (float x = xmin; x < bounds/2; x += .05) {
		stride++;
		for (float y = ymin; y < bounds/2; y += .05)
			position_map.push_back(-1);
	}

	for (int i = 0; i < num_verts; i++) {

		input_file >> vertx;
		input_file >> verty;
		verts.push_back(Position(vertx, verty));
		vert_map.push_back(0);
		already_in.push_back(false);
	}

	for (int i = 0; i < num_verts; i++) {
		float ang0;
		input_file >> ang0;
		ang0s.push_back(ang0);
	}

	for  (int i = 0; i < max_i; i++) {
		for (int j = 0; j < max_j; j++) {

			for (int num = 0; num < verts.size(); num++) {

				size_t xindex = floor(verts[num].x + i * dx1 + j * dx2 - xmin);
				size_t yindex = floor(verts[num].y + i * dy1 + j * dy2 - ymin);
				size_t index = yindex * stride + xindex;

				if (position_map[index] == -1) {
					positions.push_back(Position(xstart + verts[num].x + i *dx1 + j * dx2, ystart + verts[num].y + i * dy1 + j * dy2));
					position_map[index] = end_vert;
					vert_map[num] = end_vert++;
					already_in[num] = false;

				} else {
					vert_map[num] = position_map[index];
					already_in[num] = true;
				}
			}

			for (int num = 0; num < verts.size(); num++) {

				size_t vert1 = vert_map[num];
				size_t vert2 = vert_map[(num + 1) % verts.size()];
				bool doit = true;

				for (auto const &constraint: linconstraints) {
					if ((constraint.pos1 == vert1 && constraint.pos2 == vert2) ||
						(constraint.pos1 == vert2 && constraint.pos1 == vert1)) {

						doit = false;
						break;
					}
				}
				if (doit) linconstraints.push_back(LinConstraint(vert1, vert2));
			}

			for (int num = 0; num < verts.size(); num++)
				angconstraints.push_back(AngConstraint(vert_map[(num-1) % verts.size()], vert_map[(num+1) % verts.size()], vert_map[num], ang0s[num]));
		}
	}

	std::ofstream outfile(argv[2], std::fstream::out | std::fstream::trunc);

	outfile <<bounds << "\n";

	outfile << positions.size() << "\n";

	for (auto &pos: positions) 
		outfile << pos.x << " " << pos.y << " " << 0 << " " << 0 << " " <<  false << "\n";

	outfile << linconstraints.size() << "\n";

	for (auto &lc: linconstraints) {

		size_t indexa = lc.pos1;
		size_t indexb = lc.pos2;

		outfile << indexa << " " << indexb << " " << linalpha << "\n";
	}

	outfile << angconstraints.size() << "\n";
	for (auto &ac: angconstraints) {

		size_t indexa = ac.pos1;
		size_t indexb = ac.pos2;
		size_t indexc = ac.posc;
		float ang0 = ac.ang0;

		outfile << indexa << " " << indexb << " "  << indexc << " " << ang0 << angalpha << "\n";

	}
	
}