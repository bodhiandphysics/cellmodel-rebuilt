#include "givr.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cstdlib>
#include <iostream>



#include "givio.h"
#include "sim.hpp"
#include "panel.h"

using namespace glm;
using namespace givr;
using namespace givr::camera;
using namespace givr::geometry;
using namespace givr::style;

int main(void)
{
  namespace givio = giv::io; // perhaps better than giv::io
  givio::GLFWContext glContext;
  glContext.glMajorVesion(4)
      .glMinorVesion(0)
      .glForwardComaptability(true)
      .glCoreProfile()
      .glAntiAliasingSamples(4)
      .matchPrimaryMonitorVideoMode();

  std::cout << givio::glfwVersionString() << '\n';

  //
  // setup window (OpenGL context)
  //
  auto window =
      glContext.makeImGuiWindow(givio::Properties()
                                    .size(givio::dimensions{1000, 1000})
                                    .title("Curve surfing...")
                                    .glslVersionString("#version 330 core"));

  auto view = View(TurnTable(), Perspective());

  view.camera.translate(vec3(0,0,-5));

  auto linestyle = GL_Line(Width(2.), Colour(1.0, 1.0, 0.0));
  auto linestyle2 = GL_Line(Width(2.), Colour(1.0, 0.3, 0.3));
  auto linestyle3 = GL_Line(Width(2.), Colour(0.3, 1.0, 0.3));
  auto linestyle4 = GL_Line(Width(2.), Colour(0.6, 0.8, 1.0));
  auto boxstyle = GL_Line(Width(2.), Colour(1.0, 1.0, 1.0));


  PositionList positions;
  LinConstraintList linconstraints;
  AngConstraintList angconstraints;

  float linalpha = .000000001f;
  float angalpha = .00000001f; 
  int nsegs = 50;
  float original_length = 20.f;

  srand((unsigned)time(0));

  float x0 = -original_length/2;
  float y = 0; 
  float x = x0;
  positions.emplace_back(vec2(x,y), true);
  float dL = original_length/(float)nsegs;
  for (float n = 1; n < nsegs; n++) {
    x = dL*n+x0;
    y = (x*x-x0*x0)*0.01;//0.1*(rand() - RAND_MAX/2)/float(RAND_MAX);
    std::cout<<dL*n<<":"<<original_length<<std::endl;
    positions.emplace_back(vec2(x,y));
  }

  x = original_length/2;
  y = 0;

  positions.emplace_back(vec2(x,y), true);

  for (auto pos = positions.begin(); pos != positions.end()-1; pos++) {

    linconstraints.emplace_back(pos, pos+1, linalpha);
    if (pos != positions.begin())
     angconstraints.emplace_back(pos-1, pos, pos+1, angalpha);
   }

  AcceleratorGrid grid = AcceleratorGrid(vec2(-original_length/2 - 1.f), vec2(original_length/2 + 1.f), vec2(.5f));
  float sub_step = 0.1;
  std::vector<std::vector<Line>> animation;
  glClearColor(0.f, 0.f, 0.f, 0.f);
  mainloop(std::move(window), [&](float frame_time) {
    float frame_advance = 1.0;//frame_time;

    bool gravity = false;
    bool bounds = true;
    bool collisions = false;

    if (panel::play){ 
      for (float deltat = 0.f; deltat < frame_advance; deltat += sub_step)
        sim_iteration(positions, linconstraints, angconstraints, grid, sub_step, gravity, bounds, collisions); //use gravity, don't use bounds

     } else {
      if (panel::single_step) {
        sim_iteration(positions, linconstraints, angconstraints, grid, sub_step, gravity, bounds, collisions); //use gravity, don't use bounds
        panel::single_step = false;
      }
    }



    glClearColor(0.f, 0.f, 0.f, 0.f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    auto wall = MultiLine();
    for (auto constraint: linconstraints) {

      auto p1 = constraint.pos1;
      auto p2 = constraint.pos2;
      wall.push_back(Line(Point1(p1->position.x, p1->position.y, 0),Point2(p2->position.x, p2->position.y, 0))); 


    }
   auto box = MultiLine();
   box.push_back(Line(Point1(vec2(-original_length/2.0 ,-original_length/2),0),Point2(vec2(-original_length/2.0 ,original_length/2),0)));
   box.push_back(Line(Point1(vec2(-original_length/2.0 ,-original_length/2),0),Point2(vec2(original_length/2.0 ,-original_length/2),0)));
   box.push_back(Line(Point1(vec2(original_length/2.0 ,original_length/2),0),Point2(vec2(-original_length/2.0 ,original_length/2),0)));
   box.push_back(Line(Point1(vec2(original_length/2.0 ,original_length/2),0),Point2(vec2(original_length/2.0 ,-original_length/2),0)));

   auto f1 = MultiLine();
   auto f2 = MultiLine();
   auto f3 = MultiLine();
   // for (auto constraint: angconstraints) {
   //  f1.push_back(Line(Point1(constraint.pos1->position,0),Point2(10.0f*constraint.d1+constraint.pos1->position,0)));
   //  f2.push_back(Line(Point1(constraint.posc->position,0),Point2(10.0f*constraint.d2+constraint.posc->position,0)));
   //  f3.push_back(Line(Point1(constraint.pos2->position,0),Point2(10.0f*constraint.d3+constraint.pos2->position,0)));
   // }

    auto cellwall = createRenderable(wall, linestyle);
    draw(cellwall, view);
    auto f1rend = createRenderable(f1,linestyle2);
    draw(f1rend,view);
    auto f2rend = createRenderable(f2,linestyle3);
    draw(f2rend,view);
    auto f3rend = createRenderable(f3,linestyle4);
    draw(f3rend,view);
    auto boxrend = createRenderable(box,boxstyle);
    draw(boxrend,view);
    
  });
  exit(EXIT_SUCCESS);
}