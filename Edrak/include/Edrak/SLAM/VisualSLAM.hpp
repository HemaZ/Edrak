#ifndef __EDRAK__SLAM_VISUAL_SLAM__HPP__
#define __EDRAK__SLAM_VISUAL_SLAM__HPP__
#include "Backend.hpp"
#include "Frontend.hpp"
#include "Map.hpp"
#include "Viewer.hpp"

namespace Edrak {
class VisualSLAM {
public:
  VisualSLAM(const Edrak::StereoCamera &stereoCamera) : camera(stereoCamera) {
    frontend = std::make_shared<Edrak::Frontend>();
    map = std::make_shared<Edrak::Map>();
    backend = std::make_shared<Edrak::Backend>();
    viewer = std::make_shared<Edrak::Viewer>();

    frontend->SetMap(map);
    backend->SetMap(map);
    viewer->SetMap(map);

    frontend->SetCamera(camera);
    backend->SetCamera(camera);

    frontend->SetBackend(backend);

    frontend->SetViewer(viewer);
  }

  bool AddFrame(StereoFrame::SharedPtr frame) {
    return frontend->AddFrame(frame);
  }
  Frontend::SharedPtr frontend;
  Backend::SharedPtr backend;
  Map::SharedPtr map;
  Viewer::SharedPtr viewer;
  Edrak::StereoCamera camera;
};
} // namespace Edrak

#endif
