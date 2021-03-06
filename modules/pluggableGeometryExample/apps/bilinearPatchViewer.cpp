// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#include <vector>

#include <ospray/ospray.h>
#include "CommandLine.h"
#include "Patch.h"

#include <ospray/ospray_cpp/Device.h>
#include <ospray/ospray_cpp/Geometry.h>
#include <ospray/ospray_cpp/Data.h>
#include <ospray/ospray_cpp/Material.h>
#include "common/commandline/Utility.h"
#include "exampleViewer/widgets/imguiViewer.h"

/*! _everything_ in the ospray core universe should _always_ be in the
  'ospray' namespace. */
namespace ospray {

  /*! though not required, it is good practice to put any module into
    its own namespace (isnide of ospray:: ). Unlike for the naming of
    library and init function, the naming for this namespace doesn't
    particularlly matter. E.g., 'bilinearPatch', 'module_blp',
    'bilinar_patch' etc would all work equally well. */
  namespace bilinearPatch {

    ospcommon::vec3f translate;
    ospcommon::vec3f scale;
    bool lockFirstFrame = false;

    // use ospcommon for vec3f etc
    using namespace ospcommon;
    
    extern "C" int main(int ac, const char **av)
    {
      // initialize ospray (this also takes all ospray-related args
      // off the command-line)
      ospInit(&ac,av);

      ospray::imgui3D::init(&ac,av);

      std::deque<ospcommon::box3f>   bbox;
      std::deque<ospray::cpp::Model> models;
      ospray::cpp::Renderer renderer("ao");
      ospray::cpp::Camera   camera("perspective");

      // parse the commandline; complain about anything we do not
      // recognize
      CommandLine args(ac,av);

      // import the patches from the sample files (creates a default
      // patch if no files were specified)
      box3f worldBounds;
      std::vector<Patch> patches = readPatchesFromFiles(args.inputFiles,worldBounds);

      ospLoadModule("bilinear_patches");
      //color data
      float color[] = { 0.9f, 0.6f, 0.3f, 1.0f,
			0.8f, 0.8f, 0.3f, 1.0f,
			0.3f, 0.7f, 0.9f, 1.0f};

      float sphere_color[] = { 1.0f, 0.0f, 0.0f, 1.0f,
			0.0f, 1.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 1.0f, 1.0f,
      0.5f, 0.7f, 0.6f, 1.0f};

      float radius[] = {0.1f, 0.2f, 0.4f};


      ospray::cpp::Data color_data(3, OSP_FLOAT4, color);
      ospray::cpp::Data sphere_color_data(4, OSP_FLOAT4, sphere_color);
      ospray::cpp::Data r(3, OSP_FLOAT, radius);

      ospray::cpp::Data data(patches.size()*12,OSP_FLOAT,patches.data());
      ospray::cpp::Geometry geometry("bilinear_patches");
      geometry.set("patches",data);
      geometry.set("color", color_data);
      //geometry.set("sphere_color", sphere_color_data);
      geometry.set("radius", r);
      geometry.commit();
      
      ospray::cpp::Model model;
      model.addGeometry(geometry);
      model.commit();
      
      models.push_back(model);
      bbox.push_back(worldBounds);

      ospray::ImGuiViewer window(bbox, models, renderer, camera);
      
      window.setScale(scale);
      window.setLockFirstAnimationFrame(lockFirstFrame);
      window.setTranslation(translate);
      window.create("ospBilinearPatchViewer: OSPRay module example app");
      
      ospray::imgui3D::run();
    }
        
  } // ::ospray::bilinearPatch
} // ::ospray
