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

// ospray
#include "math/vec.ih"
#include "math/box.ih"
#include "math/LinearSpace.ih"
#include "common/Ray.ih"
#include "common/Model.ih"
#include "ospray/geometry/Geometry.ih"
// embree
#include "embree2/rtcore.isph"
#include "embree2/rtcore_scene.isph"
#include "embree2/rtcore_geometry_user.isph"



/*! input data for a single patch */
struct Patch {
  vec3f v00, v01, v10, v11;
};

struct BilinearPatches {
  /*! inherit from "Geometry" class: since ISPC doesn't support
      inheritance we simply put the "parent" class as the first
      member; this way any typecast to the parent class will get the
      right members (including 'virtual' function pointers, etc) */
  Geometry   super;

  size_t numPatches;
  Patch *uniform patchArray;
  uint8 *data;
  int32 stride;
  uint8 *color;
  int    color_stride;
  int    color_offset;
  // add sphere radius
 uint8 *radius;
  // offset param
  int offset_center;
  int offset_colorID;

  // sphere color
  uint8 *sphere_color;
  int sphere_color_stride;
  int sphere_color_offset;
  int sphere_offset_colorID;

};

/*
*   POST INTERSECT
*/
static void BilinearPatches_postIntersect(uniform Geometry *uniform geometry,
                                  uniform Model *uniform model,
                                  varying DifferentialGeometry &dg,
                                  const varying Ray &ray,
                                  uniform int64 flags)
{
  BilinearPatches *uniform self = (BilinearPatches *uniform)geometry;
  dg.Ng = dg.Ns = normalize(ray.Ng);

// edit by Mengjiao  
// color
  if ((flags & DG_COLOR) && self->color) {
     uint32 colorID = 0;
     if (self->offset_colorID >= 0) {
         uniform uint8 *varying patchPtr = self->data + self->stride * ray.primID;
         colorID = *((uniform uint32 *varying)(patchPtr+self->offset_colorID));
    } else
        colorID = ray.primID;
        uint32 colorAddr = self->color_offset + colorID * self->color_stride;
        dg.color = *((vec4f *)(self->color+colorAddr));
        dg.color.w = 1.f;
  }

    if ((flags & DG_COLOR) && self->sphere_color) {
      uint32 colorID = 0;
      if (self->sphere_offset_colorID >= 0) {
          uniform uint8 *varying spherePtr = self -> data + self -> stride * ray.primID;
          colorID = *((uniform uint32 *varying)(spherePtr + self -> sphere_offset_colorID));
     } else
        colorID = ray.primID;
        uint32 colorAddr = self -> sphere_color_offset + colorID * self -> sphere_color_stride;
        dg.color = *((vec4f *)(self->sphere_color+colorAddr));
        dg.color.w = 1.f;
  }

  // if (flags & DG_MATERIALID) {
  //   dg.materialID = 0;
  //   dg.material   = self->super.material;
  // }
}


/*! create a new ispc equivalent for the C++ class (cppEquivalent
    points back, if we ever need this. this returns a void * that the
    c++ side can use to call back to us when needed. Note this return
    values _has_ to be a untyped 'void*' because ISPC cannot currently
    export 'real' class types - with function pointers etcpp - to a c
    header file */
export void *uniform BilinearPatches_create(void *uniform cppEquivalent)
{
  BilinearPatches *uniform self = uniform new uniform BilinearPatches;
  
  /* recursively call the 'inherited' constructor, and pass our
     'overridden' functions for intersect and postintersect. note we
     do not have to set the 'intersect' and 'occluded' functions
     because we use embree to do the traversal for us (see
     BilinearPatches_set()) below */
  Geometry_Constructor(&self->super,cppEquivalent,
                       BilinearPatches_postIntersect,
                       NULL,0,NULL);
  return self;
}

export void BilinearPatches_destroy(void *uniform _self)
{
  /* _actually_ this should also destroy the created embree geometry
     here; not doing this to keep the code as small as possible, but
     usually this is where it should be done */
  BilinearPatches *uniform self = (BilinearPatches *uniform)_self;
  delete self;
}




/*! to be able to have embree build a data structure over us we have
    to be able to tell it the bounding box for each primitimve. this
    is the callback embree calls for querying the (uniform) bounding
    box for a single given primitmive 'primID' */
unmasked void BilinearPatches_bounds(BilinearPatches *uniform self,
                                     uniform size_t primID,
                                     uniform  box3fa &bbox)
{
  print("bounds\n");
  Patch *uniform patch = self->patchArray+primID;
  // bbox = box_extend(box_extend(box_extend(box_extend(make_box3fa_empty(),make_vec3fa(patch->v00)),
  // make_vec3fa(patch->v01)),
  // make_vec3fa(patch->v10)),
  // make_vec3fa(patch->v11));

  uniform uint8 *uniform radiusPtr = self -> radius + sizeof(uniform float) * primID;
  uniform float r = *((float *uniform) (radiusPtr));
     //uniform  float r =  *((uniform float *)(self -> radius + radiusAddr));

     //const uniform vec3fa r = make_vec3fa(self->radius + radiusAddr));
  
  const uniform vec3fa A00 = make_vec3fa(patch -> v00) ;
  const uniform vec3fa A01 = make_vec3fa(patch -> v01);
  const uniform vec3fa A10 = make_vec3fa(patch -> v10) ;
  const uniform vec3fa A11 = make_vec3fa(patch -> v11);

  // const uniform box3fa bbox0 = make_box3fa(A00 - self -> radius , A00 + self ->radius);
  // const uniform box3fa bbox1 = make_box3fa(A01 - self -> radius, A01 + self ->radius);
  // const uniform box3fa bbox2 = make_box3fa(A10 - self ->radius, A10 + self ->radius);
  // const uniform box3fa bbox3 = make_box3fa(A11 - self -> radius, A11 + self ->radius);

  const uniform box3fa bbox0 = make_box3fa(A00 - r , A00 + r);
  const uniform  box3fa bbox1 = make_box3fa(A01 - r , A01 + r);
  const uniform box3fa bbox2 = make_box3fa(A10 - r, A10 + r);
  const uniform box3fa bbox3 = make_box3fa(A11 - r, A11 + r);


  bbox = box_extend(box_extend( box_extend(bbox0, bbox1), bbox2), bbox3);
 
}

/*
edit by Mengjiao add sphere
 */
inline bool intersectSphere(varying Ray &ray,
			    const vec3f &A,
			    const uniform float r)
{
  bool hit = false;
  const float a = dot(ray.dir, ray.dir);
  const float b = 2.0f * dot(ray.dir, A);
  const float c = dot(A, A) - r * r;
  
  const float radical = b * b - 4.0f * a * c;
  if(radical < 0.0f) return false;

  const float srad = sqrt(radical);
  const float t_in = (b - srad) * rcpf(2.0f * a);
  const float t_out = (b + srad) * rcpf(2.0f * a);
  
  if(t_in > ray.t0 && t_in  < ray.t) {
    ray.t = t_in;
    hit = true;
    //return true;
  }else if (t_out > ray.t0 && t_out < ray.t){
    ray.t = t_out;
    hit = true;
   // return true;    
  }
  if(hit) {ray.Ng = ray.org + ray.t * ray.dir - A;}

  return hit;
}
inline bool intersectCylinder(varying Ray &ray,
                                     const vec3f &A, const vec3f &B,
                                     const uniform float r)
{
  const vec3f V = ray.dir;

  const vec3f AB = B - A;
  
  const vec3f AOxAB = cross(AB,A);
  const vec3f VxAB  = cross(V,AB);
  const float ab2   = dot(AB,AB);
  const float a     = dot(VxAB,VxAB);
  const float b     = 2 * dot(VxAB,AOxAB);
  const float c     = dot(AOxAB,AOxAB) - (sqr(r) * ab2);

  // clip to near and far cap of cylinder
  const float rVAB = rcp(dot(V,AB));
  const float tA = dot(AB,A) * rVAB;
  const float tB = dot(AB,B) * rVAB;
  const float tAB0 = max(ray.t0,min(tA,tB));
  const float tAB1 = min(ray.t, max(tA,tB));

  // ------------------------------------------------------------------
  // abc formula: t0,1 = (-b +- sqrt(b^2-4*a*c)) / 2a
  //
  const float radical = b*b-4.f*a*c;
  if (radical < 0.f) return false;
  
  const float srad = sqrt(radical);

  const float t_in = (- b - srad) *rcpf(2.f*a);
  const float t_out= (- b + srad) *rcpf(2.f*a);

  if (t_in >= tAB0 && t_in <= tAB1) {
    ray.t = t_in;
    ray.Ng = ray.org + ray.t * ray.dir - A;
    return true;
  } else if (t_out >= tAB0 && t_out <= tAB1) {
    ray.t = t_out;
    ray.Ng = ray.org + ray.t * ray.dir - A;
    return true;
  }
  return false;
}
inline bool recursiveBisection(varying Ray &ray,
                        const varying LinearSpace3f &frame,
                        const uniform vec3f p00,
                        const uniform vec3f p01,
                        const uniform vec3f p10,
                        const uniform vec3f p11,
                        const uniform float u0,
                        const uniform float u1,
                        const uniform float v0,
                        const uniform float v1,
                        uniform int recursionsLeft)
{
  // compute signs to x plane
  const float x0 = dot(p00-ray.org,frame.vx);
  const float x1 = dot(p01-ray.org,frame.vx);
  const float x2 = dot(p10-ray.org,frame.vx);
  const float x3 = dot(p11-ray.org,frame.vx);
  const float min_x = min(min(x0,x1),min(x2,x3));
  if (min_x > 0.f) return false;
  const float max_x = max(max(x0,x1),max(x2,x3));
  if (max_x < 0.f) return false;

  // compute signs to y plane
  const float y0 = dot(p00-ray.org,frame.vy);
  const float y1 = dot(p01-ray.org,frame.vy);
  const float y2 = dot(p10-ray.org,frame.vy);
  const float y3 = dot(p11-ray.org,frame.vy);
  const float min_y = min(min(y0,y1),min(y2,y3));
  if (min_y > 0.f) return false;
  const float max_y = max(max(y0,y1),max(y2,y3));
  if (max_y < 0.f) return false;

  // distance test
  const float t0 = dot(p00-ray.org,frame.vz);
  const float t1 = dot(p01-ray.org,frame.vz);
  const float t2 = dot(p10-ray.org,frame.vz);
  const float t3 = dot(p11-ray.org,frame.vz);
  // const float min_t = min(min(t0,t1),min(t2,t3));
  // if (min_t > 0.f) return false;
  const float max_t = max(max(t0,t1),max(t2,t3));
  if (max_t < 0.f) return false;
  
  const uniform float uc = 0.5f*(u0+u1);
  const uniform float vc = 0.5f*(v0+v1);

  if (recursionsLeft == 0) {
    const float t = 0.25*(t0+t1+t2+t3);
    if (t < ray.t0 || t >= ray.t) return false;
    
    ray.t = t;
    ray.u = uc;
    ray.v = vc; 


    const vec3f pu0 = p00+ray.u*(p01-p00);
    const vec3f pu1 = p10+ray.u*(p11-p10);

    const vec3f pv0 = p00+ray.v*(p10-p00);
    const vec3f pv1 = p01+ray.v*(p11-p01);
    
    ray.Ng = cross(pu1-pu0,pv1-pv0);
    return true;
  } else {
    const uniform vec3f p0c = 0.5f*(p01+p00);
    const uniform vec3f p1c = 0.5f*(p11+p10);
    const uniform vec3f pc0 = 0.5f*(p10+p00);
    const uniform vec3f pc1 = 0.5f*(p11+p01);
    const uniform vec3f pcc = 0.25f*(p00+p01+p10+p11);
    
    bool hit = false;
    hit |= recursiveBisection(ray,frame, p00,p0c,pc0,pcc, u0,uc, v0,vc, recursionsLeft-1);
    hit |= recursiveBisection(ray,frame, pc0,pcc,p10,p1c, u0,uc, vc,v1, recursionsLeft-1);

    hit |= recursiveBisection(ray,frame, p0c,p01,pcc,pc1, uc,u1, v0,vc, recursionsLeft-1);
    hit |= recursiveBisection(ray,frame, pcc,pc1,p1c,p11, uc,u1, vc,v1, recursionsLeft-1);
    return hit;
  }
}


/*! this is the function callback the embree calls to compute a single
  ray-primitive intersection. For sake of simplicity we'll use a very,
  very simple recursive test here - this is neither fast nor elegant,
  but does show that you can do mere arbitray stuff in a ISPC kernel,
  including 'true' recusion. */
void BilinearPatches_intersect(BilinearPatches *uniform self,
                               varying Ray &ray,
                               uniform size_t primID)
{
  Patch *uniform patch = self->patchArray + primID;

  
  // create a simple coordinate system with x and y directions
  // orthogonal to the ray
  const LinearSpace3f coordSys = frame(ray.dir);
  bool hit = 0;

  uniform uint8 *uniform radiusPtr = self -> radius + sizeof(uniform float) * primID;
  uniform float r = *((float *uniform) (radiusPtr));
  //print("radius", r);
  
  const vec3f A00 = make_vec3f(patch -> v00) - ray.org;
  const vec3f A01 = make_vec3f(patch -> v01) - ray.org;
  const vec3f A10 = make_vec3f(patch -> v10) - ray.org;
  const vec3f A11 = make_vec3f(patch -> v11) - ray.org;
  // hit |= intersectSphere(ray, A00, self -> radius + primID );
  // hit |= intersectSphere(ray, A01, self -> radius + primID);
  // hit |= intersectSphere(ray, A10, self -> radius + primID);
  // hit |= intersectSphere(ray, A11, self -> radius + primID);
  // hit |= intersectCylinder(ray, A00, A01, self->radius / 8.0f);
  // hit |= intersectCylinder(ray, A01, A10, self->radius / 8.0f);
  // hit |= intersectCylinder(ray, A10, A11 ,self->radius / 8.0f);
  // hit |= intersectCylinder(ray, A00, A11, self->radius / 8.0f);
  hit |= intersectSphere(ray, A00, r);
  hit |= intersectSphere(ray, A01, r);
  hit |= intersectSphere(ray, A10, r);
  hit |= intersectSphere(ray, A11, r);
  hit |= intersectCylinder(ray, A00, A01, r / 8.0f);
  hit |= intersectCylinder(ray, A01, A10, r / 8.0f);
  hit |= intersectCylinder(ray, A10, A11 ,r / 8.0f);
  hit |= intersectCylinder(ray, A00, A11, r / 8.0f);

  if (recursiveBisection(ray,coordSys,
                         patch->v00,patch->v01,patch->v10,patch->v11,
                         0.f,1.f, 0.f,1.f,
                         /* number of bisections */ 10)) {
    ray.geomID = self->super.geomID;
    ray.primID = primID;
  }
    if (hit) {
    ray.geomID = self->super.geomID;
    ray.primID = primID;
  }

}



/*! the function called by BilinearPatches::commit() to pass the
  actual user data over. for simplicity we assume that this functoin
  gets called exactly once; in practice a user may also _change_ an
  existing class'es data members, but handling this would make this
  code significantly larger */
export void BilinearPatches_finalize(void *uniform _self,
                                     void *uniform _model,
                                     float *uniform patchesDataMemory,
                                     uniform size_t numPatches,
				     void *uniform color,
				     uniform int color_offset,
				     uniform int color_stride,
				     uniform int offset_colorID,
				     void *uniform radius,
				     uniform int offset_center,
             void *uniform sphere_color,
             uniform int sphere_color_stride,
             uniform int sphere_color_offset,
             uniform int sphere_offset_colorID
             )
{
  // first, typecast to our 'real' type. since ispc can't export real
  // types to c we have to pass 'self' in as a void*, and typecast
  // here.
  BilinearPatches *uniform self = (BilinearPatches *uniform)_self;

  // set our internal data.
  self->numPatches = numPatches;
  self->patchArray = (Patch *uniform)patchesDataMemory;
  
  // edit by Mengjiao
  self -> color = (uint8 *uniform) color;
  self -> color_stride = color_stride;
  self -> color_offset = color_offset;
  self -> offset_colorID = offset_colorID;
  self -> radius = (uint8 *uniform) radius;
  self -> offset_center = offset_center;

  self -> sphere_color = (uint8 *uniform) sphere_color;
  self -> sphere_color_stride = sphere_color_stride;
  self -> sphere_color_offset = sphere_color_offset;
  self -> sphere_offset_colorID = sphere_offset_colorID;

  // ... and let embree build a bvh, with 'numPatches' primitmives and
  // our function pointers for ray-prim interseciton and
  // prim-boundingbox computation callsbacks
  Model *uniform model  = (Model *uniform)_model;

  // create a new embree geometry with numpathces prims, in the model
  // that this goemetry is in.
  uint32 uniform geomID = rtcNewUserGeometry(model->embreeSceneHandle,numPatches);
  
  // set 'us' as user data (this will be the first arg in intersect()
  // and computebounds() callbacks
  rtcSetUserData(model->embreeSceneHandle,geomID,self);
  
  // set function pointers:
  rtcSetBoundsFunction(model->embreeSceneHandle,geomID,
                       (uniform RTCBoundsFunc)&BilinearPatches_bounds);
  rtcSetIntersectFunction(model->embreeSceneHandle,geomID,
                          (uniform RTCIntersectFuncVarying)&BilinearPatches_intersect);
  rtcSetOccludedFunction(model->embreeSceneHandle,geomID,
                         (uniform RTCOccludedFuncVarying)&BilinearPatches_intersect);
}
                                    

    
 
