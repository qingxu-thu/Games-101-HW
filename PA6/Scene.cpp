//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Vector3f po = ray.origin-ray.direction;
    Ray pp(po,-1*ray.direction);
    Intersection pos = intersect(pp);
    Intersection t = pos;
    float pdf_t = 1.0f;
    sampleLight(pos,pdf_t);
    Vector3f distance = pos.coords-ray.origin;
    //Ray p(ray.origin,distance);
    float d = distance.norm();
    distance = normalize(distance);
    //Intersection tp =  intersect(p);
    Vector3f L_dir {0.0f,0.0f,0.0f};
    Ray te(pos.coords,distance);
    Intersection TT = Intersection(te);
    if ((!TT.happened)||(TT.distance>=d))
    {
     L_dir = pos.emit*t.m->eval(ray.direction,distance,t.normal)*dotProduct(distance,t.normal)*dotProduct(distance,pos.normal)/(d*d*pdf_t);
    }
    Vector3f L_indir {0.0f,0.0f,0.0f};
    float P_RR = 0.7;
    float k = get_random_float();
    if ((k<P_RR)&&depth<maxDepth)
    {Vector3f wi = t.m->sample(ray.direction,t.normal);
    Ray p(t.coords,wi);
    Intersection t2 = intersect(p);
    if ((t2.happened)&&(!t2.obj->hasEmit()))
    {depth = depth+1;
    L_indir = castRay(p,depth)*t.m->eval(ray.direction,p.direction,t.normal)*dotProduct(p.direction,t.normal)/(t.m->pdf(ray.direction,p.direction,t.normal)*P_RR);
    return L_dir+L_indir;
    }
    else {return L_dir;}
    }
    else {return L_dir;}
}