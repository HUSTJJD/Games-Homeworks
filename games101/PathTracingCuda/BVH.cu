//
// Created by LEI XU on 5/16/19.
//
#include "BVH.cuh"
#include "Triangle.cuh"

__device__ BVHBuildNode *BVHAccel::recursiveBuild(Object ** objects, int num)
{
    BVHBuildNode *node = new BVHBuildNode();
    Bounds3 bounds;
    for (int i = 0; i < num; ++i)
    {
        Bounds3 bound;
        if (objects[i]->type == EObjectType::MeshTriangle)
        {
            bound = ((MeshTriangle *)objects[i])->getBounds();
        }
        else if (objects[i]->type == EObjectType::Triangle)
        {
            bound = ((Triangle *)objects[i])->getBounds();
        }
        bounds = Union(bounds, bound);
    }
    if (num == 1)
    {
        node->left = nullptr;
        node->right = nullptr;
        if (objects[0]->type == EObjectType::MeshTriangle)
        {
            node->bounds = ((MeshTriangle *)objects[0])->getBounds();
            node->area = ((MeshTriangle *)objects[0])->getArea();
        }
        else if (objects[0]->type == EObjectType::Triangle)
        {
            node->bounds = ((Triangle *)objects[0])->getBounds();
            node->area = ((Triangle *)objects[0])->getArea();
        }
        node->object = objects[0];
        return node;
    }
    else if (num == 2)
    {
        node->left = recursiveBuild(&objects[0], 1);
        node->right = recursiveBuild(&objects[1], 1);
        
        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < num; ++i)
        {
            Vector3f centroid;
            if (objects[i]->type == EObjectType::MeshTriangle)
            {
                centroid = ((MeshTriangle *)objects[i])->getBounds().Centroid();
            }
            else if (objects[i]->type == EObjectType::Triangle)
            {
                centroid = ((Triangle *)objects[i])->getBounds().Centroid();
            }
            centroidBounds = Union(centroidBounds, centroid);
        }
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
        case 0:
            std::sort(objects, objects[num], [](auto f1, auto f2)
                      { 
                        Vector3f centroid1;
                        Vector3f centroid2;
                        if (f1->type == EObjectType::MeshTriangle)
                        {
                            centroid1 = ((MeshTriangle *)f1)->getBounds().Centroid();
                        }
                        else if (f1->type == EObjectType::Triangle)
                        {
                            centroid1 = ((Triangle *)f1)->getBounds().Centroid();
                        }
                        if (f2->type == EObjectType::MeshTriangle)
                        {
                            centroid2 = ((MeshTriangle *)f2)->getBounds().Centroid();
                        }
                        else if (f2->type == EObjectType::Triangle)
                        {
                            centroid2 = ((Triangle *)f2)->getBounds().Centroid();
                        }
                        return centroid1.x < centroid2.x; });
            break;
        case 1:
            std::sort(objects, objects[num], [](auto f1, auto f2)
                      {
                        Vector3f centroid1;
                        Vector3f centroid2;
                        if (f1->type == EObjectType::MeshTriangle)
                        {
                            centroid1 = ((MeshTriangle *)f1)->getBounds().Centroid();
                        }
                        else if (f1->type == EObjectType::Triangle)
                        {
                            centroid1 = ((Triangle *)f1)->getBounds().Centroid();
                        }
                        if (f2->type == EObjectType::MeshTriangle)
                        {
                            centroid2 = ((MeshTriangle *)f2)->getBounds().Centroid();
                        }
                        else if (f2->type == EObjectType::Triangle)
                        {
                            centroid2 = ((Triangle *)f2)->getBounds().Centroid();
                        }
                        return centroid1.y < centroid2.y; });
            break;
        case 2:
            std::sort(objects, objects[num], [](auto f1, auto f2)
                      {  
                        Vector3f centroid1;
                        Vector3f centroid2;
                        if (f1->type == EObjectType::MeshTriangle)
                        {
                            centroid1 = ((MeshTriangle *)f1)->getBounds().Centroid();
                        }
                        else if (f1->type == EObjectType::Triangle)
                        {
                            centroid1 = ((Triangle *)f1)->getBounds().Centroid();
                        }
                        if (f2->type == EObjectType::MeshTriangle)
                        {
                            centroid2 = ((MeshTriangle *)f2)->getBounds().Centroid();
                        }
                        else if (f2->type == EObjectType::Triangle)
                        {
                            centroid2 = ((Triangle *)f2)->getBounds().Centroid();
                        }
                        return centroid1.z < centroid2.z; });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (num / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(num == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }
    printf("%f %f %f, %f %f %f\n", node->bounds.pMax.x, node->bounds.pMax.y, node->bounds.pMax.z, node->bounds.pMin.x, node->bounds.pMin.y, node->bounds.pMin.z);
    return node;
}
