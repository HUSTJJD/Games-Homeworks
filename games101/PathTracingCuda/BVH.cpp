//
// Created by LEI XU on 5/16/19.
//
#include "BVH.hpp"
#include "Triangle.hpp"

BVHBuildNode *BVHAccel::recursiveBuild(std::vector<Object *> objects)
{
    BVHBuildNode *node = new BVHBuildNode();
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, ((MeshTriangle *)objects[i])->getBounds());
    if (objects.size() == 1)
    {
        node->object = objects[0];
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
        return node;
    }
    else if (objects.size() == 2)
    {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
        return node;
    }
    else
    {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds = Union(centroidBounds, ((MeshTriangle *)objects[i])->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim)
        {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return ((MeshTriangle *)f1)->getBounds().Centroid().x <
                               ((MeshTriangle *)f2)->getBounds().Centroid().x; });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return ((MeshTriangle *)f1)->getBounds().Centroid().y <
                               ((MeshTriangle *)f2)->getBounds().Centroid().y; });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2)
                      { return ((MeshTriangle *)f1)->getBounds().Centroid().z <
                               ((MeshTriangle *)f2)->getBounds().Centroid().z; });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object *>(beginning, middling);
        auto rightshapes = std::vector<Object *>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
        node->area = node->left->area + node->right->area;
    }
    return node;
}
