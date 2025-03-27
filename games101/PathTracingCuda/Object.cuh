#include "CudaMemory.cuh"

enum class EObjectType
{
    Default = 0,
    Triangle = 1,
    MeshTriangle = 2
};

class Object : public CudaMemory
{
public:
    EObjectType type = EObjectType::Default;
    Object() : type(EObjectType::Default) {}
    Object(EObjectType _type) : type(_type) {}
};