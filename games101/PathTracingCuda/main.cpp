#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char **argv)
{

    // Change the definition here to change resolution
    const int width = 784, height = 784;
    Scene *scene = new Scene(width, height);
    const int pixel_num = width * height;
    Vector3f *framebuffer = new Vector3f[pixel_num];

    Material *red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material *green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material *white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material *light = new Material(DIFFUSE, (Vector3f(0.747f + 0.058f, 0.747f + 0.258f, 0.747f) * 8.0f + Vector3f(0.740f + 0.287f, 0.740f + 0.160f, 0.740f) * 15.6f + Vector3f(0.737f + 0.642f, 0.737f + 0.159f, 0.737f) * 18.4f));
    light->Kd = Vector3f(0.65f);

    Material *whiteM = new Material(Microfacet, Vector3f(0.0f));
    whiteM->Ks = Vector3f(0.45, 0.45, 0.45);
    whiteM->Kd = Vector3f(0.3, 0.3, 0.25);

    MeshTriangle *floor = new MeshTriangle("../../models/cornellbox/floor.obj", white);
    MeshTriangle *shortbox = new MeshTriangle("../../models/cornellbox/shortbox.obj", white);
    MeshTriangle *tallbox = new MeshTriangle("../../models/cornellbox/tallbox.obj", white);
    MeshTriangle *left = new MeshTriangle("../../models/cornellbox/left.obj", red);
    MeshTriangle *right = new MeshTriangle("../../models/cornellbox/right.obj", green);
    MeshTriangle *light_ = new MeshTriangle("../../models/cornellbox/light.obj", light);

    scene->Add(floor);
    scene->Add(shortbox);
    scene->Add(tallbox);
    scene->Add(left);
    scene->Add(right);
    scene->Add(light_);

    scene->buildBVH();

    Renderer renderer;
    constexpr int spp = 64;
    std::cout << "SPP: " << spp << "\n";

    Vector3f eye_pos(278.0f, 273.0f, -800.0f);

    auto start = std::chrono::system_clock::now();
    renderer.Render(scene, spp, eye_pos, framebuffer);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";
    // save framebuffer to file
    FILE *fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", width, height);
    for (auto i = 0; i < pixel_num; ++i)
    {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(framebuffer[i].x, 0.6f));
        color[1] = (unsigned char)(255 * std::pow(framebuffer[i].y, 0.6f));
        color[2] = (unsigned char)(255 * std::pow(framebuffer[i].z, 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);

    return 0;
}