#ifndef RENDER_DATA
#define RENDER_DATA

#include <Eigen/Core>
#include <Eigen/Dense>
#include <array>
#include <map>
#include <string>
#include <vector>

namespace render_data {
struct pointR {
	Eigen::Vector4d pos;
	std::vector<size_t> tri_index;
	Eigen::Vector4d normal;
};
typedef struct pointR pointR;

struct textureR {
	size_t count = 1;
	int w, h;
	std::vector<std::vector<uint32_t>> texs;
};
typedef struct textureR texR;

struct AABB {
	double xmin, xmax, ymin, ymax;
};
typedef struct AABB AABB;

struct triangleR {
	std::array<Eigen::Vector4d, 3> points;
	std::array<uint32_t, 3> colors;
	std::array<Eigen::Vector2d, 3> uvMap;
	std::string tmName = "";
	std::string texName = "";
	std::array<double, 3> k_a = {0.3, 0.3, 0.3}; // 环境光着色率
	std::array<double, 3> k_d = {0.7, 0.7, 0.7}; // 漫反射..，颜色由colors提供
	std::array<double, 3> k_s = {1, 1, 1};       // 镜面反射..，颜色由光源提供
	std::array<uint32_t, 3> I_a;                 // 环境光色
	int shininess = 16;                          // 光泽度（幂数）
	// 以下爲需注意項
	Eigen::Vector4d normal;
	std::array<size_t, 3> point_index;
	double area = 0;
	bool valid = true;
	bool single_face = false;
};
typedef struct triangleR triR;

struct Light {
	Eigen::Vector4d position;
	double value;
	uint32_t color = 0xffffffff;
};
typedef struct Light Light;

struct ARGB {
	uint8_t R, G, B, A;
};
typedef struct ARGB ARGB;

typedef std::vector<pointR> pointRSet;
typedef std::vector<triangleR> triRSet;
typedef std::map<std::string, texR> texMap;
typedef std::map<std::string, Light> LightMap;
typedef std::vector<Light> LightSet;

struct rectangleR {
	triR u, l;
};
typedef struct rectangleR rectR;

} // namespace render_data

#endif
