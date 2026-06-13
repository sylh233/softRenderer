#ifndef RENDER_FUN
#define RENDER_FUN

#include <Eigen/Core>
#include <Eigen/Dense>
#include <SDL3/SDL.h>
#include <array>
#include <fundm.h>
#include <map>
#include <renderDataStruct.h>
#include <string>
#include <vector>

extern Eigen::Matrix4d TransMat;
void set_px(int, int, Uint32);
extern std::vector<double> zBuffer;
extern const int Wwidth;
extern const int Wheight;
extern fundm::Camera camera;

inline double cross2(Eigen::Vector2d l1, Eigen::Vector2d l2) {
	return l1[0] * l2[1] - l1[1] * l2[0];
}

template <typename T1, typename T2, typename U, std::size_t N>
inline U dot_array(std::array<T1, N> array1, std::array<T2, N> array2) {
	U sum{};
	size_t size = array1.size();
	for (size_t i = 0; i < size; i++)
		sum += array1[i] * array2[i];
	return sum;
}

namespace render_func {
using namespace render_data;

class textureSet {
  private:
	texMap texm;

  public:
	texR *get_textures(std::string texName);
	void setTexLayer(std::string texName, size_t layers);
	void appendTex(std::string address, std::string texName, size_t layers = 1);
	void deleteTex(std::string texName);
};

class geometryR {
  public:
	enum class shading_mode {
		OFF,
		FLAT_SHADING,
		GOURAUD_SHADING,
		PHONG_SHADING
	};

  private:
	triRSet tris;
	pointRSet ps;
	LightSet lights;
	shading_mode shadingM = shading_mode::OFF;
	bool inTriangle(std::array<Eigen::Vector2d, 3> tri2d, Eigen::Vector2d xy);
	AABB get_aabb(std::array<Eigen::Vector2d, 3> tri2d);
	Eigen::Vector4d get_3dpoint(Eigen::Vector2d xy,
	                            std::array<Eigen::Vector4d, 3> triTransed);
	std::array<double, 3> get_BaryCoor(Eigen::Vector3d point,
	                                   std::array<Eigen::Vector3d, 3> TRI);
	bool rangeTest(int x, int y, int w, int h);
	ARGB colorDecompo(uint32_t color);
	uint32_t colorMap(ARGB argb);
	uint32_t colorInter(std::array<double, 3> weights,
	                    std::array<uint32_t, 3> colors);
	template <typename T, typename U>
	U interpolation(std::array<double, 3> weights, std::array<T, 3> values);
	bool notALine(std::array<Eigen::Vector2d, 3> tri2d);
	std::array<double, 3> persAmend(std::array<double, 3> baryCoor,
	                                std::array<double, 3> w);
	uint32_t get_uvmapColor(std::array<double, 3> baryCoor, size_t index);
	uint32_t colorMulti(double m, uint32_t color);
	uint32_t colorPlus(uint32_t color1, uint32_t color2);
	double Blinn_PhongD(double k_d, Eigen::Vector4d LightDIr,
	                    Eigen::Vector4d normal);
	double Blinn_PhongS(double k_s, int shininess, Eigen::Vector4d LightDir,
	                    Eigen::Vector4d normal, Eigen::Vector4d cameraDir);
	double lightValue(double value, double distance);

  public:
	void appendTri(triR TRI);
	triR *changeTri(size_t index);
	void deleteTri(size_t index);
	void renderGeo();
	void preloadNormal();
	void transform(Eigen::Matrix4d tMat);
	pointRSet *get_ps() { return &ps; };
	triRSet *get_tris() { return &tris; };
	void appendLight(Light l);
	void deleteLight(size_t i);
	void setShadingM(shading_mode sm);
};

class boxR {
  private:
	geometryR geo;
	enum class rotateAxis { X, Y, Z };
	double scale;             // = 1;
	Eigen::Vector4d position; // = Eigen::Vector4d(0, 0, 0, 1);
	std::array<Eigen::Vector4d, 3> towards;
	std::array<uint32_t, 8> colors8;
	std::array<rectR, 6> surfaces;
	double k_a, k_d, k_s, shininess;
	uint32_t I_a;
	void update();

  public:
	boxR(double s = 1, Eigen::Vector4d pos = Eigen::Vector4d(0, 0, 0, 1));
	void render();
	void trans(Eigen::Matrix4d mat);
	void rotate(rotateAxis axis, double angle);
	void move(Eigen::Vector4d dir);
	void changeScale(double scale);
	void addTex(texR *tex);
	void setColor(size_t index, uint32_t color1);
	void setColors(std::array<uint32_t, 8> colors8);
	void setK(double k_a, double k_d, double k_s, double shininess);
	void preNormal();
	void addLight(Light l);
	void setShading(geometryR::shading_mode sm);
};

} // namespace render_func

#endif
