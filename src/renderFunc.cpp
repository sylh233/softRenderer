#include <algorithm>
#include <cmath>
#include <fundm.h>
#include <iostream>
#include <renderDataStruct.h>
#include <renderFunction.h>

namespace render_func {

void textureSet::appendTex(std::string address, std::string texName,
                           size_t layers) {
	texR texr;
	texr.count = layers;
	SDL_Surface *ts = SDL_LoadBMP(address.c_str());
	if (ts == NULL) {
		SDL_Log("Load BMP error:%s", SDL_GetError());
	}
	SDL_Surface *texsurf = SDL_ConvertSurface(ts, SDL_PIXELFORMAT_ARGB8888);
	if (texsurf == NULL) {
		SDL_Log("Load BMP error:%s", SDL_GetError());
	}
	SDL_DestroySurface(ts);
	texr.w = texsurf->w;
	texr.h = texsurf->h;
	uint32_t *src = (uint32_t *)texsurf->pixels;
	std::vector<uint32_t> vec;
	vec.resize(texr.w * texr.h);
	int pitch = texsurf->pitch / sizeof(uint32_t); // 每行字节数
	for (size_t y = 0; y < texr.h; y++) {          // 跳过无用内存
		memcpy(&vec[y * texr.w], &src[y * pitch], texr.w * sizeof(uint32_t));
	}
	texr.texs.push_back(std::move(vec)); // 移动语义
	texm[texName] = texr;

	SDL_DestroySurface(texsurf);
}

texR *textureSet::get_textures(std::string texName) {
	if (texm.count(texName) != 0) {
		return &texm[texName];
	} else
		return nullptr;
}

void textureSet::deleteTex(std::string texName) { texm.erase(texName); }

// 渲染几何体
void geometryR::renderGeo() {
	for (size_t i = 0; i < tris.size(); i++) {
		auto TRI = tris[i];
		if (TRI.single_face) {
			if (camera.lookat.dot(TRI.normal) > 0) {
				continue;
			}
		}
		auto triPos = TRI.points;
		// 需插值项
		std::array<uint32_t, 3> colors = TRI.colors;

		//
		// 预计算三线
		std::array<Eigen::Vector4d, 3> vertexNormal;
		std::array<Eigen::Vector4d, 3> cameraDir;
		std::vector<std::array<Eigen::Vector4d, 3>> LightDirs;
		std::vector<std::array<double, 3>> dises;

		std::vector<double> Flat_k_d, Flat_k_s;
		std::vector<std::array<double, 3>> Gour_k_d, Gour_k_s;
		if (lights.size() > 0) {
			if (shadingM == shading_mode::FLAT_SHADING) {
				Eigen::Vector4d FlatC = (camera.pos - triPos[0]).normalized();
				for (size_t l = 0; l < lights.size(); l++) {
					Eigen::Vector4d FlatL =
					    (lights[l].position - triPos[0]).normalized();
					double FlatD = (lights[l].position - triPos[0]).norm();
					double V = lightValue(lights[l].value, FlatD);
					Flat_k_d.push_back(
					    V * Blinn_PhongD(TRI.k_d[0], FlatL, TRI.normal));
					Flat_k_s.push_back(V * Blinn_PhongS(TRI.k_s[0],
					                                    TRI.shininess, FlatL,
					                                    TRI.normal, FlatC));
				}
			}
			if (shadingM == shading_mode::GOURAUD_SHADING) {
				for (size_t i = 0; i < 3; i++) {
					cameraDir[i] = (camera.pos - triPos[i]).normalized();
					vertexNormal[i] = ps[TRI.point_index[i]].normal;
				}
				for (size_t l = 0; l < lights.size(); l++) {
					std::array<Eigen::Vector4d, 3> LightDir;
					std::array<double, 3> dis;
					std::array<double, 3> gkd;
					std::array<double, 3> gks;
					for (size_t l2 = 0; l2 < 3; l2++) {
						LightDir[l2] =
						    (lights[l].position - triPos[l2]).normalized();
						dis[l2] = (lights[l].position - triPos[l2]).norm();
						double V = lightValue(lights[l].value, dis[l2]);
						gkd[l2] = V * Blinn_PhongD(TRI.k_d[l2], LightDir[l2],
						                           vertexNormal[l2]);
						gks[l2] =
						    V * Blinn_PhongS(TRI.k_s[l2], TRI.shininess,
						                     LightDir[l2], vertexNormal[l2],
						                     cameraDir[l2]);
					}
					Gour_k_d.push_back(gkd);
					Gour_k_s.push_back(gks);
				}
			}
			if (shadingM == shading_mode::PHONG_SHADING) {
				for (size_t i = 0; i < 3; i++) {
					cameraDir[i] = (camera.pos - triPos[i]).normalized();
					vertexNormal[i] = ps[TRI.point_index[i]].normal;
				}
				for (size_t l = 0; l < lights.size(); l++) {
					std::array<Eigen::Vector4d, 3> LightDir;
					std::array<double, 3> dis;
					for (size_t l2 = 0; l2 < 3; l2++) {
						LightDir[l2] =
						    (lights[l].position - triPos[l2]).normalized();
						dis[l2] = (lights[l].position - triPos[l2]).norm();
					}
					LightDirs.push_back(LightDir);
					dises.push_back(dis);
				}
			}
		}
		//
		std::array<Eigen::Vector4d, 3> triTransed;
		std::array<double, 3> W; // 齐次项w
		std::array<Eigen::Vector2d, 3> triScreen;
		for (size_t j = 0; j < 3; j++) {
			triTransed[j] = TransMat * triPos[j];
			if (triTransed[j][3] == 0) {
				triTransed[j][3] += 1e-5;
			}
			W[j] = triTransed[j][3];
			triTransed[j] /= triTransed[j][3];
			triScreen[j] = triTransed[j].head<2>();
		}
		if (!notALine(triScreen))
			continue;
		AABB aabb = get_aabb(triScreen);
		// test
		/*
		int testX = (aabb.xmax - aabb.xmin) / 2 + aabb.xmin,
		    testY = (aabb.ymax - aabb.ymin) / 2 + aabb.ymin;
		Eigen::Vector2d TestXY = {testX, testY};
		{
		    Eigen::Vector4d tPoint = get_3dpoint(TestXY, triTransed);
		    Eigen::Vector3d tP3d = tPoint.head<3>();
		    std::array<Eigen::Vector3d, 3> triTrans3d;
		    for (size_t i_ = 0; i_ < 3; i_++) {
		        triTrans3d[i_] = triTransed[i_].head<3>();
		    }
		    std::array<double, 3> baryCoor = get_BaryCoor(tP3d, triTrans3d);
		    baryCoor = persAmend(baryCoor, W);

		    Uint32 tColor = get_uvmapColor(baryCoor, i);
		    //	std::cout << triTransed[0][0] << " " << triTransed[0][1] << " "
		    //<< triTransed[0][2] << std::endl; std::cout << tPoint[0] << " " <<
		    // tPoint[1] << " " << tPoint[2] << std::endl;
		    // std::cout << baryCoor[0] << baryCoor[1] << baryCoor[2] <<
		    // std::endl;
		    std::cout << tColor << std::endl;
		}
		*/
		for (int x = (int)aabb.xmin; x <= aabb.xmax; x++) {
			for (int y = (int)aabb.ymin; y <= aabb.ymax; y++) {
				// 如需MSAA则在这里做
				if (rangeTest(x, y, Wwidth, Wheight)) {
					Eigen::Vector2d txy = {x, y};
					if (inTriangle(triScreen, txy)) {
						// 取原始点的方法需要改进！(已改进！！)
						Eigen::Vector4d tPoint = get_3dpoint(txy, triTransed);
						Eigen::Vector3d tP3d = tPoint.head<3>();
						std::array<Eigen::Vector3d, 3> triTrans3d;
						for (size_t i_ = 0; i_ < 3; i_++) {
							triTrans3d[i_] = triTransed[i_].head<3>();
						}
						std::array<double, 3> baryCoor = get_BaryCoor(
						    tP3d, triTrans3d); // 终于获得重心坐标！！
						baryCoor = persAmend(baryCoor, W);
						// 颜色插值
						uint32_t tColor;
						uint32_t shadingColor;
						if (TRI.texture == nullptr) { // 纹理映射
							tColor = colorInter(baryCoor, colors);
						} else {
							tColor = get_uvmapColor(baryCoor, i);
						}
						if (lights.size() > 0 &&
						    shadingM != shading_mode::OFF) { // 光照着色
							// 计算单点着色率
							double k_a = interpolation<double, double>(baryCoor,
							                                           TRI.k_a);
							uint32_t I_a = colorInter(baryCoor, TRI.I_a);
							for (size_t aa = 0; aa < 3; aa++) {
								shadingColor = colorMulti(k_a, I_a);
							} // 环境光

							Eigen::Vector4d N;
							Eigen::Vector4d C =
							    interpolation<Eigen::Vector4d, Eigen::Vector4d>(
							        baryCoor, cameraDir)
							        .normalized();
							std::vector<Eigen::Vector4d> Ls;
							std::vector<double> Ds;

							switch (shadingM) {
							case shading_mode::FLAT_SHADING:
								for (size_t lf = 0; lf < lights.size(); lf++) {
									shadingColor = colorPlus(
									    shadingColor,
									    colorPlus(
									        colorMulti(Flat_k_d[lf], tColor),
									        colorMulti(Flat_k_s[lf],
									                   lights[lf].color)));
								}
								break;
							case shading_mode::GOURAUD_SHADING:
								for (size_t l = 0; l < lights.size(); l++) {
									double k_d = interpolation<double, double>(
									    baryCoor, Gour_k_d[l]);
									double k_s = interpolation<double, double>(
									    baryCoor, Gour_k_s[l]);
									shadingColor = colorPlus(
									    shadingColor,
									    colorPlus(
									        colorMulti(k_d, tColor),
									        colorMulti(k_s, lights[l].color)));
								}
								break;
							case shading_mode::PHONG_SHADING:
								double k_d = interpolation<double, double>(
								    baryCoor, TRI.k_d);
								double k_s = interpolation<double, double>(
								    baryCoor, TRI.k_s);
								N = interpolation<Eigen::Vector4d,
								                  Eigen::Vector4d>(baryCoor,
								                                   vertexNormal)
								        .normalized();
								for (size_t l = 0; l < lights.size(); l++) {
									Ls.push_back(interpolation<Eigen::Vector4d,
									                           Eigen::Vector4d>(
									                 baryCoor, LightDirs[l])
									                 .normalized());
									Ds.push_back(interpolation<double, double>(
									    baryCoor, dises[l]));
									double V =
									    lightValue(lights[l].value, Ds[l]);
									shadingColor = colorPlus(
									    shadingColor,
									    colorPlus(
									        colorMulti(
									            Blinn_PhongD(k_d * V, Ls[l], N),
									            tColor),
									        colorMulti(
									            Blinn_PhongS(k_s * V,
									                         TRI.shininess,
									                         Ls[l], N, C),
									            lights[l].color)));
								}
								break;
							}
						} else
							shadingColor = tColor;
						if (tPoint[2] >=
						        zBuffer[(Wheight - 1 - y) * Wwidth + x] &&
						    tPoint[2] <= 1) {
							zBuffer[(Wheight - 1 - y) * Wwidth + x] = tPoint[2];
							fundm::set_px(x, y, shadingColor);
						}
					}
				}
			}
		}
	}
}

void geometryR::appendTri(triR TRI) {
	size_t triI;
	bool v = false;
	for (size_t a = 0; a < tris.size(); a++) {
		if (!tris[a].valid) {
			triI = a;
			tris[triI] = TRI;
			v = true;
			break;
		}
	}
	if (!v) {
		tris.push_back(TRI);
		triI = tris.size() - 1;
	}

	std::array<pointR, 3> t;
	for (size_t i = 0; i < 3; i++) {
		t[i].pos = TRI.points[i];
		t[i].tri_index.push_back(triI);
		bool exist = false;
		for (size_t j = 0; j < ps.size(); j++) {
			if (ps[j].pos == t[i].pos) {
				ps[j].tri_index.push_back(triI);
				tris[triI].point_index[i] = j;
				exist = true;
				break;
			}
		}
		if (!exist) {
			ps.push_back(t[i]);
			tris[triI].point_index[i] = ps.size() - 1;
		}
	}
	Eigen::Vector3d AB = (TRI.points[1] - TRI.points[0]).head<3>();
	Eigen::Vector3d AC = (TRI.points[2] - TRI.points[0]).head<3>();
	Eigen::Vector3d AB_cross_AC = AB.cross(AC);
	tris[triI].area = AB_cross_AC.norm();
	tris[triI].normal = Eigen::Vector4d::Zero();
	tris[triI].normal.block<3, 1>(0, 0) = AB_cross_AC.normalized();
}

void geometryR::deleteTri(size_t index) {
	tris[index].valid = false;
	if (tris.size() == index + 1) {
		tris.pop_back();
	}
	size_t c = 0;
	for (size_t i = 0; i < ps.size(); i++) {
		for (size_t j = 0; j < ps[i].tri_index.size(); j++) {
			if (ps[i].tri_index[j] == index) {
				auto iter = ps[i].tri_index.begin();
				ps[i].tri_index.erase(iter + j);
				c++;
				break;
			}
		}
		if (c >= 3)
			break;
	}
}

void geometryR::preloadNormal() {
	for (size_t t = 0; t < tris.size(); t++) {
		Eigen::Vector3d AB = (tris[t].points[1] - tris[t].points[0]).head<3>();
		Eigen::Vector3d AC = (tris[t].points[2] - tris[t].points[0]).head<3>();
		Eigen::Vector3d AB_cross_AC = AB.cross(AC);
		tris[t].area = AB_cross_AC.norm();
		tris[t].normal = Eigen::Vector4d::Zero();
		tris[t].normal.block<3, 1>(0, 0) = AB_cross_AC.normalized();
	}
	for (size_t i = 0; i < ps.size(); i++) {
		double areaSum = 0;
		for (size_t j = 0; j < ps[i].tri_index.size(); j++) {
			areaSum += tris[ps[i].tri_index[j]].area;
		}
		for (size_t j = 0; j < ps[i].tri_index.size(); j++) {
			ps[i].normal += ((tris[ps[i].tri_index[j]].area) / areaSum) *
			                (tris[ps[i].tri_index[j]].normal);
		}
		ps[i].normal.normalize();
	}
}

bool geometryR::inTriangle(std::array<Eigen::Vector2d, 3> tri2d,
                           Eigen::Vector2d xy) {
	std::array<Eigen::Vector2d, 3> sides;
	for (size_t i = 0; i < 3; i++) {
		sides[i] = tri2d[(i + 1) % 3] - tri2d[i];
	}

	std::array<size_t, 3> p;
	for (size_t i = 0; i < 3; i++) {
		if (cross2(sides[i], xy - tri2d[i]) > 0) {
			p[i] = 1;
		} else if (cross2(sides[i], xy - tri2d[i]) < 0) {
			p[i] = -1;
		} else
			p[i] = 0;
	}
	if ((p[0] == p[1] && p[1] == p[2] && p[0] == p[2]) || p[0] == 0 ||
	    p[1] == 0 || p[2] == 0) {
		return true;
	} else
		return false;
}

AABB geometryR::get_aabb(std::array<Eigen::Vector2d, 3> tri2d) {
	double xmin, xmax, ymin, ymax;
	auto X = std::minmax({tri2d[0][0], tri2d[1][0], tri2d[2][0]});
	xmin = X.first >= 0 ? X.first : 0;
	xmax = X.second < Wwidth ? X.second : Wwidth;
	auto Y = std::minmax({tri2d[0][1], tri2d[1][1], tri2d[2][1]});
	ymin = Y.first >= 0 ? Y.first : 0;
	ymax = Y.second < Wheight ? Y.second : Wheight;
	AABB aabb = {xmin, xmax, ymin, ymax};
	return aabb;
}

// 虽然公式很nb但是用不到了（哭 （好像还是能用到，但是其实意义没之前大了
Eigen::Vector4d
geometryR::get_3dpoint(Eigen::Vector2d xy,
                       std::array<Eigen::Vector4d, 3> triTransed) {
	Eigen::Vector4d r(xy[0], xy[1], 1, 1);
	Eigen::Matrix3d ar;
	Eigen::Vector4d l1 = triTransed[1] - triTransed[0],
	                l2 = triTransed[2] - triTransed[0], l3 = r - triTransed[0];
	ar << l1[0], l1[1], l1[2], l2[0], l2[1], l2[2], l3[0], l3[1], l3[2];
	l3[2] = ar.block<1, 2>(2, 0) * ar.block<2, 2>(0, 0).inverse() *
	        ar.block<2, 1>(0, 2); // Schur补公式
	r[2] = l3[2] + triTransed[0][2];
	return r;
}

// 重心坐标公式
std::array<double, 3>
geometryR::get_BaryCoor(Eigen::Vector3d point,
                        std::array<Eigen::Vector3d, 3> TRI) {
	auto l1 = TRI[1] - TRI[0], l2 = TRI[2] - TRI[0];
	double area = l1.cross(l2).norm();
	std::array<double, 3> r;
	for (size_t i = 0; i < 2; i++) {
		size_t i1 = (i + 1) % 3, i2 = (i + 2) % 3;
		r[i] = (TRI[i1] - point).cross(TRI[i2] - point).norm() / area;
	}
	r[2] = 1 - r[0] - r[1];
	return r;
}

bool geometryR::rangeTest(int x, int y, int w, int h) {
	if (x >= 0 && x < w && y >= 0 && y < h) {
		return true;
	} else
		return false;
}

ARGB geometryR::colorDecompo(uint32_t color) {
	ARGB argb;
	argb.B = (uint8_t)color;
	argb.G = (uint8_t)(color >> 8);
	argb.R = (uint8_t)(color >> 16);
	argb.A = (uint8_t)(color >> 24);
	return argb;
}

uint32_t geometryR::colorMap(ARGB argb) {
	uint32_t c = 0;
	c += argb.B + (argb.G << 8) + (argb.R << 16) + (argb.A << 24);
	return c;
}

// 颜色插值
uint32_t geometryR::colorInter(std::array<double, 3> weights,
                               std::array<uint32_t, 3> colors) {
	std::array<uint8_t, 3> R, G, B, A;
	ARGB argb;
	double Ri, Bi, Gi, Ai;
	for (size_t i = 0; i < 3; i++) {
		ARGB targb = colorDecompo(colors[i]);
		R[i] = targb.R;
		G[i] = targb.G;
		B[i] = targb.B;
		A[i] = targb.A;
	}
	Ri = dot_array<double, uint8_t, double, 3>(weights, R);
	Gi = dot_array<double, uint8_t, double, 3>(weights, G);
	Bi = dot_array<double, uint8_t, double, 3>(weights, B);
	Ai = dot_array<double, uint8_t, double, 3>(weights, A);
	argb.R = Ri <= 0xff ? Ri : 0xff;
	argb.G = Gi <= 0xff ? Gi : 0xff;
	argb.B = Bi <= 0xff ? Bi : 0xff;
	argb.A = Ai <= 0xff ? Ai : 0xff;
	return colorMap(argb);
}

template <typename T, typename U>
U geometryR::interpolation(std::array<double, 3> weights,
                           std::array<T, 3> values) {
	return dot_array<double, T, U, 3>(weights, values);
}

// 检测三角形坍缩成直线的情况
bool geometryR::notALine(std::array<Eigen::Vector2d, 3> tri2d) {
	Eigen::Vector2d l1 = tri2d[1] - tri2d[0], l2 = tri2d[2] - tri2d[0];
	if (cross2(l1, l2) == 0)
		return false;
	else
		return true;
}

// 修正透视变化下重心坐标（核心公式）
std::array<double, 3> geometryR::persAmend(std::array<double, 3> baryCoor,
                                           std::array<double, 3> w) {
	std::array<double, 3> result;
	double under = baryCoor[0] / w[0] + baryCoor[1] / w[1] + baryCoor[2] / w[2];
	result[0] = baryCoor[0] / w[0] / under;
	result[1] = baryCoor[1] / w[1] / under;
	result[2] = baryCoor[2] / w[2] / under; // 不要忘了除以w哦
	return result;
}

triR *geometryR::changeTri(size_t index) {
	if (tris[index].valid) {
		return &tris[index];
	} else
		return nullptr;
}

void geometryR::transform(Eigen::Matrix4d tMat) {
	for (size_t i = 0; i < tris.size(); i++) {
		if (tris[i].valid) {
			for (size_t j = 0; j < 3; j++) {
				tris[i].points[j] = tMat * tris[i].points[j];
			}
		}
	}
}

uint32_t geometryR::get_uvmapColor(std::array<double, 3> baryCoor,
                                   size_t index) {
	Eigen::Vector2d uvd =
	    dot_array<Eigen::Vector2d, double, Eigen::Vector2d, 3>(
	        tris[index].uvMap, baryCoor);
	Eigen::Vector2i uv = uvd.cast<int>();
	std::vector<uint32_t> tex0 = (tris[index].texture)->texs[0];
	int w = (tris[index].texture)->w, h = (tris[index].texture)->h;
	if (rangeTest(uv[0], uv[1], w, h)) {
		return tex0[(w - 1 - uv[1]) * w + uv[0]];
	} else {
		return 0x00000000;
	}
}

void geometryR::appendLight(Light l) { lights.push_back(l); }

void geometryR::deleteLight(size_t i) { lights.erase(lights.begin() + i); }

void geometryR::setShadingM(shading_mode sm) { shadingM = sm; }

void boxR::setShading(geometryR::shading_mode sm) { geo.setShadingM(sm); };

boxR::boxR(double s, Eigen::Vector4d pos)
    : colors8({0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff, 0xffffffff,
               0xffffffff, 0xffffffff, 0xffffffff}),
      towards({Eigen::Vector4d(1, 0, 0, 0), Eigen::Vector4d(0, 1, 0, 0),
               Eigen::Vector4d(0, 0, 1, 0)}),
      k_a(0.3), k_d(0.7), k_s(1), I_a(0xffffffff), shininess(16) {

	scale = s;
	position = pos;

	/*surfaces[0].u.points =
	{v4(100,100,-100,1),v4(100,100,100,1),v4(100,-100,100,1)};
	surfaces[0].l.points =
	{v4(100,100,-100,1),v4(100,-100,100,1),v4(100,-100,-100,1)};
	surfaces[0].u.colors = {0xffffff,0xffffff,0xffffff};
	surfaces[0].l.colors = {0xffffff,0xffffff,0xffffff};*/

	// 表驱动
	int mainAxis[6]{0, 0, 1, 1, 2, 2};
	int YAxis[6]{1, 1, 2, 2, 0, 0};
	int ZAxis[6]{2, 2, 0, 0, 1, 1};
	double axisDir[6]{1, -1, 1, -1, 1, -1};
	double posY[4]{1, 1, -1, -1};
	double posZ[4]{-1, 1, 1, -1};
	int uCyc[3]{0, 1, 2};
	int lCyc[3]{0, 2, 3};
	for (size_t i = 0; i < 6; i++) {
		for (size_t j = 0; j < 3; j++) {
			surfaces[i].u.points[j][mainAxis[i]] = axisDir[i];
			surfaces[i].u.points[j][YAxis[i]] = posY[uCyc[j]];
			if (i % 2 == 1)
				surfaces[i].u.points[j][ZAxis[i]] = -posZ[uCyc[j]];
			else
				surfaces[i].u.points[j][ZAxis[i]] = posZ[uCyc[j]];
			surfaces[i].u.points[j][3] = 1;

			surfaces[i].l.points[j][mainAxis[i]] = axisDir[i];
			surfaces[i].l.points[j][YAxis[i]] = posY[lCyc[j]];
			if (i % 2 == 1)
				surfaces[i].l.points[j][ZAxis[i]] = -posZ[lCyc[j]];
			else
				surfaces[i].l.points[j][ZAxis[i]] = posZ[lCyc[j]];
			surfaces[i].l.points[j][3] = 1;
		}
	}
	for (size_t i = 0; i < 6; i++) {
		surfaces[i].u.single_face = true;
		surfaces[i].l.single_face = true;
		geo.appendTri(surfaces[i].u);
		geo.appendTri(surfaces[i].l);
	}
	geo.setShadingM(geometryR::shading_mode::GOURAUD_SHADING);
	update();
}

void boxR::render() { geo.renderGeo(); }

void boxR::update() {
	Eigen::Matrix4d scaleM = Eigen::Matrix4d::Identity();
	scaleM(0, 0) = scale;
	scaleM(1, 1) = scale;
	scaleM(2, 2) = scale;
	Eigen::Matrix4d posM = Eigen::Matrix4d::Identity();
	posM.block<3, 1>(0, 3) = position.head<3>();
	for (size_t i = 0; i < 12; i++) { // 逐三角变换
		for (size_t j = 0; j < 3; j++) {
			if (i % 2 == 0) {
				(*geo.get_tris())[i].points[j] =
				    posM * scaleM * surfaces[i / 2].u.points[j];
			} else {
				(*geo.get_tris())[i].points[j] =
				    posM * scaleM * surfaces[i / 2].l.points[j];
			}
			(*geo.get_tris())[i].k_a[j] = k_a;
			(*geo.get_tris())[i].k_d[j] = k_d;
			(*geo.get_tris())[i].k_s[j] = k_s;
			(*geo.get_tris())[i].I_a[j] = I_a;
			(*geo.get_tris())[i].shininess = shininess;
		}
	}
	for (size_t i = 0; i < 8; i++) { // 逐点变换
		size_t s = (*geo.get_ps())[i].tri_index.size();
		for (size_t j = 0; j < s; j++) {
			for (size_t k = 0; k < 3; k++) {
				if ((*geo.get_tris())[(*geo.get_ps())[i].tri_index[j]]
				        .point_index[k] == i) {
					(*geo.get_tris())[(*geo.get_ps())[i].tri_index[j]]
					    .colors[k] = colors8[i];
				}
			}
		}
	}
	preNormal();
}

// Blinn_Phong光照模型的反射光着色率
double geometryR::Blinn_PhongS(double k_s, int shininess,
                               Eigen::Vector4d LightDir, Eigen::Vector4d normal,
                               Eigen::Vector4d cameraDir) {
	Eigen::Vector4d H = (LightDir + cameraDir).normalized();
	return k_s * pow(std::max(0.0, H.dot(normal)), shininess);
}

// Blinn_Phong光照模型的漫射光着色率
double geometryR::Blinn_PhongD(double k_d, Eigen::Vector4d L,
                               Eigen::Vector4d N) {
	return k_d * std::max((double)0, N.dot(L));
}

double geometryR::lightValue(double value, double dis) {
	dis = std::abs(dis);
	if (dis > 0) {
		return value * (1 / pow(dis, 2));
	} else {
		return 0;
	}
}

void boxR::preNormal() { geo.preloadNormal(); }

void boxR::addLight(Light l) { geo.appendLight(l); }

uint32_t geometryR::colorMulti(double m, uint32_t c) {
	ARGB argb = colorDecompo(c);
	double R = argb.R * m;
	double G = argb.G * m;
	double B = argb.B * m;
	argb.R = R < 0xff ? R : 0xff;
	argb.G = G < 0xff ? G : 0xff;
	argb.B = B < 0xff ? B : 0xff;
	return colorMap(argb);
}

uint32_t geometryR::colorPlus(uint32_t c1, uint32_t c2) {
	ARGB argb = colorDecompo(c1);
	ARGB argb2 = colorDecompo(c2);
	double R = argb.R + argb2.R;
	double G = argb.G + argb2.G;
	double B = argb.B + argb2.B;
	argb.R = R < 0xff ? R : 0xff;
	argb.G = G < 0xff ? G : 0xff;
	argb.B = B < 0xff ? B : 0xff;
	return colorMap(argb);
}

void boxR::setColor(size_t i, uint32_t c1) {
	colors8[i] = c1;
	update();
}

void boxR::setColors(std::array<uint32_t, 8> c8) {
	colors8 = c8;
	update();
}

void boxR::setK(double a, double d, double s, double sh) {
	k_a = a;
	k_d = d;
	k_s = s;
	shininess = sh;
}

void boxR::move(Eigen::Vector4d dir) {
	position += dir;
	update();
}

void boxR::changeScale(double s) {
	scale = s;
	update();
}

} // namespace render_func
