// hw6.cpp : 定义控制台应用程序的入口点。
//

#include "Common.h"
#include "Geometry.h"
#include "shapelib/shapefil.h"

#include "CMakeIn.h" // 配置注入

#include <gl/freeglut.h> // GLUT库头文件

#include <cstdio>
#include <ctime>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <vector>

#define USE_BPLUSTREE
//#define USE_RTREE
#ifdef USE_RTREE
#include "RTree.h"
using TreeTy = hw6::RTree;
#elif defined(USE_BPLUSTREE)
#include "BPlusTree.h"
using TreeTy = hw6::BPlusTree;
#else
#include "QuadTree.h"
using TreeTy = hw6::QuadTree;
#endif


using namespace std;

int screenWidth = 640;
int screenHeight = 480;

double pointSize = 2.0;

int mode;

vector<hw6::Feature> features;
vector<hw6::Feature> roads;
bool showRoad = true;

unique_ptr<hw6::Tree> pointTree;
unique_ptr<hw6::Tree> roadTree;
bool showTree = false;

hw6::Feature nearestFeature;

bool firstPoint = true;
hw6::Point corner[2];
hw6::Envelope selectedRect;
vector<hw6::Feature> selectedFeatures;

/*
 * shapefile文件中name和geometry属性读取
 */
vector<string> readName(const char* filename) {
	DBFHandle file = DBFOpen(filename, "r");

	vector<string> res;
	int cct = DBFGetRecordCount(file);
	res.reserve(cct);
	for (int i = 0; i < cct; ++i) {
		string a = DBFReadStringAttribute(file, i, 0);
		res.push_back(a);
	}

	DBFClose(file);

	return res;
}

vector<hw6::Geometry*> readGeom(const char* filename) {
	SHPHandle file = SHPOpen(filename, "r");

	int pnEntities, pnShapeType;
	double padfMinBound[4], padfMaxBound[4];
	SHPGetInfo(file, &pnEntities, &pnShapeType, padfMinBound, padfMaxBound);

	vector<hw6::Point> points;
	vector<hw6::Geometry*> geoms;
	geoms.reserve(pnEntities);
	switch (pnShapeType) {
	case SHPT_POINT:
		for (int i = 0; i < pnEntities; ++i) {
			SHPObject* pt = SHPReadObject(file, i);
			geoms.push_back(new hw6::Point(pt->padfY[0], pt->padfX[0]));
			SHPDestroyObject(pt);
		}
		break;

	case SHPT_ARC:
		for (int i = 0; i < pnEntities; ++i) {
			points.clear();
			SHPObject* pt = SHPReadObject(file, i);
			for (int j = 0; j < pt->nVertices; ++j) {
				points.push_back(hw6::Point(pt->padfY[j], pt->padfX[j]));
			}
			SHPDestroyObject(pt);
			geoms.push_back(new hw6::LineString(points));
		}
		break;

	case SHPT_POLYGON:
		for (int i = 0; i < pnEntities; ++i) {
			points.clear();
			SHPObject* pt = SHPReadObject(file, i);
			for (int j = 0; j < pt->nVertices; ++j) {
				points.push_back(hw6::Point(pt->padfY[j], pt->padfX[j]));
			}
			SHPDestroyObject(pt);
			hw6::LineString line(points);
			hw6::Polygon* poly = new hw6::Polygon(line);
			geoms.push_back(new hw6::Polygon(line));
		}
		break;
	}

	SHPClose(file);
	return geoms;
}

void transformValue(double& res, const char* format = "%.2lf") {
	char buf[20];
	sprintf(buf, format, res);
	sscanf(buf, "%lf", &res);
}

void wrongMessage(hw6::Envelope e1, hw6::Envelope e2, bool cal) {
	cout << "Your answer is " << cal << " for test between ";
	e1.print();
	cout << " and ";
	e2.print();
	cout << ", but the answer is " << !cal << endl;
}

void wrongMessage(const hw6::Point& pt1, const hw6::Point& pt2, double dis,
	double res) {
	cout << "Your answer is " << dis << " for test ";
	pt1.print();
	cout << " and ";
	pt2.print();
	cout << ", but the answer is " << res << endl;
}

void wrongMessage(hw6::Envelope e1, hw6::Envelope e2, hw6::Envelope cal,
	hw6::Envelope res) {
	cout << "Your answer is ";
	cal.print();
	cout << " for test between ";
	e1.print();
	cout << " and ";
	e2.print();
	cout << ", but the answer is ";
	res.print();
	cout << endl;
}

/*
 * 输出几何信息
 */
void printGeom(vector<hw6::Geometry*>& geom) {
	cout << "Geometry:" << endl;
	for (vector<hw6::Geometry*>::iterator it = geom.begin(); it != geom.end();
		++it) {
		(*it)->print();
	}
}

/*
 * 删除几何信息
 */
void deleteGeom(vector<hw6::Geometry*>& geom) {
	for (vector<hw6::Geometry*>::iterator it = geom.begin(); it != geom.end();
		++it) {
		delete* it;
		*it = NULL;
	}
	geom.clear();
}

/*
 * 读取纽约道路数据
 */
void loadRoadData() {
	vector<hw6::Geometry*> geom = readGeom(PROJ_SRC_DIR "/data/highway");

	roads.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		roads.push_back(hw6::Feature(to_string(i), geom[i]));

	cout << "road number: " << geom.size() << endl;
	roadTree->setCapacity(20);
	roadTree->constructTree(roads);
}

/*
 * 读取纽约自行车租赁点数据
 */
void loadStationData() {
	vector<hw6::Geometry*> geom = readGeom(PROJ_SRC_DIR "/data/station");
	vector<string> name = readName(PROJ_SRC_DIR "/data/station");

	features.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		features.push_back(hw6::Feature(name[i], geom[i]));

	cout << "station number: " << geom.size() << endl;
	pointTree->setCapacity(5);
	pointTree->constructTree(features);
}

/*
 * 读取纽约出租车打车点数据
 */
void loadTaxiData() {
	vector<hw6::Geometry*> geom = readGeom(PROJ_SRC_DIR "/data/taxi");
	vector<string> name = readName(PROJ_SRC_DIR "/data/taxi");

	features.clear();
	for (size_t i = 0; i < geom.size(); ++i)
		features.push_back(hw6::Feature(name[i], geom[i]));

	cout << "taxi number: " << geom.size() << endl;
	pointTree->setCapacity(100);
	pointTree->constructTree(features);
}

/*
 * 区域查询
 */
void rangeQuery() {
	vector<hw6::Feature> candidateFeatures;

	// filter step (使用四叉树获得查询区域和几何特征包围盒相交的候选集）
	if (mode == RANGEPOINT)
		pointTree->rangeQuery(selectedRect, candidateFeatures);
	else if (mode == RANGELINE)
		roadTree->rangeQuery(selectedRect, candidateFeatures);

	// refine step (精确判断时，需要去重，避免查询区域和几何对象的重复计算)
	// TODO
	/*selectedFeatures.clear();
	std::vector<std::string> seenNames;
	std::vector<const hw6::Geometry*> seenGeoms;

	for (const auto& f : candidateFeatures) {
		const std::string& id = f.getName();
		bool seen = false;
		if (!id.empty()) {
			for (const auto& s : seenNames) {
				if (s == id) { seen = true; break; }
			}
			if (seen) continue;
			seenNames.push_back(id);
		}
		else {
			const hw6::Geometry* g = f.getGeom();
			if (!g) continue;
			for (const auto& pg : seenGeoms) {
				if (pg == g) { seen = true; break; }
			}
			if (seen) continue;
			seenGeoms.push_back(g);
		}

		const hw6::Geometry* geom = f.getGeom();
		if (!geom) continue;

		// 用包围盒相交作为判断（若有更精确方法可替换）
		const hw6::Envelope& fe = f.getEnvelope();
		bool accept = !(fe.getMaxX() < selectedRect.getMinX() ||
			fe.getMinX() > selectedRect.getMaxX() ||
			fe.getMaxY() < selectedRect.getMinY() ||
			fe.getMinY() > selectedRect.getMaxY());
		if (accept) selectedFeatures.push_back(f);
	}

	glutPostRedisplay();*/
	selectedFeatures.clear();
	size_t numOfFeatures = candidateFeatures.size();
	//去重
	for (size_t i = 0; i < numOfFeatures; i++) {
		int n = selectedFeatures.size();
		bool f = true;
		for (size_t j = 0; j < n; j++) {
			if (selectedFeatures[j].getGeom() == candidateFeatures[i].getGeom()) {
				f = false;
			}
		}
		if (f && candidateFeatures[i].getGeom()->intersects(selectedRect)) {
			selectedFeatures.push_back(candidateFeatures[i]);
		}
	}
}

/*
 * 邻近查询
 */
void NNQuery(hw6::Point p) {
	vector<hw6::Feature> candidateFeatures;

	const auto& env = (mode == NNPOINT ? pointTree->getEnvelope() : roadTree->getEnvelope());
	//printf("NNQuery tree env: (%f,%f)-(%f,%f)\n", env.getMinX(), env.getMinY(), env.getMaxX(), env.getMaxY());
	// filter step (使用四叉树获得距离较近的几何特征候选集)
	if (mode == NNPOINT)
		pointTree->NNQuery(p.getX(), p.getY(), candidateFeatures);
	else if (mode == NNLINE)
		roadTree->NNQuery(p.getX(), p.getY(), candidateFeatures);

	// refine step (精确计算查询点与几何对象的距离)
	// TODO
	selectedFeatures.clear();
	double bestDist = std::numeric_limits<double>::infinity();
	int bestIdx = -1;
	for (size_t i = 0; i < candidateFeatures.size(); ++i) {
		printf("query pt=(%.12f,%.12f)\n", p.getX(), p.getY());
		const auto& f = candidateFeatures[i];
		//double d_env = envelopeDist(f.getEnvelope(), p.getX(), p.getY());
		double d_geom = f.distance(p.getX(), p.getY());
		printf("geomDist raw=%a\n", f.distance(p.getX(), p.getY()));
		printf("cand[%zu] name=%s geom=%p geomDist=%.12f\n",
			i, f.getName().c_str(), (void*)f.getGeom(),  d_geom);
		if (d_geom < bestDist) {
			bestDist = d_geom;
			bestIdx = (int)i;
		}
	}
	if (bestIdx >= 0) {
		selectedFeatures.push_back(candidateFeatures[bestIdx]);
		nearestFeature = candidateFeatures[bestIdx];
		printf("chosen idx=%d name=%s geomDist=%.12f\n", bestIdx, nearestFeature.getName().c_str(), bestDist);
	}
	else {
		nearestFeature = hw6::Feature();
	}
	/*size_t numOfFeatures = candidateFeatures.size();
	for (size_t i = 0; i < numOfFeatures; i++) {
		int n = selectedFeatures.size();
		bool f = true;
		for (size_t j = 0; j < n; j++) {
			if (selectedFeatures[j].getGeom() == candidateFeatures[i].getGeom()) {
				f = false;
			}
		}
		if (f) {
			selectedFeatures.push_back(candidateFeatures[i]);
		}
	}
	numOfFeatures = 0;
	if (!selectedFeatures.empty()) {
		numOfFeatures = selectedFeatures.size();
		size_t index = 0;
		double minDist = 0;
		if (mode == NNPOINT) {
			minDist = p.distance((hw6::Point*)(selectedFeatures[0].getGeom()));
		}
		else if (mode == NNLINE) {
			minDist = selectedFeatures[0].distance(p.getX(), p.getY());
		}
		for (size_t i = 0; i < numOfFeatures; i++) {
			double dist = 0;
			if (mode == NNPOINT) {
				dist = p.distance((hw6::Point*)(selectedFeatures[i].getGeom()));
			}
			else if (mode == NNLINE) {
				dist = selectedFeatures[i].distance(p.getX(), p.getY());
			}
			if (dist < minDist) {
				index = i;
				minDist = dist;
			}
		}
		nearestFeature = selectedFeatures[index];
	}
	else {
		nearestFeature = hw6::Feature();
	}*/
}

/*
 * 从屏幕坐标转换到地理坐标
 */
void transfromPt(hw6::Point& pt) {
	const hw6::Envelope bbox = pointTree->getEnvelope();
	double width = bbox.getMaxX() - bbox.getMinX() + 0.002;
	double height = bbox.getMaxY() - bbox.getMinY() + 0.002;

	double x = pt.getX() * width / screenWidth + bbox.getMinX() - 0.001;
	double y = pt.getY() * height / screenHeight + bbox.getMinY() - 0.001;

	x = max(bbox.getMinX(), x);
	x = min(bbox.getMaxX(), x);
	y = max(bbox.getMinY(), y);
	y = min(bbox.getMaxY(), y);
	pt = hw6::Point(x, y);
}

/*
 * 绘制代码
 */
void display() {
	// glClearColor(241 / 255.0, 238 / 255.0, 232 / 255.0, 0.0);
	// 设置背景颜色
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glClear(GL_COLOR_BUFFER_BIT);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	const hw6::Envelope bbox = pointTree->getEnvelope();

	glOrtho(bbox.getMinX() - 0.001, bbox.getMaxX() + 0.001,
		bbox.getMinY() - 0.001, bbox.getMaxY() + 0.001,
		-1.0, 1.0);

	// 道路绘制
	if (showRoad) {
		glColor3d(252 / 255.0, 214 / 255.0, 164 / 255.0);
		//glColor3d(0.6, 0.3, 0.0); // 棕色
		for (size_t i = 0; i < roads.size(); ++i)
			roads[i].draw();
	}

	// 点绘制
	if (!(mode == RANGELINE || mode == NNLINE)) {
		glPointSize((float)pointSize);
		glColor3d(0.0, 146 / 255.0, 247 / 255.0);
		for (size_t i = 0; i < features.size(); ++i)
			features[i].draw();
	}

	// 四叉树绘制
	if (showTree) {
		glColor3d(0.0, 146 / 255.0, 247 / 255.0);
		if (mode == RANGELINE || mode == NNLINE)
			roadTree->draw();
		else
			pointTree->draw();

	}

	// 离鼠标最近点绘制
	if (mode == NNPOINT) {
		glPointSize(5.0);
		glColor3d(0.9, 0.0, 0.0);
		nearestFeature.draw();
	}

	// 离鼠标最近道路绘制
	if (mode == NNLINE) {
		glLineWidth(3.0);
		glColor3d(0.9, 0.0, 0.0);
		nearestFeature.draw();
		glLineWidth(1.0);
	}

	// 区域选择绘制
	if (mode == RANGEPOINT || mode == RANGELINE) {
		glColor3d(0.0, 0.0, 0.0);
		selectedRect.draw();
		glColor3d(1.0, 0.0, 0.0);
		for (size_t i = 0; i < selectedFeatures.size(); ++i)
			selectedFeatures[i].draw();
	}

	glFlush();
	glutSwapBuffers();
}

/*
 * 鼠标和键盘交互
 */
void mouse(int button, int state, int x, int y) {
	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
		if (mode == RANGEPOINT || mode == RANGELINE) {
			if (firstPoint) {
				selectedFeatures.clear();
				corner[0] = hw6::Point(x, screenHeight - y);
				transfromPt(corner[0]);
			}
			else {
				corner[1] = hw6::Point(x, screenHeight - y);
				transfromPt(corner[1]);
				selectedRect =
					hw6::Envelope(min(corner[0].getX(), corner[1].getX()),
						max(corner[0].getX(), corner[1].getX()),
						min(corner[0].getY(), corner[1].getY()),
						max(corner[0].getY(), corner[1].getY()));
				rangeQuery();
			}
			firstPoint = !firstPoint;
			glutPostRedisplay();
		}
	}
}

void passiveMotion(int x, int y) {
	corner[1] = hw6::Point(x, screenHeight - y);

	if ((mode == RANGEPOINT || mode == RANGELINE) && !firstPoint) {
		corner[1] = hw6::Point(x, screenHeight - y);
		transfromPt(corner[1]);
		selectedRect = hw6::Envelope(min(corner[0].getX(), corner[1].getX()),
			max(corner[0].getX(), corner[1].getX()),
			min(corner[0].getY(), corner[1].getY()),
			max(corner[0].getY(), corner[1].getY()));
		rangeQuery();

		glutPostRedisplay();
	}
	else if (mode == NNPOINT || mode == NNLINE) {
		hw6::Point p(x, screenHeight - y);
		transfromPt(p);
		NNQuery(p);

		glutPostRedisplay();
	}
}

void changeSize(int w, int h) {
	screenWidth = w;
	screenHeight = h;
	glViewport(0, 0, w, h);
	glutPostRedisplay();
}

void processNormalKeys(unsigned char key, int x, int y) {
	switch (key) {
	case 27:
		exit(0);
		break;
	case 'N':
		mode = NNLINE;
		break;
	case 'n':
		mode = NNPOINT;
		break;
	case 'S':
		mode = RANGELINE;
		firstPoint = true;
		break;
	case 's':
		mode = RANGEPOINT;
		firstPoint = true;
		break;
	case 'B':
	case 'b':
		loadStationData();
		mode = Default;
		break;
	case 'T':
	case 't':
		loadTaxiData();
		mode = Default;
		break;
	case 'R':
	case 'r':
		showRoad = !showRoad;
		break;
	case 'Q':
	case 'q':
		showTree = !showTree;
		break;
	case '+':
		pointSize *= 1.1;
		break;
	case '-':
		pointSize /= 1.1;
		break;
	case '1':
	case '2':
	case '3':
	case '4':
	case '5':
	case '6':
	case '7':
	case '8':
		TreeTy::test(key - '0');
		break;
	default:
		mode = Default;
		break;
	}
	glutPostRedisplay();
}

int main(int argc, char* argv[]) {
	cout << "Key Usage:\n"
		<< "  S  : range search for roads\n"
		<< "  s  : range search for stations\n"
		<< "  N  : nearest road search\n"
		<< "  n  : nearest station search\n"
		<< "  B/b: Bicycle data\n"
		<< "  T/t: Taxi data\n"
		<< "  R/r: show Road\n"
		<< "  Q/q: show Tree\n"
		<< "  +  : increase point size\n"
		<< "  -  : decrease point size\n"
		<< "  1  : Test Envelope contain, interset and union\n"
		<< "  2  : Test distance between Point and LineString\n"
		<< "  3  : Test distance between Point and Polygon\n"
		<< "  4  : Test tree construction\n"
		<< "  5  : Test (your option here)\n"
		<< "  8  : Tree performance analysis\n"
		<< "  ESC: quit\n"
		<< endl;

	pointTree = make_unique<TreeTy>();
	roadTree = make_unique<TreeTy>();

	loadRoadData();

	loadStationData();

	glutInit(&argc, argv);
	glutInitWindowSize(screenWidth, screenHeight);
	glutInitWindowPosition(0, 0);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
	glutCreateWindow("New York");

	glutMouseFunc(mouse);
	glutDisplayFunc(display);
	glutPassiveMotionFunc(passiveMotion);
	glutReshapeFunc(changeSize);
	glutKeyboardFunc(processNormalKeys);

	glutMainLoop();

	return 0;
}
