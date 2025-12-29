#include "Geometry.h"
#include <cmath>
#include <gl/freeglut.h>

#define NOT_IMPLEMENT -1.0

namespace hw6 {

	/*
	 * Envelope functions
	 */
	bool Envelope::contain(double x, double y) const {
		return x >= minX && x <= maxX && y >= minY && y <= maxY;
	}

	bool Envelope::contain(const Envelope& envelope) const {
		// Task 测试Envelope是否包含关系
		// TODO
		return envelope.getMaxX() <= maxX && envelope.getMinX() >= minX && envelope.getMaxY() <= maxY && envelope.getMinY() >= minY;
	}

	bool Envelope::intersect(const Envelope& envelope) const {
		// Task 测试Envelope是否相交
		// TODO
		return !(envelope.getMaxX() < minX || envelope.getMinX() > maxX || envelope.getMaxY() < minY || envelope.getMinY() > maxY);
	}

	Envelope Envelope::unionEnvelope(const Envelope& envelope) const {
		// Task 合并两个Envelope生成一个新的Envelope
		// TODO
		double union_minX = std::min(minX, envelope.getMinX());
		double union_minY = std::min(minY, envelope.getMinY());
		double union_maxX = std::max(maxX, envelope.getMaxX());
		double union_maxY = std::max(maxY, envelope.getMaxY());
		return Envelope(union_minX, union_maxX, union_minY, union_maxY);
	}

	void Envelope::draw() const {
		glBegin(GL_LINE_STRIP);

		glVertex2d(minX, minY);
		glVertex2d(minX, maxY);
		glVertex2d(maxX, maxY);
		glVertex2d(maxX, minY);
		glVertex2d(minX, minY);

		glEnd();
	}

	/*
	 * Points functions
	 */
	double Point::distance(const Point* point) const {
		return sqrt((x - point->x) * (x - point->x) +
			(y - point->y) * (y - point->y));
	}

	double Point::distance(const LineString* line) const {
		double mindist = line->getPointN(0).distance(this);
		for (size_t i = 0; i < line->numPoints() - 1; ++i) {
			double dist = 0;
			double x1 = line->getPointN(i).getX();
			double y1 = line->getPointN(i).getY();
			double x2 = line->getPointN(i + 1).getX();
			double y2 = line->getPointN(i + 1).getY();
			// Task calculate the distance between Point P(x, y) and Line [P1(x1,
			// y1), P2(x2, y2)] (less than 10 lines)
			// TODO
			double px = getX(), py = getY();
			double vx = x2 - x1, vy = y2 - y1;
			double wx = px - x1, wy = py - y1;
			double c1 = vx * wx + vy * wy;
			double c2 = vx * vx + vy * vy;
			double t = (c2 == 0) ? 0 : c1 / c2;
			t = std::max(0.0, std::min(1.0, t));
			double projx = x1 + t * vx, projy = y1 + t * vy;
			dist = std::sqrt((px - projx) * (px - projx) + (py - projy) * (py - projy));

			if (dist < mindist)
				mindist = dist;
		}
		return mindist;
	}

	double Point::distance(const Polygon* polygon) const {
		LineString line = polygon->getExteriorRing();

		size_t n = line.numPoints();

		bool inPolygon = false;
		// Task whether Point P(x, y) is within Polygon (less than 15 lines)
		// TODO
		double px = this->getX();
		double py = this->getY();
		for (size_t i = 0, j = n - 1; i < n; j = i++) {
			double xi = line.getPointN(i).getX();
			double yi = line.getPointN(i).getY();
			double xj = line.getPointN(j).getX();
			double yj = line.getPointN(j).getY();
			bool intersect = ((yi > py) != (yj > py)) &&
				(px < (xj - xi) * (py - yi) / (yj - yi) + xi);
			if (intersect)
				inPolygon = !inPolygon;
		}

		double mindist = 0;
		if (!inPolygon)
			mindist = this->distance(&line);
		return mindist;
	}

	bool Point::intersects(const Envelope& rect) const {
		return (x >= rect.getMinX()) && (x <= rect.getMaxX()) &&
			(y >= rect.getMinY()) && (y <= rect.getMaxY());
	}

	void Point::draw() const {
		glBegin(GL_POINTS);
		glVertex2d(x, y);
		glEnd();
	}

	/*
	 * LineString functions
	 */
	void LineString::constructEnvelope() {
		double minX, minY, maxX, maxY;
		maxX = minX = points[0].getX();
		maxY = minY = points[0].getY();
		for (size_t i = 1; i < points.size(); ++i) {
			maxX = std::max(maxX, points[i].getX());
			maxY = std::max(maxY, points[i].getY());
			minX = std::min(minX, points[i].getX());
			minY = std::min(minY, points[i].getY());
		}
		envelope = Envelope(minX, maxX, minY, maxY);
	}
	static double pointToSegmentDist2(double px, double py,
		double x1, double y1, double x2, double y2) {
		double vx = x2 - x1;
		double vy = y2 - y1;
		double wx = px - x1;
		double wy = py - y1;
		double c1 = vx * wx + vy * wy;
		if (c1 <= 0.0) {
			double dx = px - x1, dy = py - y1;
			return dx * dx + dy * dy;
		}
		double c2 = vx * vx + vy * vy;
		if (c2 <= c1) {
			double dx = px - x2, dy = py - y2;
			return dx * dx + dy * dy;
		}
		double t = c1 / c2;
		double projx = x1 + t * vx;
		double projy = y1 + t * vy;
		double dx = px - projx, dy = py - projy;
		return dx * dx + dy * dy;
	}

	static double orient(double ax, double ay, double bx, double by, double cx, double cy) {
		return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
	}
	static bool onSegment(double ax, double ay, double bx, double by, double px, double py) {
		return std::min(ax, bx) <= px + 1e-12 && px <= std::max(ax, bx) + 1e-12 &&
			std::min(ay, by) <= py + 1e-12 && py <= std::max(ay, by) + 1e-12;
	}
	static bool segmentsIntersect(double a1x, double a1y, double a2x, double a2y,
		double b1x, double b1y, double b2x, double b2y) {
		double o1 = orient(a1x, a1y, a2x, a2y, b1x, b1y);
		double o2 = orient(a1x, a1y, a2x, a2y, b2x, b2y);
		double o3 = orient(b1x, b1y, b2x, b2y, a1x, a1y);
		double o4 = orient(b1x, b1y, b2x, b2y, a2x, a2y);

		if (fabs(o1) < 1e-12 && onSegment(a1x, a1y, a2x, a2y, b1x, b1y)) return true;
		if (fabs(o2) < 1e-12 && onSegment(a1x, a1y, a2x, a2y, b2x, b2y)) return true;
		if (fabs(o3) < 1e-12 && onSegment(b1x, b1y, b2x, b2y, a1x, a1y)) return true;
		if (fabs(o4) < 1e-12 && onSegment(b1x, b1y, b2x, b2y, a2x, a2y)) return true;

		return (o1 * o2 < 0.0) && (o3 * o4 < 0.0);
	}

	// 线段到线段平方距离（端点到对方线段最小值）
	static double segmentToSegmentDist2(double ax1, double ay1, double ax2, double ay2,
		double bx1, double by1, double bx2, double by2) {
		if (segmentsIntersect(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2)) return 0.0;
		double d1 = pointToSegmentDist2(ax1, ay1, bx1, by1, bx2, by2);
		double d2 = pointToSegmentDist2(ax2, ay2, bx1, by1, bx2, by2);
		double d3 = pointToSegmentDist2(bx1, by1, ax1, ay1, ax2, ay2);
		double d4 = pointToSegmentDist2(bx2, by2, ax1, ay1, ax2, ay2);
		return std::min(std::min(d1, d2), std::min(d3, d4));
	}

	// --- LineString::distance(LineString) ---
	double LineString::distance(const LineString* line) const {
		if (!line) return std::numeric_limits<double>::infinity();
		size_t n1 = this->numPoints();
		size_t n2 = line->numPoints();
		if (n1 == 0 || n2 == 0) return std::numeric_limits<double>::infinity();
		if (n1 == 1 && n2 == 1) {
			double dx = this->getPointN(0).getX() - line->getPointN(0).getX();
			double dy = this->getPointN(0).getY() - line->getPointN(0).getY();
			return std::sqrt(dx * dx + dy * dy);
		}

		double minDist2 = std::numeric_limits<double>::infinity();

		// 单点到线串
		if (n1 == 1) {
			double px = this->getPointN(0).getX();
			double py = this->getPointN(0).getY();
			for (size_t j = 0; j + 1 < n2; ++j) {
				const auto& b1 = line->getPointN(j);
				const auto& b2 = line->getPointN(j + 1);
				minDist2 = std::min(minDist2,
					pointToSegmentDist2(px, py, b1.getX(), b1.getY(), b2.getX(), b2.getY()));
				if (minDist2 == 0.0) return 0.0;
			}
			return std::sqrt(minDist2);
		}
		if (n2 == 1) {
			double px = line->getPointN(0).getX();
			double py = line->getPointN(0).getY();
			for (size_t i = 0; i + 1 < n1; ++i) {
				const auto& a1 = this->getPointN(i);
				const auto& a2 = this->getPointN(i + 1);
				minDist2 = std::min(minDist2,
					pointToSegmentDist2(px, py, a1.getX(), a1.getY(), a2.getX(), a2.getY()));
				if (minDist2 == 0.0) return 0.0;
			}
			return std::sqrt(minDist2);
		}

		// 一般情况：比较每一对线段（若相交则返回0）
		for (size_t i = 0; i + 1 < n1; ++i) {
			const auto& a1 = this->getPointN(i);
			const auto& a2 = this->getPointN(i + 1);
			for (size_t j = 0; j + 1 < n2; ++j) {
				const auto& b1 = line->getPointN(j);
				const auto& b2 = line->getPointN(j + 1);
				double d2 = segmentToSegmentDist2(
					a1.getX(), a1.getY(), a2.getX(), a2.getY(),
					b1.getX(), b1.getY(), b2.getX(), b2.getY()
				);
				if (d2 < minDist2) minDist2 = d2;
				if (minDist2 == 0.0) return 0.0;
			}
		}

		return std::sqrt(minDist2);
	}

	// --- LineString::distance(Polygon) ---
	double LineString::distance(const Polygon* polygon) const {
		if (!polygon) return std::numeric_limits<double>::infinity();

		const LineString& exterior = polygon->getExteriorRing();
		// 1) 若线与外环相交，则距离为0
		// 检查任一线段是否与外环某线段相交
		size_t nL = this->numPoints();
		size_t nE = exterior.numPoints();
		if (nL >= 2 && nE >= 2) {
			for (size_t i = 0; i + 1 < nL; ++i) {
				const auto& a1 = this->getPointN(i);
				const auto& a2 = this->getPointN(i + 1);
				for (size_t j = 0; j + 1 < nE; ++j) {
					const auto& b1 = exterior.getPointN(j);
					const auto& b2 = exterior.getPointN(j + 1);
					if (segmentsIntersect(a1.getX(), a1.getY(), a2.getX(), a2.getY(),
						b1.getX(), b1.getY(), b2.getX(), b2.getY())) {
						return 0.0;
					}
				}
			}
		}

		// 2) 若线的任一点在多边形内部，则距离为0
		// 假定 Polygon 提供 containsPoint(const Point&) 或 contains(x,y) —— 若没有请忽略此段。
#ifdef POLYGON_HAS_CONTAINS_POINT
		for (size_t i = 0; i < nL; ++i) {
			const auto& p = this->getPointN(i);
			if (polygon->containsPoint(p)) return 0.0;
		}
#endif

		// 3) 否则，计算线串与外环的最小距离（线段对比）
		double minDist2 = std::numeric_limits<double>::infinity();
		if (nL >= 2 && nE >= 2) {
			for (size_t i = 0; i + 1 < nL; ++i) {
				const auto& a1 = this->getPointN(i);
				const auto& a2 = this->getPointN(i + 1);
				for (size_t j = 0; j + 1 < nE; ++j) {
					const auto& b1 = exterior.getPointN(j);
					const auto& b2 = exterior.getPointN(j + 1);
					double d2 = segmentToSegmentDist2(
						a1.getX(), a1.getY(), a2.getX(), a2.getY(),
						b1.getX(), b1.getY(), b2.getX(), b2.getY()
					);
					if (d2 < minDist2) minDist2 = d2;
					if (minDist2 == 0.0) return 0.0;
				}
			}
		}

		if (minDist2 == std::numeric_limits<double>::infinity())
			return std::numeric_limits<double>::infinity();
		return std::sqrt(minDist2);
	}

	typedef int OutCode;

	const int INSIDE = 0; // 0000
	const int LEFT = 1;   // 0001
	const int RIGHT = 2;  // 0010
	const int BOTTOM = 4; // 0100
	const int TOP = 8;    // 1000

	// Compute the bit code for a point (x, y) using the clip rectangle
	// bounded diagonally by (xmin, ymin), and (xmax, ymax)
	// ASSUME THAT xmax, xmin, ymax and ymin are global constants.
	OutCode ComputeOutCode(double x, double y, double xmin, double xmax,
		double ymin, double ymax) {
		OutCode code;

		code = INSIDE; // initialised as being inside of [[clip window]]

		if (x < xmin) // to the left of clip window
			code |= LEFT;
		else if (x > xmax) // to the right of clip window
			code |= RIGHT;
		if (y < ymin) // below the clip window
			code |= BOTTOM;
		else if (y > ymax) // above the clip window
			code |= TOP;

		return code;
	}

	// Cohen–Sutherland clipping algorithm clips a line from
	// P0 = (x0, y0) to P1 = (x1, y1) against a rectangle with
	// diagonal from (xmin, ymin) to (xmax, ymax).
	bool intersectTest(double x0, double y0, double x1, double y1, double xmin,
		double xmax, double ymin, double ymax) {
		// compute outcodes for P0, P1, and whatever point lies outside the clip
		// rectangle
		OutCode outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
		OutCode outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
		bool accept = false;

		while (true) {
			if (!(outcode0 | outcode1)) {
				// bitwise OR is 0: both points inside window; trivially accept and
				// exit loop
				accept = true;
				break;
			}
			else if (outcode0 & outcode1) {
				// bitwise AND is not 0: both points share an outside zone (LEFT,
				// RIGHT, TOP, or BOTTOM), so both must be outside window; exit loop
				// (accept is false)
				break;
			}
			else {
				// failed both tests, so calculate the line segment to clip
				// from an outside point to an intersection with clip edge
				double x, y;

				// At least one endpoint is outside the clip rectangle; pick it.
				OutCode outcodeOut = outcode0 ? outcode0 : outcode1;

				// Now find the intersection point;
				// use formulas:
				//   slope = (y1 - y0) / (x1 - x0)
				//   x = x0 + (1 / slope) * (ym - y0), where ym is ymin or ymax
				//   y = y0 + slope * (xm - x0), where xm is xmin or xmax
				// No need to worry about divide-by-zero because, in each case, the
				// outcode bit being tested guarantees the denominator is non-zero
				if (outcodeOut & TOP) { // point is above the clip window
					x = x0 + (x1 - x0) * (ymax - y0) / (y1 - y0);
					y = ymax;
				}
				else if (outcodeOut & BOTTOM) { // point is below the clip window
					x = x0 + (x1 - x0) * (ymin - y0) / (y1 - y0);
					y = ymin;
				}
				else if (outcodeOut &
					RIGHT) { // point is to the right of clip window
					y = y0 + (y1 - y0) * (xmax - x0) / (x1 - x0);
					x = xmax;
				}
				else if (outcodeOut &
					LEFT) { // point is to the left of clip window
					y = y0 + (y1 - y0) * (xmin - x0) / (x1 - x0);
					x = xmin;
				}

				// Now we move outside point to intersection point to clip
				// and get ready for next pass.
				if (outcodeOut == outcode0) {
					x0 = x;
					y0 = y;
					outcode0 = ComputeOutCode(x0, y0, xmin, xmax, ymin, ymax);
				}
				else {
					x1 = x;
					y1 = y;
					outcode1 = ComputeOutCode(x1, y1, xmin, xmax, ymin, ymax);
				}
			}
		}
		return accept;
	}

	bool LineString::intersects(const Envelope& rect) const {
		double xmin = rect.getMinX();
		double xmax = rect.getMaxX();
		double ymin = rect.getMinY();
		double ymax = rect.getMaxY();

		for (size_t i = 1; i < points.size(); ++i)
			if (intersectTest(points[i - 1].getX(), points[i - 1].getY(),
				points[i].getX(), points[i].getY(), xmin, xmax, ymin,
				ymax))
				return true;
		return false;
	}

	void LineString::draw() const {
		if (points.empty()) {
			std::cerr << "Warning: LineString points array is empty." << std::endl;
			return;
		}

		glBegin(GL_LINE_STRIP);
		for (size_t i = 0; i < points.size(); ++i)
			glVertex2d(points[i].getX(), points[i].getY());
		glEnd();

		GLenum err;
		while ((err = glGetError()) != GL_NO_ERROR) {
			std::cerr << "OpenGL Error in draw(): " << gluErrorString(err) << std::endl;
		}
	}

	void LineString::print() const {
		std::cout << "LineString(";
		for (size_t i = 0; i < points.size(); ++i) {
			if (i != 0)
				std::cout << ", ";
			std::cout << points[i].getX() << " " << points[i].getY();
		}
		std::cout << ")";
	}

	/*
	 * Polygon
	 */
	double Polygon::distance(const Polygon* polygon) const {
		return std::min(exteriorRing.distance(polygon),
			polygon->getExteriorRing().distance(this));
	}

	bool Polygon::intersects(const Envelope& rect) const {
		// TODO
		return true;
	}

	void Polygon::draw() const { exteriorRing.draw(); }

	void Polygon::print() const {
		std::cout << "Polygon(";
		for (size_t i = 0; i < exteriorRing.numPoints(); ++i) {
			if (i != 0)
				std::cout << ", ";
			Point p = exteriorRing.getPointN(i);
			std::cout << p.getX() << " " << p.getY();
		}
		std::cout << ")";
	}

} // namespace hw6
