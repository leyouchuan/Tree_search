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

	double LineString::distance(const LineString* line) const {
		// TODO
		return NOT_IMPLEMENT;
	}

	double LineString::distance(const Polygon* polygon) const {
		// TODO
		return NOT_IMPLEMENT;
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
