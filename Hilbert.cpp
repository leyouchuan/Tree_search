#include "Hilbert.h"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace hw6 {

    Hilbert::Hilbert(int order) : order_(order), n_(0), bbox_(Envelope(0, 1, 0, 1)) {
        if (order_ <= 0 || order_ > 32) throw std::invalid_argument("Hilbert order must be in (0,32]");
        n_ = (1ULL << order_);
    }

    Hilbert::Hilbert(int order, const Envelope& bbox) : order_(order), n_(0), bbox_(bbox) {
        if (order_ <= 0 || order_ > 32) throw std::invalid_argument("Hilbert order must be in (0,32]");
        n_ = (1ULL << order_);
    }

    void Hilbert::setOrder(int order) {
        if (order <= 0 || order > 32) throw std::invalid_argument("Hilbert order must be in (0,32]");
        order_ = order;
        n_ = (1ULL << order_);
    }

    int Hilbert::getOrder() const { return order_; }

    void Hilbert::setBBox(const Envelope& bbox) { bbox_ = bbox; }
    const Envelope& Hilbert::getBBox() const { return bbox_; }

    uint64_t Hilbert::gridSize() const
    { return n_; }
    //将给定点 (x,y) 按 bbox 归一化并映射到 Hilbert 网格坐标 (xi, yi)
    /*void Hilbert::normalizeToGrid(double x, double y, uint32_t& xi, uint32_t& yi) const {
        double xmin = bbox_.getMinX();
        double xmax = bbox_.getMaxX();
        double ymin = bbox_.getMinY();
        double ymax = bbox_.getMaxY();

        // protect degenerate bbox
        double dx = xmax - xmin;
        double dy = ymax - ymin;
        if (dx <= 0) dx = 1e-9;
        if (dy <= 0) dy = 1e-9;

        double rx = (x - xmin) / dx;
        double ry = (y - ymin) / dy;

        if (rx < 0) rx = 0; if (rx > 1) rx = 1;
        if (ry < 0) ry = 0; if (ry > 1) ry = 1;

        // map to [0, n_-1], careful with rounding: clamp to n_-1
        double gx = rx * (static_cast<double>(n_) - 1.0);
        double gy = ry * (static_cast<double>(n_) - 1.0);

        xi = static_cast<uint32_t>(std::floor(gx));
        yi = static_cast<uint32_t>(std::floor(gy));

        if (xi >= n_) xi = static_cast<uint32_t>(n_ - 1);
        if (yi >= n_) yi = static_cast<uint32_t>(n_ - 1);
    }*/
    void Hilbert::normalizeToGrid(double x, double y, uint32_t& xi, uint32_t& yi) const {
        const Envelope& bb = bbox_;
        double xmin = bb.getMinX(), ymin = bb.getMinY();
        double xmax = bb.getMaxX(), ymax = bb.getMaxY();
        double dx = xmax - xmin; if (dx <= 0) dx = 1e-9;
        double dy = ymax - ymin; if (dy <= 0) dy = 1e-9;
        double rx = (x - xmin) / dx;
        double ry = (y - ymin) / dy;
        if (rx < 0) rx = 0; if (rx > 1) rx = 1;
        if (ry < 0) ry = 0; if (ry > 1) ry = 1;

        uint32_t n = getN(); // n = 1<<order_
        // map to [0, n-1] using floor(rx * n) but clamp to n-1
        double gx = rx * static_cast<double>(n);
        double gy = ry * static_cast<double>(n);
        uint32_t tx = static_cast<uint32_t>(std::floor(gx));
        uint32_t ty = static_cast<uint32_t>(std::floor(gy));
        if (tx >= n) tx = n - 1;
        if (ty >= n) ty = n - 1;
        xi = tx; yi = ty;
    }

    uint64_t Hilbert::pointToHilbert(double x, double y) const {
        uint32_t xi = 0, yi = 0;
        normalizeToGrid(x, y, xi, yi);
        return xyToHilbertIndex(xi, yi);
    }

    uint64_t Hilbert::xy2d(uint32_t xi, uint32_t yi) const {
        uint64_t d = 0;
        uint32_t x = xi;
        uint32_t y = yi;
        // iterate from highest bit to lowest
        for (int s = order_ - 1; s >= 0; --s) {
            uint32_t rx = (x >> s) & 1U;
            uint32_t ry = (y >> s) & 1U;
            uint32_t quad = (rx << 1) | ry; // 2*rx + ry
            d <<= 2;
            d |= quad;
            // rotate/flip
            rot(static_cast<uint32_t>(1U << s), x, y, rx, ry);
        }
        // The above builds bits in order of descending significance; however
        // many implementations produce a different bit order. The produced value
        // here yields a Hilbert-like interleaving with rotations applied.
        return d;
    }

    // rot function: rotate/flip a quadrant appropriately.
    // This is the standard helper for Hilbert transformations.
    //在逐位处理 Hilbert 编码时调整坐标以准备下一层位的处理
    void Hilbert::rot(uint32_t n, uint32_t& x, uint32_t& y, uint32_t rx, uint32_t ry) {
        if (ry == 0) {
            if (rx == 1) {
                // reflect
                x = (n - 1) - x;
                y = (n - 1) - y;
            }
            // swap x and y
            uint32_t t = x;
            x = y;
            y = t;
        }
    }
    //对离散网格坐标 (xi, yi) 做边界检查/钳制后，调用 xy2d 将其编码为 uint64_t 的 Hilbert 索引，返回该索引
    uint64_t Hilbert::xyToHilbertIndex(uint32_t xi, uint32_t yi) const {
        // validate input: xi, yi < n_
        if (xi >= n_ || yi >= n_) {
            // clamp (defensive)
            xi = std::min<uint32_t>(xi, static_cast<uint32_t>(n_ - 1));
            yi = std::min<uint32_t>(yi, static_cast<uint32_t>(n_ - 1));
        }

        // Use the mapping via xy2d.
        // Note: Different conventions exist; this produces a consistent mapping
        // suitable for ordering points along a Hilbert-like curve.
        return xy2d(xi, yi);
    }
} // namespace hw6