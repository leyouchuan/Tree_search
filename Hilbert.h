#ifndef HW6_HILBERT_H
#define HW6_HILBERT_H


#include "Tree.h"
#include <cstdint>


namespace hw6 {

    class Hilbert {
    private:
        int order_;               // bits per dimension
        uint64_t n_;              // grid size = 1 << order_
        Envelope bbox_;         
        uint64_t xy2d(uint32_t xi, uint32_t yi) const;
        // helper for bit manip operations (rotate/flip)
        static void rot(uint32_t n, uint32_t& x, uint32_t& y, uint32_t rx, uint32_t ry);
    public:
        // order: bits per dimension (typical 16). Must satisfy (2*order) <= 64.
        Hilbert(int order = 16);
        Hilbert(int order, const Envelope& bbox);

        void setOrder(int order);            // reset order (and grid)
        int getOrder() const;

        void setBBox(const Envelope& bbox);
        const Envelope& getBBox() const;

        uint64_t xyToHilbertIndex(uint32_t xi, uint32_t yi) const;
        // normalize world coords (x,y) to grid and compute index
        uint64_t pointToHilbert(double x, double y) const;

        // normalize a point to integer grid coords [0, gridSize-1]
        void normalizeToGrid(double x, double y, uint32_t& xi, uint32_t& yi) const;

        // return grid size = 2^order
        uint64_t gridSize() const;
    };

} // namespace hw6

#endif // HW6_HILBERT_H