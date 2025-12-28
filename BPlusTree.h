#ifndef BPLUS_H_INCLUDED
#define BPLUS_H_INCLUDED

#include "Geometry.h"
#include "Tree.h"
#include "Hilbert.h"

#include <vector>
#include <cstdint>
#include <memory>
#include <algorithm>

#include "CMakeIn.h"

namespace hw6 {

    // B+Tree node (raw pointers)
    struct BPlusNode {
        bool isLeaf = false;

        // For internal: keys[i] is separator; children.size() == keys.size()+1 usually
        std::vector<uint64_t> keys;

        // For internal nodes
        std::vector<BPlusNode*> children;

        // For leaf nodes: entries sorted by h
        struct Entry {
            uint64_t h = 0;
            size_t fid = SIZE_MAX;
            int gx = -1;
            int gy = -1;
        };

        std::vector<Entry> entries;

        // Links/parent
        BPlusNode* parent = nullptr;
        BPlusNode* next = nullptr;

        explicit BPlusNode(bool leaf = false) : isLeaf(leaf) {}

        size_t size() const {
            return isLeaf ? entries.size() : children.size();
        }
        uint64_t h_min;
        uint64_t h_max;
    };

    class BPlusTree : public Tree {
    private:
        BPlusNode* root = nullptr;
        const std::vector<Feature>* srcFeatures = nullptr;
        int hilbertOrder = 8;     // bits per dimension
        size_t maxEntries = 8;     // leaf capacity
        size_t minEntries = 4;     // optional; not used in bulk-load

        void destroySubtree(BPlusNode* node);

        // Bulk-load helpers
        BPlusNode* buildLeaves(const std::vector<BPlusNode::Entry>& sorted);
        BPlusNode* buildUpperLevel(const std::vector<BPlusNode*>& level);

        BPlusNode* findLeafByHilbert(uint64_t h) const;
        uint64_t h_min;
        uint64_t h_max;

    public:
        BPlusTree(size_t capacity = 8, int hilbertOrderBits = 8);
        ~BPlusTree();
        void setCapacity(int capacity) override {
			// DO NOTHING, since capacity is immutable in R tree
		}
        // (3.1) Hilbert mapping
        uint64_t getHilbertValue(double x, double y) const;

        int getHilbertOrder() const { return hilbertOrder; }


        // (3.2) Build tree by H (bulk-load)
        bool constructTree(const std::vector<Feature>& features) override;

        // Queries / stats
        void rangeQuery(const Envelope& rect, std::vector<Feature>& features) override;

        bool NNQuery(double x, double y, std::vector<Feature>& features) override;

        void countNode(int& interiorNum, int& leafNum);
        void countHeight(int& height);

        void draw() override;
        Envelope getBBox() const { return this->bbox; }
    public:
        static void test(int t);
        static void analyse();
    };

} // namespace hw6

#endif // BPLUS_H_INCLUDED