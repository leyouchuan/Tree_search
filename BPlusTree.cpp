#include "BPlusTree.h"
#include "Hilbert.h"
#include"Geometry.h"
#include <algorithm>
#include <queue>
#include <cassert>
#include <cmath>
#include <iostream>
#include<functional>

#include "BPlusTree.h"
#include "Hilbert.h"   // uses your project Hilbert implementation

#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

/*namespace hw6 {

    static inline double clamp01(double v) {
        if (v < 0.0) return 0.0;
        if (v > 1.0) return 1.0;
        return v;
    }

    BPlusTree::BPlusTree(size_t capacity, int hilbertOrderBits)
        : Tree(capacity), root(nullptr), hilbertOrder(hilbertOrderBits),
        maxEntries(capacity), minEntries(capacity / 2) {
        if (maxEntries < 2) maxEntries = 2;
        if (hilbertOrder < 1) hilbertOrder = 1;
        // bbox will be set in constructTree based on data
    }

    BPlusTree::~BPlusTree() {
        destroySubtree(root);
        root = nullptr;
    }

    void BPlusTree::destroySubtree(BPlusNode* node) {
        if (!node) return;
        if (!node->isLeaf) {
            for (auto* c : node->children) destroySubtree(c);
        }
        delete node;
    }

    uint64_t BPlusTree::getHilbertValue(double x, double y) const {
        // validate order
        if (hilbertOrder <= 0 || hilbertOrder > 32) throw std::invalid_argument("hilbertOrder must be in [1,32]");

        const double minX = bbox.getMinX();
        const double maxX = bbox.getMaxX();
        const double minY = bbox.getMinY();
        const double maxY = bbox.getMaxY();

        double w = (maxX - minX);
        double h = (maxY - minY);

        double nx = (w > 0) ? (x - minX) / w : 0.0;
        double ny = (h > 0) ? (y - minY) / h : 0.0;
        nx = clamp01(nx);
        ny = clamp01(ny);

        const uint64_t gridMax = ((uint64_t(1) << hilbertOrder) - 1); // safe for hilbertOrder <= 32

        // Map to integer grid using floor to avoid rounding inconsistencies:
        uint32_t ix = static_cast<uint32_t>(std::min<uint64_t>(gridMax, static_cast<uint64_t>(std::floor(nx * (gridMax + 1.0)))));
        uint32_t iy = static_cast<uint32_t>(std::min<uint64_t>(gridMax, static_cast<uint64_t>(std::floor(ny * (gridMax + 1.0)))));

        Hilbert hilb(hilbertOrder, bbox);
        // Directly use discrete mapping exposed by Hilbert
        return hilb.xyToHilbertIndex(ix, iy);
    }

    // helper: build leaf-level as a linked list of leaves, return root of leaf level (first leaf)
    /*hw6::BPlusNode* hw6::BPlusTree::buildLeaves(const std::vector<BPlusNode::Entry>& sorted) {
        if (sorted.empty()) return nullptr;

        BPlusNode* first = nullptr;
        BPlusNode* prev = nullptr;
        size_t n = sorted.size();
        size_t idx = 0;

        while (idx < n) {
            BPlusNode* leaf = new BPlusNode(true); // isLeaf = true
            // fill up to maxEntries entries
            for (size_t k = 0; k < maxEntries && idx < n; ++k, ++idx) {
                leaf->entries.push_back(sorted[idx]);
            }
            // link list
            if (!first) first = leaf;
            if (prev) prev->next = leaf;
            leaf->parent = nullptr;
            leaf->next = nullptr;
            prev = leaf;
        }
        return first;
    }*/
/*
    // helper: build one upper level given a vector of child nodes (level)
    // returns pointer to first node in new upper level (linked via children vector order)
    hw6::BPlusNode* hw6::BPlusTree::buildUpperLevel(const std::vector<BPlusNode*>& level) {
        if (level.empty()) return nullptr;
        std::vector<BPlusNode*> parents;
        size_t idx = 0;
        while (idx < level.size()) {
            BPlusNode* parent = new BPlusNode(false); // internal
            parent->children.clear();
            parent->keys.clear();
            parent->parent = nullptr;
            parent->next = nullptr;

            // pack up to maxEntries children into this parent (note: internal node children count <= maxEntries)
            // keys count = children.size() - 1
            size_t cnt = 0;
            while (cnt < maxEntries && idx < level.size()) {
                BPlusNode* child = level[idx++];
                parent->children.push_back(child);
                child->parent = parent;
                ++cnt;
            }
            // build separator keys: for simplicity use first key/h of each child (except first) as separator
            parent->keys.reserve((parent->children.size() >= 1) ? parent->children.size() - 1 : 0);
            for (size_t i = 1; i < parent->children.size(); ++i) {
                // derive separator: if child is leaf use its first entry.h else use child's first key
                uint64_t sep = 0;
                BPlusNode* c = parent->children[i];
                if (c->isLeaf) {
                    if (!c->entries.empty()) sep = c->entries.front().h;
                }
                else {
                    if (!c->keys.empty()) sep = c->keys.front();
                }
                parent->keys.push_back(sep);
            }
            parents.push_back(parent);
        }

        // convert parents vector into a singly-linked list via parent->next for compatibility
        for (size_t i = 0; i + 1 < parents.size(); ++i) parents[i]->next = parents[i + 1];
        if (!parents.empty()) parents.back()->next = nullptr;

        // return first parent node (root of this upper level)
        return parents.empty() ? nullptr : parents.front();
    }

/*    hw6::BPlusNode* hw6::BPlusTree::buildLeaves(const std::vector<BPlusNode::Entry>& sorted) {
        if (sorted.empty()) return nullptr;

        BPlusNode* first = nullptr;
        BPlusNode* prev = nullptr;
        size_t n = sorted.size();
        size_t idx = 0;

        while (idx < n) {
            BPlusNode* leaf = new BPlusNode(true);
            leaf->entries.clear();
            leaf->parent = nullptr;
            leaf->next = nullptr;

            // fill leaf with groups of same h, ensuring we don't split a group across leaves.
            while (idx < n) {
                uint64_t curH = sorted[idx].h;
                size_t j = idx + 1;
                while (j < n && sorted[j].h == curH) ++j; // [idx, j) is group

                size_t groupSize = j - idx;
                size_t remaining = (maxEntries > leaf->entries.size()) ? (maxEntries - leaf->entries.size()) : 0;

                if (groupSize <= remaining) {
                    for (size_t t = idx; t < j; ++t) leaf->entries.push_back(sorted[t]);
                    idx = j;
                }
                else {
                    if (leaf->entries.empty()) {
                        // allow overflow for an oversized group
                        for (size_t t = idx; t < j; ++t) leaf->entries.push_back(sorted[t]);
                        idx = j;
                    }
                    break;
                }
            }

            // set h_min/h_max safely
            if (!leaf->entries.empty()) {
                leaf->h_min = leaf->entries.front().h;
                leaf->h_max = leaf->entries.back().h;
            }
            else {
                leaf->h_min = UINT64_MAX;
                leaf->h_max = 0;
            }

            if (!first) first = leaf;
            if (prev) prev->next = leaf;
            prev = leaf;
        }

        return first;
    }
    // B+ 树按分隔键向下搜索叶节点
    BPlusNode* BPlusTree::findLeafByHilbert(uint64_t h) const {
        if (!root) return nullptr;

        BPlusNode* cur = root;
        while (cur && !cur->isLeaf) {
            // 二分查找
            size_t idx = std::upper_bound(cur->keys.begin(), cur->keys.end(), h) - cur->keys.begin();
            if (idx >= cur->children.size()) idx = cur->children.size() - 1;
            cur = cur->children[idx];
        }
        return cur;
    }

    BPlusNode* BPlusNode::createLeaf() {
        BPlusNode* n = new BPlusNode(true);
        n->isLeaf = true;
        n->parent = nullptr;
        n->next = nullptr;
        n->keys.clear();
        n->children.clear();
        n->entries.clear();
        n->h_min = 0;
        n->h_max = 0;
        return n;
    }

    BPlusNode* BPlusNode::createInternal() {
        BPlusNode* n = new BPlusNode(false);
        n->isLeaf = false;
        n->parent = nullptr;
        n->next = nullptr;
        n->keys.clear();
        n->children.clear();
        n->entries.clear();
        n->h_min = 0;
        n->h_max = 0;
        return n;
    }

    size_t BPlusNode::maxKeys() const {
        return 8; // 或你期望的默认值
    }
    // 插入单个条目到 B+ 树（按 hilbert 值排序位置插入）

    bool hw6::BPlusTree::insertByHilbert(uint64_t h, uint32_t fid, int gx, int gy) {
        // prepare entry
        BPlusNode::Entry ent;
        ent.h = h;
        ent.fid = fid;
        ent.gx = gx;
        ent.gy = gy;

        // empty tree -> create first leaf
        if (!root) {
            BPlusNode* leaf = BPlusNode::createLeaf();
            leaf->entries.push_back(ent);
            // safe because we just pushed one entry
            leaf->h_min = leaf->entries.front().h;
            leaf->h_max = leaf->entries.back().h;
            root = leaf;
            root->parent = nullptr;
            return true;
        }

        // 1) find leaf by traversing internal keys
        BPlusNode* node = root;
        while (!node->isLeaf) {
            size_t idx = 0;
            // find first key > h, so child index is idx
            while (idx < node->keys.size() && node->keys[idx] <= h) ++idx;
            if (node->children.empty()) {
                // defensive: should not happen
                return false;
            }
            if (idx >= node->children.size()) idx = node->children.size() - 1;
            node = node->children[idx];
        }

        // 2) insert entry into leaf in order (preserve order by h)
        auto it = std::lower_bound(node->entries.begin(), node->entries.end(), h,
            [](const BPlusNode::Entry& a, uint64_t val) { return a.h < val; });
        node->entries.insert(it, ent);

        // update leaf h range (safe because entries not empty)
        if (!node->entries.empty()) {
            node->h_min = node->entries.front().h;
            node->h_max = node->entries.back().h;
        }

        // 3) split while node overflows; use this->maxEntries as capacity
        size_t cap = this->maxEntries;
        while (true) {
            if (node->isLeaf) {
                if (node->entries.size() <= cap) break;
            }
            else {
                // internal overflow: keys.size() > cap OR children.size() > cap+1
                if (node->keys.size() <= cap && node->children.size() <= cap + 1) break;
            }

            if (node->isLeaf) {
                // split leaf
                BPlusNode* sibling = BPlusNode::createLeaf();
                sibling->parent = node->parent;

                size_t total = node->entries.size();
                size_t mid = total / 2; // move [mid, end) -> sibling
                // move entries
                sibling->entries.assign(node->entries.begin() + mid, node->entries.end());
                node->entries.erase(node->entries.begin() + mid, node->entries.end());

                // fix sibling links
                sibling->next = node->next;
                node->next = sibling;

                // update ranges (ensure non-empty)
                assert(!node->entries.empty());
                assert(!sibling->entries.empty());
                node->h_min = node->entries.front().h;
                node->h_max = node->entries.back().h;
                sibling->h_min = sibling->entries.front().h;
                sibling->h_max = sibling->entries.back().h;

                uint64_t promoteKey = sibling->entries.front().h;

                if (!node->parent) {
                    // make new root (internal)
                    BPlusNode* newRoot = BPlusNode::createInternal();
                    newRoot->children.clear();
                    newRoot->keys.clear();
                    newRoot->children.push_back(node);
                    newRoot->children.push_back(sibling);
                    newRoot->keys.push_back(promoteKey);
                    node->parent = newRoot;
                    sibling->parent = newRoot;
                    root = newRoot;
                    root->parent = nullptr;
                    return true;
                }
                else {
                    // insert sibling into parent
                    BPlusNode* p = node->parent;
                    // find position of node in parent's children
                    size_t pos = 0;
                    while (pos < p->children.size() && p->children[pos] != node) ++pos;
                    // node must be found in parent's children
                    assert(pos < p->children.size());

                    p->children.insert(p->children.begin() + pos + 1, sibling);
                    p->keys.insert(p->keys.begin() + pos, promoteKey);
                    sibling->parent = p;

                    // continue upward check on parent
                    node = p;
                    continue;
                }
            }
            else {
                // split internal node
                BPlusNode* sibling = BPlusNode::createInternal();
                sibling->parent = node->parent;

                // children.size() == keys.size() + 1 expected
                size_t totalChildren = node->children.size();
                assert(totalChildren >= 2);
                size_t midChild = totalChildren / 2; // move children [midChild, end) to sibling
                size_t promoteKeyIndex = midChild - 1; // index of key to promote; valid because totalChildren>=2

                // capture promote key before erasing
                assert(promoteKeyIndex < node->keys.size());
                uint64_t promoteKey = node->keys[promoteKeyIndex];

                // move children
                sibling->children.assign(node->children.begin() + midChild, node->children.end());
                node->children.erase(node->children.begin() + midChild, node->children.end());

                // move keys: keys after promoteKeyIndex move to sibling
                sibling->keys.assign(node->keys.begin() + promoteKeyIndex + 1, node->keys.end());
                // erase promoteKey and keys to its right from node (left keeps keys [0 .. promoteKeyIndex-1])
                node->keys.erase(node->keys.begin() + promoteKeyIndex, node->keys.end());

                // attach parent pointers for moved children
                for (BPlusNode* c : sibling->children) {
                    if (c) c->parent = sibling;
                }

                // update approximate ranges from child extremes (guard non-empty)
                if (!node->children.empty() && node->children.front() && node->children.back()) {
                    node->h_min = node->children.front()->h_min;
                    node->h_max = node->children.back()->h_max;
                }
                else {
                    node->h_min = node->h_max = 0;
                }
                if (!sibling->children.empty() && sibling->children.front() && sibling->children.back()) {
                    sibling->h_min = sibling->children.front()->h_min;
                    sibling->h_max = sibling->children.back()->h_max;
                }
                else {
                    sibling->h_min = sibling->h_max = 0;
                }

                if (!node->parent) {
                    // new root
                    BPlusNode* newRoot = BPlusNode::createInternal();
                    newRoot->children.clear();
                    newRoot->keys.clear();
                    newRoot->children.push_back(node);
                    newRoot->children.push_back(sibling);
                    newRoot->keys.push_back(promoteKey);
                    node->parent = newRoot;
                    sibling->parent = newRoot;
                    root = newRoot;
                    root->parent = nullptr;
                    return true;
                }
                else {
                    BPlusNode* p = node->parent;
                    // find position of node in parent->children
                    size_t pos = 0;
                    while (pos < p->children.size() && p->children[pos] != node) ++pos;
                    assert(pos < p->children.size());

                    p->children.insert(p->children.begin() + pos + 1, sibling);
                    p->keys.insert(p->keys.begin() + pos, promoteKey);
                    sibling->parent = p;

                    node = p;
                    continue;
                }
            }
        } // end while overflow

        return true;
    }

    /*bool BPlusTree::constructTree(const std::vector<Feature>& features) {
        if (features.empty()) return false;

        // compute global bbox
        bbox = features[0].getEnvelope();
        for (size_t i = 1; i < features.size(); ++i) {
            bbox = bbox.unionEnvelope(features[i].getEnvelope());
        }

        // prepare Hilbert helper (uses your Hilbert class)
        Hilbert hil(this->getHilbertOrder());
        hil.setBBox(bbox);
        const uint64_t G = hil.gridSize(); // = 1 << ORDER
        const double gridWidth = bbox.getWidth() / static_cast<double>(G);
        const double gridHeight = bbox.getHeight() / static_cast<double>(G);
        const double originX = bbox.getMinX();
        const double originY = bbox.getMinY();

        for (size_t fid = 0; fid < features.size(); ++fid) {
            const Feature& feat = features[fid];
            Envelope env = feat.getEnvelope();

            // compute grid index bounds covered by this feature, clipped to [0, G-1]
            auto clampIdx = [&](int64_t v)->size_t {
                if (v < 0) return 0u;
                if (v >= static_cast<int64_t>(G)) return static_cast<size_t>(G - 1);
                return static_cast<size_t>(v);
                };

            int64_t ix0 = static_cast<int64_t>(std::floor((env.getMinX() - originX) / gridWidth));
            int64_t iy0 = static_cast<int64_t>(std::floor((env.getMinY() - originY) / gridHeight));
            int64_t ix1 = static_cast<int64_t>(std::ceil((env.getMaxX() - originX) / gridWidth)) - 1;
            int64_t iy1 = static_cast<int64_t>(std::ceil((env.getMaxY() - originY) / gridHeight)) - 1;

            size_t sx = clampIdx(ix0);
            size_t sy = clampIdx(iy0);
            size_t ex = clampIdx(ix1);
            size_t ey = clampIdx(iy1);

            for (size_t gx = sx; gx <= ex; ++gx) {
                double cellMinX = originX + static_cast<double>(gx) * gridWidth;
                double cellMaxX = cellMinX + gridWidth;
                for (size_t gy = sy; gy <= ey; ++gy) {
                    double cellMinY = originY + static_cast<double>(gy) * gridHeight;
                    double cellMaxY = cellMinY + gridHeight;

                    Envelope gridBox(cellMinX, cellMaxX, cellMinY, cellMaxY);
                    if (!env.intersect(gridBox)) continue;

                    // compute Hilbert index using your Hilbert class
                    uint32_t xi = static_cast<uint32_t>(gx);
                    uint32_t yi = static_cast<uint32_t>(gy);
                    uint64_t h = hil.xyToHilbertIndex(xi, yi);

                    // insert using your available API
                    insertByHilbert(h, static_cast<uint32_t>(fid), static_cast<int>(gx), static_cast<int>(gy));
                }
            }
        }

        // finalize
        if (root) root->parent = nullptr;
        srcFeatures = &features;
        return true;
    }*/

    /*// main: bulk-load constructTree (single-h per feature using envelope center)
    bool hw6::BPlusTree::constructTree(const std::vector<Feature>& features) {
        if (features.empty()) return false;
        this->srcFeatures = &features;

        // compute global bbox
        bbox = features[0].getEnvelope();
        for (size_t i = 1; i < features.size(); ++i)
            bbox = bbox.unionEnvelope(features[i].getEnvelope());

        const int ORDER = getHilbertOrder();
        const size_t G = (1ULL << ORDER);
        double gridWidth = bbox.getWidth() / static_cast<double>(G);
        double gridHeight = bbox.getHeight() / static_cast<double>(G);
        double originX = bbox.getMinX();
        double originY = bbox.getMinY();

        std::vector<BPlusNode::Entry> all;
        all.reserve(features.size());
        Hilbert hil(ORDER, bbox);

        auto floorIndex = [&](double x, double origin, double cellSize)->int64_t {
            double rel = (x - origin) / cellSize;
            int64_t idx = static_cast<int64_t>(std::floor(rel));
            if (idx < 0) idx = 0;
            if (idx >= static_cast<int64_t>(G)) idx = static_cast<int64_t>(G) - 1;
            return idx;
            };
        auto ceilIndexMinusOne = [&](double x, double origin, double cellSize)->int64_t {
            double rel = (x - origin) / cellSize;
            int64_t idx = static_cast<int64_t>(std::ceil(rel)) - 1;
            if (idx < 0) idx = 0;
            if (idx >= static_cast<int64_t>(G)) idx = static_cast<int64_t>(G) - 1;
            return idx;
            };

        for (size_t fid = 0; fid < features.size(); ++fid) {
            const Feature& f = features[fid];
            Envelope e = f.getEnvelope();

            int64_t xi0 = floorIndex(e.getMinX(), originX, gridWidth);
            int64_t yi0 = floorIndex(e.getMinY(), originY, gridHeight);
            int64_t xi1 = ceilIndexMinusOne(e.getMaxX(), originX, gridWidth);
            int64_t yi1 = ceilIndexMinusOne(e.getMaxY(), originY, gridHeight);

            if (xi0 > xi1) std::swap(xi0, xi1);
            if (yi0 > yi1) std::swap(yi0, yi1);

            for (int64_t ix = xi0; ix <= xi1; ++ix) {
                for (int64_t iy = yi0; iy <= yi1; ++iy) {
                    double cellMinX = originX + ix * gridWidth;
                    double cellMaxX = originX + (ix + 1) * gridWidth;
                    double cellMinY = originY + iy * gridHeight;
                    double cellMaxY = originY + (iy + 1) * gridHeight;
                    Envelope cellEnv(cellMinX, cellMaxX, cellMinY, cellMaxY);
                    if (!e.intersect(cellEnv)) continue;

                    uint64_t h = hil.xyToHilbertIndex(static_cast<uint32_t>(ix), static_cast<uint32_t>(iy));

                    BPlusNode::Entry ent;
                    ent.h = h;
                    ent.fid = static_cast<uint32_t>(fid);
                    ent.gx = static_cast<int>(ix);
                    ent.gy = static_cast<int>(iy);
                    all.push_back(ent);
                }
            }
        }

        if (all.empty()) {
            root = nullptr;
            return true;
        }

        std::sort(all.begin(), all.end(), [](const BPlusNode::Entry& a, const BPlusNode::Entry& b) {
            return a.h < b.h;
            });

        BPlusNode* firstLeaf = buildLeaves(all);
        if (!firstLeaf) {
            root = nullptr;
            return true;
        }

        std::vector<BPlusNode*> level;
        for (BPlusNode* p = firstLeaf; p != nullptr; p = p->next) level.push_back(p);

        while (level.size() > 1) {
            BPlusNode* firstParent = buildUpperLevel(level);
            level.clear();
            for (BPlusNode* p = firstParent; p != nullptr; p = p->next) level.push_back(p);
        }

        root = level.empty() ? firstLeaf : level.front();
        if (root) root->parent = nullptr;
        return true;
    }

    /*bool hw6::BPlusTree::constructTree(const std::vector<Feature>& features) {
        if (features.empty()) return false;

        // compute global bbox
        bbox = features[0].getEnvelope();
        for (size_t i = 1; i < features.size(); ++i)
            bbox = bbox.unionEnvelope(features[i].getEnvelope());

        const int ORDER = getHilbertOrder();
        const uint32_t G = (1U << ORDER);
        const double gridWidth = bbox.getWidth() / static_cast<double>(G);
        const double gridHeight = bbox.getHeight() / static_cast<double>(G);
        const double originX = bbox.getMinX();
        const double originY = bbox.getMinY();

        Hilbert hil(ORDER, bbox);

        std::vector<BPlusNode::Entry> all;
        all.reserve(features.size() * 2); // 预留一些空间

        // helper: map coordinate to grid index (floor mapping), clamped to [0, G-1]
        auto coordToIndex = [&](double x, double origin, double cellSize)->uint32_t {
            if (x <= origin) return 0;
            double rel = (x - origin) / cellSize;
            uint32_t idx = static_cast<uint32_t>(std::floor(rel));
            if (idx >= G) idx = G - 1;
            return idx;
            };

        for (size_t t = 0; t < features.size(); ++t) {
            const Feature& f = features[t];
            Envelope e = f.getEnvelope();

            // compute covered grid index range
            uint32_t xi0 = coordToIndex(e.getMinX(), originX, gridWidth);
            uint32_t yi0 = coordToIndex(e.getMinY(), originY, gridHeight);
            uint32_t xi1 = coordToIndex(e.getMaxX(), originX, gridWidth);
            uint32_t yi1 = coordToIndex(e.getMaxY(), originY, gridHeight);

            if (xi0 > xi1) std::swap(xi0, xi1);
            if (yi0 > yi1) std::swap(yi0, yi1);

            // iterate only the covered cells
            for (uint32_t i = xi0; i <= xi1; ++i) {
                for (uint32_t j = yi0; j <= yi1; ++j) {
                    // optional fine check: keep to avoid false positives at boundaries
                    Envelope gridBox(originX + i * gridWidth,
                        originX + (i + 1) * gridWidth,
                        originY + j * gridHeight,
                        originY + (j + 1) * gridHeight);
                    if (!f.getEnvelope().intersect(gridBox)) continue;

                    // create entry with this grid cell's hilbert value
                    uint32_t xi = i, yi = j;
                    uint64_t h = hil.xyToHilbertIndex(xi, yi);

                    BPlusNode::Entry ent;
                    ent.h = h;
                    ent.feature = f;               // 保持原语义：Entry 持有 Feature
                    ent.gx = static_cast<int>(xi);
                    ent.gy = static_cast<int>(yi);
                    all.push_back(std::move(ent));
                }
            }
        }

        if (all.empty()) {
            // no entries (possible if all envelopes degenerate outside bbox)
            root = nullptr;
            return true;
        }

        // sort by Hilbert value (stable order not strictly required but keep it)
        std::sort(all.begin(), all.end(), [](const BPlusNode::Entry& a, const BPlusNode::Entry& b) {
            return a.h < b.h;
            });

        // build leaf level
        BPlusNode* firstLeaf = buildLeaves(all);
        if (!firstLeaf) {
            root = nullptr;
            return true;
        }

        // collect leaves in order
        std::vector<BPlusNode*> level;
        for (BPlusNode* p = firstLeaf; p != nullptr; p = p->next) level.push_back(p);

        // iteratively build upper levels
        while (level.size() > 1) {
            BPlusNode* firstParent = buildUpperLevel(level);
            level.clear();
            for (BPlusNode* p = firstParent; p != nullptr; p = p->next) level.push_back(p);
        }

        root = level.empty() ? firstLeaf : level.front();
        if (root) root->parent = nullptr;
        return true;
    }*/

    /*void BPlusTree::rangeQuery(const Envelope& rect, std::vector<Feature>& result) {
        result.clear();
        if (!root) return;

        // 上层将 rect 的两角映射到 Hilbert 值区间
        uint64_t h1 = getHilbertValue(rect.getMinX(), rect.getMinY());
        uint64_t h2 = getHilbertValue(rect.getMaxX(), rect.getMaxY());
        uint64_t hmin = std::min(h1, h2);
        uint64_t hmax = std::max(h1, h2);

        // 从包含 hmin 的叶开始（findLeafByHilbert 已由你实现）
        BPlusNode* leaf = findLeafByHilbert(hmin);
        if (!leaf) return;

        // 扫描叶链（按 Hilbert 顺序），遇到第一个叶内第一个 entry.h > hmax 则结束
        for (BPlusNode* p = leaf; p != nullptr; p = p->next) {
            // 叶为空则跳过
            if (p->entries.empty()) continue;

            // 若叶中最小 h 已大于 hmax，可提前停止
            if (p->entries.front().h > hmax) break;

            // 线性扫描叶内条目（假定 entries 按 h 升序）
            for (size_t i = 0; i < p->entries.size(); ++i) {
                uint64_t h = p->entries[i].h;
                if (h < hmin) continue;
                if (h > hmax) break; // 当前叶后续也更大，跳到下一叶
                // 便宜的包围盒过滤，符合“返回候选集”的要求
                if (p->entries[i].feature.getGeom() &&
                    p->entries[i].feature.getGeom()->getEnvelope().intersect(rect)) {
                    result.push_back(p->entries[i].feature);
                }
                else {
                    // 如果你希望仅基于 Hilbert 返回候选集（不做任何几何过滤），可取消上面判断并直接 push
                    // features.push_back(p->entries[i].feature);
                }
            }
        }
    }*/

    /*void hw6::BPlusTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
        features.clear();
        if (!root) return;

        const int ORDER = getHilbertOrder();
        const uint32_t G = (1U << ORDER);
        const double gridWidth = bbox.getWidth() / static_cast<double>(G);
        const double gridHeight = bbox.getHeight() / static_cast<double>(G);
        const double originX = bbox.getMinX();
        const double originY = bbox.getMinY();
        Hilbert hil(ORDER, bbox);

        auto floorIndex = [&](double x, double origin, double cellSize)->uint32_t {
            if (x <= origin) return 0;
            double rel = (x - origin) / cellSize;
            int64_t idx = static_cast<int64_t>(std::floor(rel));
            if (idx < 0) idx = 0;
            if (idx >= static_cast<int64_t>(G)) idx = static_cast<int64_t>(G) - 1;
            return static_cast<uint32_t>(idx);
            };
        auto ceilIndexMinusOne = [&](double x, double origin, double cellSize)->uint32_t {
            if (x <= origin) return 0;
            double rel = (x - origin) / cellSize;
            int64_t idx = static_cast<int64_t>(std::ceil(rel)) - 1;
            if (idx < 0) idx = 0;
            if (idx >= static_cast<int64_t>(G)) idx = static_cast<int64_t>(G) - 1;
            return static_cast<uint32_t>(idx);
            };

        uint32_t xi0 = floorIndex(rect.getMinX(), originX, gridWidth);
        uint32_t yi0 = floorIndex(rect.getMinY(), originY, gridHeight);
        uint32_t xi1 = ceilIndexMinusOne(rect.getMaxX(), originX, gridWidth);
        uint32_t yi1 = ceilIndexMinusOne(rect.getMaxY(), originY, gridHeight);

        if (xi0 > xi1) std::swap(xi0, xi1);
        if (yi0 > yi1) std::swap(yi0, yi1);

        std::vector<std::pair<uint32_t, uint32_t>> cells;
        for (uint32_t i = xi0; i <= xi1; ++i)
            for (uint32_t j = yi0; j <= yi1; ++j)
                cells.emplace_back(i, j);

        std::sort(cells.begin(), cells.end());
        cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
        if (cells.empty()) return;

        features.reserve(256);
        std::unordered_set<size_t> seen;

        for (auto [ci, cj] : cells) {
            uint64_t h = hil.xyToHilbertIndex(ci, cj);
            BPlusNode* leaf = findLeafByHilbert(h);
            if (!leaf) continue;

            // scan this leaf and successive right leaves while their h_min <= h <= h_max
            for (BPlusNode* L = leaf; L != nullptr; L = L->next) {
                if (L->entries.empty()) break;
                if (L->h_min > h) break;            // right leaf min already greater -> stop
                if (L->h_max < h) continue;         // this leaf's max < h -> skip

                for (const auto& ent : L->entries) {
                    if (ent.h != h) continue;
                    if (static_cast<uint32_t>(ent.gx) != ci || static_cast<uint32_t>(ent.gy) != cj) continue;
                    size_t fid = ent.fid;
                    if (!srcFeatures) continue;
                    const auto& allFeatures = *srcFeatures;
                    if (fid >= allFeatures.size()) continue;
                    const Feature& feat = allFeatures[fid];
                    if (!feat.getEnvelope().intersect(rect)) continue;
                    if (seen.insert(fid).second) features.push_back(feat);
                }
            }
        }
    }

    bool BPlusTree::NNQuery(double x, double y, std::vector<Feature>& features) {
        features.clear();
        if (!root) return false;

        uint64_t hq = getHilbertValue(x, y);
        BPlusNode* leaf = findLeafByHilbert(hq);
        if (!leaf) return false;

        const size_t maxCandidates = std::max<size_t>(64, maxEntries * 8);
        features.reserve(maxCandidates);
        for (BPlusNode* p = leaf; p != nullptr && features.size() < maxCandidates; p = p->next) {
            for (const auto& ent : p->entries) {
                size_t fid = ent.fid;
                // 边界检查（可选但推荐）
                //if (fid == SIZE_MAX || fid >= (features.size()) continue;
                const Feature& feat = (*srcFeatures)[fid];
                features.push_back(feat);               // 拷贝 Feature；若需指针，可 push_back(&feat)
                if (features.size() >= maxCandidates) break;
            }
        }

        // fallback: if not enough, scan from leftmost leaf
        if (features.size() < maxCandidates) {
            BPlusNode* cur = root;
            while (cur && !cur->isLeaf) cur = cur->children.front();
            for (BPlusNode* p = cur; p != nullptr && features.size() < maxCandidates; p = p->next) {
                for (const auto& ent : p->entries) {
                    size_t fid = ent.fid;
                    //if (fid == SIZE_MAX || fid >= features.size()) continue;
                    const Feature& feat = (*srcFeatures)[fid];
                    features.push_back(feat);
                    if (features.size() >= maxCandidates) break;
                }
            }
        }
        return !features.empty();
    }

    void BPlusTree::countNode(int& interiorNum, int& leafNum) {
        interiorNum = 0;
        leafNum = 0;
        if (!root) return;

        std::queue<BPlusNode*> q;
        q.push(root);
        while (!q.empty()) {
            BPlusNode* n = q.front();
            q.pop();
            if (n->isLeaf) {
                ++leafNum;
            }
            else {
                ++interiorNum;
                for (auto* c : n->children) q.push(c);
            }
        }
    }

    void BPlusTree::countHeight(int& height) {
        height = 0;
        if (!root) return;

        BPlusNode* cur = root;
        while (cur) {
            ++height;
            if (cur->isLeaf) break;
            cur = cur->children.empty() ? nullptr : cur->children.front();
        }
    }
    
    void BPlusTree::draw() {
        double gridHeight = bbox.getHeight() / (pow(2, 4));
        double gridWidth = bbox.getWidth() / (pow(2, 4));
        double originX = bbox.getMinX();
        double originY = bbox.getMinY();
        for (size_t i = 0; i < (pow(2,6)); i++) {
            for (size_t j = 0; j < (pow(2, 6)); j++) {
                Envelope drawBox(originX + i * gridWidth, originX + (i + 1) * gridWidth, originY + j * gridHeight, originY + (j + 1) * gridHeight);
                drawBox.draw();
            }
        }
    }
    /*void BPlusTree::draw() {
        if (!root) return;

        std::function<void(BPlusNode*)> walk = [&](BPlusNode* node) {
            if (!node) return;
            if (node->isLeaf) {
                // compute leaf bbox from entries (if entries empty skip)
                if (!node->entries.empty()) {
                    Envelope e = node->entries.front().feature.getGeom()->getEnvelope();
                    for (size_t i = 1; i < node->entries.size(); ++i)
                        e = e.unionEnvelope(node->entries[i].feature.getGeom()->getEnvelope());
                    e.draw();
                }
            }
            else {
                for (BPlusNode* c : node->children) walk(c);
            }
            };

        walk(root);
    }*/
/*}*/