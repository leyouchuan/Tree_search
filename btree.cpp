#include "btree.h"
#include "Hilbert.h"
#include"Geometry.h"

#include "Common.h"
#include <ctime>
#include "CMakeIn.h"

#include <unordered_set>
#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include<functional>


namespace hw6 {

    // ============================================================================
    // BPlusNode 实现
    // ============================================================================

    BPlusNode::BPlusNode(bool isLeaf, int maxKeys)
        : isLeaf_(isLeaf), parent_(nullptr), next_(nullptr), prev_(nullptr), maxKeys_(maxKeys) {
        keys_.reserve(maxKeys + 1);
        if (isLeaf_) {
            features_.reserve(maxKeys + 1);
        }
        else {
            children_.reserve(maxKeys + 2);
        }
    }

    BPlusNode::~BPlusNode() {
        // 不删除子节点，由BPlusTree统一管理
    }
    //在叶节点中按Hilbert 值 key，保持有序地插入一条记录（Feature）
    void BPlusNode::insertLeaf(uint64_t key, const Feature& feature) {
        // 找到插入位置（保持有序）
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        size_t pos = it - keys_.begin();

        keys_.insert(keys_.begin() + pos, key);
        features_.insert(features_.begin() + pos, feature);
    }
    //在内部节点中插入一个键（key）和与之对应的子节点指针（child）
    void BPlusNode::insertInternal(uint64_t key, BPlusNode* child) {
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        size_t pos = it - keys_.begin();

        keys_.insert(keys_.begin() + pos, key);
        children_.insert(children_.begin() + pos + 1, child);

        if (child) {
            child->setParent(this);
        }
    }
    //当节点（叶或内部）超过容量上限时，将节点分裂成两个节点，返回新创建的右侧节点。
    BPlusNode* BPlusNode::split() {
        int mid = (maxKeys_ + 1) / 2;
        BPlusNode* newNode = new BPlusNode(isLeaf_, maxKeys_);

        if (isLeaf_) {
            // 叶节点分裂：中点及之后的数据移到新节点
            newNode->keys_.assign(keys_.begin() + mid, keys_.end());
            newNode->features_.assign(features_.begin() + mid, features_.end());

            keys_.erase(keys_.begin() + mid, keys_.end());
            features_.erase(features_.begin() + mid, features_.end());

            // 维护叶节点链表
            newNode->next_ = this->next_;
            newNode->prev_ = this;
            if (this->next_) {
                this->next_->prev_ = newNode;
            }
            this->next_ = newNode;
        }
        else {
            // 内部节点分裂：中点key上移，右边数据移到新节点
            newNode->keys_.assign(keys_.begin() + mid + 1, keys_.end());
            newNode->children_.assign(children_.begin() + mid + 1, children_.end());

            // 更新子节点的父指针
            for (auto* child : newNode->children_) {
                if (child) child->setParent(newNode);
            }

            keys_.erase(keys_.begin() + mid, keys_.end());
            children_.erase(children_.begin() + mid + 1, children_.end());
        }

        return newNode;
    }
    //在节点的 keys_ 中找到第一个不小于 key 的位置索引
    // 此索引用于决定应该访问哪个子节点或在哪个位置插入新键。
    int BPlusNode::findKeyIndex(uint64_t key) const {
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        return static_cast<int>(it - keys_.begin());
    }
    //在内部节点中根据 key 决定应下钻到哪个子节点进行继续查找或插入。
    BPlusNode* BPlusNode::findChild(uint64_t key) const {
        if (isLeaf_ || children_.empty()) return nullptr;

        // 找第一个 key <= keys[i] 的位置
        size_t i = 0;
        while (i < keys_.size() && key > keys_[i]) {
            i++;
        }

        return children_[i];
    }
    //统计以当前节点为根的子树中内部节点和叶节点的数量
    void BPlusNode::countNodes(int& interiorNum, int& leafNum) const {
        if (isLeaf_) {
            ++leafNum;
        }
        else {
            ++interiorNum;
            for (auto* child : children_) {
                if (child) {
                    child->countNodes(interiorNum, leafNum);
                }
            }
        }
    }
    //计算以当前节点为根的子树高度
    int BPlusNode::countHeight(int height) const {
        if (isLeaf_) {
            return height + 1;
        }
        else {
            int maxH = height + 1;
            for (auto* child : children_) {
                if (child) {
                    maxH = std::max(maxH, child->countHeight(height + 1));
                }
            }
            return maxH;
        }
    }
    //将当前节点子树中的所有叶节点记录（Feature）按叶子遍历的顺序收集到传入的向量中
    void BPlusNode::collectLeafFeatures(std::vector<Feature>& features) const {
        if (isLeaf_) {
            features.insert(features.end(), features_.begin(), features_.end());
        }
        else {
            for (auto* child : children_) {
                if (child) {
                    child->collectLeafFeatures(features);
                }
            }
        }
    }

    void BPlusNode::rangeQuery(const Envelope& rect, uint64_t key, std::vector<Feature>& features) {
        if (isLeaf_) {
            // 在叶节点上，keys_ 与 features_ 应当一一对应
            for (size_t i = 0; i < keys_.size(); ++i) {
                if (keys_[i] == key) {
                    if (features_[i].getEnvelope().intersect(rect)) {
                        features.push_back(features_[i]);
                    }
                }
            }
        }
        else {
            size_t i = 0;
            while (i < keys_.size() && key > keys_[i]) ++i;
            // 递归调用从 i 开始到 children_.size()-1
            for (; i < children_.size(); ++i) {
                if (children_[i]) children_[i]->rangeQuery(rect, key, features);
            }
        }
    }

    // ============================================================================
    // BPlusTree 实现
    // ============================================================================

    BPlusTree::BPlusTree(int order, int hilbertOrder)
        : Tree(order), root_(nullptr), leftmost_(nullptr),
        order_(order), hilbert_(hilbertOrder) {
        if (order < 3) {
            throw std::invalid_argument("B+ tree order must be >= 3");
        }
    }

    BPlusTree::BPlusTree(int order, int hilbertOrder, const Envelope& bbox)
        : Tree(order), root_(nullptr), leftmost_(nullptr),
        order_(order), hilbert_(hilbertOrder, bbox) {
        if (order < 3) {
            throw std::invalid_argument("B+ tree order must be >= 3");
        }
        this->bbox = bbox;
    }

    BPlusTree::~BPlusTree() {
        deleteTree(root_);
    }
    //递归删除以 node 为根的子树，释放所有节点内存
    void BPlusTree::deleteTree(BPlusNode* node) {
        if (!node) return;

        if (!node->isLeaf()) {
            for (size_t i = 0; i < node->getChildCount(); ++i) {
                deleteTree(node->getChild(i));
            }
        }
        delete node;
    }
    //批量构建（bulk-load）B+ 树：先用 Hilbert 值把所有要素排序，然后自底向上构造叶层与内部索引层。
    bool BPlusTree::constructTree(const std::vector<Feature>& features) {
        // 清空现有树
        deleteTree(root_);
        root_ = nullptr;
        leftmost_ = nullptr;

        if (features.empty()) {
            return true;
        }

        // 计算包围盒
        bbox = features[0].getEnvelope();
        for (size_t i = 1; i < features.size(); ++i) {
            bbox = bbox.unionEnvelope(features[i].getEnvelope());
        }
        hilbert_.setBBox(bbox);

        // 创建带Hilbert值的Feature列表并排序
        std::vector<std::pair<uint64_t, Feature>> sorted;
        sorted.reserve(features.size());

        for (const auto& f : features) {
            const Envelope& env = f.getEnvelope();
            double cx = (env.getMinX() + env.getMaxX()) / 2.0;
            double cy = (env.getMinY() + env.getMaxY()) / 2.0;
            uint64_t hValue = hilbert_.pointToHilbert(cx, cy);
            sorted.emplace_back(hValue, f);
        }

        std::sort(sorted.begin(), sorted.end(),
            [](const auto& a, const auto& b) { return a.first < b.first; });

        // 批量构建叶节点
        std::vector<BPlusNode*> leafNodes;
        BPlusNode* currentLeaf = nullptr;

        for (const auto& pair : sorted) {
            if (!currentLeaf || currentLeaf->isFull()) {
                currentLeaf = new BPlusNode(true, order_);
                leafNodes.push_back(currentLeaf);

                // 叶节点链表
                if (leafNodes.size() > 1) {
                    BPlusNode* prevLeaf = leafNodes[leafNodes.size() - 2];
                    prevLeaf->setNext(currentLeaf);
                    currentLeaf->setPrev(prevLeaf);
                }
            }
            currentLeaf->insertLeaf(pair.first, pair.second);
        }

        if (leafNodes.empty()) return true;

        leftmost_ = leafNodes[0];

        // 自底向上构建索引层
        std::vector<BPlusNode*> currentLevel = leafNodes;

        /*while (currentLevel.size() > 1) {
            std::vector<BPlusNode*> nextLevel;
            BPlusNode* parent = nullptr;

            for (size_t i = 0; i < currentLevel.size(); ++i) {
                if (!parent || static_cast<int>(parent->getKeyCount()) >= order_) {
                    parent = new BPlusNode(false, order_);
                    nextLevel.push_back(parent);
                    // 第一个子节点
                    parent->children_.push_back(currentLevel[i]);
                    currentLevel[i]->setParent(parent);
                }
                else {
                    // 添加key和子节点
                    uint64_t key = currentLevel[i]->getKey(0);
                    parent->keys_.push_back(key);
                    parent->children_.push_back(currentLevel[i]);
                    currentLevel[i]->setParent(parent);
                }
            }

            currentLevel = nextLevel;
        }*/
        while (currentLevel.size() > 1) {
            std::vector<BPlusNode*> nextLevel;
            BPlusNode* parent = nullptr;

            for (size_t i = 0; i < currentLevel.size(); ++i) {
                if (!parent || static_cast<int>(parent->children_.size()) > order_) {
                    // 创建新的父节点
                    parent = new BPlusNode(false, order_);
                    nextLevel.push_back(parent);
                }

                // 添加子节点
                if (parent->children_.empty()) {
                    // 第一个子节点，不需要前置分隔键
                    parent->children_.push_back(currentLevel[i]);
                }
                else {
                    // 后续子节点，需要添加分隔键
                    uint64_t key = currentLevel[i]->getKey(0);
                    parent->keys_.push_back(key);
                    parent->children_.push_back(currentLevel[i]);
                }

                currentLevel[i]->setParent(parent);
            }

            currentLevel = nextLevel;
        }
        root_ = currentLevel[0];
        return true;
    }
    //将单条 Feature 插入树，动态更新用的，不需要了
    void BPlusTree::insert(const Feature& feature) {
        const Envelope& env = feature.getEnvelope();
        double cx = (env.getMinX() + env.getMaxX()) / 2.0;
        double cy = (env.getMinY() + env.getMaxY()) / 2.0;
        uint64_t hValue = hilbert_.pointToHilbert(cx, cy);

        if (!root_) {
            root_ = new BPlusNode(true, order_);
            leftmost_ = root_;
            root_->insertLeaf(hValue, feature);
            return;
        }

        BPlusNode* leaf = findLeaf(hValue);
        leaf->insertLeaf(hValue, feature);

        if (leaf->isFull() && static_cast<int>(leaf->getKeyCount()) > order_) {
            // 分裂叶节点
            BPlusNode* newLeaf = leaf->split();
            uint64_t newKey = newLeaf->getKey(0);

            if (leaf == root_) {
                splitRoot();
                root_->insertInternal(newKey, newLeaf);
            }
            else {
                insertInternal(leaf->getParent(), newKey, newLeaf);
            }
        }
    }
    //从 root下钻找到包含 hValue 的叶节点
    BPlusNode* BPlusTree::findLeaf(uint64_t hValue) const {
        if (!root_) return nullptr;

        BPlusNode* current = root_;
        while (!current->isLeaf()) {
            int idx = current->findKeyIndex(hValue);
            current = current->getChild(idx);
            if (!current) break;
        }
        return current;
    }
    //将上升键 key 与对应的新子节点 newChild 插入到父节点 node（内部节点）。
    // 如插入后父节点超过容量，则分裂并把新的上升键递归插入更高一层
    void BPlusTree::insertInternal(BPlusNode* node, uint64_t key, BPlusNode* newChild) {
        if (!node) return;

        node->insertInternal(key, newChild);

        if (static_cast<int>(node->getKeyCount()) > order_) {
            BPlusNode* newNode = node->split();
            uint64_t upKey = newNode->getKey(0);

            if (node == root_) {
                splitRoot();
                root_->insertInternal(upKey, newNode);
            }
            else {
                insertInternal(node->getParent(), upKey, newNode);
            }
        }
    }
    //当根节点分裂或需要提升层级时，创建一个新的根节点并把旧根作为新根的第一个子，从而增加树的高度一层。
    void BPlusTree::splitRoot() {
        BPlusNode* oldRoot = root_;
        root_ = new BPlusNode(false, order_);
        root_->children_.push_back(oldRoot);
        oldRoot->setParent(root_);
    }
    //统计树中内部节点数和叶节点数
    void BPlusTree::countNode(int& interiorNum, int& leafNum) {
        interiorNum = leafNum = 0;
        if (root_) {
            root_->countNodes(interiorNum, leafNum);
        }
    }

    void BPlusTree::countHeight(int& height) {
        height = 0;
        if (root_) {
            height = root_->countHeight(0);
        }
    }

    // ============================================================================
    // 区域查询实现
    // ============================================================================
    //hilbert找叶
    BPlusNode* BPlusTree::findLeafByHilbert(uint64_t hValue) const {
        return findLeaf(hValue);
    }

    /*void BPlusTree::computeHilbertRange(const Envelope& rect,
        std::vector<std::pair<uint64_t, uint64_t>>& ranges) const {
        ranges.clear();
        // grid size
        uint32_t n = static_cast<uint32_t>(1u << hilbert_.getOrder());
        const Envelope& bbox = hilbert_.getBBox();
        double xmin = bbox.getMinX(), ymin = bbox.getMinY();
        double xmax = bbox.getMaxX(), ymax = bbox.getMaxY();
        double dx = xmax - xmin; if (dx <= 0) dx = 1e-9;
        double dy = ymax - ymin; if (dy <= 0) dy = 1e-9;
        auto cellEnvelope = [&](uint32_t cx, uint32_t cy, uint32_t size)->Envelope {
            double cellW = dx / static_cast<double>(n);
            double cellH = dy / static_cast<double>(n);
            double x0 = xmin + static_cast<double>(cx) * cellW;
            double y0 = ymin + static_cast<double>(cy) * cellH;
            double x1 = xmin + static_cast<double>(cx + size) * cellW;
            double y1 = ymin + static_cast<double>(cy + size) * cellH;
            return Envelope(x0, y0, x1, y1);
            };
        auto cellHilbertRange = [&](uint32_t cx, uint32_t cy, uint32_t size)->std::pair<uint64_t, uint64_t> {
            uint32_t x0 = cx, y0 = cy;
            uint32_t x1 = std::min<uint32_t>(cx + size - 1, n - 1);
            uint32_t y1 = std::min<uint32_t>(cy + size - 1, n - 1);
            uint64_t h0 = hilbert_.xyToHilbertIndex(x0, y0);
            uint64_t h1 = hilbert_.xyToHilbertIndex(x1, y1);
            if (h0 <= h1) return { h0, h1 };
            return { h1, h0 };
            };

        std::vector<std::pair<uint64_t, uint64_t>> raw;
        std::function<void(uint32_t, uint32_t, uint32_t)> dfs = [&](uint32_t cx, uint32_t cy, uint32_t size) {
            Envelope cenv = cellEnvelope(cx, cy, size);
            if (!cenv.intersect(rect)) return;
            if (rect.contain(cenv) || size == 1) {
                raw.push_back(cellHilbertRange(cx, cy, size));
                return;
            }
            uint32_t h = size / 2;
            if (h == 0) h = 1;
            dfs(cx, cy, h);
            if (cx + h < n) dfs(cx + h, cy, h);
            if (cy + h < n) dfs(cx, cy + h, h);
            if (cx + h < n && cy + h < n) dfs(cx + h, cy + h, h);
            };

        dfs(0, 0, n);
        if (raw.empty()) return;
        std::sort(raw.begin(), raw.end(), [](auto& a, auto& b) { if (a.first != b.first) return a.first < b.first; return a.second < b.second; });
        auto cur = raw[0];
        for (size_t i = 1; i < raw.size(); ++i) {
            if (raw[i].first <= cur.second + 1) cur.second = std::max(cur.second, raw[i].second);
            else { ranges.push_back(cur); cur = raw[i]; }
        }
        ranges.push_back(cur);
    }*/

    /*void BPlusTree::rangeQueryByHilbert(uint64_t hMin, uint64_t hMax,
        const Envelope& rect,
        std::vector<Feature>& result) const {
        if (!root_) return;

        // 使用B+树索引快速定位第一个叶节点
        BPlusNode* startLeaf = findLeafContainingOrAfter(hMin);
        if (!startLeaf) return;

        // 从起始叶节点开始顺序遍历
        BPlusNode* current = startLeaf;
        while (current) {
            const auto& keys = current->getKeys();
            const auto& features = current->getFeatures();

            for (size_t i = 0; i < keys.size(); ++i) {
                uint64_t key = keys[i];

                // 如果key还没到范围内，跳过
                if (key < hMin) continue;

                // 如果key超过范围，提前终止整个查询
                if (key > hMax) return;

                // key在范围内，进行几何相交测试
                if (features[i].getEnvelope().intersect(rect)) {
                    result.push_back(features[i]);
                }
            }

            // 移动到下一个叶节点
            current = current->getNext();
        }
    }*/

    void BPlusTree::rangeQueryByHilbert(uint64_t hMin, uint64_t hMax,
        const Envelope& rect,
        std::vector<Feature>& result) const
    {
        if (!root_) return;

        std::unordered_set<uint64_t> localSeen;
        BPlusNode* leaf = findLeafContainingOrAfter(hMin);
        if (!leaf) return;

        BPlusNode* cur = leaf;
        while (cur) {
            const auto& keys = cur->getKeys();
            const auto& feats = cur->getFeatures();

            // 二分找到第一个 >= hMin
            size_t lo = 0, hi = keys.size();
            while (lo < hi) {
                size_t m = (lo + hi) / 2;
                if (keys[m] < hMin) lo = m + 1; else hi = m;
            }

            for (size_t i = lo; i < keys.size(); ++i) {
                uint64_t k = keys[i];
                if (k > hMax) return;
                if (localSeen.find(k) != localSeen.end()) continue;
                if (feats[i].getEnvelope().intersect(rect)) {
                    result.push_back(feats[i]);
                    localSeen.insert(k);
                }
            }

            cur = cur->getNext();
        }
    }

    void BPlusTree::computeHilbertRange(const Envelope& rect,
        std::vector<std::pair<uint64_t, uint64_t>>& rawIntervals) const
    {
        rawIntervals.clear();
        uint32_t n = static_cast<uint32_t>(1u << hilbert_.getOrder());
        const Envelope& hb = hilbert_.getBBox();
        double xmin = hb.getMinX(), ymin = hb.getMinY();
        double xmax = hb.getMaxX(), ymax = hb.getMaxY();
        double dx = xmax - xmin; if (dx <= 0) dx = 1e-9;
        double dy = ymax - ymin; if (dy <= 0) dy = 1e-9;
        double cellW = dx / static_cast<double>(n);
        double cellH = dy / static_cast<double>(n);

        std::function<void(uint32_t, uint32_t, uint32_t)> dfs =
            [&](uint32_t cx, uint32_t cy, uint32_t size) {
            double x0 = xmin + static_cast<double>(cx) * cellW;
            double y0 = ymin + static_cast<double>(cy) * cellH;
            double x1 = xmin + static_cast<double>(std::min<uint32_t>(cx + size, n)) * cellW;
            double y1 = ymin + static_cast<double>(std::min<uint32_t>(cy + size, n)) * cellH;
            Envelope cellEnv(x0, x1, y0, y1);

            if (!cellEnv.intersect(rect)) return;

            if (rect.contain(cellEnv) || size == 1) {
                uint32_t xStart = cx;
                uint32_t yStart = cy;
                uint32_t xEnd = std::min<uint32_t>(cx + size - 1, n - 1);
                uint32_t yEnd = std::min<uint32_t>(cy + size - 1, n - 1);

                uint64_t h0 = hilbert_.xyToHilbertIndex(xStart, yStart);
                uint64_t h1 = hilbert_.xyToHilbertIndex(xEnd, yEnd);
                if (h0 <= h1) rawIntervals.emplace_back(h0, h1);
                else rawIntervals.emplace_back(h1, h0);
                return;
            }
            uint32_t half = size / 2;
            if (half == 0) half = 1;
            dfs(cx, cy, half);
            if (cx + half < n) dfs(cx + half, cy, half);
            if (cy + half < n) dfs(cx, cy + half, half);
            if (cx + half < n && cy + half < n) dfs(cx + half, cy + half, half);
            };

        dfs(0, 0, n);

        if (rawIntervals.empty()) return;
        std::sort(rawIntervals.begin(), rawIntervals.end(),
            [](const std::pair<uint64_t, uint64_t>& a, const std::pair<uint64_t, uint64_t>& b) {
                if (a.first != b.first) return a.first < b.first;
                return a.second < b.second;
            });
        std::vector<std::pair<uint64_t, uint64_t>> merged;
        auto cur = rawIntervals[0];
        for (size_t i = 1; i < rawIntervals.size(); ++i) {
            if (rawIntervals[i].first <= cur.second + 1) {
                cur.second = std::max(cur.second, rawIntervals[i].second);
            }
            else {
                merged.push_back(cur);
                cur = rawIntervals[i];
            }
        }
        merged.push_back(cur);
        rawIntervals.swap(merged);
    }

    void BPlusTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
        features.clear();
        if (!root_) return;

        std::vector<std::pair<uint64_t, uint64_t>> intervals;
        computeHilbertRange(rect, intervals);
        if (intervals.empty()) return;

        std::unordered_set<uint64_t> seenKeys;
        std::vector<Feature> candidates;

        for (const auto& pr : intervals) {
            uint64_t hMin = pr.first;
            uint64_t hMax = pr.second;

            BPlusNode* startLeaf = findLeafContainingOrAfter(hMin);
            if (startLeaf && !startLeaf->getKeys().empty() &&
                startLeaf->getKeys().front() > hMin && startLeaf->getPrev()) {
                startLeaf = startLeaf->getPrev();
            }
            if (!startLeaf) startLeaf = leftmost_;
            if (!startLeaf) continue;

            BPlusNode* current = startLeaf;
            while (current) {
                const auto& keys = current->getKeys();
                const auto& feats = current->getFeatures();

                // 二分找到起始索引 >= hMin
                size_t lo = 0, hi = keys.size();
                while (lo < hi) {
                    size_t m = (lo + hi) / 2;
                    if (keys[m] < hMin) lo = m + 1;
                    else hi = m;
                }

                for (size_t i = lo; i < keys.size(); ++i) {
                    uint64_t key = keys[i];
                    if (key > hMax) {
                        goto next_interval;
                    }
                    if (seenKeys.find(key) != seenKeys.end()) continue;

                    if (!feats[i].getEnvelope().intersect(rect)) continue;

                    candidates.push_back(feats[i]);
                    seenKeys.insert(key);
                }

                current = current->getNext();
            }
        next_interval:
            continue;
        }
        //精确判断，快一点
        for (const auto& f : candidates) {
            bool preciseIntersect = false;
            if (!preciseIntersect) {
                if (f.getEnvelope().intersect(rect)) preciseIntersect = true;
            }
            if (preciseIntersect) {
                features.push_back(f);
            }
        }
    }

    BPlusNode* BPlusTree::findLeafContainingOrAfter(uint64_t hValue) const {
        if (!root_) return nullptr;

        BPlusNode* current = root_;

        while (!current->isLeaf()) {
            const auto& keys = current->getKeys();
            const auto& children = current->children_;
            size_t i = 0;
            while (i < keys.size() && hValue > keys[i]) {
                i++;
            }

            current = children[i];
            if (!current) return nullptr;
        }

        return current;
    }


    // ============================================================================
    // 最邻近查询实现
    // ============================================================================
    bool BPlusTree::NNQuery(double x, double y, std::vector<Feature>& features) {
        features.clear();
        if (!root_) return false;

        uint64_t qh = hilbert_.pointToHilbert(x, y);
        BPlusNode* startLeaf = findLeafByHilbert(qh);
        if (!startLeaf) return false;

        auto leafMinMaxCellEnvelope = [&](BPlusNode* leaf)->Envelope {
            const auto& ks = leaf->getKeys();
            if (ks.empty()) return Envelope(0, 0, 0, 0);
            uint64_t hmin = ks.front();
            uint64_t hmax = ks.back();
            return hilbert_.getBBox();
            };

        auto envMinDist = [&](const Envelope& env)->double {
            return pointToEnvelopeDist(x, y, env);
            };

        using PQItem = std::pair<double, BPlusNode*>;
        struct Cmp {
            bool operator()(const PQItem& a, const PQItem& b) const 
        { 
            return a.first > b.first;
        } 
        };
        std::priority_queue<PQItem, std::vector<PQItem>, Cmp> pq;

        std::set<BPlusNode*> pushed;
        auto pushLeaf = [&](BPlusNode* leaf) {
            if (!leaf || pushed.count(leaf)) return;
            Envelope env = leafMinMaxCellEnvelope(leaf);
            double lb = envMinDist(env);
            pq.emplace(lb, leaf);
            pushed.insert(leaf);
            };

        pushLeaf(startLeaf);
        if (startLeaf->getPrev()) pushLeaf(startLeaf->getPrev());
        if (startLeaf->getNext()) pushLeaf(startLeaf->getNext());

        double bestDist = std::numeric_limits<double>::infinity();
        std::vector<Feature> cand;

        while (!pq.empty()) {
            auto top = pq.top(); pq.pop();
            double lb = top.first;
            BPlusNode* leaf = top.second;
            if (lb > bestDist) break; 

            for (size_t i = 0; i < leaf->getFeatureCount(); ++i) {
                const Feature& f = leaf->getFeature(i);
                double d = f.distance(x, y);
                if (d < bestDist) {
                    bestDist = d;
                    cand.clear();
                    cand.push_back(f);
                }
                else if (d == bestDist) {
                    cand.push_back(f);
                }
            }

            if (leaf->getPrev()) pushLeaf(leaf->getPrev());
            if (leaf->getNext()) pushLeaf(leaf->getNext());
        }


        if (cand.empty()) return false;

        double minD = std::numeric_limits<double>::infinity();
        for (const auto& f : cand) {
            double d = f.distance(x, y);
            if (d < minD) minD = d;
        }
        for (const auto& f : cand) {
            if (std::abs(f.distance(x, y) - minD) < 1e-12) features.push_back(f);
        }
        return !features.empty();
    }

    // ============================================================================
    // 基于距离的空间关联实现_1，比较快
    double BPlusTree::pointToEnvelopeDist(double x, double y, const Envelope& env) {
        double dx = 0.0;
        if (x < env.getMinX()) {
            dx = env.getMinX() - x;
        }
        else if (x > env.getMaxX()) {
            dx = x - env.getMaxX();
        }

        double dy = 0.0;
        if (y < env.getMinY()) {
            dy = env.getMinY() - y;
        }
        else if (y > env.getMaxY()) {
            dy = y - env.getMaxY();
        }

        return std::sqrt(dx * dx + dy * dy);
    }
    static Envelope envelopeFromNode(const BPlusNode* node) {
        Envelope env;
        bool init = false;
        if (!node) return env;
        if (node->isLeaf()) {
            size_t n = node->getFeatureCount();
            for (size_t i = 0; i < n; ++i) {
                const Feature& f = node->getFeature(i);
                const Geometry* g = f.getGeom();
                if (!g) continue;
                Envelope e = g->getEnvelope();
                if (!init) {
                    env = e;
                    init = true;
                }
                else {
                    env = env.unionEnvelope(e);  // 修复：赋值回env
                }
            }
        }
        else {
            size_t n = node->getChildCount();
            for (size_t i = 0; i < n; ++i) {
                BPlusNode* c = node->getChild(i);
                if (!c) continue;
                Envelope e = envelopeFromNode(c);
                if (!init) {
                    env = e;
                    init = true;
                }
                else {
                    env = env.unionEnvelope(e);  // 修复：赋值回env
                }
            }
        }
        return env;
    }

    // Envelope 最小距离（平方）
    static inline double envelopeMinDistSquared(const Envelope& a, const Envelope& b) {
        double dx = 0.0;
        if (a.getMaxX() < b.getMinX()) dx = b.getMinX() - a.getMaxX();
        else if (b.getMaxX() < a.getMinX()) dx = a.getMinX() - b.getMaxX();
        double dy = 0.0;
        if (a.getMaxY() < b.getMinY()) dy = b.getMinY() - a.getMaxY();
        else if (b.getMaxY() < a.getMinY()) dy = a.getMinY() - b.getMaxY();
        return dx * dx + dy * dy;
    }

    // 基本几何距离辅助（点/线段/LineString）——与之前 RTree 版本兼容
    static double pointToPointDist2(double x1, double y1, double x2, double y2) {
        double dx = x1 - x2, dy = y1 - y2; return dx * dx + dy * dy;
    }
    static double pointToSegmentDist2(double px, double py,
        double ax, double ay, double bx, double by) {
        double vx = bx - ax, vy = by - ay;
        double wx = px - ax, wy = py - ay;
        double c1 = vx * wx + vy * wy;
        if (c1 <= 0.0) return pointToPointDist2(px, py, ax, ay);
        double c2 = vx * vx + vy * vy;
        if (c2 <= c1) return pointToPointDist2(px, py, bx, by);
        double t = c1 / c2;
        double projx = ax + t * vx, projy = ay + t * vy;
        return pointToPointDist2(px, py, projx, projy);
    }
    static double pointToLineStringDist2(double px, double py, const LineString* line) {
        if (!line || line->numPoints() == 0) return std::numeric_limits<double>::infinity();
        size_t n = line->numPoints();
        if (n == 1) {
            const Point& p0 = line->getPointN(0);
            return pointToPointDist2(px, py, p0.getX(), p0.getY());
        }
        double minD = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i + 1 < n; ++i) {
            const Point& a = line->getPointN(i), & b = line->getPointN(i + 1);
            double d2 = pointToSegmentDist2(px, py, a.getX(), a.getY(), b.getX(), b.getY());
            if (d2 < minD) minD = d2;
        }
        return minD;
    }
    static double segmentToSegmentDist2(double a1x, double a1y, double a2x, double a2y,
        double b1x, double b1y, double b2x, double b2y) {
        double d1 = pointToSegmentDist2(a1x, a1y, b1x, b1y, b2x, b2y);
        double d2 = pointToSegmentDist2(a2x, a2y, b1x, b1y, b2x, b2y);
        double d3 = pointToSegmentDist2(b1x, b1y, a1x, a1y, a2x, a2y);
        double d4 = pointToSegmentDist2(b2x, b2y, a1x, a1y, a2x, a2y);
        return std::min({ d1,d2,d3,d4 });
    }
    static double lineStringToLineStringDist2(const LineString* A, const LineString* B) {
        if (!A || !B || A->numPoints() == 0 || B->numPoints() == 0) return std::numeric_limits<double>::infinity();
        size_t nA = A->numPoints(), nB = B->numPoints();
        if (nA == 1) { const Point& p = A->getPointN(0); return pointToLineStringDist2(p.getX(), p.getY(), B); }
        if (nB == 1) { const Point& p = B->getPointN(0); return pointToLineStringDist2(p.getX(), p.getY(), A); }
        double minD = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i + 1 < nA; ++i) {
            const Point& a1 = A->getPointN(i), & a2 = A->getPointN(i + 1);
            for (size_t j = 0; j + 1 < nB; ++j) {
                const Point& b1 = B->getPointN(j), & b2 = B->getPointN(j + 1);
                double d2 = segmentToSegmentDist2(a1.getX(), a1.getY(), a2.getX(), a2.getY(),
                    b1.getX(), b1.getY(), b2.getX(), b2.getY());
                if (d2 < minD) minD = d2;
                if (minD == 0.0) return 0.0;
            }
        }
        return minD;
    }
    static double computeGeometryDist2(const Geometry* A, const Geometry* B) {
        if (!A || !B) return std::numeric_limits<double>::infinity();
        const Point* pA = dynamic_cast<const Point*>(A);
        const Point* pB = dynamic_cast<const Point*>(B);
        const LineString* lA = dynamic_cast<const LineString*>(A);
        const LineString* lB = dynamic_cast<const LineString*>(B);
        if (pA && pB) return pointToPointDist2(pA->getX(), pA->getY(), pB->getX(), pB->getY());
        if (pA && lB) return pointToLineStringDist2(pA->getX(), pA->getY(), lB);
        if (lA && pB) return pointToLineStringDist2(pB->getX(), pB->getY(), lA);
        if (lA && lB) return lineStringToLineStringDist2(lA, lB);
        return std::numeric_limits<double>::infinity();
    }

    // BPlusTree::featureDistanceWithin（头文件声明的静态方法）实现
    bool BPlusTree::featureDistanceWithin(const Feature& a, const Feature& b, double D2, bool inclusive) {
        const Geometry* ga = a.getGeom();
        const Geometry* gb = b.getGeom();
        double d2 = computeGeometryDist2(ga, gb);
        return inclusive ? (d2 <= D2) : (d2 < D2);
    }

    // joinByDistance：对两个要素集合进行逐对精确匹配（头文件中声明）
    void BPlusTree::joinByDistance(const std::vector<Feature>& setA,
        const std::vector<Feature>& setB,
        double D2,
        std::vector<std::pair<Feature, Feature>>* out,
        MatchCallback cb, void* userData,
        bool inclusive) const
    {
        for (const Feature& fa : setA) {
            const Geometry* ga = fa.getGeom();
            if (!ga) continue;
            for (const Feature& fb : setB) {
                const Geometry* gb = fb.getGeom();
                if (!gb) continue;
                double d2 = computeGeometryDist2(ga, gb);
                bool match = inclusive ? (d2 <= D2) : (d2 < D2);
                if (match) {
                    if (out) out->push_back(std::make_pair(fa, fb));
                    if (cb) cb(fa, fb, userData);
                }
            }
        }
    }
    // 递归匹配实现：treeMatchNodesByDist（签名与 btree.h 一致）
    void BPlusTree::treeMatchNodesByDist(BPlusNode* a, BPlusNode* b, double D2,
        std::vector<std::pair<Feature, Feature>>* out,
        BPlusTree::MatchCallback cb, void* userData, bool inclusive)
    {
        if (!a || !b) return;

        Envelope envA = envelopeFromNode(a);
        Envelope envB = envelopeFromNode(b);

        double mind2 = envelopeMinDistSquared(envA, envB);
        if (mind2 > D2) return;

        // 都是叶节点：按要素逐对检查（利用 joinByDistance）
        if (a->isLeaf() && b->isLeaf()) {
            std::vector<Feature> aFeatures = a->getFeatures();
            std::vector<Feature> bFeatures = b->getFeatures();
            joinByDistance(aFeatures, bFeatures, D2, out, cb, userData, inclusive);
            return;
        }

        // a 叶, b 内部
        if (a->isLeaf() && !b->isLeaf()) {
            size_t nb = b->getChildCount();
            for (size_t j = 0; j < nb; ++j) {
                BPlusNode* childB = b->getChild(j);
                if (!childB) continue;
                Envelope envChildB = envelopeFromNode(childB);
                double lb = envelopeMinDistSquared(envA, envChildB);
                if (lb > D2) continue;
                treeMatchNodesByDist(a, childB, D2, out, cb, userData, inclusive);
            }
            return;
        }

        // a 内部, b 叶
        if (!a->isLeaf() && b->isLeaf()) {
            size_t na = a->getChildCount();
            for (size_t i = 0; i < na; ++i) {
                BPlusNode* childA = a->getChild(i);
                if (!childA) continue;
                Envelope envChildA = envelopeFromNode(childA);
                double lb = envelopeMinDistSquared(envChildA, envB);
                if (lb > D2) continue;
                treeMatchNodesByDist(childA, b, D2, out, cb, userData, inclusive);
            }
            return;
        }

        // 两个内部节点
        size_t na = a->getChildCount();
        size_t nb = b->getChildCount();
        for (size_t i = 0; i < na; ++i) {
            BPlusNode* childA = a->getChild(i);
            if (!childA) continue;
            Envelope envChildA = envelopeFromNode(childA);
            for (size_t j = 0; j < nb; ++j) {
                BPlusNode* childB = b->getChild(j);
                if (!childB) continue;
                Envelope envChildB = envelopeFromNode(childB);
                double lb = envelopeMinDistSquared(envChildA, envChildB);
                if (lb > D2) continue;
                treeMatchNodesByDist(childA, childB, D2, out, cb, userData, inclusive);
            }
        }
    }

    // 对外接口：vector 返回版本（头文件已声明）
    std::vector<std::pair<Feature, Feature>> BPlusTree::spatialJoinWithin(BPlusTree& other, double D, bool inclusive) {
        std::vector<std::pair<Feature, Feature>> out;
        if (!this->root_ || !other.root_) return out;
        double D2 = D * D;
        treeMatchNodesByDist(this->root_, other.root_, D2, &out, nullptr, nullptr, inclusive);
        return out;
    }

    // 对外接口：回调版本（头文件已声明）
    void BPlusTree::spatialJoinWithin(BPlusTree& other, double D, MatchCallback cb, void* userData, bool inclusive) {
        if (!this->root_ || !other.root_) return;
        double D2 = D * D;
        treeMatchNodesByDist(this->root_, other.root_, D2, nullptr, cb, userData, inclusive);
    }

    void BPlusNode::draw() {
        if (isLeaf()) {
            // 叶节点：绘制叶节点的包围盒（如果有），并绘制其中的 features
            Envelope bbox = envelopeFromNode(this);
            bbox.draw();
            for (size_t i = 0; i < getFeatureCount(); ++i) {
                const Feature& f = getFeature(i);
                // Feature::draw() 通常会绘制几何本身（点/线）
                f.draw();
            }
        }
        else {
            // 内部节点：仅递归调用子节点的 draw（不绘制自身 bbox，或可选择绘制）
            size_t n = getChildCount();
            for (size_t i = 0; i < n; ++i) {
                BPlusNode* c = getChild(i);
                if (c) c->draw();
            }
        }
    }

    // BPlusTree::draw — 覆盖自 Tree，调用 root_
    void BPlusTree::draw() {
        if (root_ != nullptr) {
            root_->draw();
        }
    }
} // namespace hw6