#include "btree.h"
#include "Hilbert.h"
#include"Geometry.h"

#include "Common.h"
#include <ctime>
#include "CMakeIn.h"

#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>

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

    void BPlusNode::insertLeaf(uint64_t key, const Feature& feature) {
        // 找到插入位置（保持有序）
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        size_t pos = it - keys_.begin();

        keys_.insert(keys_.begin() + pos, key);
        features_.insert(features_.begin() + pos, feature);
    }

    void BPlusNode::insertInternal(uint64_t key, BPlusNode* child) {
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        size_t pos = it - keys_.begin();

        keys_.insert(keys_.begin() + pos, key);
        children_.insert(children_.begin() + pos + 1, child);

        if (child) {
            child->setParent(this);
        }
    }

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

    int BPlusNode::findKeyIndex(uint64_t key) const {
        auto it = std::lower_bound(keys_.begin(), keys_.end(), key);
        return static_cast<int>(it - keys_.begin());
    }

    BPlusNode* BPlusNode::findChild(uint64_t key) const {
        if (isLeaf_ || children_.empty()) return nullptr;

        int idx = findKeyIndex(key);
        // 如果key >= keys_[idx]，应该去idx+1的子节点
        // 但lower_bound返回的是第一个>=key的位置
        // 在B+树中，keys_[i]是children_[i+1]的最小值
        return children_[idx];
    }

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

    void BPlusTree::deleteTree(BPlusNode* node) {
        if (!node) return;

        if (!node->isLeaf()) {
            for (size_t i = 0; i < node->getChildCount(); ++i) {
                deleteTree(node->getChild(i));
            }
        }
        delete node;
    }

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

        // Bulk-loading: 批量构建叶节点
        std::vector<BPlusNode*> leafNodes;
        BPlusNode* currentLeaf = nullptr;

        for (const auto& pair : sorted) {
            if (!currentLeaf || currentLeaf->isFull()) {
                currentLeaf = new BPlusNode(true, order_);
                leafNodes.push_back(currentLeaf);

                // 维护叶节点链表
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

        while (currentLevel.size() > 1) {
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
        }

        root_ = currentLevel[0];
        return true;
    }

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

    void BPlusTree::splitRoot() {
        BPlusNode* oldRoot = root_;
        root_ = new BPlusNode(false, order_);
        root_->children_.push_back(oldRoot);
        oldRoot->setParent(root_);
    }

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

    void BPlusTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
        features.clear();
        if (!root_) return;

        // 计算查询矩形对应的Hilbert值范围
        std::vector<std::pair<uint64_t, uint64_t>> ranges;
        computeHilbertRange(rect, ranges);

        // 对每个Hilbert范围执行查询
        for (const auto& range : ranges) {
            rangeQueryByHilbert(range.first, range.second, rect, features);
        }
    }

    void BPlusTree::computeHilbertRange(const Envelope& rect,
        std::vector<std::pair<uint64_t, uint64_t>>& ranges) const {
        ranges.clear();

        // 简化版本：计算矩形四个角和中心的Hilbert值，取最小和最大值
        double minX = rect.getMinX();
        double maxX = rect.getMaxX();
        double minY = rect.getMinY();
        double maxY = rect.getMaxY();
        double centerX = (minX + maxX) / 2.0;
        double centerY = (minY + maxY) / 2.0;

        std::vector<uint64_t> cornerValues;
        cornerValues.push_back(hilbert_.pointToHilbert(minX, minY));
        cornerValues.push_back(hilbert_.pointToHilbert(maxX, minY));
        cornerValues.push_back(hilbert_.pointToHilbert(minX, maxY));
        cornerValues.push_back(hilbert_.pointToHilbert(maxX, maxY));
        cornerValues.push_back(hilbert_.pointToHilbert(centerX, centerY));

        // 采样矩形边界上的更多点以获得更精确的范围
        int samples = 10;
        for (int i = 0; i <= samples; ++i) {
            double t = static_cast<double>(i) / samples;
            cornerValues.push_back(hilbert_.pointToHilbert(minX + t * (maxX - minX), minY));
            cornerValues.push_back(hilbert_.pointToHilbert(minX + t * (maxX - minX), maxY));
            cornerValues.push_back(hilbert_.pointToHilbert(minX, minY + t * (maxY - minY)));
            cornerValues.push_back(hilbert_.pointToHilbert(maxX, minY + t * (maxY - minY)));
        }

        std::sort(cornerValues.begin(), cornerValues.end());
        cornerValues.erase(std::unique(cornerValues.begin(), cornerValues.end()),
            cornerValues.end());

        // 创建连续的范围（简化：使用min到max的单一范围）
        if (!cornerValues.empty()) {
            ranges.emplace_back(cornerValues.front(), cornerValues.back());
        }
    }

    void BPlusTree::rangeQueryByHilbert(uint64_t hMin, uint64_t hMax,
        const Envelope& rect,
        std::vector<Feature>& result) const {
        if (!leftmost_) return;

        // 从最左叶节点开始扫描
        BPlusNode* current = leftmost_;

        while (current) {
            const auto& keys = current->getKeys();
            const auto& features = current->getFeatures();

            for (size_t i = 0; i < keys.size(); ++i) {
                uint64_t hValue = keys[i];

                // 如果超出范围，可以提前退出（因为是有序的）
                if (hValue > hMax) {
                    return;
                }

                // 在范围内，检查几何相交
                if (hValue >= hMin && hValue <= hMax) {
                    const Feature& f = features[i];
                    if (f.getEnvelope().intersect(rect)) {
                        result.push_back(f);
                    }
                }
            }

            current = current->getNext();
        }
    }

    // ============================================================================
    // 最邻近查询实现
    // ============================================================================

    bool BPlusTree::NNQuery(double x, double y, std::vector<Feature>& features) {
        features.clear();
        if (!root_) return false;

        // 计算查询点的Hilbert值
        uint64_t queryH = hilbert_.pointToHilbert(x, y);

        // 找到包含该Hilbert值的叶节点
        BPlusNode* leaf = findLeaf(queryH);
        if (!leaf) return false;

        // 初始化最小距离
        double minDist = std::numeric_limits<double>::infinity();
        std::vector<Feature> candidates;

        // 从当前叶节点开始向两侧扩展搜索
        // 首先检查当前叶节点
        for (size_t i = 0; i < leaf->getFeatureCount(); ++i) {
            const Feature& f = leaf->getFeature(i);
            double dist = pointToEnvelopeDist(x, y, f.getEnvelope());
            if (dist < minDist) {
                minDist = dist;
                candidates.clear();
                candidates.push_back(f);
            }
            else if (dist == minDist) {
                candidates.push_back(f);
            }
        }

        // 向左右扩展搜索，直到不可能有更近的点
        // 向左搜索
        BPlusNode* leftNode = leaf->getPrev();
        while (leftNode && minDist > 0) {
            bool foundCloser = false;
            for (size_t i = 0; i < leftNode->getFeatureCount(); ++i) {
                const Feature& f = leftNode->getFeature(i);
                double dist = pointToEnvelopeDist(x, y, f.getEnvelope());
                if (dist < minDist) {
                    minDist = dist;
                    candidates.clear();
                    candidates.push_back(f);
                    foundCloser = true;
                }
                else if (dist == minDist) {
                    candidates.push_back(f);
                }
            }

            // 如果左侧节点的最远点都比当前minDist远，可以停止
            if (!foundCloser && leftNode->getFeatureCount() > 0) {
                const Feature& farthest = leftNode->getFeature(leftNode->getFeatureCount() - 1);
                double maxDist = pointToEnvelopeDist(x, y, farthest.getEnvelope());
                if (maxDist > minDist * 2) break; // 启发式剪枝
            }

            leftNode = leftNode->getPrev();
        }

        // 向右搜索
        BPlusNode* rightNode = leaf->getNext();
        while (rightNode && minDist > 0) {
            bool foundCloser = false;
            for (size_t i = 0; i < rightNode->getFeatureCount(); ++i) {
                const Feature& f = rightNode->getFeature(i);
                double dist = pointToEnvelopeDist(x, y, f.getEnvelope());
                if (dist < minDist) {
                    minDist = dist;
                    candidates.clear();
                    candidates.push_back(f);
                    foundCloser = true;
                }
                else if (dist == minDist) {
                    candidates.push_back(f);
                }
            }

            // 如果右侧节点的最近点都比当前minDist远，可以停止
            if (!foundCloser && rightNode->getFeatureCount() > 0) {
                const Feature& nearest = rightNode->getFeature(0);
                double minDistToNode = pointToEnvelopeDist(x, y, nearest.getEnvelope());
                if (minDistToNode > minDist * 2) break; // 启发式剪枝
            }

            rightNode = rightNode->getNext();
        }

        // 使用精确距离重新过滤候选集
        minDist = std::numeric_limits<double>::infinity();
        for (const auto& f : candidates) {
            double dist = f.distance(x, y);
            if (dist < minDist) {
                minDist = dist;
                features.clear();
                features.push_back(f);
            }
            else if (dist == minDist) {
                features.push_back(f);
            }
        }

        return !features.empty();
    }

    // ============================================================================
    // 基于距离的空间关联实现
    // ============================================================================

    std::vector<std::pair<Feature, Feature>> BPlusTree::spatialJoinWithin(
        BPlusTree& other, double D, bool inclusive) {

        std::vector<std::pair<Feature, Feature>> result;
        if (!root_ || !other.root_) return result;

        double D2 = D * D;

        // 收集所有features
        std::vector<Feature> setA, setB;
        if (leftmost_) {
            BPlusNode* node = leftmost_;
            while (node) {
                const auto& features = node->getFeatures();
                setA.insert(setA.end(), features.begin(), features.end());
                node = node->getNext();
            }
        }

        if (other.leftmost_) {
            BPlusNode* node = other.leftmost_;
            while (node) {
                const auto& features = node->getFeatures();
                setB.insert(setB.end(), features.begin(), features.end());
                node = node->getNext();
            }
        }

        // 执行距离关联
        joinByDistance(setA, setB, D2, &result, nullptr, nullptr, inclusive);

        return result;
    }

    void BPlusTree::spatialJoinWithin(BPlusTree& other, double D,
        MatchCallback cb, void* userData,
        bool inclusive) {
        if (!root_ || !other.root_ || !cb) return;

        double D2 = D * D;

        // 收集所有features
        std::vector<Feature> setA, setB;
        if (leftmost_) {
            BPlusNode* node = leftmost_;
            while (node) {
                const auto& features = node->getFeatures();
                setA.insert(setA.end(), features.begin(), features.end());
                node = node->getNext();
            }
        }

        if (other.leftmost_) {
            BPlusNode* node = other.leftmost_;
            while (node) {
                const auto& features = node->getFeatures();
                setB.insert(setB.end(), features.begin(), features.end());
                node = node->getNext();
            }
        }

        // 执行距离关联
        joinByDistance(setA, setB, D2, nullptr, cb, userData, inclusive);
    }

    void BPlusTree::joinByDistance(const std::vector<Feature>& setA,
        const std::vector<Feature>& setB,
        double D2,
        std::vector<std::pair<Feature, Feature>>* out,
        MatchCallback cb, void* userData,
        bool inclusive) const {
        // 使用包围盒快速过滤
        for (const auto& fa : setA) {
            const Envelope& envA = fa.getEnvelope();

            for (const auto& fb : setB) {
                const Envelope& envB = fb.getEnvelope();

                // 包围盒层面的距离剪枝
                double envDist2 = envelopeMinDist(envA, envB) * envelopeMinDist(envA, envB);
                if (envDist2 > D2) continue;

                // 精确距离计算
                if (featureDistanceWithin(fa, fb, D2, inclusive)) {
                    if (out) {
                        out->emplace_back(fa, fb);
                    }
                    if (cb) {
                        cb(fa, fb, userData);
                    }
                }
            }
        }
    }

    bool BPlusTree::featureDistanceWithin(const Feature& a, const Feature& b,
        double D2, bool inclusive) {
        // 计算两个几何对象中心点之间的距离
        const Envelope& envA = a.getEnvelope();
        const Envelope& envB = b.getEnvelope();

        double cxA = (envA.getMinX() + envA.getMaxX()) / 2.0;
        double cyA = (envA.getMinY() + envA.getMaxY()) / 2.0;
        double cxB = (envB.getMinX() + envB.getMaxX()) / 2.0;
        double cyB = (envB.getMinY() + envB.getMaxY()) / 2.0;

        double dist2 = (cxA - cxB) * (cxA - cxB) + (cyA - cyB) * (cyA - cyB);

        if (inclusive) {
            return dist2 <= D2;
        }
        else {
            return dist2 < D2;
        }
    }

    // ============================================================================
    // 辅助函数实现
    // ============================================================================

    double BPlusTree::pointDistance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

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

    double BPlusTree::envelopeMinDist(const Envelope& a, const Envelope& b) {
        double dx = 0.0;
        if (a.getMaxX() < b.getMinX()) {
            dx = b.getMinX() - a.getMaxX();
        }
        else if (b.getMaxX() < a.getMinX()) {
            dx = a.getMinX() - b.getMaxX();
        }

        double dy = 0.0;
        if (a.getMaxY() < b.getMinY()) {
            dy = b.getMinY() - a.getMaxY();
        }
        else if (b.getMaxY() < a.getMinY()) {
            dy = a.getMinY() - b.getMaxY();
        }

        return std::sqrt(dx * dx + dy * dy);
    }

    void BPlusTree::collectFeaturesInRange(uint64_t hMin, uint64_t hMax,
        std::vector<Feature>& result) const {
        if (!leftmost_) return;

        BPlusNode* current = leftmost_;
        while (current) {
            const auto& keys = current->getKeys();
            const auto& features = current->getFeatures();

            for (size_t i = 0; i < keys.size(); ++i) {
                uint64_t h = keys[i];
                if (h > hMax) return; // 提前退出
                if (h >= hMin) {
                    result.push_back(features[i]);
                }
            }
            current = current->getNext();
        }
    }

    void BPlusTree::draw() {
        // 可视化实现（可选）
        if (!root_) return;

        // 绘制所有feature的包围盒
        if (leftmost_) {
            BPlusNode* node = leftmost_;
            while (node) {
                for (size_t i = 0; i < node->getFeatureCount(); ++i) {
                    node->getFeature(i).draw();
                }
                node = node->getNext();
            }
        }
    }

} // namespace hw6