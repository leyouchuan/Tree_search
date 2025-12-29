#ifndef BPLUSTREE_H_INCLUDED
#define BPLUSTREE_H_INCLUDED

#include "Geometry.h"
#include "Tree.h"
#include "Hilbert.h"
#include <algorithm>
#include <vector>
#include <queue>
#include <memory>
#include <cmath>

namespace hw6 {

    // B+树节点：存储Hilbert值索引的Feature
    class BPlusNode {
    private:
        bool isLeaf_;
        BPlusNode* parent_;
        std::vector<uint64_t> keys_;           // Hilbert值作为键
        std::vector<Feature> features_;        // 叶节点：存储Feature
        std::vector<BPlusNode*> children_;     // 内部节点：子节点指针
        BPlusNode* next_;                      // 叶节点链表指针
        BPlusNode* prev_;                      // 叶节点链表指针（用于范围查询优化）
        int maxKeys_;

    public:
        BPlusNode(bool isLeaf, int maxKeys);
        ~BPlusNode();

        bool isLeaf() const { return isLeaf_; }
        BPlusNode* getParent() const { return parent_; }
        void setParent(BPlusNode* p) { parent_ = p; }

        BPlusNode* getNext() const { return next_; }
        void setNext(BPlusNode* n) { next_ = n; }
        BPlusNode* getPrev() const { return prev_; }
        void setPrev(BPlusNode* p) { prev_ = p; }

        size_t getKeyCount() const { return keys_.size(); }
        uint64_t getKey(size_t i) const { return i < keys_.size() ? keys_[i] : 0; }
        const std::vector<uint64_t>& getKeys() const { return keys_; }

        size_t getFeatureCount() const { return features_.size(); }
        const Feature& getFeature(size_t i) const { return features_[i]; }
        const std::vector<Feature>& getFeatures() const { return features_; }

        size_t getChildCount() const { return children_.size(); }
        BPlusNode* getChild(size_t i) const { return i < children_.size() ? children_[i] : nullptr; }

        bool isFull() const { return static_cast<int>(keys_.size()) >= maxKeys_; }
        int getMaxKeys() const { return maxKeys_; }

        // 插入操作
        void insertLeaf(uint64_t key, const Feature& feature);
        void insertInternal(uint64_t key, BPlusNode* child);

        // 分裂操作
        BPlusNode* split();

        // 查找操作
        int findKeyIndex(uint64_t key) const;
        BPlusNode* findChild(uint64_t key) const;

        void countNodes(int& interiorNum, int& leafNum) const;
        int countHeight(int height) const;
        void collectLeafFeatures(std::vector<Feature>& features) const;
        //区域查询
        void rangeQuery(const Envelope& rect, uint64_t key, std::vector<Feature>& features);
        void draw();

        friend class BPlusTree;
    };

    class BPlusTree : public Tree {
    private:
        BPlusNode* root_;
        BPlusNode* leftmost_;      // 最左叶节点（最小Hilbert值）
        int order_;                 // B+树的阶（最大子节点数）
        Hilbert hilbert_;          // Hilbert曲线编码器

        // 辅助函数
        BPlusNode* findLeaf(uint64_t hValue) const;
        void insertInternal(BPlusNode* parent, uint64_t key, BPlusNode* newChild);
        void splitRoot();

        // 区域查询辅助
        void computeHilbertRange(const Envelope& rect,
            std::vector<std::pair<uint64_t, uint64_t>>& ranges) const;
        void rangeQueryByHilbert(uint64_t hMin, uint64_t hMax,
            const Envelope& rect,
            std::vector<Feature>& result) const;
        BPlusNode* findLeafContainingOrAfter(uint64_t hValue) const;
        BPlusNode* BPlusTree::findLeafByHilbert(uint64_t hValue) const;
        // 距离计算辅助
        static double pointToEnvelopeDist(double x, double y, const Envelope& env);

    public:
        BPlusTree(int order = 50, int hilbertOrder = 16);
        BPlusTree(int order, int hilbertOrder, const Envelope& bbox);
        ~BPlusTree();

        void setCapacity(int capacity) override { order_ = capacity; }

        // Hilbert相关
        void setHilbertOrder(int order) { hilbert_.setOrder(order); }
        int getHilbertOrder() const { return hilbert_.getOrder(); }
        void setHilbertBBox(const Envelope& bbox) { hilbert_.setBBox(bbox); }
        const Envelope& getHilbertBBox() const { return hilbert_.getBBox(); }

        // 树操作接口
        bool constructTree(const std::vector<Feature>& features) override;
        void insert(const Feature& feature);

        void countNode(int& interiorNum, int& leafNum) override;
        void countHeight(int& height) override;

        void rangeQuery(const Envelope& rect, std::vector<Feature>& features) override;
        bool NNQuery(double x, double y, std::vector<Feature>& features) override;
        // 空间关联
        using MatchCallback = void(*)(const Feature&, const Feature&, void* userData);
        void treeMatchNodesByDist(BPlusNode* a, BPlusNode* b, double D2, std::vector<std::pair<Feature, Feature>>* out, BPlusTree::MatchCallback cb, void* userData, bool inclusive);
        std::vector<std::pair<Feature, Feature>> spatialJoinWithin(
            BPlusTree& other, double D, bool inclusive = true);
        void spatialJoinWithin(BPlusTree& other, double D,
            MatchCallback cb, void* userData = nullptr,
            bool inclusive = true);

        void draw() override;


    private:
        void deleteTree(BPlusNode* node);

        // 距离关联的递归实现
        void joinByDistance(const std::vector<Feature>& setA,
            const std::vector<Feature>& setB,
            double D2,
            std::vector<std::pair<Feature, Feature>>* out,
            MatchCallback cb, void* userData,
            bool inclusive) const;

        static bool featureDistanceWithin(const Feature& a, const Feature& b,
            double D2, bool inclusive);
    public:
        static void test(int t);
        static void analyse();
    };

} // namespace hw6

#endif // BPLUSTREE_H_INCLUDED