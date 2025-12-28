#ifndef RTREE_H_INCLUDED
#define RTREE_H_INCLUDED

#include "Geometry.h"
#include "Tree.h"

#include <algorithm>
#include <array>
#include <queue>
#include <string>
#include <vector>

#include "CMakeIn.h"

namespace hw6 {

	/// <summary>
	/// </summary>
	class RNode {
	private:
		RNode* parent = nullptr;
		int maxChildren;
		std::vector<RNode*> children;
		int childrenNum = 0;
		Envelope bbox;
		std::vector<Feature> features;

	public:
		RNode() = delete;
		RNode(const Envelope& box) : bbox(box) { children.clear(); }

		bool isLeafNode() { return children.empty();}

		const Envelope& getEnvelope() { return bbox; }

		RNode* getParent() { return parent; }

		void setEnvelope(const Envelope& box) { bbox = box; }

		RNode* getChildNode(size_t i) {
			return i < children.size() ? children[i] : nullptr;
		}

		const RNode* getChildNode(size_t i) const {
			return i < children.size() ? children[i] : nullptr;
		}

		int getChildNum() const { return children.size(); }

		//envelope的面积计算
		double envelopeArea(const Envelope& e) {
			double w = e.getWidth();
			double h = e.getHeight();
			if (w < 0 || h < 0) return 0.0;
			return w * h;
		}

		double enlargementToInclude(const Envelope& base, const Envelope& e) {
			Envelope u = base.unionEnvelope(e);
			return envelopeArea(u) - envelopeArea(base);
		}

		size_t getFeatureNum() const { return features.size(); }

		const Feature& getFeature(size_t i) const { return features[i]; }

		const std::vector<Feature>& getFeatures() const { return features; }

		void add(const Feature& f) {
			features.push_back(f);
		}

		void add(RNode* child);

		void remove(const Feature& f);

		void remove(RNode* child);

		Feature popBackFeature();

		RNode* popBackChildNode();

		void countNode(int& interiorNum, int& leafNum);

		int countHeight(int height);

		void draw();

		void rangeQuery(const Envelope& rect, std::vector<Feature>& features);

		RNode* pointInLeafNode(double x, double y);

		void recalcBBox();

		RNode* RNode::splitNode(RNode* node);

		friend void deleteSubtree(RNode* node);

	};

	class RTree : public Tree {
	private:
		int maxChildren;
		RNode* root = nullptr;

	public:
		// 默认构造函数，maxChildren 默认为 8
		RTree() : RTree(8) {}
		explicit RTree(int maxChildren);
		~RTree() {
			if (root != nullptr) delete root;
			root = nullptr;
		}

		void setCapacity(int capacity) override {
			// DO NOTHING, since capacity is immutable in R tree
		}

		RNode* chooseleaf(RNode* node, const Envelope& box);
		void RTree::insertFeature(const Feature& f);
		void RTree::updateTree(RNode* n, RNode* nn);

		bool constructTree(const std::vector<Feature>& features) override;

		void countNode(int& interiorNum, int& leafNum) override;

		void countHeight(int& height) override;

		void rangeQuery(const Envelope& rect, std::vector<Feature>& features) override;

		bool NNQuery(double x, double y, std::vector<Feature>& features) override;

		RNode* pointInLeafNode(double x, double y) {
			if (root != nullptr) return root->pointInLeafNode(x, y);
			else return nullptr;
		}

		void draw() override { if (root != nullptr) root->draw();}

		using MatchCallback = void(*)(const Feature&, const Feature&, void* userData);

		std::vector<std::pair<Feature, Feature>> spatialJoinWithin(RTree& other, double D, bool inclusive = true);
		void spatialJoinWithin(RTree& other, double D, MatchCallback cb, void* userData = nullptr, bool inclusive = true);

		void treeMatchNodesByDist(RNode* a, RNode* b, double D2,
			std::vector<std::pair<Feature, Feature>>* out,
			MatchCallback cb, void* userData, bool inclusive);

	public:
		static void test(int t);
		static void analyse();
	};

} // namespace hw6

#endif // !RTREE_H_INCLUDED
