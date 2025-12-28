#include "QuadTree.h"
#include <set>

namespace hw6 {

	/*
	 * QuadNode
	 */
	void QuadNode::split(size_t capacity) {
		for (int i = 0; i < 4; ++i) {
			delete children[i];
			children[i] = nullptr;
		}

		// Task construction
		// TODO
	}

	void QuadNode::countNode(int& interiorNum, int& leafNum) {
		if (isLeafNode()) {
			++leafNum;
		}
		else {
			++interiorNum;
			for (int i = 0; i < 4; ++i)
				children[i]->countNode(interiorNum, leafNum);
		}
	}

	int QuadNode::countHeight(int height) {
		++height;
		if (!isLeafNode()) {
			int cur = height;
			for (int i = 0; i < 4; ++i) {
				height = std::max(height, children[i]->countHeight(cur));
			}
		}
		return height;
	}

	void QuadNode::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
		if (!bbox.intersect(rect))
			return;

		// Task range query
		// TODO
	}

	QuadNode* QuadNode::pointInLeafNode(double x, double y) {
		// Task NN query
		// TODO

		return nullptr;
	}

	void QuadNode::draw() {
		if (isLeafNode()) {
			bbox.draw();
		}
		else {
			for (int i = 0; i < 4; ++i)
				children[i]->draw();
		}
	}

	/*
	 * QuadTree
	 */
	bool QuadTree::constructTree(const std::vector<Feature>& features) {
		if (features.empty())
			return false;

		// Task construction
		// TODO

		bbox = Envelope(-74.1, -73.8, 40.6, 40.8); // 注意此行代码需要更新为features的包围盒，或根节点的包围盒

		return true;
	}

	void QuadTree::countNode(int& interiorNum, int& leafNum) {
		interiorNum = 0;
		leafNum = 0;
		if (root)
			root->countNode(interiorNum, leafNum);
	}

	void QuadTree::countHeight(int& height) {
		height = 0;
		if (root)
			height = root->countHeight(0);
	}

	void QuadTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
		features.clear();

		// Task range query
		// TODO

		// filter step (选择查询区域与几何对象包围盒相交的几何对象)

		// 注意四叉树区域查询仅返回候选集，精炼步在hw6的rangeQuery中完成
	}

	bool QuadTree::NNQuery(double x, double y, std::vector<Feature>& features) {
		if (!root || !(root->getEnvelope().contain(x, y)))
			return false;

		// Task NN query
		// TODO

		// filter step
		// (使用maxDistance2Envelope函数，获得查询点到几何对象包围盒的最短的最大距离，然后区域查询获得候选集)

		const Envelope& envelope = root->getEnvelope();
		double minDist = std::max(envelope.getWidth(), envelope.getHeight());

		// 注意四叉树邻近查询仅返回候选集，精炼步在hw6的NNQuery中完成

		return true;
	}

	void QuadTree::draw() {
		if (root)
			root->draw();
	}

} // namespace hw6
