#include "QuadTree.h"
#include <set>

namespace hw6 {

	/*
	 * QuadNode
	 */

	 //分裂
	 //*
	void QuadNode::split(size_t capacity) {
		for (int i = 0; i < 4; ++i) {
			delete children[i];
			children[i] = nullptr;
		}

		double midx = (bbox.getMinX() + bbox.getMaxX()) / 2.0;
		double midy = (bbox.getMinY() + bbox.getMaxY()) / 2.0;

		children[0] = new QuadNode(Envelope(bbox.getMinX(), midx, midy, bbox.getMaxY()));
		children[1] = new QuadNode(Envelope(midx, bbox.getMaxX(), midy, bbox.getMaxY()));
		children[2] = new QuadNode(Envelope(bbox.getMinX(), midx, bbox.getMinY(), midy));
		children[3] = new QuadNode(Envelope(midx, bbox.getMaxX(), bbox.getMinY(), midy));

		for (const auto& f : features)
			for (int i = 0; i < 4; ++i)
				if (children[i]->getEnvelope().intersect(f.getEnvelope()))
					children[i]->features.push_back(f);

		features.clear();

		for (int i = 0; i < 4; ++i)
			if (children[i]->features.size() > capacity)
				children[i]->split(capacity);
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

	//区域查询
	//*
	void QuadNode::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
		if (!bbox.intersect(rect))
			return;

		// Task range query
		// TODO
		// 若是叶子节点，直接检查 feature
		if (isLeafNode()) {
			for (const Feature& f : this->features) {
				if (f.getEnvelope().intersect(rect)) {
					features.push_back(f);
				}
			}
		}
		// 若不是叶子节点，递归访问子节点
		else {
			for (int i = 0; i < 4; ++i) {
				if (children[i] != nullptr) {
					children[i]->rangeQuery(rect, features);
				}
			}
		}

	}


	//返回叶子节点
	//*
	QuadNode* QuadNode::pointInLeafNode(double x, double y) {
		// Task NN query
		// TODO
		// 如果是叶子节点，直接返回自己
		if (isLeafNode())
			return this;

		// 否则递归到包含该点的子节点
		for (int i = 0; i < 4; ++i) {
			if (children[i] != nullptr &&
				children[i]->getEnvelope().contain(x, y)) {
				return children[i]->pointInLeafNode(x, y);
			}
		}
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
	 //建立树
	 //*
	bool QuadTree::constructTree(const std::vector<Feature>& features) {
		if (features.empty())
			return false;

		// Task construction
		// TODO

		bbox = Envelope(std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity());
		for (const Feature f : features) {
			bbox = bbox.unionEnvelope(f.getEnvelope());
		}
		root = new QuadNode(bbox);
		root->add(features);
		root->split(capacity);

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

	//*
	void QuadTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
		features.clear();

		// Task range query
		// TODO
		if (root != nullptr) {
			root->rangeQuery(rect, features);
		}

		// filter step (选择查询区域与几何对象包围盒相交的几何对象)

		// 注意四叉树区域查询仅返回候选集，精炼步在hw6的rangeQuery中完成
	}

	//*
	bool QuadTree::NNQuery(double x, double y, std::vector<Feature>& features) {
		if (!root || !(root->getEnvelope().contain(x, y)))
			return false;

		features.clear();
		// Task NN query
		// TODO

		// filter step
		// (使用maxDistance2Envelope函数，获得查询点到几何对象包围盒的最短的最大距离，然后区域查询获得候选集)

		const Envelope& envelope = root->getEnvelope();
		double minDist = std::max(envelope.getWidth(), envelope.getHeight());

		// 注意四叉树邻近查询仅返回候选集，精炼步在hw6的NNQuery中完成

		QuadNode* pLeaf = root->pointInLeafNode(x, y);

		if (pLeaf != nullptr) {
			for (size_t i = 0; i < pLeaf->getFeatureNum(); i++) {
				minDist = std::min(minDist, pLeaf->getFeature(i).maxDistance2Envelope(x, y));
			}
		}

		Envelope rect(x - minDist, x + minDist, y - minDist, y + minDist);
		rangeQuery(rect, features);

		return true;
	}

	void QuadTree::draw() {
		if (root)
			root->draw();
	}

} // namespace hw6