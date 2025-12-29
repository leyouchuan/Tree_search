#include "Geometry.h"
#include "RTree.h"
#include"assert.h"
#include<random>
namespace hw6 {

	// RNode 实现
	void RNode::add(RNode* child) {
		/*
		children[childrenNum] = child;
		child->parent = this;
		++childrenNum;*/
		if (!child) return;
		if (child->parent && child->parent != this) child->parent->remove(child);
		children.push_back(child);
		child->parent = this;
		++childrenNum;
		recalcBBox();
	}

	void RNode::remove(const Feature& f) {
		auto where = [&]() {
			for (auto itr = features.begin(); itr != features.end(); ++itr)
				if (itr->getName() == f.getName())
					return itr;
			}();
		features.erase(where);
		if (features.empty())
			features.shrink_to_fit();
		/*features.erase(where);
		if (features.empty())
			features.shrink_to_fit(); // free memory unused but allocated*/
	}

	void RNode::remove(RNode* child) {

		for (int i = 0; i < childrenNum; ++i)
			if (children[i] == child) {
				--childrenNum;
				std::swap(children[i], children[childrenNum]);
				children[childrenNum] = nullptr;
				break;
			}
		recalcBBox();
		/*if (!child) return;

		auto it = std::find(children.begin(), children.end(), child);
		if (it == children.end()) return;
		if (it != children.end() - 1) std::iter_swap(it, children.end() - 1);
		children.pop_back();
		if (childrenNum > 0) --childrenNum;
		child->parent = nullptr;
		recalcBBox();*/
	}

	Feature RNode::popBackFeature() {
		auto ret = features.back();
		features.pop_back();
		return ret;
	}

	RNode* RNode::popBackChildNode() {
		--childrenNum;
		auto ret = children[childrenNum];
		children[childrenNum] = nullptr;
		return ret;
		/*if (childrenNum == 0) return nullptr;
		--childrenNum;
		assert(childrenNum == static_cast<int>(children.size()));
		RNode* ret = children.back();
		children.pop_back();
		if (ret) ret->parent = nullptr;
		return ret;*/
	}

	void RNode::countNode(int& interiorNum, int& leafNum) {
		if (isLeafNode()) {
			++leafNum;
		}
		else {
			++interiorNum;
			for (int i = 0; i < childrenNum; ++i)
				children[i]->countNode(interiorNum, leafNum);
			assert(childrenNum == static_cast<int>(children.size()));
			/*for (auto* c : children)
				if (c)
					c->countNode(interiorNum, leafNum);*/
		}
	}

	int RNode::countHeight(int height) {
		++height;
		if (!isLeafNode()) {
			int cur = height;
			for (int i = 0; i < childrenNum; ++i)
				height = std::max(height, children[i]->countHeight(cur));
		}
		return height;
	}

	void RNode::draw() {
		if (isLeafNode()) {
			bbox.draw();
		}
		else
			for (int i = 0; i < childrenNum; ++i)
				children[i]->draw();
		assert(childrenNum == static_cast<int>(children.size()));
	}

	void RNode::rangeQuery(const Envelope& rect, std::vector<Feature>& result) {
		// Task rangeQuery
		/* TODO */
		if (!bbox.intersect(rect)) return;
		if (isLeafNode()) {
			for (const auto& f : features) {
				if (f.getEnvelope().intersect(rect)) result.push_back(f);
			}
		}
		else {
			for (int i = 0; i < childrenNum; ++i) {
				RNode* c = children[i];
				if (c) c->rangeQuery(rect, result);
				assert(childrenNum == static_cast<int>(children.size()));
			}
		}
		// filter step (选择查询区域与几何对象包围盒相交的几何对象)
		// 注意R树区域查询仅返回候选集，精炼步在hw6的rangeQuery中完成
	}

	//递归选择要插入的节点，新增面积越小越好
	RNode* RNode::pointInLeafNode(double x, double y) {
		// Task pointInLeafNode
		/* TODO */
		if (isLeafNode()) return this;
		if (children.empty()) return nullptr;
		Envelope pEnv(x, x, y, y); // 点的包围盒
		double bestInc = std::numeric_limits<double>::infinity();
		double bestArea = std::numeric_limits<double>::infinity();
		int bestIdx = -1;

		for (size_t i = 0; i < childrenNum; ++i) {
			RNode* c = children[i];
			if (!c) continue;
			double inc = enlargementToInclude(c->getEnvelope(), pEnv);
			double area = envelopeArea(c->getEnvelope());
			if (inc < bestInc || (inc == bestInc && area < bestArea)) {
				bestInc = inc;
				bestArea = area;
				bestIdx = static_cast<int>(i);
			}
		}

		if (bestIdx < 0) return nullptr;
		return children[bestIdx]->pointInLeafNode(x, y);
	}

	//Rnode重新计算包围盒
	void RNode::recalcBBox() {
		if (isLeafNode()) {
			if (features.empty()) { bbox = Envelope(); return; }
			Envelope e = features[0].getEnvelope();
			for (size_t i = 1; i < features.size(); ++i) e = e.unionEnvelope(features[i].getEnvelope());
			bbox = e;
		}
		else {
			if (children.empty()) { bbox = Envelope(); return; }
			Envelope e = children[0]->getEnvelope();
			for (size_t i = 1; i < childrenNum; ++i) e = e.unionEnvelope(children[i]->getEnvelope());
			assert(childrenNum == static_cast<int>(children.size()));
			bbox = e;
		}
	}

	// RTree 实现
	RTree::RTree(int maxChildren) : Tree(maxChildren), maxChildren(maxChildren) {
		if (maxChildren < 4) throw std::invalid_argument("maxChildren must be >= 4");
	}

	void RTree::countNode(int& interiorNum, int& leafNum) {
		interiorNum = leafNum = 0;
		if (root != nullptr)
			root->countNode(interiorNum, leafNum);
	}

	void RTree::countHeight(int& height) {
		height = 0;
		if (root != nullptr)
			height = root->countHeight(height);
	}

	//节点在分裂后或删除后应维持的最小子条目数
	static int MIN_CHILDREN_FROM_MAX(int maxChildren) {
		return std::max(1, (int)std::ceil(maxChildren * 0.4)); // 40% 最小填充
	}
	//删除递归节点
	static void deleteSubtree(RNode* node) {
		if (!node) return;
		if (!node->isLeafNode()) {
			int n = node->getChildNum();
			for (int i = 0; i < n; ++i) {
				RNode* c = node->getChildNode(i);
				deleteSubtree(c);
			}
		}
		delete node;

	}

	RNode* RTree::chooseleaf(RNode* node, const Envelope& box) {
		if (node->isLeafNode())
			return node;
		double bestInc = std::numeric_limits<double>::infinity();
		double bestArea = std::numeric_limits<double>::infinity();
		int bestIndex = -1;
		for (int i = 0; i < node->getChildNum(); ++i) {
			RNode* child = node->getChildNode(i);
			double inc = child->getEnvelope().unionEnvelope(box).getArea() - child->getEnvelope().getArea();
			double area = child->getEnvelope().getArea();
			if (inc < bestInc || (inc == bestInc && area < bestArea)) {
				bestInc = inc;
				bestArea = area;
				bestIndex = i;
			}
		}
		if (bestIndex < 0) return node;
		return chooseleaf(node->getChildNode(bestIndex), box);
	}

	void RTree::insertFeature(const Feature& f) {
		if (!root) {
			root = new RNode(f.getEnvelope());
			root->add(f);
			return;
		}
		Envelope env = f.getEnvelope();
		RNode* leaf = this->pointInLeafNode(env.getMaxX(), env.getMinY());
		if (!leaf) {
			leaf = chooseleaf(root, env);
			if (!leaf) leaf = root;
		}
		leaf->add(f);
		if ((int)leaf->getFeatureNum() > maxChildren) {
			RNode* newNode = leaf->splitNode(leaf);
			updateTree(leaf, newNode);
		}
		else {
			RNode* cur = leaf;
			while (cur) {
				cur->recalcBBox();
				cur = cur->getParent();
			}
		}
	}
	//quatric spliting 当超过节点所能存储的最大几何特征时分裂节点。
	RNode* RNode::splitNode(RNode* node) {
		int minChildren = MIN_CHILDREN_FROM_MAX(maxChildren);
		if (node->isLeafNode()) {
			int N = static_cast<int>(node->getFeatureNum());
			if (N <= 1) return nullptr;
			std::vector<bool> assigned(N, false);
			int seed1 = -1, seed2 = -1;
			double worstWaste = -std::numeric_limits<double>::infinity();
			for (int i = 0; i < N; ++i) {
				for (int j = i + 1; j < N; ++j) {
					Envelope ui = node->getFeature(i).getEnvelope().unionEnvelope(node->getFeature(j).getEnvelope());
					double waste = envelopeArea(ui) - envelopeArea(node->getFeature(i).getEnvelope())
						- envelopeArea(node->getFeature(j).getEnvelope());
					if (waste > worstWaste) { worstWaste = waste; seed1 = i; seed2 = j; }
				}
			}
			if (seed1 < 0 || seed2 < 0) {
				seed1 = 0; seed2 = 1;
			}
			RNode* group2 = new RNode(Envelope());
			std::vector<Feature> g1_feats; std::vector<Feature> g2_feats;
			g1_feats.push_back(node->getFeature(seed1)); assigned[seed1] = true;
			g2_feats.push_back(node->getFeature(seed2)); assigned[seed2] = true;
			Envelope g1_box = g1_feats[0].getEnvelope();
			Envelope g2_box = g2_feats[0].getEnvelope();
			int remain = N - 2;
			while (remain > 0) {
				int need1 = std::max(0, minChildren - (int)g1_feats.size());
				int need2 = std::max(0, minChildren - (int)g2_feats.size());
				if (need1 == remain) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) { g1_feats.push_back(node->getFeature(i)); assigned[i] = true; --remain; }
					break;
				}
				if (need2 == remain) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) { g2_feats.push_back(node->getFeature(i)); assigned[i] = true; --remain; }
					break;
				}

				double bestDiff = -1.0; int bestIdx = -1; bool assignToG1 = true;
				for (int i = 0; i < N; ++i) if (!assigned[i]) {
					Envelope e = node->getFeature(i).getEnvelope();
					double inc1 = enlargementToInclude(g1_box, e);
					double inc2 = enlargementToInclude(g2_box, e);
					double diff = std::abs(inc1 - inc2);
					if (diff > bestDiff) { bestDiff = diff; bestIdx = i; assignToG1 = inc1 < inc2; }
					else if (diff == bestDiff) {
						double area1 = envelopeArea(g1_box), area2 = envelopeArea(g2_box);
						if (area1 < area2) assignToG1 = true; else assignToG1 = false;
					}
				}
				if (bestIdx < 0) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) { bestIdx = i; break; }
					if (bestIdx < 0) break;
				}
				if (assignToG1) {
					g1_feats.push_back(node->getFeature(bestIdx));
					g1_box = g1_box.unionEnvelope(node->getFeature(bestIdx).getEnvelope());
				}
				else {
					g2_feats.push_back(node->getFeature(bestIdx));
					g2_box = g2_box.unionEnvelope(node->getFeature(bestIdx).getEnvelope());
				}
				assigned[bestIdx] = true;
				--remain;
			}

			node->features = std::move(g1_feats);
			node->recalcBBox();
			group2->features = std::move(g2_feats);
			group2->recalcBBox();
			assert(N == static_cast<int>(node->getFeatureNum()) + static_cast<int>(group2->getFeatureNum()));
			return group2;
		}
		else {
			int N = node->childrenNum;
			if (N <= 1) return nullptr;
			std::vector<bool> assigned(N, false);
			int seed1 = -1, seed2 = -1;
			double worstWaste = -std::numeric_limits<double>::infinity();
			for (int i = 0; i < N; ++i) for (int j = i + 1; j < N; ++j) {
				Envelope ui = node->children[i]->getEnvelope().unionEnvelope(node->children[j]->getEnvelope());
				double waste = envelopeArea(ui) - envelopeArea(node->children[i]->getEnvelope()) - envelopeArea(node->children[j]->getEnvelope());
				if (waste > worstWaste) { worstWaste = waste; seed1 = i; seed2 = j; }
			}
			if (seed1 < 0 || seed2 < 0) {
				seed1 = 0; seed2 = 1;
			}
			RNode* group2 = new RNode(Envelope());
			std::vector<RNode*> g1_children; std::vector<RNode*> g2_children;
			g1_children.push_back(node->children[seed1]); assigned[seed1] = true;
			g2_children.push_back(node->children[seed2]); assigned[seed2] = true;
			Envelope g1_box = g1_children[0]->getEnvelope();
			Envelope g2_box = g2_children[0]->getEnvelope();
			int remain = N - 2;
			while (remain > 0) {
				int need1 = std::max(0, minChildren - (int)g1_children.size());
				int need2 = std::max(0, minChildren - (int)g2_children.size());
				if (need1 == remain) {
					for (int i = 0; i < N; ++i) if (!assigned[i])
					{ 
						g1_children.push_back(node->children[i]);
						assigned[i] = true; --remain;
					}
					break;
				}
				if (need2 == remain) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) 
					{
						g2_children.push_back(node->children[i]);
						assigned[i] = true; --remain; 
					}
					break;
				}
				double bestDiff = -1.0; int bestIdx = -1; bool assignToG1 = true;
				for (int i = 0; i < N; ++i) if (!assigned[i]) {
					Envelope e = node->children[i]->getEnvelope();
					double inc1 = enlargementToInclude(g1_box, e);
					double inc2 = enlargementToInclude(g2_box, e);
					double diff = std::abs(inc1 - inc2);
					if (diff > bestDiff) { bestDiff = diff; bestIdx = i; assignToG1 = inc1 < inc2; }
					else if (diff == bestDiff) {
						double area1 = envelopeArea(g1_box), area2 = envelopeArea(g2_box);
						assignToG1 = (area1 < area2);
					}
				}

				if (bestIdx < 0) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) { bestIdx = i; break; }
					if (bestIdx < 0) break;
				}
				if (assignToG1) {
					g1_children.push_back(node->children[bestIdx]);
					g1_box = g1_box.unionEnvelope(node->children[bestIdx]->getEnvelope());
				}
				else {
					g2_children.push_back(node->children[bestIdx]);
					g2_box = g2_box.unionEnvelope(node->children[bestIdx]->getEnvelope());
				}
				assigned[bestIdx] = true;
				--remain;
			}
			node->children = std::move(g1_children);
			node->childrenNum = static_cast<int>(node->children.size());
			for (int i = 0; i < node->childrenNum; ++i) if (node->children[i]) 
				node->children[i]->parent = node;
			node->recalcBBox();;
			group2->children = std::move(g2_children);
			group2->childrenNum = static_cast<int>(group2->children.size());

			for (int i = 0; i < group2->childrenNum; ++i) if (group2->children[i])
				group2->children[i]->parent = group2;

			group2->recalcBBox();
			return group2;
		}
	}

	void RTree::updateTree(RNode* n, RNode* nn) {
		if (n == root) {
			Envelope newRootBox = n->getEnvelope().unionEnvelope(nn->getEnvelope());
			RNode* newRoot = new RNode(newRootBox);
			newRoot->add(n);
			newRoot->add(nn);
			root = newRoot;
			return;
		}
		RNode* parent = n->getParent();
		if (!parent) {
			Envelope newRootBox = n->getEnvelope().unionEnvelope(nn->getEnvelope());
			RNode* newRoot = new RNode(newRootBox);
			newRoot->add(n);
			newRoot->add(nn);
			root = newRoot;
			return;
		}

		parent->add(nn);
		parent->recalcBBox();
		if (parent->getChildNum() > maxChildren) {
			RNode* newParent = parent->splitNode(parent);
			if (newParent) updateTree(parent, newParent);
		}
		else {
			RNode* cur = parent;
			while (cur) { cur->recalcBBox(); cur = cur->getParent(); }
		}
		if (root) root->recalcBBox();
		bbox = root ? root->getEnvelope() : hw6::Envelope();
	}

	bool RTree::constructTree(const std::vector<Feature>& features) {
		// Task RTree construction
		/* TODO
		构建可以采用几何特征按x轴的顺序逐个插入，
		基于节点新增面积越小越好的原则选择插入的节点，
		当超过节点所能存储的最大几何特征时，
		基于二次分裂(quadratic split)算法，
		选择最左和最右两个几何特征作为种子点，
		对几何特征进行分组。为了保持R-Tree的平衡性，
		建议随机选择几何特征插入，或类似AVL树，
		在插入后调整R-Tree结构。
		*/
		if (root) 
		{ 
			deleteSubtree(root);
			root = nullptr;
		}
		if (features.empty())
		{ 
			root = nullptr;
			return true; 
		}
		std::vector<Feature> shuffled = features;
		std::mt19937_64 rng(std::random_device{}());
		std::shuffle(shuffled.begin(), shuffled.end(), rng);
		for (const Feature& f : shuffled)
			insertFeature(f);
		if (root)
			root->recalcBBox();
		bbox = root ? root->getEnvelope() : hw6::Envelope();

		return true;
	}

	void RTree::rangeQuery(const Envelope& rect, std::vector<Feature>& features) {
		features.clear();
		if (root != nullptr)
			root->rangeQuery(rect, features);
	}

	bool RTree::NNQuery(double x, double y, std::vector<Feature>& features) {
		if (!root) return false;
		const Envelope& rootEnv = root->getEnvelope();
		if (!rootEnv.contain(x, y)) return false;

		features.clear();

		double minDist = std::max(rootEnv.getWidth(), rootEnv.getHeight());

		RNode* pNode = root->pointInLeafNode(x, y);
		if (!pNode) { 
			Envelope rect(x - minDist, x + minDist, y - minDist, y + minDist);
			rangeQuery(rect, features);
			return !features.empty();
		}

		if (pNode->isLeafNode()) {
			size_t fn = pNode->getFeatureNum();
			for (size_t i = 0; i < fn; ++i) {
				minDist = std::min(minDist, pNode->getFeature(i).maxDistance2Envelope(x, y));
				if (minDist <= 0.0) break;
			}
		}
		else {
			int cn = pNode->getChildNum();
			for (int i = 0; i < cn; ++i) {
				RNode* child = pNode->getChildNode(i);
				if (!child) continue;
				size_t fn = child->getFeatureNum();
				for (size_t j = 0; j < fn; ++j) {
					minDist = std::min(minDist, child->getFeature(j).maxDistance2Envelope(x, y));
					if (minDist <= 0.0) break;
				}
				if (minDist <= 0.0) break;
			}
		}
		if (minDist < 0.0) minDist = 0.0;
		Envelope rect(x - minDist, x + minDist, y - minDist, y + minDist);
		rangeQuery(rect, features);

		return !features.empty();
	}

	//基于距离的空间关联=====================================
	static inline double envelopeMinDistSquared(const Envelope& a, const Envelope& b) {
		double dx = 0.0;
		if (a.getMaxX() < b.getMinX()) dx = b.getMinX() - a.getMaxX();
		else if (b.getMaxX() < a.getMinX()) dx = a.getMinX() - b.getMaxX();
		double dy = 0.0;
		if (a.getMaxY() < b.getMinY()) dy = b.getMinY() - a.getMaxY();
		else if (b.getMaxY() < a.getMinY()) dy = a.getMinY() - b.getMaxY();
		return dx * dx + dy * dy;
	}

	std::vector<std::pair<Feature, Feature>> RTree::spatialJoinWithin(RTree& other, double D, bool inclusive) {
		std::vector<std::pair<Feature, Feature>> out;
		if (!this->root || !other.root) return out;
		double D2 = D * D;
		treeMatchNodesByDist(this->root, other.root, D2, &out, nullptr, nullptr, inclusive);
		return out;
	}

	void RTree::spatialJoinWithin(RTree& other, double D, RTree::MatchCallback cb, void* userData, bool inclusive) {
		if (!this->root || !other.root) return;
		this->root->getEnvelope().print();
		double D2 = D * D;
		treeMatchNodesByDist(this->root, other.root, D2, nullptr, cb, userData, inclusive);
	}

	// ============== 距离计算辅助函数 ==============

// 点到点的平方距离
	static double pointToPointDist2(double x1, double y1, double x2, double y2) {
		double dx = x1 - x2;
		double dy = y1 - y2;
		return dx * dx + dy * dy;
	}

	// 点到线段的平方距离（已有，保持不变）
	static double pointToSegmentDist2(double px, double py,
		double ax, double ay,
		double bx, double by) {
		double vx = bx - ax, vy = by - ay;
		double wx = px - ax, wy = py - ay;
		double c1 = vx * wx + vy * wy;
		if (c1 <= 0.0) {
			double dx = px - ax, dy = py - ay;
			return dx * dx + dy * dy;
		}
		double c2 = vx * vx + vy * vy;
		if (c2 <= c1) {
			double dx = px - bx, dy = py - by;
			return dx * dx + dy * dy;
		}
		double t = c1 / c2;
		double projx = ax + t * vx, projy = ay + t * vy;
		double dx = px - projx, dy = py - projy;
		return dx * dx + dy * dy;
	}

	// 点到LineString的平方距离
	static double pointToLineStringDist2(double px, double py, const LineString* line) {
		if (!line || line->numPoints() == 0) {
			return std::numeric_limits<double>::infinity();
		}

		size_t n = line->numPoints();

		// 单点LineString
		if (n == 1) {
			const Point& p0 = line->getPointN(0);
			return pointToPointDist2(px, py, p0.getX(), p0.getY());
		}

		// 多点LineString：计算到每个线段的最小距离
		double minDist2 = std::numeric_limits<double>::infinity();
		for (size_t i = 0; i + 1 < n; ++i) {
			const Point& p1 = line->getPointN(i);
			const Point& p2 = line->getPointN(i + 1);
			double d2 = pointToSegmentDist2(px, py,
				p1.getX(), p1.getY(),
				p2.getX(), p2.getY());
			if (d2 < minDist2) minDist2 = d2;
		}

		return minDist2;
	}

	// 线段到线段的平方距离
	static double segmentToSegmentDist2(double a1x, double a1y, double a2x, double a2y,
		double b1x, double b1y, double b2x, double b2y) {
		// 计算四个端点到对方线段的距离，取最小值
		double d1 = pointToSegmentDist2(a1x, a1y, b1x, b1y, b2x, b2y);
		double d2 = pointToSegmentDist2(a2x, a2y, b1x, b1y, b2x, b2y);
		double d3 = pointToSegmentDist2(b1x, b1y, a1x, a1y, a2x, a2y);
		double d4 = pointToSegmentDist2(b2x, b2y, a1x, a1y, a2x, a2y);

		return std::min({ d1, d2, d3, d4 });
	}

	// LineString到LineString的平方距离
	static double lineStringToLineStringDist2(const LineString* lineA, const LineString* lineB) {
		if (!lineA || !lineB || lineA->numPoints() == 0 || lineB->numPoints() == 0) {
			return std::numeric_limits<double>::infinity();
		}

		size_t nA = lineA->numPoints();
		size_t nB = lineB->numPoints();

		// 如果任一是单点，转为点到线串距离
		if (nA == 1) {
			const Point& p = lineA->getPointN(0);
			return pointToLineStringDist2(p.getX(), p.getY(), lineB);
		}
		if (nB == 1) {
			const Point& p = lineB->getPointN(0);
			return pointToLineStringDist2(p.getX(), p.getY(), lineA);
		}

		// 计算所有线段对之间的最小距离
		double minDist2 = std::numeric_limits<double>::infinity();

		for (size_t i = 0; i + 1 < nA; ++i) {
			const Point& a1 = lineA->getPointN(i);
			const Point& a2 = lineA->getPointN(i + 1);

			for (size_t j = 0; j + 1 < nB; ++j) {
				const Point& b1 = lineB->getPointN(j);
				const Point& b2 = lineB->getPointN(j + 1);

				double d2 = segmentToSegmentDist2(
					a1.getX(), a1.getY(), a2.getX(), a2.getY(),
					b1.getX(), b1.getY(), b2.getX(), b2.getY()
				);

				if (d2 < minDist2) minDist2 = d2;
				if (minDist2 == 0.0) return 0.0; // 早期退出优化
			}
		}

		return minDist2;
	}

	// 计算两个几何对象之间的平方距离（不使用Geometry的distance方法）
	static double computeGeometryDist2(const Geometry* geomA, const Geometry* geomB) {
		if (!geomA || !geomB) {
			return std::numeric_limits<double>::infinity();
		}

		// 尝试转换为具体类型
		const Point* pA = dynamic_cast<const Point*>(geomA);
		const Point* pB = dynamic_cast<const Point*>(geomB);
		const LineString* lA = dynamic_cast<const LineString*>(geomA);
		const LineString* lB = dynamic_cast<const LineString*>(geomB);

		// Point to Point
		if (pA && pB) {
			return pointToPointDist2(pA->getX(), pA->getY(), pB->getX(), pB->getY());
		}

		// Point to LineString
		if (pA && lB) {
			return pointToLineStringDist2(pA->getX(), pA->getY(), lB);
		}

		// LineString to Point
		if (lA && pB) {
			return pointToLineStringDist2(pB->getX(), pB->getY(), lA);
		}

		// LineString to LineString
		if (lA && lB) {
			return lineStringToLineStringDist2(lA, lB);
		}

		// 其他类型（如Polygon）返回无穷大
		return std::numeric_limits<double>::infinity();
	}

	// ============== Tree Matching 实现 ==============

	void hw6::RTree::treeMatchNodesByDist(RNode* a, RNode* b, double D2,
		std::vector<std::pair<Feature, Feature>>* out,
		RTree::MatchCallback cb, void* userData, bool inclusive) {

		if (!a || !b) return;

		// 包围盒最小距离剪枝
		double mind2 = envelopeMinDistSquared(a->getEnvelope(), b->getEnvelope());
		if (mind2 > D2) return;  // 包围盒距离大于阈值，直接返回

		// 两个都是叶节点 - 计算精确几何距离
		if (a->isLeafNode() && b->isLeafNode()) {
			size_t numA = a->getFeatureNum();
			size_t numB = b->getFeatureNum();

			for (size_t i = 0; i < numA; ++i) {
				const Feature& fa = a->getFeature(i);
				const Geometry* geomA = fa.getGeom();
				if (!geomA) continue;

				for (size_t j = 0; j < numB; ++j) {
					const Feature& fb = b->getFeature(j);
					const Geometry* geomB = fb.getGeom();
					if (!geomB) continue;

					// 计算精确几何距离的平方
					double dist2 = computeGeometryDist2(geomA, geomB);

					// 判断是否满足距离条件
					bool match = false;
					if (inclusive) {
						match = (dist2 <= D2);  // 距离 ≤ D
					}
					else {
						match = (dist2 < D2);   // 距离 < D
					}

					if (match) {
						// 输出结果
						if (out) {
							out->push_back(std::make_pair(fa, fb));
						}
						if (cb) {
							cb(fa, fb, userData);
						}
					}
				}
			}
			return;
		}

		// a是叶节点，b是内部节点
		if (a->isLeafNode() && !b->isLeafNode()) {
			int numChildren = b->getChildNum();
			for (int i = 0; i < numChildren; ++i) {
				RNode* childB = b->getChildNode(i);
				if (childB) {
					treeMatchNodesByDist(a, childB, D2, out, cb, userData, inclusive);
				}
			}
			return;
		}

		// a是内部节点，b是叶节点
		if (!a->isLeafNode() && b->isLeafNode()) {
			int numChildren = a->getChildNum();
			for (int i = 0; i < numChildren; ++i) {
				RNode* childA = a->getChildNode(i);
				if (childA) {
					treeMatchNodesByDist(childA, b, D2, out, cb, userData, inclusive);
				}
			}
			return;
		}

		// 两个都是内部节点 - 递归所有子节点对
		if (!a->isLeafNode() && !b->isLeafNode()) {
			int numChildrenA = a->getChildNum();
			int numChildrenB = b->getChildNum();

			for (int i = 0; i < numChildrenA; ++i) {
				RNode* childA = a->getChildNode(i);
				if (!childA) continue;

				for (int j = 0; j < numChildrenB; ++j) {
					RNode* childB = b->getChildNode(j);
					if (!childB) continue;

					// 递归处理
					treeMatchNodesByDist(childA, childB, D2, out, cb, userData, inclusive);
				}
			}
			return;
		}
	}
} // namespace hw6

