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
					double waste = envelopeArea(ui) - envelopeArea(node->getFeature(i).getEnvelope()) - envelopeArea(node->getFeature(j).getEnvelope());
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
		for (const Feature& f : features)
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

	static double pointToSegmentDist2(double px, double py, double ax, double ay, double bx, double by) {
		double vx = bx - ax, vy = by - ay;
		double wx = px - ax, wy = py - ay;
		double c1 = vx * wx + vy * wy;
		if (c1 <= 0.0) {
			double dx = px - ax, dy = py - ay; return dx * dx + dy * dy;
		}
		double c2 = vx * vx + vy * vy;
		if (c2 <= c1) {
			double dx = px - bx, dy = py - by; return dx * dx + dy * dy;
		}
		double t = c1 / c2;
		double projx = ax + t * vx, projy = ay + t * vy;
		double dx = px - projx, dy = py - projy;
		return dx * dx + dy * dy;
	}

	// 点到折线平方距离（poly: vector<pair<double,double>>）
	static double pointToPolylineDist2(double px, double py, const std::vector<std::pair<double, double>>& poly) {
		double best = std::numeric_limits<double>::infinity();
		if (poly.size() == 1) {
			double dx = px - poly[0].first, dy = py - poly[0].second;
			return dx * dx + dy * dy;
		}
		for (size_t i = 0; i + 1 < poly.size(); ++i) {
			double d2 = pointToSegmentDist2(px, py, poly[i].first, poly[i].second, poly[i + 1].first, poly[i + 1].second);
			if (d2 < best) best = d2;
		}
		return best;
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

	// 递归实现
	void hw6::RTree::treeMatchNodesByDist(RNode* a, RNode* b, double D2,
		std::vector<std::pair<Feature, Feature>>* out,
		RTree::MatchCallback cb, void* userData, bool inclusive) {
		if (!a || !b) return;

		// 快速包围盒下界剪枝
		double mind2 = envelopeMinDistSquared(a->getEnvelope(), b->getEnvelope());
		if (mind2 > D2) return;

		// 两叶逐对比较

	}

} // namespace hw6

