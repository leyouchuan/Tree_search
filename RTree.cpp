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
		if (it == children.end()) return;       // 子节点不在此节点中
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
			// 计算将点包含进 child 的增加量（或直接使用 unionArea - area）
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
		// 假定 f 是点（Envelope 的 min==max）
		if (!root) {
			root = new RNode(f.getEnvelope());
			root->add(f);
			return;
		}

		Envelope env = f.getEnvelope();
		// 额外校验（可选）：
		// if (!(env.minX == env.maxX && env.minY == env.maxY)) { /* 处理非点或抛错 */ }

		// 优先使用点专用选择函数
		RNode* leaf = this->pointInLeafNode(env.getMaxX(), env.getMinY());
		if (!leaf) {
			// 回退：使用通用选择
			leaf = chooseleaf(root, env);
			if (!leaf) leaf = root;
		}

		// 插入
		leaf->add(f);

		// 处理溢出或更新 bbox
		if ((int)leaf->getFeatureNum() > maxChildren) {
			// 按你实现的签名调用 splitNode（此处假定无参数）
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
	//quatric spliting 当超过节点所能存储的最大几何特征时分裂节点。推荐的优化：bulk-loading（STR）
	RNode* RNode::splitNode(RNode* node) {
		int minChildren = MIN_CHILDREN_FROM_MAX(maxChildren);
		if (node->isLeafNode()) {
			// operate on features
			int N = static_cast<int>(node->getFeatureNum());
			if (N <= 1) return nullptr;
			//if (N < 2) return new RNode(Envelope()); // 或直接返回 nullptr/不分裂
			std::vector<bool> assigned(N, false);
			// indices
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
				// fallback: assign first to group1, second to group2 if possible
				seed1 = 0; seed2 = 1;
			}
			// 如果 N==1 或 seed 未找到，直接创建新节点并移动一项
			RNode* group2 = new RNode(Envelope());
			//group2->parent = node->parent;//=============

			// initialize group1 as node (we will rebuild both)
			std::vector<Feature> g1_feats; std::vector<Feature> g2_feats;
			g1_feats.push_back(node->getFeature(seed1)); assigned[seed1] = true;
			g2_feats.push_back(node->getFeature(seed2)); assigned[seed2] = true;
			Envelope g1_box = g1_feats[0].getEnvelope();
			Envelope g2_box = g2_feats[0].getEnvelope();
			int remain = N - 2;
			while (remain > 0) {
				// forced assignment to satisfy minChildren
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

				// pick entry with greatest difference in enlargement
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
				if (bestIdx < 0) { // 防御：若未找到，随便取一个未分配的
					for (int i = 0; i < N; ++i) if (!assigned[i]) { bestIdx = i; break; }
					if (bestIdx < 0) break;
				}
				// assign
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

			// replace node's features with g1, create group2 with g2
			node->features = std::move(g1_feats);
			node->recalcBBox();
			group2->features = std::move(g2_feats);
			group2->recalcBBox();
			assert(N == static_cast<int>(node->getFeatureNum()) + static_cast<int>(group2->getFeatureNum()));
			//group2->parent = node->parent;
			return group2;
		}
		else {
			// internal node: split children vector
			//int N = static_cast<int>(node->children.size());
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
				// fallback: assign first to group1, second to group2 if possible
				seed1 = 0; seed2 = 1;
			}
			RNode* group2 = new RNode(Envelope());
			//group2->parent = node->parent;//==================
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
					for (int i = 0; i < N; ++i) if (!assigned[i]) { g1_children.push_back(node->children[i]); assigned[i] = true; --remain; }
					break;
				}
				if (need2 == remain) {
					for (int i = 0; i < N; ++i) if (!assigned[i]) { g2_children.push_back(node->children[i]); assigned[i] = true; --remain; }
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
			// set node children = g1_children, group2 children = g2_children
			node->children = std::move(g1_children);
			node->childrenNum = static_cast<int>(node->children.size());
			for (int i = 0; i < node->childrenNum; ++i) if (node->children[i]) node->children[i]->parent = node;
			node->recalcBBox();
			//node->recalcBBox();
			group2->children = std::move(g2_children);
			group2->childrenNum = static_cast<int>(group2->children.size());
			// fix parent pointers

			for (int i = 0; i < group2->childrenNum; ++i) if (group2->children[i]) group2->children[i]->parent = group2;

			//group2->parent = node->parent;

			group2->recalcBBox();
			return group2;
		}
	}

	void RTree::updateTree(RNode* n, RNode* nn) {
		if (n == root) {
			// create new root
			Envelope newRootBox = n->getEnvelope().unionEnvelope(nn->getEnvelope());
			RNode* newRoot = new RNode(newRootBox);
			newRoot->add(n);
			newRoot->add(nn);
			// add 已设置 n->parent/nn->parent
			root = newRoot;
			return;
		}
		RNode* parent = n->getParent();
		if (!parent) {
			// 防御：若 parent 为空则把新节点提升为 root
			Envelope newRootBox = n->getEnvelope().unionEnvelope(nn->getEnvelope());
			RNode* newRoot = new RNode(newRootBox);
			newRoot->add(n);
			newRoot->add(nn);
			root = newRoot;
			return;
		}

		// 将 nn 加入 parent
		parent->add(nn);
		parent->recalcBBox();
		// 如果超出 capacity，则分裂 parent
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
		if (root) { deleteSubtree(root); root = nullptr; }

		if (features.empty()) { root = nullptr; return true; }

		std::vector<Feature> shuffled = features;
		std::mt19937_64 rng(std::random_device{}());
		std::shuffle(shuffled.begin(), shuffled.end(), rng);

		for (const Feature& f : features) insertFeature(f);

		if (root) root->recalcBBox();

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

		// 初始 minDist：根包围盒的较大边长（保守）
		double minDist = std::max(rootEnv.getWidth(), rootEnv.getHeight());

		// 尝试用包含点的叶节点或其子节点快速缩小 minDist
		RNode* pNode = root->pointInLeafNode(x, y);
		if (!pNode) { // 保护性检查（若实现保证不为空可去掉）
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
				// 用子节点包围盒的 maxDistance2Envelope（假设 RNode 提供该接口），
				// 否则退回为遍历其 features（但那更慢）
				// 这里仍使用 child->getFeatureNum 遍历 features 保持通用性
				size_t fn = child->getFeatureNum();
				for (size_t j = 0; j < fn; ++j) {
					minDist = std::min(minDist, child->getFeature(j).maxDistance2Envelope(x, y));
					if (minDist <= 0.0) break;
				}
				if (minDist <= 0.0) break;
			}
		}

		// 若 minDist 非负（通常是），构造查询矩形并执行 rangeQuery
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

		// 两叶：逐对比较（外层选较小集合）

	}

} // namespace hw6

