# Tree_search — 项目简介

这是一个基于空间索引结构（四叉树、R 树等）实现的空间查询与距离计算项目。目标是提供常用的空间数据操作函数与索引构建、查询功能。提供了一个示例。

主要功能

- 欧式距离计算

  - 计算 Point 到 LineString 的欧式距离
  - 计算 Point 到 Polygon 的欧式距离
- Envelope（包围盒）工具函数

  - contain：判断包含关系
  - intersect：判断相交关系
  - unionEnvelope：合并包围盒
- 四叉树（QuadTree / QuadNode）

  - 构建四叉树（constructQuadTree）
  - 节点划分（split）
  - 范围查询（rangeQuery）
  - 最近邻查询（NNQuery）
  - 叶节点点查询（pointInLeafNode）
  - 空间连接（SpatialJoin）
- R-树（R-Tree）

  - 构建 R 树及相关函数
  - 范围查询、最近邻查询
  - 叶节点点查询、空间连接
- 精炼步

  - 对查询结果的精化（refinement）实现
  - rangeQuery、NNQuery 的实现与优化
- 扩展

  - 分层空间数据类型与结构完善
  - 多边形数据的空间查询支持
  - 基于空间填充曲线的 B+Tree 实现

使用说明

- 源代码按功能模块组织，包含四叉树、R 树、距离计算与工具类实现。
- 编译与运行：在项目根目录使用常见 C++ 编译工具（例如 g++ / Visual Studio）编译相应源文件，或根据提供的 Makefile / 项目文件构建。
- 示例：仓库内含示例调用（或测试用例）展示如何构建索引、插入数据并执行 range / NN / SpatialJoin 查询。
