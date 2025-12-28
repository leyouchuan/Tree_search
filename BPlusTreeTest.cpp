#include "Common.h"
#include "BPlusTree.h"
#include <ctime>
#include "CMakeIn.h"

using namespace hw6;

extern int mode;
extern std::vector<Geometry*> readGeom(const char* filename);
extern std::vector<std::string> readName(const char* filename);
extern void transformValue(double& res, const char* format);
extern void wrongMessage(Envelope e1, Envelope e2, bool cal);
extern void wrongMessage(const Point& pt1, const Point& pt2, double dis,
    double res);
extern void wrongMessage(Envelope e1, Envelope e2, Envelope cal, Envelope res);

namespace hw6 {

    void BPlusTree::test(int t) {
        using namespace std;
        cout << "***************BPlusTree Test Start****************" << endl;

        if (t == 4) { // 对应 RTree 的 TEST4
            cout << "TEST4-BPlus: BPlusTree Construction" << endl;
            int ncase = 2, cct = 2;

            // 用例1：station 数据集
            {
                BPlusTree bpt(8, 16);
                vector<Geometry*> geom = readGeom(PROJ_SRC_DIR "/data/station");
                vector<Feature> features;
                features.reserve(geom.size());
                for (size_t i = 0; i < geom.size(); ++i) features.emplace_back("", geom[i]);

                bool ok = bpt.constructTree(features); // 若你的实现返回 bool
                (void)ok; // 忽略或可用于断言

                int height = 0, interiorNum = 0, leafNum = 0;
                bpt.countHeight(height);
                bpt.countNode(interiorNum, leafNum);

                cout << "BPlusTree: height=" << height
                    << " interior=" << interiorNum
                    << " leaf=" << leafNum << endl;

                if (height <= 0 || leafNum == 0) {
                    cout << "BPlusTree construction suspicious: height or leafNum invalid\n";
                    --cct;
                }

                for (size_t i = 0; i < geom.size(); ++i) delete geom[i];
                geom.clear();
            }

            // 用例2：highway 数据集
            {
                vector<Geometry*> geom2 = readGeom(PROJ_SRC_DIR "/data/highway");
                vector<Feature> features2;
                features2.reserve(geom2.size());

                BPlusTree bpt2(8, 16);
                for (size_t i = 0; i < geom2.size(); ++i) features2.emplace_back("", geom2[i]);

                bool ok2 = bpt2.constructTree(features2);
                (void)ok2;

                int height2 = 0, interiorNum2 = 0, leafNum2 = 0;
                bpt2.countHeight(height2);
                bpt2.countNode(interiorNum2, leafNum2);

                cout << "BPlusTree2: height=" << height2
                    << " interior=" << interiorNum2
                    << " leaf=" << leafNum2 << endl;
                if (height2 <= 0 || leafNum2 == 0) --cct;

                for (size_t i = 0; i < geom2.size(); ++i) delete geom2[i];
                geom2.clear();
            }

            cout << "BPlusTree Construction: " << cct << " / " << ncase << " tests passed" << endl;
        }
        else if (t == 8) {
            cout << "TEST8-BPlus: Analysis placeholder" << endl;
            // 可实现类似 RTree::analyse 的分析函数
        }

        cout << "***************BPlusTree Test End******************" << endl;
    }
    void forConstCapAnalyseBPlusTree(const std::vector<Feature>& features, int childNum, int maxNum, int step) {
        if (childNum <= maxNum) {
            BPlusTree rtree(childNum);
            rtree.constructTree(features);

            // TODO 与四叉树进行比较

            forConstCapAnalyseBPlusTree(features, childNum + step, maxNum, step);
        }
    }

    void BPlusTree::analyse() {
        using namespace std;

        vector<Feature> features;
        vector<Geometry*> geom = readGeom(PROJ_SRC_DIR "/data/taxi");
        vector<string> name = readName(PROJ_SRC_DIR "/data/taxi");

        features.clear();
        features.reserve(geom.size());
        for (size_t i = 0; i < geom.size(); ++i)
            features.push_back(Feature(name[i], geom[i]));

        cout << "taxi number: " << geom.size() << endl;

        srand(time(nullptr));

        /*TODO:实现forConstCapAnalyseRTree */
        forConstCapAnalyseBPlusTree(features, 70, 200, 10);
    }
} // namespace hw6