#include "quad_tree_map.hpp"

int main() {
    QuadTreeMap qtm(0.02,0.02,0.8,0.8,40,80);
    qtm.loadMatFromDir("images");
    qtm.addToTree();
    cv::Mat mat;
    qtm.toCvMat(mat);
    qtm.printTree();
    cv::imwrite("output.png", mat);
    // std::vector<std::vector<int>> matrix(6, std::vector<int>(6,-1));
    // for (int i=0; i<6; i++) {
    //     for (int j=0; j<6; j++) {
    //         if (i==j || (i>2 && j<2))
    //             matrix[i][j] = 6;
    //     }
    // }
    // QuadTree qt(0,0,3,3,0.5,0.5);
    // qt.printMatrix(matrix);
    // qt.fromMatrix(qt.getRoot(),matrix);
    // debugFile << "----------" << std::endl;
    // std::vector<std::vector<int>> newMatrix(6, std::vector<int>(6,-2));
    // qt.toMatrix(qt.getRoot(), newMatrix);
    // qt.printMatrix(newMatrix);
    // debugFile << "==========" << std::endl;
    // Point point(1.6,1.6,5);
    // qt.tryInsert(qt.getRoot(),point);
    // std::vector<std::vector<int>> newerMatrix(12, std::vector<int>(12,-3));
    // qt.toMatrix(qt.getRoot(), newerMatrix);
    // qt.printMatrix(newerMatrix);
    return 0;
}