#include <cstdint>	
#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <tuple>
#include <queue>
#include <vector>
#include <fstream>	
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <functional>
#include <opencv2/opencv.hpp>

/*
for cv mat coordinate, origins at the top left corner, x-right y-down
for global coordinate, its origin is at the first BEV bottom middle, x-forward, y-left
*/

struct Point {
    double x, y;
    static const int invalidVal = 255;
    int val=invalidVal;
    Point() {}
    Point(double _x, double _y, int _val=Point::invalidVal) : x(_x), y(_y), val(_val) {}
    bool operator == (const Point& _point) {
        return x==_point.x && y==_point.y;
    }
};

struct Box {
    double x, y, w, h;
    Box() {};
    Box(double _x, double _y, double _h, double _w) : 
        x(_x), y(_y), h(_h), w(_w){}
    bool isOverLappedAABB(const Box& _box) const {
        return this->x - this->h/2.0 < _box.x + _box.h/2.0 && \
               this->x + this->h/2.0 > _box.x - _box.h/2.0 && \
               this->y - this->w/2.0 < _box.y + _box.w/2.0 && \
               this->y + this->w/2.0 > _box.y - _box.w/2.0;
    }
    bool isContainingAABB(const Point& _point) const {
        return this->x - this->h/2.0 <  _point.x && \
               this->x + this->h/2.0 >= _point.x && \
               this->y - this->w/2.0 <  _point.y && \
               this->y + this->w/2.0 >= _point.y;
    }
};

class QuadTree {
public:
	struct Node
	{
        int                                     depth       = 0;
        bool                                    leafFlag    = true;
        Box                                     bound       = Box();
        Point                                   point       = Point();
		std::shared_ptr<Node>                   children[4] = {nullptr, nullptr, nullptr, nullptr}; // NW, NE, SW, SE
        Node() = delete;
        Node(bool _leafFlag, int _depth, Box _box) : leafFlag(_leafFlag), depth(_depth), bound(_box) {}
	};
private:
    std::shared_ptr<Node>         sptr_root_;
    double                        x_, y_, h_, w_, cellH_, cellW_;
    int                           gridNumH_, gridNumW_;
    int                           maxDepth_;
public:
    QuadTree(double _x=0.0, double _y=0.0, double _h=2.0, double _w=2.0, double _cellH=0.5, double _cellW=0.5) : \
    x_(_x), y_(_y), h_(_h), w_(_w), cellH_(_cellH), cellW_(_cellW){
        assert(cellH_>0&&cellW_>0);
        gridNumH_     = std::ceil(h_ / cellH_);
        gridNumW_     = std::ceil(w_ / cellW_);
        sptr_root_    = std::make_shared<Node>(true, 0, Box(x_, y_, h_, w_));
        double ratio  = std::max(gridNumH_, gridNumW_);
        maxDepth_     = std::ceil(std::log2(ratio));
    }
    std::shared_ptr<Node> getRoot() const { return sptr_root_; }
    int getMaxDepth() const { return maxDepth_; }
    std::tuple<double,double,double,double> getBoundary() const {return std::tuple<double,double,double,double>{x_, y_, h_, w_};}
    std::pair<double,double> getCell() const {return std::pair<double,double>{cellH_,cellW_};}
    std::pair<int,int> getGridNum() const {return std::pair<int,int>{gridNumH_,gridNumW_};}
    void clearChildren(std::shared_ptr<Node> _sptr_node) {
        for (int i=0; i<4; i++) {
            _sptr_node->children[i].reset();
        }
        _sptr_node->leafFlag = true;
    }
    std::array<Box,4> generateSubBoxes(std::shared_ptr<Node> _sptr_node) {
        int numH = std::ceil(_sptr_node->bound.h / cellH_);
        int numW = std::ceil(_sptr_node->bound.w / cellW_);
        double upH    = double(numH / 2) * cellH_;
        double downH  = _sptr_node->bound.h - upH;
        double leftW  = double(numW / 2) * cellW_;
        double rightW = _sptr_node->bound.w - leftW;
        return  { Box(_sptr_node->bound.x + upH/2.0,    _sptr_node->bound.y + leftW/2.0,  downH, rightW),
                  Box(_sptr_node->bound.x + upH/2.0,    _sptr_node->bound.y - rightW/2.0, downH, leftW),
                  Box(_sptr_node->bound.x - downH/2.0,  _sptr_node->bound.y + leftW/2.0,  upH, rightW),
                  Box(_sptr_node->bound.x - downH/2.0,  _sptr_node->bound.y - rightW/2.0, upH, leftW)};
    }
    bool trySplit(std::shared_ptr<Node> _sptr_node) {
        if(!_sptr_node) return false;
        std::array<Box,4> boxes = generateSubBoxes(_sptr_node);
        if (_sptr_node->depth+1<=maxDepth_) {
            _sptr_node->leafFlag     = false;
            for (int i=0; i<4; i++) {
                if (!_sptr_node->children[i])
                    _sptr_node->children[i]  = std::make_shared<Node>(true, _sptr_node->depth + 1, boxes[i]);
            }
            return true;
        } else {
            return false;
        }
    }
    int sectionContainPoint(std::shared_ptr<Node> _sptr_node, const Point& _point) {
        if(!_sptr_node) return -100;
        if (_sptr_node->bound.isContainingAABB(_point)) {
            std::array<Box,4> boxes = generateSubBoxes(_sptr_node);
            for (int i=0; i<4; i++) {
                if (boxes[i].isContainingAABB(_point)) {
                    return i;
                }
            }
            return -1;
        } else {
            return -2;
        }
    }
    bool tryInsert(std::shared_ptr<Node> _sptr_node, const Point& _point) {
        if (!_sptr_node ) return false;
        int index = sectionContainPoint(_sptr_node, _point);
        if (index == -2) {        
            if (_sptr_node == sptr_root_) {
                tryExpand(_point);
                tryInsert(sptr_root_,_point);
                return true;
            }
            return false;
        } else if (index == -1) {
            _sptr_node->point = _point;
            return true;
        } else if (trySplit(_sptr_node)) {
                if (_sptr_node->children[index])
                    return tryInsert(_sptr_node->children[index], _point);
                else 
                    return false;
        } else {
            _sptr_node->point = _point;
            return true;
        }
    }
    void addDepthByOne(std::shared_ptr<Node> _sptr_node) {
        if (!_sptr_node) return;
        if (_sptr_node!=sptr_root_)
            _sptr_node->depth++;
        for (int i=0; i<4; i++) {
            if (_sptr_node->children[i]) addDepthByOne(_sptr_node->children[i]);
        }
    }
    bool tryExpand(const Point& _point) {
        if (sptr_root_->bound.isContainingAABB(_point)) {
            return false;
        } else {
            double deltaX = _point.x - sptr_root_->bound.x;
            double deltaY = _point.y - sptr_root_->bound.y;
            h_*=2.0;
            w_*=2.0;
            gridNumH_     = std::ceil(h_ / cellH_);
            gridNumW_     = std::ceil(w_ / cellH_);
            double ratio  = std::max(gridNumH_, gridNumW_);
            maxDepth_     = std::ceil(std::log2(ratio));
            if (deltaX > 0 && deltaY > 0) {
                x_+=(h_/4.0);
                y_+=(w_/4.0);
                std::shared_ptr<Node> sptr_newRoot = std::make_shared<Node>(false, 0, Box(x_, y_, h_, w_));
                sptr_newRoot->children[3]          = sptr_root_;
                sptr_root_                         = sptr_newRoot;
            } else if (deltaX > 0 && deltaY < 0) {
                x_+=(h_/4.0);
                y_-=(w_/4.0);
                std::shared_ptr<Node> sptr_newRoot = std::make_shared<Node>(false, 0, Box(x_, y_, h_, w_));
                sptr_newRoot->children[2]          = sptr_root_;
                sptr_root_                         = sptr_newRoot;
            } else if (deltaX < 0 && deltaY > 0) {
                x_-=(h_/4.0);
                y_+=(w_/4.0);
                std::shared_ptr<Node> sptr_newRoot = std::make_shared<Node>(false, 0, Box(x_, y_, h_, w_));
                sptr_newRoot->children[1]          = sptr_root_;
                sptr_root_                         = sptr_newRoot;
            } else if (deltaX < 0 && deltaY < 0) {
                x_-=(w_/4.0);
                y_-=(h_/4.0);
                std::shared_ptr<Node> sptr_newRoot = std::make_shared<Node>(false, 0, Box(x_, y_, h_, w_));
                sptr_newRoot->children[0]          = sptr_root_;
                sptr_root_                         = sptr_newRoot;
            }
            addDepthByOne(sptr_root_);
            return true;
        }
    }
    bool tryMerge(std::shared_ptr<Node> _sptr_node) {
        if (!_sptr_node || _sptr_node->leafFlag) return true;
        std::vector<int> indices;
        for (int i=0; i<4; i++) {
            tryMerge(_sptr_node->children[i]);
            if (_sptr_node->children[i])
                indices.push_back(i);
        }
        for (const auto& i : indices) {
            if (_sptr_node->children[i]->leafFlag && (_sptr_node->children[i]->point.val == _sptr_node->children[indices[0]]->point.val)) {
            } else {
                return false;
            }
        }

        _sptr_node->point.val = _sptr_node->children[indices[0]]->point.val;
        clearChildren(_sptr_node);
        return true;
    }
    void getVal(std::shared_ptr<Node> _sptr_node, Point& _point) {
        if (_sptr_node->bound.isContainingAABB(_point) && _sptr_node->leafFlag) {
            _point.val = _sptr_node->point.val;
        } else {
            for (int i=0; i<4; i++) {
                if (_sptr_node->children[i])
                    getVal(_sptr_node->children[i], _point);
            }
        }
    }
    void query(std::shared_ptr<Node> _sptr_node, const Box& _box, std::vector<Point>& _points) {
        if (!_sptr_node->bound.isOverLappedAABB(_box)) return;
        if (_box.isContainingAABB(_sptr_node->point)) {
            _points.push_back(_sptr_node->point);
        }
        for (int i=0; i<4; i++) {
            if (_sptr_node->children[i]) {
                query(_sptr_node->children[i], _box, _points);
            }
        }
    }
    bool tryRemove(std::shared_ptr<Node> _sptr_node, const Point& _point) {
        if (!_sptr_node) return false;
        if (_sptr_node->point == _point) {
            _sptr_node->point.val = 0;
            return true;
        } else {
            for (int i=0; i<4; i++) {
                if (_sptr_node->children[i] && tryRemove(_sptr_node->children[i], _point)) {
                    tryMerge(_sptr_node);
                    return true;
                }     
            }
        }
        return false;
    }
    void toMatrix(std::shared_ptr<Node> _sptr_node, std::vector<std::vector<int>>& _matrix, bool searchFlag=false) {
        if (!_sptr_node) return;
        if (searchFlag) {
            for (int i=0; i<_matrix.size(); i++) {
                for (int j=0; j<_matrix[i].size(); j++) {
                    Point point = Point(-(double(i)*cellH_ - (x_+h_/2.0)) - cellH_/2.0,  -(double(j)*cellW_ - (y_+w_/2.0)) - cellW_/2.0, _matrix[i][j]);
                    getVal(_sptr_node, point);
                    _matrix[i][j] = point.val;
                }
            }
        } else {
            if (!_sptr_node->leafFlag) {
                for (int i = 0; i < 4; i++) {
                    toMatrix(_sptr_node->children[i], _matrix);
                } 
            } else if (_sptr_node->bound.w>1e-6 && _sptr_node->bound.h>1e-6){
                int indexX = std::round(((x_ + h_/2.0) - (_sptr_node->bound.x + _sptr_node->bound.h/2.0)) / cellH_);
                int indexY = std::round(((y_ + w_/2.0) - (_sptr_node->bound.y + _sptr_node->bound.w/2.0)) / cellW_);
                _matrix[indexX][indexY] = static_cast<unsigned char>(_sptr_node->point.val);  
            }
        } 
    }
    void fromMatrix(std::shared_ptr<Node> _sptr_node, const std::vector<std::vector<int>>& _matrix) {
        for (int i=0; i<_matrix.size(); i++) {
            for (int j=0; j<_matrix[i].size(); j++) {
                Point point = Point(-(double(i)*cellH_ - (x_+h_/2.0)) - cellH_/2.0,  -(double(j)*cellW_ - (y_+w_/2.0)) - cellW_/2.0, _matrix[i][j]);
                tryInsert(_sptr_node, point);
            }
        }
    }
    void toCvMat(std::shared_ptr<Node> _sptr_node, cv::Mat& _matrix) {
        if (!_sptr_node) return;
        if (!_sptr_node->leafFlag) {
            for (int i = 0; i < 4; i++) {
                toCvMat(_sptr_node->children[i], _matrix);
            } 
        } else if (_sptr_node->bound.w>1e-5 && _sptr_node->bound.h>1e-5){
            int indexX = std::round(((x_ + h_/2.0) - (_sptr_node->bound.x + _sptr_node->bound.h/2.0)) / cellH_);
            int indexY = std::round(((y_ + w_/2.0) - (_sptr_node->bound.y + _sptr_node->bound.w/2.0)) / cellW_);
            _matrix.at<unsigned char>(indexX,indexY) = static_cast<unsigned char>(_sptr_node->point.val);  
        }
    }
};

class QuadTreeMap {
    private:
    std::mutex                     pointQueueMutex_;
    std::queue<std::vector<Point>> pointsQueue_;
    double                         resX_, resY_, robotOriginInBevX_, robotOriginInBevY_;
    int                            matRow_, matCol_;
    QuadTree                       quadtree_;
    public:
    QuadTreeMap(double _resX, double _resY, double _robotOriginInBevX, double _robotOriginInBevY, int _matRow, int _matCol) : \
    resX_(_resX), resY_(_resY), robotOriginInBevX_(_robotOriginInBevX), robotOriginInBevY_(_robotOriginInBevY), matRow_(_matRow), matCol_(_matCol) {
        quadtree_ = QuadTree(double(matRow_)*resX_/2.0, 0.0, double(matRow_)*resX_, double(matCol_)*resY_, resX_, resY_);
    }
    void extractPointsFromImage(const cv::Mat& _mat, std::vector<Point>& _points) {
        _points.clear();
        for (int i=0; i<_mat.rows; i++) {
            for (int j=0; j<_mat.cols; j++) {
                _points.push_back(Point(-double(i)*resX_-resX_/2.0 + robotOriginInBevX_, \
                                        -double(j)*resY_-resY_/2.0 + robotOriginInBevY_, \
                                        static_cast<int>(_mat.at<unsigned char>(i,j))));
            }
        }
    }
    void addToTree() {
        std::lock_guard<std::mutex> lock(pointQueueMutex_);
        while(!pointsQueue_.empty()) {
            std::vector<Point> points = pointsQueue_.front();
            for (const auto& point : points) {
                quadtree_.tryInsert(quadtree_.getRoot(), point);
            }
            pointsQueue_.pop();
        }
        quadtree_.tryMerge(quadtree_.getRoot());
    }
    void toCvMat(cv::Mat& _mat) {
        std::pair<int,int> gridNum = quadtree_.getGridNum();
        int gridNumX=gridNum.first, gridNumY=gridNum.second;
        _mat = cv::Mat(gridNumX, gridNumY, CV_8UC1);
        quadtree_.toCvMat(quadtree_.getRoot(), _mat);
    }
    
    // ------------------- for debug ------------------------------------------------------------//
    void readMatFromFile(const std::string& _file) {
        std::lock_guard<std::mutex> lock(pointQueueMutex_);
        cv::Mat img_gray = cv::imread(_file, cv::IMREAD_GRAYSCALE);
        std::vector<Point> points;
        extractPointsFromImage(img_gray,points);
        pointsQueue_.push(points);
    }
    void loadMatFromDir(const std::string& _dir) {
        if (!std::filesystem::exists(_dir) || !std::filesystem::is_directory(_dir)) {
            return;
        }
        for (const auto& entry : std::filesystem::directory_iterator(_dir)) {
            if (entry.is_regular_file()) {
                readMatFromFile("./" + _dir + "/" +entry.path().filename().string());
            }
        }
    }
    void printTree() {
        std::pair<int,int> gridNum = quadtree_.getGridNum();
        int gridNumX=gridNum.first, gridNumY=gridNum.second;
        std::vector<std::vector<int>> matrix(gridNumX,std::vector<int>(gridNumY,0));
        quadtree_.toMatrix(quadtree_.getRoot(), matrix);
        std::ofstream file("output.log");
        for (const auto& vec: matrix) {
            for (const auto& d : vec) {
                file<<static_cast<unsigned char>(d);
            }
            file<<std::endl;
        }
    }
};
