#include "../../include/ConvexHullHeader/convexHull.h"

ch::Point::Point(double X, double Y) 
    : x{X}, y{Y} {} 

bool ch::Point::operator<(const ch::Point& other) {
    return (x < other.getX()) || (x == other.getX() && y < other.y);
}

bool ch::Point::operator==(const Point& other) const {
    return (x == other.x) && (y == other.y);
}

void ch::Point::setX(double X) {
    x = X;
}

void ch::Point::setY(double Y) {
    y = Y;
}

double ch::Point::getX() const {
    return x;
}
double ch::Point::getY() const {
    return y;
}


std::vector<ch::Point> ch::ConvexHull::convexHullGrahamScan(std::vector<Point>& points) {
    int n = points.size();
    if (n < 3) {
        throw std::runtime_error("The set of points should contain at least 3 points.");
    }
    int lowest = 0;
    for (int i = 0; i < n; ++i) {
        if (points[i] < points[lowest]) {
            lowest = i;
        }
    }
    std::swap(points[lowest], points[0]);
    std::sort(points.begin(), points.end(), [&](const Point& p1, const Point& p2) {
        double ang1 = atan2(p1.getY() - points[0].getY(), p1.getX() - points[0].getX());
        double ang2 = atan2(p2.getY() - points[0].getY(), p2.getX() - points[0].getX());
        return ang1 < ang2;
    });

    std::vector<Point> hull;
    hull.push_back(points[0]);
    hull.push_back(points[1]);
    hull.push_back(points[2]);

    for (int i = 3; i < n; ++i) {
        while (crossProduct(hull[hull.size() - 2], hull[hull.size() - 1], points[i]) <= 0) {
            hull.pop_back();
        }
        hull.push_back(points[i]);
    }
    return hull;
}

double ch::ConvexHull::crossProduct(const Point& p1, const Point& p2, const Point& p3) {
        return (p2.getX() - p1.getX()) * (p3.getY() - p1.getY()) - (p2.getY() - p1.getY()) * (p3.getX() - p1.getX());
}


std::vector<ch::Point> ch::ConvexHull::convexHullJarvisMarch(std::vector<Point>& points) {
     int n = points.size();
    if (n < 3) {
        throw std::runtime_error("The set of points should contain at least 3 points.");
    }

    int startPoint = 0;
    for (int i = 1; i < n; ++i) {
        if (points[i] < points[startPoint]) {
            startPoint = i;
        }
    }

    int currentPoint = startPoint;
    std::vector<ch::Point> hull;

    do {
        hull.push_back(points[currentPoint]);
        int nextPoint = (currentPoint + 1) % n; 

        for (int i = 0; i < n; ++i) {
            if (crossProduct(points[currentPoint], points[i], points[nextPoint]) < 0) {
                nextPoint = i;
            }
        }
        currentPoint = nextPoint;
    } while (currentPoint != startPoint);
    return hull;
}



void ch::ConvexHull::setUp(std::vector<Point>& points) {

    cv::Mat canvas = cv::Mat::zeros(500, 500, CV_8UC3);
    int num = 20;

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> x_dist(num, canvas.cols - num);
    std::uniform_int_distribution<int> y_dist(num, canvas.rows - num);

    for (int i = 0; i < 50; i++) {
        double x = x_dist(gen);
        double y = y_dist(gen);
        points.push_back({x, y});
    }

    int n = points.size();
    if (n < 3) {
        throw std::runtime_error("The set of points should contain at least 3 points.");
    }
   
    std::sort(points.begin(), points.end(), [&](const Point& p1, const Point& p2) {
        return p1.getX() < p2.getX();
    });

    cv::imshow("Convex Hull", canvas);
    cv::waitKey(1);
}


void ch::ConvexHull::draw(std::vector<Point>& points) {
    bool flag = false;
    bool flag2 = false;

    std::vector<Point> hull;
    Point lowest = points[0];
    Point curr = lowest;
    hull.push_back(curr);
    Point next = points[1];
    int currInd = 2;

    while (!flag2) {
        cv::Mat canvas = cv::Mat::zeros(500, 500, CV_8UC3);

        for (const Point& p : points) {
            cv::circle(canvas, cv::Point(p.getX(), p.getY()), 1, cv::Scalar(255, 255, 255), cv::FILLED);
        }

        for (size_t j = 0; j < hull.size(); ++j) {
            cv::line(canvas, cv::Point(hull[j].getX(), hull[j].getY()), cv::Point(hull[(j + 1) % hull.size()].getX(), hull[(j + 1) % hull.size()].getY()), cv::Scalar(0, 0, 255), 1);
        }

        cv::circle(canvas, cv::Point(lowest.getX(), lowest.getY()), 2, cv::Scalar(0, 255, 0), cv::FILLED);
        
        cv::circle(canvas, cv::Point(curr.getX(), curr.getY()), 2, cv::Scalar(200, 0, 255), cv::FILLED);

        cv::line(canvas, cv::Point(curr.getX(), curr.getY()), cv::Point(next.getX(), next.getY()), cv::Scalar(0, 255, 0), 1);

        Point checking = points[currInd];
        cv::line(canvas, cv::Point(curr.getX(), curr.getY()), cv::Point(checking.getX(), checking.getY()), cv::Scalar(255, 255, 255), 1);

        double cross = crossProduct(curr, next, checking);

        if (cross < 0) {
            next = checking;
        }

        if (flag){
            flag2 = true;
        }

        currInd = currInd + 1;
        if (currInd == points.size() ) {
            if (next.getX() == lowest.getX() && next.getY() == lowest.getY()) {
                std::cout << "done" << std::endl;
                flag = true;
            }
            hull.push_back(next);
            curr = next;
            currInd = 0;
            next = lowest;
        }

        cv::imshow("Convex Hull", canvas);
        cv::waitKey(1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


void ch::ConvexHull::visualize() {
    std::vector<Point> points;
    setUp(points);
    draw(points);
    cv::waitKey(0);
}

void ch::ConvexHull::benchmarkConvexHullAlgorithm(std::function<std::vector<Point>(std::vector<Point>&)> algorithm,std::vector<Point>& points) {
    auto start = std::chrono::steady_clock::now();
    std::vector<Point> convexHull = algorithm(points);
    auto end = std::chrono::steady_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);

    std::cout << "Algorithm execution time: " << duration.count() << " microseconds\n";
}


std::vector<ch::Point> ch::ConvexHull::deserialize(const nlohmann::json& inputJson, const std::string& str) {
    std::vector<ch::Point> points;
    for (const auto& pointJson : inputJson[str]) {
        ch::Point point(pointJson["x"], pointJson["y"]);
        points.push_back(point);
    }
    return points;
}
