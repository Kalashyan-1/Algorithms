#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

#include "../includes.h"

namespace ch {

    /**
     * @struct Point
     * Represents a 2D point with x and y coordinates.
     */
    struct Point {
        Point(double x, double y);
        void setX(double x);
        void setY(double y);
        double getX() const;
        double getY() const;
        bool operator<(const Point& other);
        bool operator==(const Point& other) const;
    private:
        double x;  
        double y;  
    };
    
    /**
     * @class ConvexHull
     * Provides methods for computing the convex hull of a set of points and visualization.
     */
    class ConvexHull {
    public:
        ConvexHull() = default;
        ~ConvexHull() = default;

        /**
         * Computes the convex hull of a set of points using the Graham Scan algorithm.
         *
         * @param points A vector of Point objects representing the input points.
         * @return A vector of Point objects representing the vertices of the convex hull.
         *
         * Complexity: O(n*log(n)) where n is the number of input points.
         */
        std::vector<Point> convexHullGrahamScan(std::vector<Point>& points);

        /**
         * Computes the convex hull of a set of points using the Jarvis March algorithm (Gift Wrapping).
         *
         * @param points A vector of Point objects representing the input points.
         * @return A vector of Point objects representing the vertices of the convex hull.
         *
         * Complexity: O(n*h) where n is the number of input points, and h is the number of vertices in the convex hull.
         */
        std::vector<Point> convexHullJarvisMarch(std::vector<Point>& points);

        /**
         * Visualizes the convex hull in a graphical form.
         */
        void visualize();

        /**
         * Benchmarks the execution time of a convex hull algorithm for a given set of points.
         *
         * @param algorithm A function pointer to the convex hull algorithm to benchmark.
         * @param points A vector of Point objects representing the input points.
         */
        void benchmarkConvexHullAlgorithm(std::function<std::vector<Point>(std::vector<Point>&)> algorithm, std::vector<Point>& points);

        /**
         * Deserializes a JSON object into a vector of Point objects.
         *
         * @param json The JSON data to deserialize.
         * @param key The key within the JSON object where the points are located.
         * @return A vector of Point objects.
         *
         * Complexity: O(n) where n is the number of points in the JSON data.
         */
        std::vector<Point> deserialize(const nlohmann::json&, const std::string&);
  
    private:
        /**
         * Calculates the cross product of three points (p1, p2, p3).
         *
         * @param p1 The first point.
         * @param p2 The second point.
         * @param p3 The third point.
         * @return The cross product value.
         */
        double crossProduct(const Point& p1, const Point& p2, const Point& p3);

        /**
         * Initializes the ConvexHull object with a set of input points.
         *
         * @param points A vector of Point objects representing the input points.
         */
        void setUp(std::vector<Point>& points);

        /**
         * Draws the convex hull and the input points.
         *
         * @param p A vector of Point objects representing the vertices of the convex hull.
         */
        void draw(std::vector<Point>& p);
    };
}

#endif  // CONVEX_HULL_H
