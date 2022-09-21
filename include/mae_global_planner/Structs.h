#ifndef STRUCTS_H
#define STRUCTS_H
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
struct Point
{
    /**
     * @brief Simple Point struct with overloaded operators
     */

    float x;
    float y;
    float z;

    Point(){};

    Point(float x, float y, float z) : x(x), y(y), z(z) {}

    inline float dist2(const Point &p) const
    {
        return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2));
    }
    inline float dist3(const Point &p) const
    {
        return sqrt(pow(x - p.x, 2) + pow(y - p.y, 2) + pow(z - p.z, 2));
    }

    inline bool operator==(const Point &p) const
    {
        float thr = 0.01;
        return (fabs(x - p.x) < thr) && (fabs(y - p.y) < thr) && (fabs(z - p.z) < thr);
    }

    inline bool operator!=(const Point &p) const
    {
        return !(*this == p);
    }
};

struct Path
{
    /**
     * @brief Struct that represents a single Subtour and exposes some useful methods
     *        for its manipulation and overloads operatos for common operations.
     *         It also keeps track of the path's length when it is changed.
     */
private:
    std::vector<Point>
        points_;
    float length_;

public:
    Path(const std::vector<Point> &points) : points_(points), length_(0.0f)
    {
        calculateLength();
    }

    Path() : points_(std::vector<Point>()), length_(0.0f) {}

    inline void calculateLength()
    {
        length_ = 0;
        if (points_.size() > 1)
        {
            for (int i = 0; i < points_.size() - 1; i++)
            {
                length_ += points_[i].dist3(points_[i + 1]);
            }
        }
    }

    inline void updatePath(const std::vector<Point> &points)
    {
        points_ = points;
        calculateLength();
    }

    inline void swapPathPoints(int index1, int index2)
    {
        std::swap(points_[index1], points_[index2]);
        calculateLength();
    }

    inline Point getPoint(int index) const
    {
        return points_[index];
    }

    inline void setPoint(int index, Point point)
    {
        points_[index] = point;
        calculateLength();
    }

    inline const std::vector<Point> &getPoints() const
    {
        return points_;
    }

    inline float getLength() const
    {
        return length_;
    }

    inline int getSize() const
    {
        return points_.size();
    }

    inline Path getSubPath(int start, int end) const
    {
        std::vector<Point> subpath = {};
        for (int i = start; i < end; i++)
        {
            subpath.push_back(points_[i]);
        }
        return Path(subpath);
    }

    inline bool operator>(const Path &p) const
    {
        return this->length_ > p.length_;
    }

    inline bool operator<(const Path &p) const
    {
        return this->length_ < p.length_;
    }

    inline bool operator==(const Path &p) const
    {
        for (int i = 0; i < points_.size(); i++)
        {
            if (points_[i] != p.points_[i])
            {
                return false;
            }
        }
        return true;
    }

    inline bool operator!=(const Path &p) const
    {
        return !(*this == p);
    }

    inline Path operator+(const Path &p) const
    {
        std::vector<Point> new_points = points_;
        new_points.insert(new_points.end(), p.points_.begin(), p.points_.end());
        return Path(new_points);
    }

    inline bool operator==(const std::vector<Point> &p) const
    {
        for (int i = 0; i < points_.size(); i++)
        {
            if (points_[i] != p[i])
            {
                return false;
            }
        }
        return true;
    }
};

typedef struct Path Path;
typedef struct Point Point;
#endif // STRUCTS_H