#ifndef POINT2D_H
#define POINT2D_H

// STD library
#include <sstream>

template<typename T>
class Point2d
{
public:
    Point2d(T x, T y)
    {
        m_data[0] = x;
        m_data[1] = y;
    }

    Point2d(T data[2])
    {
        m_data[0] = data[0];
        m_data[1] = data[1];
    }

    Point2d(const Point2d& other):
        Point2d(other.x(),other.y())
    {

    }

    T x() const
    {
        return m_data[0];
    }
    T y() const
    {
        return m_data[1];
    }

    Point2d& operator+=(const Point2d& right)
    {
        m_data[0] += right.x();
        m_data[1] += right.y();
        return *this;
    }

    friend Point2d operator+(Point2d lhs, const Point2d& rhs)
    {
      lhs += rhs;
      return lhs;
    }

    Point2d& operator-=(const Point2d& right)
    {
        m_data[0] -= right.x();
        m_data[1] -= right.y();
        return *this;
    }

    friend Point2d operator-(Point2d lhs, const Point2d& rhs)
    {
      lhs -= rhs;
      return lhs;
    }

    void distance(const Point2d& other, Point2d error) const
    {
        error = Point2d(other.x() - this->x(),other.y() - this->y());
    }

    T distance(const Point2d& other) const
    {
        return sqrt(pow(other.x() - this->x(),2) + pow(other.y() - this->y(),2));
    }

    template<typename P>
    friend std::ostream& operator<<(std::ostream& os, const Point2d<P>& obj);

    void copyTo(T* data) const
    {
        data[0] = m_data[0];
        data[1] = m_data[1];
    }

    T* getPtr()
    {
        return m_data;
    }

    void setX(T x)
    {
        m_data[0] = x;
    }

    void setY(T y)
    {
        m_data[1] = y;
    }

private:
    T m_data[2];
};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Point2d<T>& obj)
{
    os<<"("<<obj.x()<<","<<obj.y()<<")";
    return os;
}

#endif // POINT2D_H
