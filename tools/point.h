#ifndef POINT_H
#define POINT_H

// PCL
#ifdef COMPILE_PCL
#include <pcl/point_types.h>
#endif

// Eigen
#include <Eigen/Dense>

// Local
#include "tools/exception.h"

template<typename T>
class Point
{
public:

    enum{
        X = 0,
        Y = 1,
        Z = 2
    };

    ~Point()
    {
        deallocate();
    }

    Point(const Eigen::Matrix<T,3,1>& point): m_owns_data(false)
    {
        allocate();
        fromEigen(point);
    }

    Point(const Point& other): m_owns_data(false)
    {
        allocate();
        set(other.x(),other.y(),other.z());
    }

    Point(): m_owns_data(false)
    {
        allocate();
        set(T(0),T(0),T(0));
    }

    Point(T const* data): m_owns_data(false)
    {
        allocate();
        set(data);
    }

    Point(T* data): m_owns_data(false)
    {
        set(data);
    }

    Point(T x, T y, T z): m_owns_data(false)
    {
        allocate();
        set(x,y,z);
    }

    T x() const
    {
        checkInitialized();
        return m_data[X];
    }

    T y() const
    {
        checkInitialized();
        return m_data[Y];
    }

    T z() const
    {
        checkInitialized();
        return m_data[Z];
    }

    void set(T x, T y, T z)
    {
        checkInitialized();
        m_data[X] = x;
        m_data[Y] = y;
        m_data[Z] = z;
    }

    void setX(T x)
    {
        checkInitialized();
        m_data[X] = x;
    }

    void setY(T y)
    {
        checkInitialized();
        m_data[Y] = y;
    }

    void setZ(T z)
    {
        checkInitialized();
        m_data[Z] = z;
    }

    void set(const Eigen::Matrix<T,3,1>& point)
    {
        checkInitialized();
        setX(point(0,0));
        setY(point(1,0));
        setZ(point(2,0));
    }

    void fromEigen(const Eigen::Matrix<T,3,1>& point)
    {
        set(point);
    }

    Eigen::Matrix<T,3,1> toEigen()
    {
        checkInitialized();
        return Eigen::Matrix<T,3,1>(x(),y(),z());
    }

    void set(T const* data)
    {
        allocate();
        for ( int i = 0; i< 3; i++ ) m_data[i] = data[i];
    }

    void set(T* data)
    {
        deallocate();
        m_data = data;
    }

    T mod2() const
    {
        checkInitialized();
        T result(0);
        result += pow(x(),2);
        result += pow(y(),2);
        result += pow(z(),2);
        return result;
    }

    T mod() const
    {
        T md2 = mod2();
        if (md2 == T(0))
            return T(0);
        else
            return sqrt(mod2());
    }

    Point cross(const Point& other) const
    {
        T x = this->y()*other.z() - this->z()*other.y();
        T y = this->z()*other.x() - this->x()*other.z();
        T z = this->x()*other.y() - this->y()*other.x();
        return Point(x,y,z);
    }

    T dot(const Point& other) const
    {
        T result(0);
        result += other.x()*this->x();
        result += other.y()*this->y();
        result += other.z()*this->z();
        return result;
    }

    Point& operator+=(const Point& rhs)
    {
        checkInitialized();
        m_data[X] += rhs.x();
        m_data[Y] += rhs.y();
        m_data[Z] += rhs.z();
        return *this;
    }

    friend Point& operator+(Point& lhs, const Point& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    friend Point operator+(const Point& lhs, const Point& rhs)
    {
        Point l(lhs);
        l += rhs;
        return l;
    }

    Point& operator-=(const Point& rhs)
    {
        checkInitialized();
        m_data[X] -= rhs.x();
        m_data[Y] -= rhs.y();
        m_data[Z] -= rhs.z();
        return *this;
    }

    friend Point& operator-(Point& lhs, const Point& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    friend Point operator-(const Point& lhs, const Point& rhs)
    {
        Point l(lhs);
        l -= rhs;
        return l;
    }

    T* data()
    {
        checkInitialized();
        return m_data;
    }

    Point& operator*=(const T& scale)
    {
        checkInitialized();
        m_data[X] *= scale;
        m_data[Y] *= scale;
        m_data[Z] *= scale;
        return *this;
    }

    friend Point operator*(T scale, const Point& rhs)
    {
        Point out(rhs);
        out *= scale;
        return out;
    }

    friend Point operator*(const Point& lhs, T scale)
    {
        Point out(lhs);
        out *= scale;
        return out;
    }

    Point& operator/=(const T& scale)
    {
        checkInitialized();
        m_data[X] /= scale;
        m_data[Y] /= scale;
        m_data[Z] /= scale;
        return *this;
    }

    friend Point operator/(const Point& lhs, T scale)
    {
        Point out(lhs);
        out /= scale;
        return out;
    }

    Point& operator=(const Point& other)
    {
        if (this == &other)
            return *this;
        set(other.x(),other.y(),other.z());
        return *this;
    }

    Point normalize() const
    {
        return *this/this->mod();
    }

#ifdef COMPILE_PCL
    pcl::PointXYZ pcl()
    {
        return pcl::PointXYZ(x(),y(),z());
    }
#endif

protected:

    void checkInitialized() const
    {
        if (m_data == NULL)
            EXCEPTION("Point data has not been initialized")
    }

    void deallocate()
    {
        if (m_owns_data)
            delete[] m_data;
    }

    void allocate()
    {
        deallocate();
        m_data = new T[6];
        m_owns_data = true;
    }

private:
    T* m_data;
    bool m_owns_data;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Point<T>& p)
{
    out<<"["<<p.x()<<","<<p.y()<<","<<p.z()<<"]";
    return out;
}

template<typename T>
std::istream& operator>>(std::istream& in, Point<T>& point)
{
    double x,y,z;
    in>>x>>y>>z;
    point.setX(x);
    point.setY(y);
    point.setZ(z);
    return in;
}

#endif // POINT_H
