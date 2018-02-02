#ifndef LINE_H
#define LINE_H

#include <Eigen/Dense>

#ifdef COMPILE_PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#endif

#include "tools/exception.h"
#include "tools/point.h"
#include "tools/functions.h"

template<typename T>
class Line
{
public:

    enum{
        V_X = 0,
        V_Y = 1,
        V_Z = 2,
        P_X = 3,
        P_Y = 4,
        P_Z = 5
    };

    ~Line()
    {
        deallocate();
    }

    Line(): m_owns_data(false)
    {
        allocate();
        set(T(0),T(0),T(0),T(0),T(0),T(0));
    }

    Line(const Point<T>& vector, const Point<T>& point): m_owns_data(false)
    {
        allocate();
        setVector(vector);
        setPoint(point);
    }

    Line(const Line& other): m_owns_data(false)
    {
        allocate();
        set(other.vector(),other.point());
    }

    Line(T const* data): m_owns_data(false)
    {
        allocate();
        set(data);
    }

    Line(T* data): m_owns_data(false)
    {
        set(data);
    }

    Line(T v_x, T v_y, T v_z, T p_x, T p_y, T p_z): m_owns_data(false)
    {
        allocate();
        set(v_x,v_y,v_z,p_x,p_y,p_z);
    }

    Point<T> operator()(T t) const
    {
        Point<T> point = this->point();
        Point<T> vector = this->vector();
        Point<T> out = point + t*vector;
        return std::move(out);
    }

    void set(T v_x, T v_y, T v_z, T p_x, T p_y, T p_z)
    {
        setVector(v_x,v_y,v_z);
        setPoint(p_x,p_y,p_z);
    }

    void setVector(T v_x, T v_y, T v_z)
    {
        setVector(Point<T>(v_x,v_y,v_z));
    }

    void setPoint(T p_x, T p_y, T p_z)
    {
        setPoint(Point<T>(p_x,p_y,p_z));
    }

    void setVector(const Point<T>& vector)
    {
        checkInitialized();
        Point<T> v = vector.normalize();
        m_data[V_X] = v.x();
        m_data[V_Y] = v.y();
        m_data[V_Z] = v.z();
    }

    void setPoint(const Point<T>& point)
    {
        checkInitialized();
        m_data[P_X] = point.x();
        m_data[P_Y] = point.y();
        m_data[P_Z] = point.z();
    }

    void set(const Point<T>& vector, const Point<T>& point)
    {
        setVector(vector);
        setPoint(point);
    }

    void set(T const* data)
    {
        allocate();
        for ( int i = 0; i< 6; i++ ) m_data[i] = data[i];
    }

    void set(T* data)
    {
        deallocate();
        m_data = data;
    }

    Point<T> point() const
    {
        checkInitialized();
        T const* ptr = m_data+3;
        return Point<T>(ptr);
    }

    Point<T> vector() const
    {
        checkInitialized();
        T const* ptr = m_data;
        return Point<T>(ptr);
    }

    T* data()
    {
        checkInitialized();
        return m_data;
    }

    Line& operator+=(const Line& rhs)
    {
        checkInitialized();
        this->setVector(this->vector()+rhs.vector());
        this->setPoint(this->point()+rhs.point());
        return *this;
    }

    friend Line& operator+(Line& lhs, const Line& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    friend Line operator+(const Line& lhs, const Line& rhs)
    {
        Line l(lhs);
        l += rhs;
        return l;
    }

    Line& operator-=(const Line& rhs)
    {
        checkInitialized();
        this->setVector(this->vector()-rhs.vector());
        this->setPoint(this->point()-rhs.point());
        return *this;
    }

    friend Line& operator-(Line& lhs, const Line& rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    friend Line operator-(const Line& lhs, const Line& rhs)
    {
        Line l(lhs);
        l -= rhs;
        return l;
    }

    Line& operator*=(const T& scale)
    {
        checkInitialized();
        this->setVector(this->vector()*scale);
        this->setPoint(this->point()*scale);
        return *this;
    }

    friend Line operator*(T scale, const Line& rhs)
    {
        Line out(rhs);
        out *= scale;
        return out;
    }

    friend Line operator*(Line& lhs, T scale)
    {
        Line out(lhs);
        out *= scale;
        return out;
    }

    Line& operator/=(const T& scale)
    {
        checkInitialized();
        this->setVector(this->vector()/scale);
        this->setPoint(this->point()/scale);
        return *this;
    }

    friend Line operator/(const Line& lhs, T scale)
    {
        Line out(lhs);
        out /= scale;
        return out;
    }

    Line& operator=(const Line& other)
    {
        if (this == &other)
            return *this;
        setVector(other.vector());
        setPoint(other.point());
        return *this;
    }

    Line normalize() const
    {
        return Line(this->vector().normalize(),this->point());
    }

    Point<T> closestPoint(Point<T> p = Point<T>(0,0,0))
    {
        // Vector from line to point
        Point<T> line_to_point = p - point();

        // Project on the line
        T pr = line_to_point.dot(vector().normalize());

        // Get point
        return (vector().normalize()*pr) + point();
    }

    Point<T> pointError(Point<T> p = Point<T>(0,0,0))
    {
        return p - closestPoint(p);
    }

    T distance(const Point<T>& x)
    {
        checkInitialized();
        return (closestPoint(x) - x).mod();
    }

    void from4parameters(T r, T roll, T pitch, T yaw)
    {
        // Line
        std::cout<<"Line: "<<r<<","<<todeg(yaw)<<","<<todeg(pitch)<<","<<todeg(roll)<<std::endl;
/*
        // Create unit vectors
        Eigen::Matrix<T,3,1> unit_z = Eigen::Matrix<T,3,1>::UnitZ();
        Eigen::Matrix<T,3,1> unit_y = Eigen::Matrix<T,3,1>::UnitY();
        Eigen::Matrix<T,3,1> unit_x = Eigen::Matrix<T,3,1>::UnitX();

        // Create rotations 0
        Eigen::Matrix<T,3,3> rot_yaw = Eigen::AngleAxis<T>(yaw,unit_z).toRotationMatrix();
        Eigen::Matrix<T,3,3> rot_pitch = Eigen::AngleAxis<T>(pitch,unit_y).toRotationMatrix();
        Eigen::Matrix<T,3,3> rot_roll = Eigen::AngleAxis<T>(roll,unit_x).toRotationMatrix();

        // Closest point
        Eigen::Matrix<T,3,1> point_0(r,T(0),T(0));
        Eigen::Matrix<T,3,1> point_1 = rot_pitch*point_0;
        Eigen::Matrix<T,3,1> point_2 = rot_yaw*point_1;
        setPoint(Point<T>(point_2));

        // Vector
        Eigen::Matrix<T,3,1> vec;
        Eigen::Matrix<T,3,3> mrt1 = rot_yaw*rot_pitch*rot_roll;
        vec = mrt1*unit_z;
        setVector(vec);
        */

        // Closest point
        Eigen::Matrix<T,3,1> point_0(r,T(0),T(0));
        Eigen::Matrix<T,3,1> point_1 = mrot(T(0),pitch,yaw)*point_0;
        setPoint(Point<T>(point_1));

        // Vector
        Eigen::Matrix<T,3,1> vec;
        vec = mrot(roll,pitch,yaw)*Eigen::Matrix<T,3,1>::UnitZ();
        setVector(vec);
    }

    void to4parameters(T& r, T& roll, T& pitch, T& yaw)
    {
        // compute radious
        r = closestPoint().mod();

        // Get z directiron
        Eigen::Matrix<T,3,1> unit_z = vector().toEigen();

        // Get x direction
        Eigen::Matrix<T,3,1> unit_x = closestPoint().normalize().toEigen();

        // Get y direction
        Eigen::Matrix<T,3,1> unit_y = unit_z.cross(unit_x);

        // Rotation matrix
        Eigen::Matrix<T,3,3> mrt;
        mrt.col(0) = unit_x;
        mrt.col(1) = unit_y;
        mrt.col(2) = unit_z;

        // Compute rotations
        getrpy(mrt,roll,pitch,yaw);
    }

protected:

    void checkInitialized() const
    {
        if (m_data == NULL)
            EXCEPTION("Line data has not been initialized")
    }

    void deallocate()
    {
        if (m_owns_data)
            delete[] m_data;
        m_owns_data = false;
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
std::ostream& operator<<(std::ostream& out, const Line<T>& l)
{
    out<<l.vector()<<"*t + "<<l.point();
    return out;
}

#endif // LINE_H
