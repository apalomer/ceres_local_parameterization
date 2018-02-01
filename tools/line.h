#ifndef LINE_H
#define LINE_H

#include "tools/exception.h"
#include "tools/point.h"

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

    T distance(const Point<T>& x)
    {
        checkInitialized();
        const Point<T> x_1 = point();
        const Point<T> x_2 = point()+vector();
        Point<T> v_0 = x - x_1;
        Point<T> v_1 = x - x_2;
        Point<T> v_3 = v_0.cross(v_1);

        return v_3.mod()/vector().mod();
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
