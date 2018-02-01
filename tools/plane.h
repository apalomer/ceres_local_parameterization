#ifndef PLANE_H
#define PLANE_H

#include "tools/exception.h"
#include "tools/point.h"

template<typename T>
class Plane
{
public:
    enum{
        N_X = 0,
        N_Y = 1,
        N_Z = 2,
        D = 3
    };

    ~Plane()
    {
        deallocate();
    }

    Plane(): m_owns_data(false)
    {
        allocate();
        set(T(0),T(0),T(0),T(0),T(0),T(0));
    }

    Plane(const Point<T>& vector, T d): m_owns_data(false)
    {
        allocate();
        setNormal(vector);
        setDistance(d);
    }

    Plane(const Plane& other): m_owns_data(false)
    {
        allocate();
        set(other.normal(),other.d());
    }

    Plane(T const* data): m_owns_data(false)
    {
        allocate();
        set(data);
    }

    Plane(T* data): m_owns_data(false)
    {
        set(data);
    }

    Plane(T n_x, T n_y, T n_z, T d): m_owns_data(false)
    {
        allocate();
        set(n_x,n_y,n_z,d);
    }

    void setNormal(const Point<T>& normal)
    {
        checkInitialized();
        m_data[N_X] = normal.x();
        m_data[N_Y] = normal.y();
        m_data[N_Z] = normal.z();
    }

    void setNormal(T n_x, T n_y, T n_z)
    {
        setNormal(Point<T>(n_x,n_y,n_z));
    }

    void set(T n_x, T n_y, T n_z, T d)
    {
        setNormal(n_x,n_y,n_z);
        setDistance(d);
    }

    void setDistance(T d)
    {
        checkInitialized();
        m_data[D] = d;
    }

    void set(const Point<T>& vector, T d)
    {
        setNormal(vector);
        setDistance(d);
    }

    void set(T const* data)
    {
        allocate();
        for ( int i = 0; i< 4; i++ ) m_data[i] = data[i];
    }

    void set(T* data)
    {
        deallocate();
        m_data = data;
    }

    T d() const
    {
        checkInitialized();
        return m_data[D];
    }

    Point<T> normal() const
    {
        checkInitialized();
        T const* ptr = m_data;
        return Point<T>(ptr);
    }

    T distance(const Point<T>& x) const
    {
        return (normal().dot(x) - d())/normal().mod();
    }

    T* data()
    {
        checkInitialized();
        return m_data;
    }

    Plane& operator+=(const Plane& rhs)
    {
        checkInitialized();
        this->setNormal(this->normal()+rhs.normal());
        this->setDistance(this->distance()+rhs.distance());
        return *this;
    }

    friend Plane& operator+(Plane& lhs, const Plane& rhs)
    {
        lhs += rhs;
        return lhs;
    }

    friend Plane operator+(const Plane& lhs, const Plane& rhs)
    {
        Plane l(lhs);
        l += rhs;
        return l;
    }

    Plane& operator-=(const Plane& rhs)
    {
        checkInitialized();
        this->setNormal(this->normal()-rhs.normal());
        this->setDistance(this->d()-rhs.d());
        return *this;
    }

    friend Plane operator-(const Plane& lhs, const Plane& rhs)
    {
        Plane l(lhs);
        l -= rhs;
        return l;
    }

    Plane& operator*=(const T& scale)
    {
        checkInitialized();
        this->setNormal(this->normal()*scale);
        this->setDistance(this->distance()*scale);
        return *this;
    }

    friend Plane operator*(T scale, const Plane& rhs)
    {
        Plane out(rhs);
        out *= scale;
        return out;
    }

    friend Plane operator*(Plane& lhs, T scale)
    {
        Plane out(lhs);
        out *= scale;
        return out;
    }

    Plane& operator/=(const T& scale)
    {
        checkInitialized();
        this->setNormal(this->normal()/scale);
        this->setDistance(this->distance()/scale);
        return *this;
    }

    friend Plane operator/(const Plane& lhs, T scale)
    {
        Plane out(lhs);
        out /= scale;
        return out;
    }

    Plane& operator=(const Plane& other)
    {
        if (this == &other)
            return *this;
        setNormal(other.normal());
        setDistance(other.distance());
        return *this;
    }

    Plane normalize() const
    {
        T module = normal().mod();
        return Plane(normal()/module,d()/module);
    }

protected:

    void checkInitialized() const
    {
        if (m_data == NULL)
            EXCEPTION("Plane data has not been initialized")
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
        m_data = new T[4];
        m_owns_data = true;
    }

private:
    T* m_data;
    bool m_owns_data;
};

template<typename T>
std::ostream& operator<<(std::ostream& out, const Plane<T>& pl)
{
    out<<pl.normal()<<"*X="<<pl.d();
    return out;
}

#endif // PLANE_H
