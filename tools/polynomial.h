#ifndef POLYNOMIAL_H
#define POLYNOMIAL_H

// STD library
#include <vector>
#include <sstream>
#include <cmath>

template<typename T>
class Polynomial
{
public:
    Polynomial()
    {

    }
    Polynomial(std::vector<T> coeficients):
        m_coeficients(coeficients)
    {

    }
    Polynomial(const Polynomial& other)
    {
        m_coeficients = other.m_coeficients;
    }

    T f(T x) const
    {
        T out(0);
        for (int i = 0;i < m_coeficients.size();i++)
            out += m_coeficients[i]*pow(x,i);
        return out;
    }

    T jacobian(T x) const
    {
        T out(0);
        for (int i = 1; i < m_coeficients.size();i++)
            out += i*pow(x,i-1)*m_coeficients[i];
        return out;
    }

    T operator()(T x) const
    {
        return f(x);
    }

    std::vector<T> getCoefficients() const
    {
        return m_coeficients;
    }

    int grade() const
    {
        return m_coeficients.size()-1;
    }

    T at(int i) const
    {
        return m_coeficients[i];
    }

    T operator[](int i) const
    {
        return at(i);
    }

    void push_back(T coeficient)
    {
        m_coeficients.push_back(coeficient);
    }

    std::vector<T> values(T init, T step, T end)
    {
        std::vector<T> out;
        for (T i = init ; i <= end ; i+= step)
            out.push_back(f(i));
        return out;
    }

    template<typename P>
    friend std::ostream& operator<<(std::ostream& os, const Polynomial<P>& obj);

private:
    std::vector<T> m_coeficients;

};

template<typename T>
std::ostream& operator<<(std::ostream& os, const Polynomial<T>& obj)
{
    for (int i = 0 ;i<obj.m_coeficients.size(); i++)
    {
        os<<obj.m_coeficients[i]<<"*x^"<<i;
        if (i<obj.m_coeficients.size()-1)
            os<<" + ";
    }
    return os;
}

#endif // POLYNOMIAL_H
