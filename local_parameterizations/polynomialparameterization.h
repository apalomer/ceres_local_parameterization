#ifndef POLYNOMIALPARAMETERIZATION_H
#define POLYNOMIALPARAMETERIZATION_H


// Local parametrization
class PolynomialParameterization: public ceres::LocalParameterization {
public:
    explicit PolynomialParameterization(Polynomial<double> polynomial):
        m_polynomial(polynomial)
    {
    }
    virtual ~PolynomialParameterization() {}

    virtual bool Plus(const double* x,const double* delta,double* x_plus_delta) const {
        x_plus_delta[0] = delta[0] + x[0];
        x_plus_delta[1] = m_polynomial(x_plus_delta[0]);
        return true;
    }

    virtual bool ComputeJacobian(const double* x, double* jacobian) const{
        jacobian[0] = 1;
        jacobian[1] = m_polynomial.jacobian(x[0]);
        return true;
    }

    virtual int GlobalSize() const {return 2;}
    virtual int LocalSize() const {return 1;}

private:
    Polynomial<double> m_polynomial;
};

#endif // POLYNOMIALPARAMETERIZATION_H
