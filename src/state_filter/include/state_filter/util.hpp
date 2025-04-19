#ifndef util_hpp
#define util_hpp

#include <Eigen/Dense>

namespace filter {

template <typename _MatrixType, int _UpLo = Eigen::Lower>
class Cholesky : public Eigen::LLT<_MatrixType, _UpLo> {


public:
    Cholesky() : Eigen::LLT<_MatrixType, _UpLo>() {}

    Cholesky(const _MatrixType& m) : Eigen::LLT<_MatrixType, _UpLo>(m) {}

    Cholesky& set_identity() {
        this->m_matrix.setIdentity();
        this->m_isInitialized = true;
        return *this;
    }

    bool is_identity() const {
        eigen_assert(this -> m_isInitialized && "LLT is not initialized.");
        return this->m_matrix.isIdentity();
    }

    template <typename Derived>
    Cholesky& set_lower(const Eigen::MatrixBase<Derived>& matrix) {
        this->m_matrix = matrix.template triangularView<Eigen::Lower>();
        this->m_isInitialized = true;
        return *this;
    }

    template <typename Derived>
    Cholesky& set_upper(const Eigen::MatrixBase<Derived>& matrix) {
        this->m_matrix = matrix.template triangularView<Eigen::Upper>().adjoint();
        this->m_isInitialized = true;
        return *this;
    }

};

} /* namespace filter */

#endif /* util_hpp */
