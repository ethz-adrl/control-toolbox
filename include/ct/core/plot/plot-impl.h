#pragma once

#ifdef PLOTTING_ENABLED
// -----------------------------------------------------------------------------
template <typename ALLOC>
bool plot(const std::vector<double, ALLOC>& y, const std::string& format)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> y_e(y.data(), y.size());
    return plot(y_e, format);
}

// -----------------------------------------------------------------------------
template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x,
    const std::vector<double, ALLOC2>& y,
    const std::map<std::string, std::string>& keywords)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> y_e(y.data(), y.size());
    return plot(x_e, y_e, keywords);
}

template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x, const std::vector<double, ALLOC2>& y, const std::string& s)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> x_e(x.data(), x.size());
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> y_e(y.data(), y.size());
    return plot(x_e, y_e, s);
}

#else  // PLOTTING_ENABLED

template <typename ALLOC>
bool plot(const std::vector<double, ALLOC>& y, const std::string& format)
{
    Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, 1>> y_e(y.data(), y.size());
    warn();
    return false;
}

template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x,
    const std::vector<double, ALLOC2>& y,
    const std::map<std::string, std::string>& keywords)
{
    warn();
    return false;
}

template <typename ALLOC, typename ALLOC2>
bool plot(const std::vector<double, ALLOC>& x, const std::vector<double, ALLOC2>& y, const std::string& s)
{
    warn();
    return false;
}

#endif
