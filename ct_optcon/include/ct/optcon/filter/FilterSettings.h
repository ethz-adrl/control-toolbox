/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

#include <boost/property_tree/info_parser.hpp>

namespace ct {
namespace optcon {

//! Settings for setting up a StateObserver
/*!
 * \ingroup
 *
 * The StateObserver settings are designed to make the initialization smoother and possible through a file
 * configuration.
 *
 */
template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
struct StateObserverSettings
{
    //! default constructor for the state Observer settings
    StateObserverSettings() {}
    //! check if the currently set parameters are meaningful
    bool parametersOk() const { return true; }
    ct::core::OutputStateMatrix<OUTPUT_DIM, STATE_DIM, SCALAR> C; /*!< Observation matrix C time. */
    ct::core::StateMatrix<STATE_DIM, SCALAR> Q;                   /*!< Weighing matrix Q. */
    ct::core::OutputMatrix<OUTPUT_DIM, SCALAR> R;                 /*!< Weighing matrix R. */
    ct::core::StateMatrix<STATE_DIM, SCALAR> dFdv;

    //! print the current settings
    void print() const
    {
        std::cout << "State Observer Settings: " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "C:\n" << C << std::endl;
        std::cout << "Q:\n" << Q << std::endl;
        std::cout << "R:\n" << R << std::endl;
        std::cout << "dFdv:\n" << dFdv << std::endl;
        std::cout << "              =======" << std::endl;
        std::cout << std::endl;
    }

    //! load settings from file
    void load(const std::string& filename, bool verbose, const std::string& ns)
    {
        if (verbose)
            std::cout << "Trying to load state observer settings from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        ct::core::loadMatrix(filename, "C", C, ns);
        ct::core::loadMatrix(filename, "Q", Q, ns);
        ct::core::loadMatrix(filename, "R", R, ns);
        ct::core::loadMatrix(filename, "dFdv", dFdv, ns);

        if (verbose)
        {
            std::cout << "Loaded state observer settings from " << filename << ": " << std::endl;
            print();
        }
    }
};

//! Settings for setting up a DisturbanceObserver
/*!
 * \ingroup
 *
 * The DisturbanceObserver settings are designed to make the initialization smoother and possible through a file
 * configuration.
 *
 */
template <size_t OUTPUT_DIM, size_t STATE_DIM, typename SCALAR = double>
using DisturbanceObserverSettings = StateObserverSettings<OUTPUT_DIM, STATE_DIM, SCALAR>;

//! Settings for setting up a SteadyStateKF
/*!
 * \ingroup
 *
 * The SteadyStateKF settings are designed to make the initialization smoother and possible through a file
 * configuration.
 *
 */
template <size_t STATE_DIM, typename SCALAR = double>
struct SteadyStateKalmanFilterSettings
{
    ct::core::StateVector<STATE_DIM> x0; /*!< Initial state estimate. */
    size_t maxDAREIterations;            /*!< Max number of iteration for solving DARE. */

    //! default constructor
    SteadyStateKalmanFilterSettings() : maxDAREIterations(1000u) {}
    //! print the current settings
    void print() const
    {
        std::cout << "State Observer Settings: " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "x0:\n" << x0 << std::endl;
        std::cout << "maxDAREIterations:\t" << maxDAREIterations << std::endl;
        std::cout << "              =======" << std::endl;
        std::cout << std::endl;
    }

    //! load settings from file
    void load(const std::string& filename, bool verbose, const std::string& ns)
    {
        if (verbose)
            std::cout << "Trying to load steady state KF settings from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        maxDAREIterations = pt.get<size_t>(ns + ".maxDAREIterations", 1000);
        ct::core::loadMatrix(filename, "x0", x0, ns);

        if (verbose)
        {
            std::cout << "Loaded steady state KF settings from " << filename << ": " << std::endl;
            print();
        }
    }
};

//! Settings for setting up an ExtendedKF
/*!
 * \ingroup
 *
 * The ExtendedKF settings are designed to make the initialization smoother and possible through a file
 * configuration.
 *
 */
template <size_t STATE_DIM, typename SCALAR = double>
struct ExtendedKalmanFilterSettings
{
    ct::core::StateVector<STATE_DIM> x0;         /*!< Initial state estimate. */
    ct::core::StateMatrix<STATE_DIM, SCALAR> P0; /*!< Initial covariance matrix. */

    //! default constructor
    ExtendedKalmanFilterSettings() {}
    //! print the current settings
    void print() const
    {
        std::cout << "State Observer Settings: " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "x0:\n" << x0 << std::endl;
        std::cout << "P0:\n" << P0 << std::endl;
        std::cout << "              =======" << std::endl;
        std::cout << std::endl;
    }

    //! load settings from file
    void load(const std::string& filename, bool verbose, const std::string& ns)
    {
        if (verbose)
            std::cout << "Trying to load steady state KF settings from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        ct::core::loadMatrix(filename, "x0", x0, ns);
        ct::core::loadMatrix(filename, "P0", P0, ns);

        if (verbose)
        {
            std::cout << "Loaded ExtendedKF settings from " << filename << ": " << std::endl;
            print();
        }
    }
};

//! Settings for setting up an UnscentedKF
/*!
 * \ingroup
 *
 * The UnscentedKF settings are designed to make the initialization smoother and possible through a file
 * configuration.
 *
 */
template <size_t STATE_DIM, typename SCALAR = double>
struct UnscentedKalmanFilterSettings
{
    ct::core::StateVector<STATE_DIM> x0; /*!< Initial state estimate. */
    SCALAR alpha;
    SCALAR beta;
    SCALAR kappa;
    ct::core::StateMatrix<STATE_DIM, SCALAR> P0; /*!< Initial covariance matrix. */

    //! default constructor
    UnscentedKalmanFilterSettings() {}
    //! print the current settings
    void print() const
    {
        std::cout << "State Observer Settings: " << std::endl;
        std::cout << "=====================" << std::endl;
        std::cout << "x0:\n" << x0 << std::endl;
        std::cout << "alpha:\n" << alpha << std::endl;
        std::cout << "beta:\n" << beta << std::endl;
        std::cout << "kappa:\n" << kappa << std::endl;
        std::cout << "P0:\n" << P0 << std::endl;
        std::cout << "              =======" << std::endl;
        std::cout << std::endl;
    }

    //! load settings from file
    void load(const std::string& filename, bool verbose, const std::string& ns)
    {
        if (verbose)
            std::cout << "Trying to load steady state KF settings from " << filename << ": " << std::endl;

        boost::property_tree::ptree pt;
        boost::property_tree::read_info(filename, pt);

        ct::core::loadMatrix(filename, "x0", x0, ns);
        alpha = pt.get<bool>(ns + ".alpha");
        beta = pt.get<bool>(ns + ".beta");
        kappa = pt.get<bool>(ns + ".kappa");
        ct::core::loadMatrix(filename, "P0", P0, ns);

        if (verbose)
        {
            std::cout << "Loaded UnscentedKF settings from " << filename << ": " << std::endl;
            print();
        }
    }
};

}  // namespace optcon
}  // namespace ct
