/**********************************************************************************************************************
This file is part of the Control Toolbox (https://github.com/ethz-adrl/control-toolbox), copyright by ETH Zurich.
Licensed under the BSD-2 license (see LICENSE file in main directory)
**********************************************************************************************************************/

#pragma once

namespace ct {
namespace optcon {


//! select a mode in which MPC is supposed to run
/*!
 *  There are four default implementations for time horizon strategies in MPC, which can be selected using the following "MPC_MODE" enum.
 *
 * - FIXED_FINAL_TIME
 * 		plan until a fixed final time T is reached. Time Horizon will continuously shrink and eventually be zero.
 *
 * - FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON
 * 		this option continuously shrinks the time horizon from the initil time horizon until some minimum time is reached.
 * 		This minimum time can be specified in the mpc_settings struct as "minimumTimeHorizonMpc_"
 *
 * - CONSTANT_RECEDING_HORIZON
 * 		Time Horizon remains constant for all times and is equal to the initial time horizon specified in the optimal control problem.
 *
 * - RECEDING_HORIZON_WITH_FIXED_FINAL_TIME
 * 		Time Horizon remains constant until a fixed final time is near. It shrinks and eventually becomes zero.
 * 		The overall time horizon gets set trough the initial problem time horizon. The smaller, receding time
 * 		horizon can be specified in the mpc_settings struct as "minimumTimeHorizonMpc_"
 */
enum MPC_MODE
{

    FIXED_FINAL_TIME = 0,  //!< FIXED_FINAL_TIME

    CONSTANT_RECEDING_HORIZON,  //!< CONSTANT_RECEDING_HORIZON

    FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON,  //!< FIXED_FINAL_TIME_WITH_MIN_TIME_HORIZON

    RECEDING_HORIZON_WITH_FIXED_FINAL_TIME  //!< RECEDING_HORIZON_WITH_FIXED_FINAL_TIME
};


//! MPC Settings struct
struct mpc_settings
{
    /*!
	 * State prediction.
	 * Use state prediction (based on a given delay, the given initial state and a given policy)?
	 * If set to true, MPC will either use a measurement of the delay or use the fixed delay given below
	 * as "fixedDelayUs_".
	 */
    bool stateForwardIntegration_ = false;

    /*!
     * Integrator type to use for the state prediction
     */
    ct::core::IntegrationType stateForwardIntegratorType_ = ct::core::IntegrationType::RK4;

    /*!
     * time step size employed for the forward integration
     */
    double stateForwardIntegration_dt_ = 0.001;

    /*!
	 * Delay compensation.
	 * If measureDelay_ is set to true, the MPC timer automatically keeps track of the planning times and
	 * uses them for the state forward prediction.
	 * Only applies if stateForwardIntegration_ is set to true.
	 * */
    bool measureDelay_ = false;

    /*!
	 * The delayMeasurementMultiplier_  quantifies the level of trust you have in our planning time measurement.
	 * In case you wish to be conservative, for example because the planning times vary a lot, reduce this multiplier.
	 * Maximum confidence means, set it to 1.0.
	 * In essence, this number says "how much of our measured delay do we use for pre-integration".
	 * */
    double delayMeasurementMultiplier_ = 0.7;

    /*!
	 * If not using measureDelay_ = true, we can instead set a fixed, known delay in microseconds (!) here.
	 */
    int fixedDelayUs_ = 0;

    /*!
	 * Additional delay in microseconds (!), which gets added to either the measured delay of above fixedDelay.
	 */
    int additionalDelayUs_ = 0;

    /*!
	 * If solving the problem takes longer than predicted, we can post-truncate the solution trajectories.
	 */
    bool postTruncation_ = false;

    /*!
	 * MPC Time horizon modus
	 */
    MPC_MODE mpc_mode = CONSTANT_RECEDING_HORIZON;

    /*!
	 * see description of the different modes above
	 */
    core::Time minimumTimeHorizonMpc_ = 1.0;  // seconds

    /*!
	 * cold starting or warm starting.
	 * Warm starting will either use a default strategy for common controller types, or a user specified
	 * custom warm starting strategy.
	 */
    bool coldStart_ = false;


    /*!
     * override the internal clock with external timing.
     * \warning when employing this option, the setExternalTime() method needs to be called in MPC
     */
    bool useExternalTiming_ = false;


    //! Print MPC settings to console
    void print()
    {
        std::cout << " ========================= MPC SETTINGS =============================" << std::endl;
        std::cout << " stateForwardIntegration: \t " << stateForwardIntegration_ << std::endl;
        std::cout << " stateForwardIntegrator: \t " << stateForwardIntegratorType_ << std::endl;
        std::cout << " stateForwardIntegration_dt: \t " << stateForwardIntegration_dt_ << std::endl;
        std::cout << " measureDelay: \t " << measureDelay_ << std::endl;
        std::cout << " delayMeasurementMultiplier: \t " << delayMeasurementMultiplier_ << std::endl;
        std::cout << " fixedDelayUs: \t " << fixedDelayUs_ << std::endl;
        std::cout << " additionalDelayUs: \t " << additionalDelayUs_ << std::endl;
        std::cout << " postTruncation_: \t " << postTruncation_ << std::endl;
        std::cout << " mpc_mode: \t " << (int)mpc_mode << std::endl;
        std::cout << " minimumTimeHorizonMpc: \t " << minimumTimeHorizonMpc_ << std::endl;
        std::cout << " coldStart: \t " << coldStart_ << std::endl;
        std::cout << " useExternalTiming: \t " << useExternalTiming_ << std::endl;
        std::cout << " ============================== END =================================" << std::endl;
    }
};


//! load the mpc settings from file
/*!
 * @param filename
 * 	path of the file you wish to load from
 * @param settings
 *  the loaded settings
 *
 *  @todo make part of mpc_settings struct
 */
inline void loadMpcSettings(const std::string& filename, mpc_settings& settings)
{
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    settings.measureDelay_ = pt.get<bool>("mpc.measureDelay");
    settings.stateForwardIntegration_ = pt.get<bool>("mpc.stateForwardIntegration");
    settings.stateForwardIntegration_dt_ = pt.get<double>("mpc.stateForwardIntegration_dt");
    settings.stateForwardIntegratorType_ = (ct::core::IntegrationType)pt.get<int>("mpc.stateForwardIntegratorType");
    settings.fixedDelayUs_ = pt.get<int>("mpc.fixedDelayUs");
    settings.additionalDelayUs_ = pt.get<int>("mpc.additionalDelayUs");
    settings.minimumTimeHorizonMpc_ = pt.get<double>("mpc.timeHorizon");
    settings.coldStart_ = pt.get<bool>("mpc.coldStart");
    settings.postTruncation_ = pt.get<bool>("mpc.postTruncation");
    settings.delayMeasurementMultiplier_ = pt.get<double>("mpc.delayMeasurementMultiplier");
    settings.useExternalTiming_ = pt.get<bool>("mpc.useExternalTiming");
}


}  // namespace optcon
}  // namespace ct
