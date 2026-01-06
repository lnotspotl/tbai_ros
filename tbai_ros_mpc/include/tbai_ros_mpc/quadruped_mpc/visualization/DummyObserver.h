#pragma once

#include <ocs2_mpc/CommandData.h>
#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

namespace ocs2 {

/**
 * This class can be used to observe the dummy loop. Every loop of the dummy, the update method is called on all subscribed observers.
 */
class DummyObserver {
 public:
  virtual ~DummyObserver() = default;

  /**
   * Update is called at the end of every timestep in the dummy loop.
   *
   * @param observation : system state and input at the current time
   * @param primalSolution : latest MPC primal solution
   * @param command : latest command on which the MPC solution is based
   */
  virtual void update(const SystemObservation& observation, const PrimalSolution& primalSolution, const CommandData& command) = 0;
};
}  // namespace ocs2
