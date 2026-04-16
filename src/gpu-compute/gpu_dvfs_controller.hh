//---------------------------------------------------------------------------------//


//---------------------------------------------------------------------------------//


#ifndef __GPU_COMPUTE_GPU_DVFS_CONTROLLER_HH__
#define __GPU_COMPUTE_GPU_DVFS_CONTROLLER_HH__

#include <vector>
#include "params/GPUDVFSController.hh"
#include "sim/sim_object.hh"

namespace gem5
{

class ComputeUnit;
class DVFSHandler;

class GPUDVFSController : public SimObject
{
  public:
    typedef GPUDVFSControllerParams Params;
    GPUDVFSController(const Params &p);
    void evaluate(uint64_t tMemory, uint64_t T_active,
                  uint64_t T_overlapped,
                  uint64_t T_pure_compute);
    void onFrequencyTransitionComplete();
    Tick getEvaluationPeriod() const { return evaluationPeriod; }

  private:
    // Configuration
    DVFSHandler *dvfsHandler;
    ComputeUnit *computeUnit;
    Tick evaluationPeriod;
    bool enableFrequencyTransitions;

    // CRISP methods
    uint64_t calculateCRISPDelay(uint64_t tMemory,
                                 uint64_t T_overlapped,
                                 uint64_t T_pure_compute,
                                 double currentFreqMHz,
                                 double targetFreqMHz) const;

    double calculateCRISPEDP(uint64_t tMemory,
                             uint64_t T_overlapped,
                             uint64_t T_pure_compute,
                             uint64_t T_active,
                             double staticPower, double dynamicPower,
                             double currentFreqMHz, double targetFreqMHz,
                             double voltageCurrent, double voltageTarget) const;

    // Helper methods
    double tickToFrequencyMHz(Tick clkPeriod) const;
    int selectOptimalFrequencyEDP(uint64_t tMemory, uint64_t T_active,
                                  uint64_t T_overlapped,
                                  uint64_t T_pure_compute) const;

    bool extractAveragePower(double &staticPower,
                             double &dynamicPower) const;

    // Core policy methods
    void adjustFrequency(int newLevel);
};

} // namespace gem5

#endif // __GPU_COMPUTE_GPU_DVFS_CONTROLLER_HH__
