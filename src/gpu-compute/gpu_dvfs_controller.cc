#include "gpu-compute/gpu_dvfs_controller.hh" 
#include "debug/DVFS.hh" // DPRINTF
#include "gpu-compute/compute_unit.hh" // cu_id, CRISP counters
#include "sim/dvfs_handler.hh" // Perf levels
#include "sim/clocked_object.hh"
#include "sim/core.hh" // for sim_clock::Frequency
#include "sim/power/power_model.hh"
#include <cmath>
#include <limits> // for std::numeric_limits

namespace gem5
{

GPUDVFSController::GPUDVFSController(const Params &p)
    : SimObject(p),
      dvfsHandler(p.dvfs_handler),
      computeUnit(p.compute_unit),
      evaluationPeriod(p.evaluation_period),
      enableFrequencyTransitions(p.enable_frequency_transitions)
{
    DPRINTF(DVFS, "GPU DVFS Controller created for CU %d, "
            "eval period %lu ticks\n",
            computeUnit->cu_id, evaluationPeriod);
}

double
GPUDVFSController::tickToFrequencyMHz(Tick clkPeriod) const
{
    if (clkPeriod == 0) {
        DPRINTF(DVFS, "GPUDVFSController: Clock period is 0, returning 0 MHz\n");
        return 0.0;
    }

    double freqHz = static_cast<double>(sim_clock::Frequency) / clkPeriod;
    double freqMHz = freqHz / 1e6; // Convert to MHz
    return freqMHz;
}


// TODO : currently the energy numbers are a part of crispcounters. need 
// to move them to a better named struct. 
bool
GPUDVFSController::extractAveragePower(
    double &staticPower, double &dynamicPower) const
{
    const auto &clockedParams =
        static_cast<const ClockedObjectParams &>(computeUnit->params());

    if (clockedParams.power_model.empty() || !clockedParams.power_model[0]) {
        DPRINTF(DVFS, "CU %d: No power model attached\n",
                computeUnit->cu_id);
        return false;
    }

    dynamicPower = clockedParams.power_model[0]->getDynamicPower();
    staticPower = clockedParams.power_model[0]->getStaticPower();

    DPRINTF(DVFS, "CU %d: Sampled power - Static: %.3f W, Dynamic: %.3f W\n",
            computeUnit->cu_id, staticPower, dynamicPower);

    return true;
}

uint64_t
GPUDVFSController::calculateCRISPDelay(uint64_t tMemory,
                                       uint64_t T_overlapped,
                                       uint64_t T_pure_compute,
                                       double currentFreqMHz,
                                       double targetFreqMHz) const
{
    uint64_t Tcomp_LCP_scaled = std::ceil(
        (currentFreqMHz / targetFreqMHz) * T_overlapped);
    uint64_t TLCP = std::max(tMemory, Tcomp_LCP_scaled);

    uint64_t Tcomp_CSP_scaled = std::ceil(
        (currentFreqMHz / targetFreqMHz) * T_pure_compute);
    uint64_t TCSP = std::max(T_pure_compute, Tcomp_CSP_scaled);

    return TLCP + TCSP;
}

double
GPUDVFSController::calculateCRISPEDP(uint64_t tMemory,
                                     uint64_t T_overlapped,
                                     uint64_t T_pure_compute,
                                     uint64_t T_active,
                                     double staticPower,    double dynamicPower,
                                     double currentFreqMHz, double targetFreqMHz,
                                     double voltageCurrent, double voltageTarget) const
{
    // Guard against division by zero
    if (currentFreqMHz == 0.0 || targetFreqMHz == 0.0) {
        DPRINTF(DVFS, "GPUDVFSController: Invalid frequency (current=%.2f, target=%.2f)\n",
             currentFreqMHz, targetFreqMHz);
        return std::numeric_limits<double>::max();
    }

    uint64_t Tdelay = calculateCRISPDelay(tMemory, T_overlapped,
                                          T_pure_compute, currentFreqMHz,
                                          targetFreqMHz);

    uint64_t Tcurrent = T_active;

    double staticPowerScaled = staticPower * (voltageTarget / voltageCurrent);
    
    double dynamicPowerScaled = dynamicPower * ((voltageTarget * voltageTarget) / (voltageCurrent * voltageCurrent)) * (targetFreqMHz / currentFreqMHz);
    
    double Tdelay_s = static_cast<double>(Tdelay) *
        computeUnit->clockPeriod() * 1e-12;
    double Tcurrent_s = static_cast<double>(Tcurrent) *
        computeUnit->clockPeriod() * 1e-12;
    
    double Estatic = staticPowerScaled * Tdelay_s;
    
    double Edynamic = dynamicPowerScaled * Tcurrent_s;
    
    double Etotal = Estatic + Edynamic;
    
    double EDP = Etotal * Tdelay_s;
    
    return EDP;
}

int
GPUDVFSController::selectOptimalFrequencyEDP(
    uint64_t tMemory, uint64_t T_active, uint64_t T_overlapped,
    uint64_t T_pure_compute) const
{
    (void)T_active;

    int domain_id = computeUnit->cu_id;
    DVFSHandler::PerfLevel currentLevel = dvfsHandler->perfLevel(domain_id);
    DVFSHandler::PerfLevel numLevels = dvfsHandler->numPerfLevels(domain_id);

    // Edge case: No CRISP data collected
    bool hasData = (tMemory > 0 ||
                    T_overlapped > 0 ||
                    T_pure_compute > 0);

    if (!hasData) {
        DPRINTF(DVFS, "CU %d: No CRISP data, staying at level %d\n",
                computeUnit->cu_id, currentLevel);
        return currentLevel;
    }

    // Edge case: Only one performance level
    if (numLevels <= 1) {
        return 0;
    }

    // Get current frequency and voltage
    Tick currentPeriod = dvfsHandler->clkPeriodAtPerfLevel(domain_id, currentLevel);
    double currentFreqMHz = tickToFrequencyMHz(currentPeriod);
    double currentVoltage = dvfsHandler->voltageAtPerfLevel(domain_id, currentLevel);

    // Extract average power from accumulated energy measurements
    double staticPower, dynamicPower;
    if (!extractAveragePower(staticPower, dynamicPower)) {
        // No valid power data - stay at current level
        DPRINTF(DVFS, "CU %d: No valid power data, staying at level %d\n",
                computeUnit->cu_id, currentLevel);
        return currentLevel;
    }

    // Calculate EDP for each performance level
    double minEDP = std::numeric_limits<double>::max();
    int optimalLevel = currentLevel;

    for (DVFSHandler::PerfLevel level = 0; level < numLevels; level++) {
        Tick targetPeriod = dvfsHandler->clkPeriodAtPerfLevel(domain_id, level);
        double targetFreqMHz = tickToFrequencyMHz(targetPeriod);
        double targetVoltage = dvfsHandler->voltageAtPerfLevel(domain_id, level);

        // Calculate EDP for this target level using measured power
        double edp = calculateCRISPEDP(tMemory, T_overlapped,
                                       T_pure_compute, T_active, staticPower,
                                       dynamicPower, currentFreqMHz,
                                       targetFreqMHz, currentVoltage,
                                       targetVoltage);

        DPRINTF(DVFS, "  Level %d: freq=%.2f MHz, V=%.3f, EDP=%.6e\n",
                level, targetFreqMHz, targetVoltage, edp);

        if (edp < minEDP) {
            minEDP = edp;
            optimalLevel = level;
        }
    }

    DPRINTF(DVFS, "CU %d: Optimal level=%d (minEDP=%.6e)\n",
            computeUnit->cu_id, optimalLevel, minEDP);

    return optimalLevel;
}

void
GPUDVFSController::evaluate(uint64_t tMemory, uint64_t T_active,
                            uint64_t T_overlapped,
                            uint64_t T_pure_compute)
{
    // Use CRISP counters for EDP-based frequency selection
    int currentLevel = dvfsHandler->perfLevel(computeUnit->cu_id);
    int newLevel = selectOptimalFrequencyEDP(
        tMemory, T_active, T_overlapped, T_pure_compute);

    // Debug: Log CRISP counter summary
    DPRINTF(DVFS, "CU %d CRISP: TMemory=%lu, "
            "TActive=%lu, Overlap=%lu, Pure=%lu\n",
            computeUnit->cu_id,
            tMemory,
            T_active,
            T_overlapped,
            T_pure_compute);

    // Apply frequency change if needed
    if (newLevel != currentLevel && enableFrequencyTransitions) {
        DPRINTF(DVFS, "CU %d: EDP transition %d -> %d\n",
                computeUnit->cu_id, currentLevel, newLevel);
        adjustFrequency(newLevel);
    }

}
void
GPUDVFSController::adjustFrequency(int newLevel)
{
    // Use cu_id as domain_id (CU 0 has domain_id 0, CU 1 has domain_id 1, etc.)
    int domain_id = computeUnit->cu_id;
    int currentLevel = dvfsHandler->perfLevel(domain_id);

    DPRINTF(DVFS, "Adjusting CU %d frequency: Level %d -> %d at tick %lu\n",
            computeUnit->cu_id, currentLevel, newLevel, curTick());

    // Request DVFS change for this CU's domain
    bool success = dvfsHandler->perfLevel(domain_id, newLevel);

    if (!success) {
        DPRINTF(DVFS, "CU %d DVFS: Failed to transition from level %d to %d\n",
             computeUnit->cu_id, currentLevel, newLevel);
    }
}

void
GPUDVFSController::onFrequencyTransitionComplete()
{
    computeUnit->recomputeWindowCycles();
}

} // namespace gem5
