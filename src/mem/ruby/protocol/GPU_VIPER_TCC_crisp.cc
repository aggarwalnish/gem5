/*
 * CRISP TCC miss routing helpers for the GPU_VIPER protocol.
 */

#include "mem/ruby/protocol/GPU_VIPER/TCC_Controller.hh"

#include <cstddef>

#include "mem/ruby/common/MachineID.hh"
#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/GPUCoalescer.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/Sequencer.hh"

namespace gem5
{
namespace ruby
{
namespace GPU_VIPER
{

void
TCC_Controller::crispMissDetectedFromTCC(Addr addr, MachineID requestor)
{
    if (!m_ruby_system) {
        return;
    }

    const auto type = requestor.getType();
    const auto num = requestor.getNum();
    if (static_cast<size_t>(type) >= m_ruby_system->m_abstract_controls.size()) {
        return;
    }

    auto &by_num = m_ruby_system->m_abstract_controls[type];
    auto it = by_num.find(num);
    if (it == by_num.end() || !it->second) {
        return;
    }

    AbstractController *cntrl = it->second;
    if (auto *coal = cntrl->getGPUCoalescer()) {
        coal->crispMissDetected(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispMissDetected(addr);
    }
}

} // namespace GPU_VIPER
} // namespace ruby
} // namespace gem5
