/*
 * CRISP TCC miss routing helper for the GPU_VIPER protocol.
 */

#include "mem/ruby/slicc_interface/RubySlicc_Util.hh"

#include "mem/ruby/slicc_interface/AbstractController.hh"
#include "mem/ruby/system/GPUCoalescer.hh"
#include "mem/ruby/system/RubySystem.hh"
#include "mem/ruby/system/Sequencer.hh"

namespace gem5
{
namespace ruby
{

void
crispMissDetectedFromTCCImpl(Addr addr, MachineID requestor)
{
    RubySystem *rs = g_ruby_system;
    if (!rs) {
        return;
    }

    if (requestor.type >= MachineType_NUM) {
        return;
    }

    auto &by_num = rs->m_abstract_controls[requestor.type];
    auto it = by_num.find(requestor.num);
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

} // namespace ruby
} // namespace gem5
