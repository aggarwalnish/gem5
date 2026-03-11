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
        coal->crispL2MissDetected(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispMissDetected(addr);
        seq->crispL2MissDetected(addr);
        return;
    }
}

void
crispL1MissImpl(Addr addr, MachineID requestor)
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
        coal->crispL1MissDetected(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispL1MissDetected(addr);
        return;
    }
}

void
crispL2HitDetectedImpl(Addr addr, MachineID requestor)
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
        coal->crispL2HitDetected(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispL2HitDetected(addr);
        return;
    }
}

void
crispLoadIssuedImpl(Addr addr, MachineID requestor)
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
        coal->crispLoadIssued(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispLoadIssued(addr);
        return;
    }
}

void
crispL1HitImpl(Addr addr, MachineID requestor)
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
        coal->crispL1HitDetected(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispL1HitDetected(addr);
        return;
    }
}

void
crispStoreIssuedImpl(Addr addr, MachineID requestor)
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
        coal->crispStoreIssued(addr);
        return;
    }
    if (auto *seq = cntrl->getCPUSequencer()) {
        seq->crispStoreIssued(addr);
        return;
    }
}

} // namespace ruby
} // namespace gem5
