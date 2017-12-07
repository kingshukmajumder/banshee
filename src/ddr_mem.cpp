/** $lic$
 * Copyright (C) 2012-2015 by Massachusetts Institute of Technology
 * Copyright (C) 2010-2013 by The Board of Trustees of Stanford University
 *
 * This file is part of zsim.
 *
 * zsim is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, version 2.
 *
 * If you use this software in your research, we request that you reference
 * the zsim paper ("ZSim: Fast and Accurate Microarchitectural Simulation of
 * Thousand-Core Systems", Sanchez and Kozyrakis, ISCA-40, June 2013) as the
 * source of the simulator in any publications that use this software, and that
 * you send us a citation of your work.
 *
 * zsim is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "ddr_mem.h"
#include <algorithm>
#include <string>
#include <vector>
#include "bithacks.h"
#include "config.h"  // for Tokenize
#include "contention_sim.h"
#include "event_recorder.h"
#include "timing_event.h"
#include "zsim.h"

//#define DEBUG(args...) info(args)
#define DEBUG(args...)

// Recorder-allocated event, represents one read or write request
class DDRMemoryAccEvent : public TimingEvent {
    private:
        DDRMemory* mem;
        Address addr;
		uint32_t data_size;
        bool write;
    public:
        DDRMemoryAccEvent(DDRMemory* _mem, bool _isWrite, Address _addr, uint32_t _data_size, int32_t domain, uint32_t preDelay, uint32_t postDelay)
            : TimingEvent(preDelay, postDelay, domain), mem(_mem), addr(_addr), data_size(_data_size), write(_isWrite) {}

        Address getAddr() const {return addr;}
        bool isWrite() const {return write;}
		uint32_t getDataSize() const {return data_size;}
        void simulate(uint64_t startCycle) {
            mem->enqueue(this, startCycle);
        }
};

// Globally allocated event that calls us every tREFI cycles
class RefreshEvent : public TimingEvent, public GlobAlloc {
    private:
        DDRMemory* mem;
        uint32_t refInterval;  // in sysCycles

    public:
        RefreshEvent(DDRMemory* _mem, uint32_t _refInterval, int32_t domain) :
            TimingEvent(0, 0, domain), mem(_mem), refInterval(_refInterval)
        {
            setMinStartCycle(0);
            zinfo->contentionSim->enqueueSynced(this, 0);
        }

        void parentDone(uint64_t startCycle) {
            panic("This is queued directly");
        }

        void simulate(uint64_t startCycle) {
            mem->refresh(startCycle);
            requeue(startCycle + refInterval);
        }

        // Use glob mem
        using GlobAlloc::operator new;
        using GlobAlloc::operator delete;
};

/** Globally allocated event for scheduling
 *
 * NOTE: This event plus the bit of logic in DDRMemory that deals with event
 * management can be generalized to deal with event-driven classes that need to
 * be ticked according to varying constraints.
 */
class SchedEvent : public TimingEvent, public GlobAlloc {
    private:
        DDRMemory* const mem;
        enum State { IDLE, QUEUED, RUNNING, ANNULLED };
        State state;

    public:
        /// for event freelist
        SchedEvent* next;  

        SchedEvent(DDRMemory* _mem, int32_t domain) : TimingEvent(0, 0, domain), mem(_mem) {
            setMinStartCycle(0);
            setRunning();
            hold();
            state = IDLE;
            next = nullptr;
        }

        void parentDone(uint64_t startCycle) {
            panic("This is queued directly");
        }

        /**
         * This function is called from contention sim SimulatePhaseThread. 
         * Calls mem->tick to get nextCycle.
         * If nextCycle > 0, then requeue(nextCycle)
         * else it calls hold() and calls mem->recycleEvent()
         * */
        void simulate(uint64_t startCycle) {
            if (state == QUEUED) {
                state = RUNNING;
				assert(mem);
                uint64_t nextCycle = mem->tick(startCycle);
                if (nextCycle) {
                    requeue(nextCycle);
                    state = QUEUED;
                } else {
                    state = IDLE;
                    hold();
                    mem->recycleEvent(this);
                }
            } else {
                assert(state == ANNULLED);
                state = IDLE;
                hold();
                mem->recycleEvent(this);
            }
        }

        void enqueue(uint64_t cycle) {
            assert(state == IDLE);
            state = QUEUED;
            requeue(cycle);
        }

        void annul() {
            assert_msg(state == QUEUED, "sched state %d", state);
            state = ANNULLED;
        }

        // Use glob mem
        using GlobAlloc::operator new;
        using GlobAlloc::operator delete;
};


/* Init & bound phase functionality */

DDRMemory::DDRMemory(uint32_t _lineSize, uint32_t _colSize, uint32_t _ranksPerChannel, uint32_t _banksPerRank,
        uint32_t _sysFreqMHz, const char* tech, const char* addrMapping, uint32_t _controllerSysLatency,
        uint32_t _queueDepth, uint32_t _rowHitLimit, bool _deferredWrites, bool _closedPage,
        uint32_t _domain, g_string& _name, uint32_t _tBL, double time_scale)
    : lineSize(_lineSize), ranksPerChannel(_ranksPerChannel), banksPerRank(_banksPerRank),
      controllerSysLatency(_controllerSysLatency), queueDepth(_queueDepth), rowHitLimit(_rowHitLimit),
      deferredWrites(_deferredWrites), closedPage(_closedPage), domain(_domain), name(_name)
{
    sysFreqKHz = 1000 * _sysFreqMHz;
    initTech(tech, time_scale);  // sets all tXX and memFreqKHz
	tBL = _tBL;
    if (memFreqKHz >= sysFreqKHz/2) {
        panic("You may need to tweak the scheduling code, which works with system cycles." \
            "With these frequencies, events (which run on system cycles) can't hit us every memory cycle.");
    }

    //minRdLatency = controllerSysLatency + memToSysCycle(tCL+tBL-1);
    minRdLatency = controllerSysLatency + memToSysCycle(tCL+2-1);
    minWrLatency = controllerSysLatency;
    preDelay = controllerSysLatency;
    postDelayRd = minRdLatency - preDelay;
    postDelayWr = 0;

    rdQueue.init(queueDepth);
    wrQueue.init(queueDepth);

    info("%s: domain %d, %d ranks/ch %d banks/rank, tech %s, boundLat %d rd / %d wr",
            name.c_str(), domain, ranksPerChannel, banksPerRank, tech, minRdLatency, minWrLatency);

    minRespCycle = tCL + tBL + 1; // We subtract tCL + tBL from this on some checks; this avoids overflows

    banks.resize(ranksPerChannel);
    for (uint32_t i = 0; i < ranksPerChannel; i++) banks[i].resize(banksPerRank);

    rankActWindows.resize(ranksPerChannel);
    for (uint32_t i = 0; i < ranksPerChannel; i++) rankActWindows[i].init(4);  // we only model FAW; for TAW (other technologies) change this to 2

    // We get line addresses, and for a 64-byte line, there are _colSize/(JEDEC_BUS_WIDTH/8) lines/page
    uint32_t colBits = ilog2(_colSize/(JEDEC_BUS_WIDTH/8)*64/lineSize);
    uint32_t bankBits = ilog2(banksPerRank);
    uint32_t rankBits = ilog2(ranksPerChannel);

    // Parse config string, has to be some combination of rank, bank, and col separated by semicolons
    // (row is always MSB bits, since we don't actually know how many bits it is to begin with...)
    std::vector<std::string> tokens;
    Tokenize(addrMapping, tokens, ":");
    if (tokens.size() != 3) panic("Invalid addrMapping %s, need all row/col/rank tokens separated by colons", addrMapping);
    std::reverse(tokens.begin(), tokens.end()); // want lowest bits first

    colMask = rankMask = bankMask = 0;
    uint32_t startBit = 0;
    auto computeShiftAndMask = [&startBit, addrMapping](const std::string& field, const uint32_t fieldBits, uint32_t& shift, uint32_t& mask) {
        if (mask) panic("Repeated field %s in addrMapping %s", field.c_str(), addrMapping);
        shift = startBit;
        mask = (1 << fieldBits) - 1;
        startBit += fieldBits;
    };
    for (auto t : tokens) {
        if (t == "col")       computeShiftAndMask(t, colBits,  colShift,  colMask);
        else if (t == "rank") computeShiftAndMask(t, rankBits, rankShift, rankMask);
        else if (t == "bank") computeShiftAndMask(t, bankBits, bankShift, bankMask);
        else panic("Invalid token %s in addrMapping %s (only row/col/rank)", t.c_str(), addrMapping);
    }
    rowShift = startBit;  // row has no mask

    info("%s: Address mapping %s row %d:%ld col %d:%d rank %d:%d bank %d:%d",
            name.c_str(), addrMapping, 63, rowShift, ilog2(colMask << colShift), colShift,
            ilog2(rankMask << rankShift), rankShift, ilog2(bankMask << bankShift), bankShift);

    // Weave phase events
    new RefreshEvent(this, memToSysCycle(tREFI), domain);

    nextSchedCycle = -1ul;
    nextSchedEvent = nullptr;
    eventFreelist = nullptr;
}

void DDRMemory::initStats(AggregateStat* parentStat) {
    AggregateStat* memStats = new AggregateStat();
    memStats->init(name.c_str(), "Memory controller stats");
    profReads.init("rd", "Read requests"); memStats->append(&profReads);
    profWrites.init("wr", "Write requests"); memStats->append(&profWrites);
    bytesReads.init("tot_rd", "Total Bytes Read"); memStats->append(&bytesReads);
    bytesWrites.init("tot_wr", "Total Bytes Write"); memStats->append(&bytesWrites);
    profTotalRdLat.init("rdlat", "Total latency experienced by read requests"); memStats->append(&profTotalRdLat);
    profTotalWrLat.init("wrlat", "Total latency experienced by write requests"); memStats->append(&profTotalWrLat);
    profReadHits.init("rdhits", "Read row hits"); memStats->append(&profReadHits);
    profWriteHits.init("wrhits", "Write row hits"); memStats->append(&profWriteHits);
    latencyHist.init("mlh", "latency histogram for memory requests", NUMBINS); 
	// XXX //memStats->append(&latencyHist);
    parentStat->append(memStats);
}

/* Bound phase interface */
// data_size is the number of bursts
uint64_t DDRMemory::access(MemReq& req, int type, uint32_t data_size) {
    switch (req.type) {
        case PUTS:
        case PUTX:
            *req.state = I;
            break;
        case GETS:
            *req.state = req.is(MemReq::NOEXCL)? S : E;
            break;
        case GETX:
            *req.state = M;
            break;

        default: panic("!?");
    }
	assert(data_size % 2 == 0);
    if (req.type == PUTS) {
        return req.cycle; //must return an absolute value, 0 latency
    } else {
        bool isWrite = (req.type == PUTX);
		// TODO If length > 1 cacheline, add 4 cycle for each cacheline
        uint64_t respCycle = req.cycle + (isWrite? minWrLatency : minRdLatency) + memToSysCycle(data_size - 1);
        if (zinfo->eventRecorders[req.srcId]) {
			// accessing multiple lines is modeled as multiple requests.
			// All the requests can be processed in parallel.
			//  
            DDRMemoryAccEvent* memEv = new (zinfo->eventRecorders[req.srcId]) DDRMemoryAccEvent(this,
                    isWrite, req.lineAddr, data_size, domain, preDelay, isWrite? postDelayWr : postDelayRd);
			if (type == 0) // default. The only record. 
            {
            	memEv->setMinStartCycle(req.cycle);
				TimingRecord tr = {req.lineAddr, req.cycle, respCycle, req.type, memEv, memEv};
				assert(!zinfo->eventRecorders[req.srcId]->hasRecord());
           	 	zinfo->eventRecorders[req.srcId]->pushRecord(tr);
			} else if (type == 1) { // append the current event to the end of the previous one
           	 	TimingRecord tr = zinfo->eventRecorders[req.srcId]->popRecord();
            	memEv->setMinStartCycle(tr.reqCycle);
				assert(tr.endEvent);
				tr.endEvent->addChild(memEv, zinfo->eventRecorders[req.srcId]);
				// XXX when to update respCycle 
				//tr.respCycle = respCycle;
				tr.type = req.type;
				tr.endEvent = memEv;
           	 	zinfo->eventRecorders[req.srcId]->pushRecord(tr);
			} else if (type == 2) { 
				// append the current event to the end of the previous one
				// but the current event is not on the critical path
           	 	TimingRecord tr = zinfo->eventRecorders[req.srcId]->popRecord();
            	memEv->setMinStartCycle(tr.reqCycle);
				assert(tr.endEvent);
				tr.endEvent->addChild(memEv, zinfo->eventRecorders[req.srcId]);
				//tr.respCycle = respCycle;
				tr.type = req.type;
           	 	zinfo->eventRecorders[req.srcId]->pushRecord(tr);
			}
        }
        //info("Access to %lx at %ld, %ld latency", req.lineAddr, req.cycle, minLatency);
        return respCycle;
    }
}

/* Weave phase functionality */

//Address mapping:
// For now, row:col:bank:rank:channel for max parallelism (same as scheme7 from DRAMSim)
// NOTE: channel is external (from SplitAddrMem)
// Change or reorder to define your own mappings
DDRMemory::AddrLoc DDRMemory::mapLineAddr(Address lineAddr) {
    AddrLoc l;
    l.col  = (lineAddr >> colShift)  & colMask;
    l.rank = (lineAddr >> rankShift) & rankMask;
    l.bank = (lineAddr >> bankShift) & bankMask;
    l.row  = lineAddr >> rowShift;

    //info("0x%lx r%ld:c%d b%d:r%d", lineAddr, l.row, l.col, l.bank, l.rank);
    assert(l.rank < ranksPerChannel);
    assert(l.bank < banksPerRank);

    return l;
}

/**
 * This function is called from DDRMemAccessEvent->simulate().
 * This is in Weave phase.
 * If rdQueue is full or  wrQueue is full, then overflow.
 * Then move the contents of Event ev to Request rq.
 * If overflow, push to overflow queue.
 * else call queue(req, memCycle); This function 
 * 
 */
void DDRMemory::enqueue(DDRMemoryAccEvent* ev, uint64_t sysCycle) {
    uint64_t memCycle = sysToMemCycle(sysCycle);
    DEBUG("%ld: enqueue() addr 0x%lx wr %d", memCycle, ev->getAddr(), ev->isWrite());

    // Create request
    Request ovfReq;
    bool overflow = rdQueue.full() || wrQueue.full();
    bool useWrQueue = deferredWrites && ev->isWrite();
    Request* req = overflow? &ovfReq : useWrQueue? wrQueue.alloc() : rdQueue.alloc();

    req->addr = ev->getAddr();
    req->loc = mapLineAddr(ev->getAddr());
	req->data_size = ev->getDataSize();
    req->write = ev->isWrite();
    req->arrivalCycle = memCycle;
    req->startSysCycle = sysCycle;

    req->ev = ev;
    ev->hold();

    if (overflow) {
        overflowQueue.push_back(*req);
    } else {
        queue(req, memCycle);

        // If needed, schedule an event to handle this new request
        if (!req->prev /* first in bank */) {
			// XXX I don't know what this code is doing, but just adding data_size anyway.
            uint64_t minSchedCycle = std::max(memCycle, minRespCycle - tCL - tBL); // * req->data_size);
            if (nextSchedCycle > minSchedCycle) minSchedCycle = std::max(minSchedCycle, findMinCmdCycle(*req));
            if (nextSchedCycle > minSchedCycle) {
                if (nextSchedEvent) nextSchedEvent->annul();
                if (eventFreelist) {
                    nextSchedEvent = eventFreelist;
                    eventFreelist = eventFreelist->next;
                    nextSchedEvent->next = nullptr;
                } else {
                    nextSchedEvent = new SchedEvent(this, domain);
                }
                DEBUG("queued %ld", minSchedCycle);

                // Under memFreq < sysFreq/2, sysToMemCycle translates back to the same memCycle
                uint64_t enqSysCycle = std::max(matchingMemToSysCycle(minSchedCycle), sysCycle);
                nextSchedEvent->enqueue(enqSysCycle);
                nextSchedCycle = minSchedCycle;
            }
        }
    }
}

/**
 * This method is called from DDRMemory::enqueue and DDRMemory::tick.
 * Most likely this is also Weave phase.
 * Check the newest request which has the same row as req and put req next to this reuqest. This is for FRFCFS.
 * Also sets value of rowHitSeq (used for fairness).
 */
void DDRMemory::queue(Request* req, uint64_t memCycle) {
    // If it's a write, respond to it immediately
    if (req->write) {
        auto ev = req->ev;
        req->ev = nullptr;

        ev->release();
        uint64_t respCycle = memToSysCycle(memCycle) + minWrLatency;
        ev->done(respCycle - preDelay - postDelayWr);
    }

    req->arrivalCycle = memCycle;  // if this comes from the overflow queue, update

    // Test: Skip writes
#if 0
    if (req->write) {
        assert(wrQueue.size() == 1);
        wrQueue.remove(wrQueue.begin());
        return;
    }
#endif

    // Alloc in per-bank queue, in FR order
    Bank& bank = banks[req->loc.rank][req->loc.bank];
    InList<Request>& q = (deferredWrites && req->write)? bank.wrReqs : bank.rdReqs;

    // Print bak queue? Use to verify FR-FCFS
#if 0
    auto printQ = [&](const char* id) {
        info("%8ld: %s r%db%d : %s  %s", memCycle, name.c_str(), req->loc.rank, req->loc.bank, (deferredWrites && req->write)? "WQ" : "RQ", id);
        Request* pr = q.front();
        while (pr) {
            info("     0x%08lx | %ld | %ld", pr->loc.row, pr->rowHitSeq, pr->arrivalCycle);
            pr = pr->next;
        }
    };
    printQ("PRE");
#endif

    Request* m = q.back();
    while (m) {
        if (m->loc.row == req->loc.row) {
            if (m->rowHitSeq < rowHitLimit) {
                // queue after last same-row access
                req->rowHitSeq = m->rowHitSeq + 1;
                q.insertAfter(m, req);
            } else {
                // queue last to get some fairness
                req->rowHitSeq = 0;
                q.push_back(req);
            }
            break;
        }
        m = m->prev;
    }

    // No matches...
    if (!m) {
        if (bank.open && req->loc.row == bank.openRow && bank.curRowHits < rowHitLimit && q.empty()) {
            // ... but row is open (& bank queue empty), bypass everyone
            /* NOTE: If the bank queue is not empty, don't go before the
             * current request. We assume that the request could have issued
             * PRE/ACT commands by now, but those are not recorded till
             * trySchedule. If you choose to bypass to the front, you should
             * check whether the next request would have issued a PRE or ACT by
             * now (o/w you have oracular knowledge...).
             */
             req->rowHitSeq = bank.curRowHits + 1;
            q.push_front(req);
        } else {
            // ... and row is closed or has too many hits, maintain FCFS
            req->rowHitSeq = 0;
            q.push_back(req);
        }
    }
#if 0
    printQ("POST");
#endif
}

// For external ticks
/**
 * Called by schedule above.
 * First gets the earliest cycle where 
 *
 * */
uint64_t DDRMemory::tick(uint64_t sysCycle) {
    uint64_t memCycle = sysToMemCycle(sysCycle);
    assert_msg(memCycle == nextSchedCycle, "%ld != %ld", memCycle, nextSchedCycle);
    uint64_t minSchedCycle = trySchedule(memCycle, sysCycle);
    assert(minSchedCycle >= memCycle);
    // Move request from overflow to  write/read queues.
    if (!rdQueue.full() && !wrQueue.full() && !overflowQueue.empty()) {
        Request& ovfReq = overflowQueue.front();
        bool useWrQueue = deferredWrites && ovfReq.write;
        Request* req = useWrQueue? wrQueue.alloc() : rdQueue.alloc();
        *req = ovfReq;
        overflowQueue.pop_front();

        //? Put the overflowen request on the read/write queue.
        queue(req, memCycle);

        // This request may be schedulable before trySchedule's minSchedCycle
        if (!req->prev /*first in bank queue*/) {
			// XXX I don't know what this code is doing, but just adding data_size anyway.
            uint64_t minQueuedSchedCycle = std::max(memCycle, minRespCycle - tCL - tBL); // * req->data_size);
            if (minSchedCycle > minQueuedSchedCycle) minSchedCycle = std::max(minQueuedSchedCycle, findMinCmdCycle(*req));
            if (minSchedCycle > minQueuedSchedCycle) {
                DEBUG("Overflowed request lowered minSchedCycle %ld -> %ld (memCycle %ld)", minSchedCycle, minQueuedSchedCycle, memCycle);
                minSchedCycle = minQueuedSchedCycle;
            }
        }
    }

    nextSchedCycle = minSchedCycle;
    if (nextSchedCycle == -1ul) {
        nextSchedEvent = nullptr;
        return 0;
    } else {
        // sysToMemCycle translates this back to nextSchedCycle
        uint64_t enqSysCycle = std::max(matchingMemToSysCycle(nextSchedCycle), sysCycle);
        return enqSysCycle;
    }
}

void DDRMemory::recycleEvent(SchedEvent* ev) {
    assert(ev != nextSchedEvent);
    assert(ev->next == nullptr);
    ev->next = eventFreelist;
    eventFreelist = ev;
}

/**
 * Returns the cycle when the next column access can be issued.
 */
uint64_t DDRMemory::findMinCmdCycle(const Request& r) const {
    const Bank& bank = banks[r.loc.rank][r.loc.bank];
    uint64_t minCmdCycle = std::max(r.arrivalCycle, bank.lastCmdCycle + 1);
    if (r.loc.row == bank.openRow && bank.open) {
        // Row buffer hit
    } else {
        // Either row closed, or row buffer miss
        uint64_t preCycle;
        if (!bank.open) {
            // minPreCycle is the cycle the bank was last PRE.
            preCycle = bank.minPreCycle;
        } else {
            assert(r.loc.row != bank.openRow);
            // minPreCycle is the cycle the bank can be precharged next.
            preCycle = std::max(r.arrivalCycle, bank.minPreCycle);
        }
        // tRP is minimum time from PRE to ACT.
        // tRRD ACT to ACT
        // The actCycle can be delayed by three factors:
        // the arrival time of the request, the delay between precharge to activate, or the delay between activate to activate.
        uint64_t actCycle = std::max(r.arrivalCycle, std::max(preCycle + tRP, bank.lastActCycle + tRRD));
        // minActCycle return the cycle of the latest request in the activation window of the bank.
        // tFAW is the time in which activate can be sent to 4 different banks. ?????????
        actCycle = std::max(actCycle, rankActWindows[r.loc.rank].minActCycle() + tFAW);
        // time ACT to CAS
        minCmdCycle = actCycle + tRCD;
    }
    return minCmdCycle;
}

/**
 * called from tick
 * Weave phase maybe
 * Takes first request from each bank's queue(bank's queue is in FR-FCFS order, where FR means row is open) and checks if and when the command bus is available for it.
 * Find the earliest cycle the write/read command can be issued.
 * minRespCycle is updated in this function: it is used to calculate the cycle for issuing commands during the next time trySchedule is called. 
 * The bank is closed(PRE) if closed policy is used and there aren't any requests to the same row to be serviced.
 * The time to  service the request is calculated and logged into the  stats framework.
 */
uint64_t DDRMemory::trySchedule(uint64_t curCycle, uint64_t sysCycle) {
    /* Implement FR-FCFS scheduling to maximize bus utilization
     *
     * This model is issue-centric: We queue our events at the appropriate
     * COLUMN ACCESS issue time, and compute constraints on when we can
     * actually do the column access. This ensures we put the column access at
     * the right time. But be careful... you have more information here than
     * you'd have in a cycle-by-cycle model, and it's easy to modify this
     * algorithm to have oracular characteristics. If you're writing a shiny
     * new scheduler algorithm, think about what you know when.
     *
     * Here, we're not using future knowledge because requests queue in FR-FCFS
     * order at *arrival* time, and we obey the appropriate timing constraints.
     */

    if (rdQueue.empty() && wrQueue.empty()) return -1ul;
    if (curCycle + tCL < minRespCycle) return minRespCycle - tCL;  // too far ahead

    // Writes have priority if the write queue is getting full...
    bool prioWrites = (wrQueue.size() > (3*queueDepth/4)) || (lastCmdWasWrite && wrQueue.size() > queueDepth/4);
    bool isWriteQueue = rdQueue.empty() || prioWrites;

    RequestQueue<Request>& queue = isWriteQueue? wrQueue : rdQueue;
    assert(!queue.empty());

    Request* r = nullptr;
    RequestQueue<Request>::iterator ir = queue.begin();
    uint64_t minSchedCycle = -1ul;
    while (ir != queue.end()) {
        //Bank& bank = banks[(*ir)->loc.rank][(*ir)->loc.bank];
        //if ((isWriteQueue? bank.wrReqs : bank.rdReqs).front() == *ir) {
        if (!(*ir)->prev) {  // FASTAH!. Performs the same thing as the previous commented lines. Check function queue for more details.
            uint64_t minCmdCycle = findMinCmdCycle(**ir); // The minimum cycle at which Column Access command can be issued. The returned value includes both PRE and ACT itmes.
            minSchedCycle = std::min(minSchedCycle, minCmdCycle);
            if (minCmdCycle <= curCycle) {
                r = *ir;
                break;
            }
            //DEBUG("Skipping 0x%lx, not ready %ld", (*ir)->ev->getAddr(), minCmdCycle);
        } else {
            //DEBUG("Skipping 0x%lx, not first", (*ir)->ev->getAddr());
        }
		//printf("ir=%ld\n", (uint64_t)&ir);
        ir.inc();
    }
    if (!r) {
        /* Because we have an event-driven model that uses the same timing
         * constraints to schedule a tick, this rarely happens. For example,
         * refreshes trigger these.
         */
        DEBUG("%ld : First req ready at %ld", curCycle, minSchedCycle);
        return minSchedCycle;  // no requests are ready to issue yet
    }

    DEBUG("%ld : Found ready request 0x%lx %s %ld (%ld / %ld)", curCycle, r->addr, r->write? "W" : "R", r->arrivalCycle, rdQueue.size(), wrQueue.size());

    Bank& bank = banks[r->loc.rank][r->loc.bank];

    // Compute the minimum cycle at which the read or write command can be issued,
    // without column access or data bus constraints
    uint64_t minCmdCycle = std::max(curCycle, minRespCycle - tCL);
    // tWTR : write to read.
    if (lastCmdWasWrite && !r->write) minCmdCycle = std::max(minCmdCycle, minRespCycle + tWTR);
    bool rowHit = false;
    if (r->loc.row == bank.openRow && bank.open) {
        // Row buffer hit
        rowHit = true;
    } else {
        // Either row closed, or row buffer miss
        uint64_t preCycle;
        // preCycle is when PRE can be/was issued.
        // bank.open = false implies PRE has been done.
        bool preIssued = bank.open;
        if (!bank.open) {
            preCycle = bank.minPreCycle;
        } else {
            assert(r->loc.row != bank.openRow);
            preCycle = std::max(r->arrivalCycle, bank.minPreCycle);
        }

        // actCycle is when ACT can be issued for the row since it is row  buffer miss.
        // tRP: PRE to ACT; tRRD: ACT to ACT
        uint64_t actCycle = std::max(r->arrivalCycle, std::max(preCycle + tRP, bank.lastActCycle + tRRD));
        // check findMinCmdCycle for more info on this. 
        actCycle = std::max(actCycle, rankActWindows[r->loc.rank].minActCycle() + tFAW);

        // Record ACT
        bank.open = true;
        bank.openRow = r->loc.row;
        // if PRE is issued(around 20 lines above), reflect it in the bank.
        if (preIssued) bank.minPreCycle = preCycle + tRAS;
        // DRAM has restrictions on the number of row activates that can be issued to a particular bank in a time interval.
        rankActWindows[r->loc.rank].addActivation(actCycle);
        bank.lastActCycle = actCycle;

        minCmdCycle = std::max(minCmdCycle, actCycle + tRCD);
    }

    // Figure out data bus constraints, find actual time at which command is issued
    uint64_t cmdCycle = std::max(minCmdCycle, minRespCycle - tCL);
	// To support accessing granularity greater than a cacheline. 
    //minRespCycle = cmdCycle + tCL + tBL;
    //minRespCycle = cmdCycle + tCL + tBL * r->data_size;
    minRespCycle = cmdCycle + tCL + r->data_size;
    lastCmdWasWrite = r->write;

    // Record PRE
    // if closed-page, close (auto-precharge) if no more row buffer hits
    // if open-page, minPreCycle is used for row buffer misses
    if (closedPage && !(r->next && r->next->rowHitSeq != 0)) bank.open = false;
    bank.minPreCycle = std::max(
            bank.minPreCycle,  // for mixed read and write commands, minPreCycle may not be monotonic without this
            std::max(bank.lastActCycle + tRAS,  // RAS constraint
            r->write? minRespCycle + tWR : cmdCycle + tRTP  // read to precharge for reads, write recovery for writes
            ));

    // Record RD or WR
    assert(bank.lastCmdCycle < cmdCycle);
    bank.lastCmdCycle = cmdCycle;
    bank.curRowHits = r->rowHitSeq;

    // Issue response
    if (r->ev) {
        auto ev = r->ev;
        // Only reads have ev.
        // Writes were stripped off their events in DDRMemory::queue function(above).
        // They were responded to immediately.
        assert(!ev->isWrite() && !r->write);  // reads only

        uint64_t doneSysCycle = memToSysCycle(minRespCycle) + controllerSysLatency;
        assert(doneSysCycle >= sysCycle);

        ev->release();
        ev->done(doneSysCycle - preDelay - postDelayRd);

        uint32_t scDelay = doneSysCycle - r->startSysCycle;
        profReads.inc();
		//if (tBL == 4)
	    //    bytesReads.inc(64 * r->data_size);
		//else if (tBL == 1)
	    //    bytesReads.inc(32 * r->data_size);
		//else 
		//	assert(false);
        bytesReads.inc(16 * r->data_size);
        profTotalRdLat.inc(scDelay);
        if (rowHit) profReadHits.inc();
        uint32_t bucket = std::min(NUMBINS-1, scDelay/BINSIZE);
        latencyHist.inc(bucket, 1);
    } else {
        uint32_t scDelay = memToSysCycle(minRespCycle) + controllerSysLatency - r->startSysCycle;
        profWrites.inc();
        bytesWrites.inc(16 * r->data_size);
		//if (tBL == 4)
        //	bytesWrites.inc(64 * r->data_size);
		//else if (tBL == 1)
        //	bytesWrites.inc(32 * r->data_size);
		//else 
		//	assert(false);

        profTotalWrLat.inc(scDelay);
        if (rowHit) profWriteHits.inc();
    }

    DEBUG("Served 0x%lx lat %ld clocks", r->addr, minRespCycle-curCycle);

    // Dequeue this req
    // from both the queues. Ie., DDRMemory's rd or wr queue and the bank's queue.
    queue.remove(ir);
    (isWriteQueue? bank.wrReqs : bank.rdReqs).pop_front();

    return (rdQueue.empty() && wrQueue.empty())? -1ul : minRespCycle - tCL;
}

void DDRMemory::refresh(uint64_t sysCycle) {
    uint64_t memCycle = sysToMemCycle(sysCycle);
    uint64_t minRefreshCycle = memCycle;
    for (auto& rankBanks : banks) {
        for (auto& bank : rankBanks) {
            minRefreshCycle = std::max(minRefreshCycle, std::max(bank.minPreCycle, bank.lastCmdCycle));
        }
    }
    assert(minRefreshCycle >= memCycle);

    uint64_t refreshDoneCycle = minRefreshCycle + tRFC;
    assert(tRFC >= tRP);
    for (auto& rankBanks : banks) {
        for (auto& bank : rankBanks) {
            // Close and force the ACT to happen at least at tRFC
            // PRE <-tRP-> ACT, so discount tRP
            bank.minPreCycle = refreshDoneCycle - tRP;
            bank.open = false;
        }
    }

    DEBUG("Refresh %ld start %ld done %ld", memCycle, minRefreshCycle, refreshDoneCycle);
}


/* Tech/Device timing parameters */

void DDRMemory::initTech(const char* techName, double time_scale) {
    std::string tech(techName);
    double tCK;

    // tBL's below are for 64-byte lines; we adjust as needed

    // Please keep this orderly; go from faster to slower technologies
    if (tech == "DDR3-1333-CL10") {
        // from DRAMSim2/ini/DDR3_micron_16M_8B_x4_sg15.ini (Micron)
        tCK = 1.5 / 2;  // ns; all other in mem cycles
        tBL = 4;
        tCL = uint32_t(10 / time_scale);
        tRCD = uint32_t( 10 / time_scale);
        tRTP = uint32_t( 5 / time_scale);
        tRP = uint32_t( 10 / time_scale);
        tRRD = uint32_t( 4 / time_scale);
        tRAS = uint32_t( 24 / time_scale);
        tFAW = uint32_t( 20 / time_scale);
        tWTR = uint32_t( 5 / time_scale);
        tWR = uint32_t( 10 / time_scale);
        tRFC = uint32_t( 74 / time_scale);
        tREFI = uint32_t( 5200 / time_scale);
    } else if (tech == "DDR3-1066-CL7") {
        // from DDR3_micron_16M_8B_x4_sg187.ini
        // see http://download.micron.com/pdf/datasheets/dram/ddr3/1Gb_DDR3_SDRAM.pdf, cl7 variant, copied from it; tRRD is widely different, others match
        tCK = 1.875;
        tBL = 4;
        tCL = 7;
        tRCD = 7;
        tRTP = 4;
        tRP = 7;
        tRRD = 4;
        tRAS = 18;
        tFAW = 18;
        tWTR = 4;
        tWR = 7;
        tRFC = 59;
        tREFI = 4160;
    } else if (tech == "DDR3-1066-CL8") {
        // from DDR3_micron_16M_8B_x4_sg187.ini
        tCK = 1.875;
        tBL = 4;
        tCL = 8;
        tRCD = 8;
        tRTP = 4;
        tRP = 8;
        tRRD = 4;
        tRAS = 20;
        tFAW = 20;
        tWTR = 4;
        tWR = 8;
        tRFC = 59;
        tREFI = 4160;
    } else {
        panic("Unknown technology %s, you'll need to define it", techName);
    }

    // Check all params were set
    assert(tCK > 0.0);
    assert(tBL && tCL && tRCD && tRTP && tRP && tRRD && tRAS && tFAW && tWTR && tWR && tRFC && tREFI);

    if (isPow2(lineSize) && lineSize >= 64) {
        tBL = lineSize*tBL/64;
    } else if (lineSize == 32) {
        tBL = tBL/2;
    } else {
        // If we wanted shorter lines, we'd have to start really caring about contention in the command bus;
        // even 32 bytes is pushing it, 32B probably calls for coalescing buffers
        panic("Unsupported line size %d", lineSize);
    }

    memFreqKHz = (uint64_t)(1e9/tCK/1e3);
}

