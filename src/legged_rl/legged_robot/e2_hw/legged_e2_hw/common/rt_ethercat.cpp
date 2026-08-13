//
// Created by han on 2021/6/15.
//

#include "rt_ethercat.h"
#include "EthercatWkcMonitor.h"

#include "ethercat.h"
#include <atomic>
#include <chrono>
#include <exception>
#include <thread>
#include <vector>



std::unordered_map<int, std::shared_ptr<EthercatSlaveBase>> slave_dict;


static char IOmap[4096];
static int expectedWKC;
static boolean inOP;
static int wkc;
// Written and read only by the RT control thread after initialization. It
// describes the IOmap consumed by the following read() call.
static bool last_cycle_valid = false;
static std::atomic_bool ethercat_monitor_running{false};
static std::atomic_bool ethercat_fault_latched{false};
static std::atomic_bool ethercat_fault_event{false};
static std::atomic_int rt_wkc_dropped_pending{0};
static std::atomic_int rt_wkc_dropped_total{0};
static std::thread ethercat_monitor_thread;
static constexpr std::uint32_t kEthercatErrorWindowCycles = 100U;
static constexpr std::uint32_t kEthercatMaxErrorsPerWindow = 20U;
static legged::EthercatWkcMonitor wkc_error_monitor(
    kEthercatErrorWindowCycles, kEthercatMaxErrorsPerWindow);
static legged::EthercatCycleTracker ethercat_cycle_tracker;
// Configured before the RT loop starts and then read only by that thread.
// Keep this below the 2 ms control period so one missing datagram cannot
// consume the entire cycle budget.
static int processdata_receive_timeout_us = 500;

static std::uint64_t steady_now_ns() noexcept
{
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
}

//#define ADAPTER_NAME "eno1"

static bool validate_configured_slave_mapping()
{
    const size_t configured_count = slave_dict.size();
    if (configured_count == 0U)
    {
        printf("[EtherCAT Error] No software slave definitions were configured.\n");
        return false;
    }
    if (ec_slavecount != static_cast<int>(configured_count))
    {
        printf("[EtherCAT Error] Slave count mismatch: configured %zu, discovered %d.\n",
               configured_count, ec_slavecount);
        return false;
    }

    std::vector<bool> used_addresses(configured_count + 1U, false);
    for (size_t index = 0; index < configured_count; ++index)
    {
        const auto entry = slave_dict.find(static_cast<int>(index));
        if (entry == slave_dict.end() || !entry->second)
        {
            printf("[EtherCAT Error] Missing software slave definition at index %zu.\n", index);
            return false;
        }

        const auto& configured_slave = entry->second;
        const uint32_t address = configured_slave->get_address();
        if (address == 0U || address > static_cast<uint32_t>(ec_slavecount) ||
            used_addresses[address])
        {
            printf("[EtherCAT Error] Invalid or duplicate EtherCAT address %u for '%s'.\n",
                   address, configured_slave->get_name().c_str());
            return false;
        }
        used_addresses[address] = true;

        const ec_slavet& mapped_slave = ec_slave[address];
        if (mapped_slave.inputs == nullptr || mapped_slave.outputs == nullptr)
        {
            printf("[EtherCAT Error] Slave %u ('%s') has null PDO mapping: inputs=%p outputs=%p.\n",
                   address, configured_slave->get_name().c_str(),
                   static_cast<void*>(mapped_slave.inputs), static_cast<void*>(mapped_slave.outputs));
            return false;
        }

        const size_t mapped_input_bytes = static_cast<size_t>(mapped_slave.Ibytes) +
            (mapped_slave.Ibits > 0 ? 1U : 0U);
        const size_t mapped_output_bytes = static_cast<size_t>(mapped_slave.Obytes) +
            (mapped_slave.Obits > 0 ? 1U : 0U);
        if (mapped_input_bytes < configured_slave->expectedInputBytes() ||
            mapped_output_bytes < configured_slave->expectedOutputBytes())
        {
            printf("[EtherCAT Error] Slave %u ('%s') PDO size mismatch: "
                   "input %zu/%zu bytes, output %zu/%zu bytes (mapped/required).\n",
                   address, configured_slave->get_name().c_str(),
                   mapped_input_bytes, configured_slave->expectedInputBytes(),
                   mapped_output_bytes, configured_slave->expectedOutputBytes());
            return false;
        }

        if (mapped_input_bytes > configured_slave->expectedInputBytes() ||
            mapped_output_bytes > configured_slave->expectedOutputBytes())
        {
            printf("[EtherCAT Warning] Slave %u ('%s') PDO has a trailing extension: "
                   "input %zu/%zu bytes, output %zu/%zu bytes (mapped/known). "
                   "Unknown trailing bytes will be ignored and output extensions remain zero.\n",
                   address, configured_slave->get_name().c_str(),
                   mapped_input_bytes, configured_slave->expectedInputBytes(),
                   mapped_output_bytes, configured_slave->expectedOutputBytes());
        }
    }
    return true;
}

static int run_ethercat(const char *ifname) {
    int i, oloop, iloop, chk;
    bool socket_initialized = false;
    inOP = FALSE;


    /* initialise SOEM, bind socket to ifname */
    if (ec_init(ifname))
    {
        socket_initialized = true;
        printf("[EtherCAT Init] Initialization on device %s succeeded.\n",ifname);
        /* find and auto-config slaves */

        if ( ec_config_init(FALSE) > 0 )
        {
            printf("[EtherCAT Init] %d slaves found and configured.\n",ec_slavecount);
            if(ec_slavecount < 1)
            {
                printf("[RT EtherCAT] Warning: Expected %d legs, found %d.\n", 1, ec_slavecount);
            }

            /*if ((ec_slavecount >= 1)) {
                for (int cnt = 1; cnt <= ec_slavecount; cnt++) {
                    printf("Found %s at position %d\n", ec_slave[cnt].name, cnt);
                    ec_slave[cnt].PO2SOconfig = &slave_dc_config;
                }
            }*/

            const int mapped_bytes = ec_config_map(&IOmap);
            if (mapped_bytes <= 0 || mapped_bytes > static_cast<int>(sizeof(IOmap)))
            {
                printf("[EtherCAT Error] Invalid PDO map size %d (buffer capacity %zu).\n",
                       mapped_bytes, sizeof(IOmap));
                ec_close();
                return 0;
            }
            if (!validate_configured_slave_mapping())
            {
                ec_close();
                return 0;
            }
            ec_configdc();

            printf("[EtherCAT Init] Mapped slaves.\n");
            /* wait for all slaves to reach SAFE_OP state */
            ec_statecheck(0, EC_STATE_SAFE_OP,  EC_TIMEOUTSTATE * 4);

            for(int slave_idx = 1; slave_idx <= ec_slavecount; slave_idx++) {
                printf("[SLAVE %d]\n", slave_idx);
                printf("  IN  %d bytes, %d bits\n", ec_slave[slave_idx].Ibytes, ec_slave[slave_idx].Ibits);
                printf("  OUT %d bytes, %d bits\n", ec_slave[slave_idx].Obytes, ec_slave[slave_idx].Obits);
                printf("  %s \n", ec_slave[slave_idx].name);
                printf("\n");
            }

            oloop = ec_slave[0].Obytes;
            if ((oloop == 0) && (ec_slave[0].Obits > 0)) oloop = 1;
            if (oloop > 8) oloop = 8;
            iloop = ec_slave[0].Ibytes;
            if ((iloop == 0) && (ec_slave[0].Ibits > 0)) iloop = 1;
            if (iloop > 8) iloop = 8;

            printf("[EtherCAT Init] segments : %d : %d %d %d %d\n",ec_group[0].nsegments ,ec_group[0].IOsegment[0],ec_group[0].IOsegment[1],ec_group[0].IOsegment[2],ec_group[0].IOsegment[3]);

            printf("[EtherCAT Init] Requesting operational state for all slaves...\n");
            expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
            printf("[EtherCAT Init] Calculated workcounter %d\n", expectedWKC);
            if (expectedWKC <= 0)
            {
                printf("[EtherCAT Error] Invalid expected workcounter %d.\n", expectedWKC);
                ec_close();
                return 0;
            }
            ec_slave[0].state = EC_STATE_OPERATIONAL;
            /* send one valid process data to make outputs in slaves happy*/
            ec_send_processdata();
            ec_receive_processdata(EC_TIMEOUTRET);
            /* request OP state for all slaves */
            ec_writestate(0);
            chk = 40;
            /* wait for all slaves to reach OP state */
            do
            {
                ec_send_processdata();
                ec_receive_processdata(EC_TIMEOUTRET);
                ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
            }
            while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

            ec_readstate();
            bool all_slaves_operational = ec_slave[0].state == EC_STATE_OPERATIONAL;
            for(i = 1; i <= ec_slavecount; ++i)
            {
                if(ec_slave[i].state != EC_STATE_OPERATIONAL)
                {
                    all_slaves_operational = false;
                    printf("[EtherCAT Error] Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                           i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                           ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                }
            }

            if (all_slaves_operational)
            {
                bool valid_workcounter_seen = false;
                for (int sample = 0; sample < 5; ++sample)
                {
                    ec_send_processdata();
                    const int initial_wkc = ec_receive_processdata(EC_TIMEOUTRET);
                    if (initial_wkc >= expectedWKC)
                    {
                        valid_workcounter_seen = true;
                        break;
                    }
                }
                if (valid_workcounter_seen)
                {
                    printf("[EtherCAT Init] Operational state and valid workcounter reached for all slaves.\n");
                    inOP = TRUE;
                    return 1;
                }
                printf("[EtherCAT Error] Operational state reached, but no valid workcounter was received.\n");
            }
            else
            {
                printf("[EtherCAT Error] Not all slaves reached operational state.\n");
            }
        }
        else
        {
            printf("[EtherCAT Error] No slaves found!\n");
        }
    }
    else
    {
        printf("[EtherCAT Error] No socket connection on %s - are you running real_23dof.sh?\n",ifname);
    }
    // ec_init() may have opened a raw socket even when discovery or OP-state
    // negotiation failed. Close it before a retry and before returning failure.
    if (socket_initialized)
    {
        ec_close();
    }
    inOP = FALSE;
    return 0;
}

// The monitor deliberately consumes atomic health snapshots only. All SOEM
// state and API access remains owned by the RT control thread after init.
static void ecatcheck()
{
    while(ethercat_monitor_running.load(std::memory_order_acquire))
    {
        const int dropped_packets = rt_wkc_dropped_pending.exchange(0, std::memory_order_relaxed);
        if(dropped_packets > 0)
        {
            printf("[EtherCAT Error] Dropped %d packet(s) in RT loop, total %d (Bad WKC!).\n",
                   dropped_packets, rt_wkc_dropped_total.load(std::memory_order_relaxed));
        }

        if(ethercat_fault_event.exchange(false, std::memory_order_acq_rel))
        {
            printf("[EtherCAT Error] Error count too high; EtherCAT fault latched. "
                   "Automatic recovery is disabled to keep SOEM single-threaded.\n");
        }
        osal_usleep(50000);
    }
}

bool rt_ethercat_init(const std::string& if_name, int receive_timeout_us)
{
    printf("[EtherCAT] Initializing EtherCAT\n");
    if (receive_timeout_us < 50 || receive_timeout_us > 1500)
    {
        printf("[EtherCAT Error] receive_timeout_us must be in [50, 1500], got %d.\n",
               receive_timeout_us);
        return false;
    }
    processdata_receive_timeout_us = receive_timeout_us;
    if (ethercat_monitor_thread.joinable() ||
        ethercat_monitor_running.load(std::memory_order_acquire) || inOP)
    {
        printf("[EtherCAT Error] Initialization requested while EtherCAT is already active.\n");
        return false;
    }

    ethercat_monitor_running.store(false, std::memory_order_release);
    ethercat_fault_event.store(false, std::memory_order_release);
    rt_wkc_dropped_pending.store(0, std::memory_order_relaxed);
    rt_wkc_dropped_total.store(0, std::memory_order_relaxed);
    expectedWKC = 0;
    wkc = 0;
    last_cycle_valid = false;
    wkc_error_monitor.reset();
    ethercat_cycle_tracker.reset();

    int successful_attempt = 0;
    for(int attempt = 1; attempt <= 3; ++attempt)
    {
        printf("[EtherCAT] Attempting to start EtherCAT, try %d of 3.\n", attempt);
        if (run_ethercat(if_name.c_str()))
        {
            successful_attempt = attempt;
            break;
        }
        if (attempt < 3)
        {
            osal_usleep(1000000);
        }
    }

    if (successful_attempt == 0)
    {
        printf("[EtherCAT Error] Failed to initialize EtherCAT after 3 tries.\n");
        ethercat_fault_latched.store(true, std::memory_order_release);
        return false;
    }

    printf("[EtherCAT] EtherCAT successfully initialized on attempt %d.\n", successful_attempt);
    ethercat_fault_latched.store(false, std::memory_order_release);
    ethercat_monitor_running.store(true, std::memory_order_release);
    try
    {
        ethercat_monitor_thread = std::thread(ecatcheck);
    }
    catch (const std::exception& exception)
    {
        printf("[EtherCAT Error] Failed to start monitor thread: %s.\n", exception.what());
        ethercat_monitor_running.store(false, std::memory_order_release);
        ethercat_fault_latched.store(true, std::memory_order_release);
        ec_close();
        inOP = FALSE;
        return false;
    }

    return true;
}

void rt_ethercat_shutdown()
{
    last_cycle_valid = false;
    ethercat_cycle_tracker.markShutdown(steady_now_ns());
    ethercat_monitor_running.store(false, std::memory_order_release);
    if (ethercat_monitor_thread.joinable()) {
        ethercat_monitor_thread.join();
    }
    if (inOP) {
        ec_close();
        inOP = FALSE;
    }
}

bool rt_ethercat_faulted()
{
    return ethercat_fault_latched.load(std::memory_order_acquire);
}

bool rt_ethercat_last_cycle_valid() noexcept
{
    return last_cycle_valid;
}

legged::EthercatCycleSnapshot rt_ethercat_cycle_snapshot() noexcept
{
    return ethercat_cycle_tracker.snapshot();
}

void rt_ethercat_run()
{
    //send
    //command_mutex.lock();
    ec_send_processdata();
    //command_mutex.unlock();

    //receive
    //data_mutex.lock();
    wkc = ec_receive_processdata(processdata_receive_timeout_us);
    const bool cycle_valid = expectedWKC > 0 && wkc >= expectedWKC;
    //data_mutex.unlock();

    //check for dropped packet
    if(!cycle_valid)
    {
        rt_wkc_dropped_pending.fetch_add(1, std::memory_order_relaxed);
        rt_wkc_dropped_total.fetch_add(1, std::memory_order_relaxed);
    }

    // Evaluate this sample before rolling the fixed window. In particular, the
    // 21st error on cycle 100 must latch instead of being cleared at the edge.
    if (wkc_error_monitor.observe(cycle_valid))
    {
        const bool was_faulted = ethercat_fault_latched.exchange(true, std::memory_order_acq_rel);
        if (!was_faulted) {
            ethercat_fault_event.store(true, std::memory_order_release);
        }
    }

    const auto& snapshot = ethercat_cycle_tracker.observe(
        wkc, expectedWKC, steady_now_ns(),
        ethercat_fault_latched.load(std::memory_order_acquire));
    last_cycle_valid = snapshot.dataValid;
}

void rt_ethercat_get_data() {
    //data_mutex.lock();
    //printf("enter here");
    for (int i = 0; i < slave_dict.size(); ++i) {
        slave_dict[i]->updateRead();
    }
    //data_mutex.unlock();

}
void rt_ethercat_set_command() {

    //command_mutex.lock();
    for (int i = 0; i < slave_dict.size(); ++i) {
        slave_dict[i]->updateWrite();
    }
    //command_mutex.unlock();
}

/*void rt_ethercat_config()
{
    /*std::string name_1("last_leg");
    std::shared_ptr<EthercatSlaveBase> tmp = std::make_shared<LegBoard>(name_1, 0);
    slave_dict[0] = tmp;

    std::string name_2("head_leg");
    std::shared_ptr<EthercatSlaveBase> tmp_1 = std::make_shared<LegBoard>(name_2, 1);
    slave_dict[1] = tmp_1;

    std::string name_3("imu_rc");
    std::shared_ptr<EthercatSlaveBase> tmp_2 = std::make_shared<ImuRc>(name_3, 2);
    slave_dict[2] = tmp_2;*/

    /*std::string name_3("imu_rc");
    std::shared_ptr<EthercatSlaveBase> tmp_2 = std::make_shared<ImuRc>(name_3, 0);
    slave_dict[0] = tmp_2;*/

    /*std::string name_2("head_leg");
    std::shared_ptr<EthercatSlaveBase> tmp_1 = std::make_shared<LegBoard>(name_2, 2);
    slave_dict[1] = tmp_1;

    std::string name_1("last_leg");
    std::shared_ptr<EthercatSlaveBase> tmp = std::make_shared<LegBoard>(name_1, 1);
    slave_dict[0] = tmp;


    std::string name_3("imu_rc");
    std::shared_ptr<EthercatSlaveBase> tmp_2 = std::make_shared<ImuRc>(name_3, 3);
    slave_dict[2] = tmp_2;

}*/
