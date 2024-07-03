#ifndef _CARMAKERNODE_CONTROL_H__
#define _CARMAKERNODE_CONTROL_H__

#include <functional>
#include "stdint.h"
#include "SimCore.h"
#include <chrono>
#include "sim_logger.h"
#include <deque>
/// Framework Base Class
#include <ecal/ecal.h>
#include <ecal/ecal_publisher.h>
#include <memory>

/// GENERAL TIMESTAMP, CYCLE NUMBER
extern uint64_t timestamp;
extern uint16_t cyclenumber;
extern SimLogger* simLogger;
extern uint32_t start_system_time;
using namespace std::chrono_literals;
using namespace std::chrono;

/*!
 * @brief CarmakerNode class that provides functions to communicates with Carmaker
*/
class CarMakerNode : public std::enable_shared_from_this<CarMakerNode>
{
    bool mCemSimulationActive{ false };

public:
    /// Funtion to get the Reference Carmaker Node 
    static std::shared_ptr<CarMakerNode> getReferenceCarmakerNode () {
        static std::shared_ptr<CarMakerNode> Carmakernode = std::make_shared<CarMakerNode>("CarMaker_CEM_Node");
        return Carmakernode; 
    }
    /// Default constructor of CarmakerNode Class
    CarMakerNode(const std::string & node_name) { };
    /// Destructor of CarmakerNode Class
    ~CarMakerNode() { }
public:
    /*!
     * @brief CarmakerNode Initialization for creation of publishers and subscribers for eCAL 
     * @param  
    */
    void Init(void);
    /*!
     * @brief 
     * @param  
    */
    void TestRun_Start(void);
    /*!
     * @brief Calc function to start the CEM components to publish the data
     * @param  
    */
    void Calc(void);
    /*!
     * @brief TestRun_End function to reset the timestamp and also reset the 
              publishers of all the CEM components
     * @param  
    */
    void TestRun_End(void);
    /*!
     * @brief 
     * @param  
    */
    void MemDeclquant(void);
    /*!
     * @brief Carmaker Node and eCAL shutdown
     * @param  
    */
    void Shutdown(void);

    /*!
     * @brief Set whether the CarMaker simulation is executed with the CEM_LSM component.
    */
    void SetCemSimulationActive(const unsigned char cemSimulationActive);

    CarMakerNode(CarMakerNode const&) = delete; // prevent copying the singleton
    CarMakerNode& operator= (CarMakerNode const&) = delete;
};

/*!
 * @brief Templated Carmaker Publisher class to create, fill, publish and reset the data
 * @tparam PT Message structure with which the data is created and published
 * @tparam N1 Cycle time
 * @tparam N2 Latency time
 * @param  
 * @return 
*/
template<typename PT, typename N1 = uint64_t, typename N2 = uint16_t>
class CarMakerNode_Publisher_eCAL
{
public:
    /// Publish Cyclenumber for the respective SIP Components
    uint16_t publish_cyclenumber = 0;
protected:
    /// Templated function for the respective SIP Components
    std::function<void(typename PT &,typename N1,typename N2)> Fill_Publisher_Input;
    /// eCAL Publisher for SIP Componets 
    eCAL::CPublisher publisher;
    /// Publisher name for the SIP Components 
    std::string publisher_name;
    /// Sensor Loop time for the respective sensors
    int SensorLoopTime;
	/// Shift time for Sensor
	int ShiftTime;
    /// Variable to determone the size of the message for respective SIP Components
    typename PT publisher_size;
    /// Message to be sent for respective SIP Components
    typename PT* publisher_message = new PT;
   
#if CAN_SENSOR_CEM_LATENCY
    /// Publisher queue to be filled for the respective component
    std::deque<typename PT> publisher_queue;
    typename PT* publisher_delayed_message = new PT;
    uint16_t  *transfer_time;
#endif
public:
    /*!
     * @brief Constructor to create the class CarMakerNode_Publisher_eCAL with parameters
     * @param publishername Name of the publisher to be published in the eCAL
     * @param fill_publisher_fcn Function to call for filling of the data for the respective sensor
     * @param cycletime The sensor loop time for the respective sensor
     * @param input_transfer_time Transfer time for the respective sensor
    */
    CarMakerNode_Publisher_eCAL(std::string publishername, std::function<void(typename PT &,typename N1 ,typename N2 )> fill_publisher_fcn, int cycletime, int starttime
    #if defined(CAN_SENSOR_CEM_LATENCY)
    ,uint16_t  *input_transfer_time
    #endif
    )
    {
        Fill_Publisher_Input = fill_publisher_fcn;
        SensorLoopTime = cycletime;
		ShiftTime = starttime;
        publisher = eCAL::CPublisher(publishername);

        publisher_name = publishername;
        #if defined(CAN_SENSOR_CEM_LATENCY)
        transfer_time = input_transfer_time;
        #endif
    }
    /*!
     * @brief Publisher function to publish the data of respective sensor in eCAL
     * @param  
    */
    void publish_data(void)
    {   
        #if defined(CAN_SENSOR_CEM_LATENCY)
        uint32_t cycle_compensation = uint32_t((*transfer_time + SensorLoopTime) / SensorLoopTime);
        uint64_t cycletime = (std::max(int(publish_cyclenumber - cycle_compensation), int(0))) * SensorLoopTime;
        if (timestamp > 0 && !publisher_queue.empty())
        {
            int test_loop_ = (cycletime + *transfer_time + SensorLoopTime);
            if ((timestamp % (cycletime + *transfer_time + SensorLoopTime) == 0))
            {
                publisher.Send(reinterpret_cast<unsigned char *>(publisher_delayed_message), sizeof(publisher_size));
                publisher_queue.pop_back();
                auto system_time = std::chrono::duration_cast<std::chrono::milliseconds>(system_clock::now().time_since_epoch()) % 1000;
                simLogger->logExecution(system_time.count() - start_system_time, 1000 * SimCore.Time, uint32_t(publish_cyclenumber), "Topic " + publisher_name, "CarMaker Publish", 0);
            }
        }
        #else
        if (SimCore.Time > 0 && (static_cast<int>(std::round(SimCore.Time * 1000) - ShiftTime) % SensorLoopTime == 0))
        {
            publisher.Send(reinterpret_cast<unsigned char*>(publisher_message), sizeof(publisher_size));
            auto system_time = std::chrono::duration_cast<std::chrono::microseconds>(system_clock::now().time_since_epoch());
            simLogger->logExecution(system_time.count() - start_system_time, 1000 * SimCore.Time, uint32_t(publish_cyclenumber), "Topic " + publisher_name, "CarMaker Publish", 0);           
        }
        #endif

    }

    /*!
     * @brief Function to fill the message of the respective sensors
     * @param  
    */
    void fill_message(void)
    {
        //Log("fill_message template %d", int(SimCore.Time));
        if (SimCore.Time <= 0)
        {
            publish_cyclenumber = 0;
        }
        //int test_time = static_cast<int>(std::round(SimCore.Time * 1000)) % SensorLoopTime;
        if (SimCore.Time > 0 && (static_cast<int>(std::round(SimCore.Time * 1000) - ShiftTime) % SensorLoopTime == 0))
        {
            publish_cyclenumber++;
            Fill_Publisher_Input(*publisher_message, timestamp, publish_cyclenumber);
    #if defined(CAN_SENSOR_CEM_LATENCY)
            publisher_queue.push_front(*publisher_message);
        }
        uint32_t cycle_compensation = uint32_t((*transfer_time + SensorLoopTime) / SensorLoopTime);
        uint32_t cycletime = (std::max(int(publish_cyclenumber - cycle_compensation), int(0))) * SensorLoopTime;
        if (timestamp > 0 && !publisher_queue.empty())
        {
            if ((timestamp % (cycletime + *transfer_time + SensorLoopTime) == 0))
            {
                *publisher_delayed_message = publisher_queue.front();
            }
        }
    #else
        }
    #endif
    }

    /*!
     * @brief Function to get the message pointer
     * @return Returns the message pointer to the publisher message
    */
    typename PT* get_message(void)
    {
        #if defined(CAN_SENSOR_CEM_LATENCY)
        return publisher_delayed_message;
        #else
        return publisher_message;
        #endif
    }

    /*!
     * @brief Function to reset the publisher messages
     * @param
    */
    void reset_messages(void)
    {
        #if defined(CAN_SENSOR_CEM_LATENCY)
        memset(publisher_delayed_message, 0, sizeof(*publisher_delayed_message));
        #endif
        memset(publisher_message, 0, sizeof(*publisher_message));
    }

    /*!
     * @brief Function to reset the publisher queue
     * @param  
    */
    void reset_queues(void) 
    {
        #if defined(CAN_SENSOR_CEM_LATENCY)
        while (!publisher_queue.empty())
            publisher_queue.pop_front();
        #endif
    }
};
#endif	/* #ifndef _CARMAKERNODE_CONTROL_H__ */
