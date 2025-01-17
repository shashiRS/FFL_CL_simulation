// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef AP_COMMONVEHSIGPROVIDER_GPSDATA_H_
#define AP_COMMONVEHSIGPROVIDER_GPSDATA_H_

#include "Platform_Types.h"
#include "ap_commonvehsigprovider/hemisphere.h"
#include "ap_commonvehsigprovider/gps_receiver_status.h"


namespace ap_commonvehsigprovider
{

  /// Data specifications of the GPS System.
  struct GPSData
  {
    ///@unit{m}
    ///@range{0,99999.9}
    ///
    float32 gpsAntennaHeight_m{};
    ///@unit{deg}
    ///@range{0,90}
    ///
    sint32 gpsLatitude_dd{};
    ///@unit{min}
    ///@range{0,60}
    ///
    float32 gpsLatitude_mm{};
    ///@unit{grad}
    ///@range{0,180}
    ///
    sint32 gpsLongitude_dd{};
    ///@unit{min}
    ///@range{0,60}
    ///
    float32 gpsLongitude_mm{};
    ///@unit{m/s}
    ///@range{0,500}
    ///GPS speed over ground
    float32 gpsSpeed_mps{};
    ///@unit{m/s}
    ///@range{0,500}
    ///GPS speed over ground. (Filled signal)
    float32 gpsR32SpeedOverGround_mps{};
    ///@unit{rad}
    ///@range{0,6.28}
    ///
    float32 gpsCourseOverGround{};
    ///@unit{nu}
    ///@range{0,25.575}
    ///Vertical Dilution Of Precision (VDOP) and its is a value of probability for the height inaccuracy effect on GPS accuracy
    float32 verticalDOP{};
    ///@unit{nu}
    ///@range{0,25.575}
    ///Horizontal Dilution Of Precision (HDOP) and it is a value of probability for the horizontal geometric effect (latitude and longitude) on GPS accuracy.
    float32 horizontalDOP{};
    ///@unit{nu}
    ///@range{0,25.575}
    ///Time Dilution Of Precision (TDOP) and its is a value of probability for the time inaccuracy effect on GPS accuracy.
    float32 timeDOP{};
    ///@unit{nu}
    ///@range{0,25.575}
    ///Geometric dilution of precision (Overall accuracy including 3D position und Time)
    float32 geometricDOP{};
    ///@unit{nu}
    ///@range{0,25.575}
    ///Positional dilution of precision (Position accuracy, 3D)
    float32 positionDOP{};
    ///@unit{std}
    ///@range{0,24}
    ///
    uint16 gpsUtcTime_hh{};
    ///@unit{min}
    ///@range{0,59}
    ///
    uint16 gpsUtcTime_mm{};
    ///@unit{sec}
    ///@range{0,59}
    ///
    uint16 gpsUtcTime_ss{};
    ///
    Hemisphere gpsLatitudeHemisphere_nu{};
    ///
    Hemisphere gpsLongitudeHemisphere_nu{};
    ///@unit{day}
    ///@range{0,31}
    ///
    uint8 gpsDateDay_dd{};
    ///@unit{month}
    ///@range{0,12}
    ///
    uint8 gpsDateMonth_mm{};
    ///@unit{year}
    ///@range{0,99}
    ///
    uint8 gpsDateYear_yy{};
    ///@unit{nu}
    ///@range{0,3}
    ///Dimension accuracy of current GPS measurement. (ex: 2 -> 2D, 3 -> 3D)
    uint8 gpsFix{};
    ///@unit{nu}
    ///@range{0,255}
    ///GPS number of satellites
    uint8 gpsNoOfSatellites{};
    ///GPS Receiver status
    GpsReceiverStatus ReceiverStatus_nu{};
  };

} // namespace ap_commonvehsigprovider

#endif // AP_COMMONVEHSIGPROVIDER_GPSDATA_H_
