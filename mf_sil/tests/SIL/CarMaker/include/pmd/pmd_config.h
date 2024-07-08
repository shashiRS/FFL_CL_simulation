#include "MfSilTypes.h"
#include "Platform_Types.h"

namespace pmd {

    //! List of possible camera ID's
    typedef enum CAMERAIDX {
        CAM_MIN = 0,
        CAM_FRONT = CAM_MIN,
        CAM_RIGHT,
        CAM_REAR,
        CAM_LEFT,
        CAM_FIFTH, /**< customer requested the ability to plug in an additional camera
                      with an undefined viewing angle. */
       // CAM_MAX,
      //  MAX_CAMIDX_VALUE =
      /*  0xFFFFFFFF *//**< MAX_VALUE is added due to difference in the sizeof enum in
                    M4 core and DSP
                    core, by adding MAX_VALUE the size will be four bytes in the
                    both the cores */
    }CAMERAIDX_t;
    /**
 * Line Segmentation Configuration
 */
    typedef struct LineSegmentConfig
    {
        //// Histogram stretching
        //float32_t    percentageToSaturate;  //!< Threshold saturate value */
        //uint8_t      minHmaxLevel;          //!< Minimum of histogram maximum value */

        // ROI for thresholding
        //it's in meters not like in pmd_alogrithm where it is in pixels
        float32_t roiThresholdingX_m;      // X [m] of the ROI for thresholding
        float32_t roiThresholdingY_m;      // Y [m] of the ROI for thresholding
        float32_t roiThresholdingWidth_m;  // Width [m] of the ROI for thresholding
        float32_t roiThresholdingHeight_m; // Height [m] of the ROI for thresholding

        //// Thresholding
        //uint8_t    threshold;   //!< Binary threshold value */

        //// TopHat Choice
        //uint8_t    topHatMode;   //!< Two modes to choose: 0 and default = serial TopHatfrom, 1 = parallel TopHat in horizontal and vertical applied to input images */
    } LineSegmentConfig_t;

    /**
 * Pmd Configuration
 */
    typedef struct PmdCameraConfig
    {
        pmd::CAMERAIDX          camId;    //!< Camera ID (direction) */
        bool        cameraAvailability;
        float32_t    topDownViewHeight_m;     //!< In the direction of the camera */
        float32_t    topDownViewWidth_m;      //!< Perpendicular to the camera */

        //int32_t     topDownShiftX_mm;         //!< topdown window shifting in pixel in horizontal direction */
        //int32_t     topDownShiftY_mm;         //!< topdown window shifting in pixel in vertical direction */

        pmd::LineSegmentConfig   lineSegmentConfig;  //!< Line segment configuration */

        //uint16_t    imageWidth;  //!< image width */
        //uint16_t    imageHeight; //!< image height */

        //alg::CameraIntrinsics   cam_intr; //!< Camera intrinsics */
        //alg::CameraExtrinsics   cam_extr; //!< Camera extrinsics */

        // Top Down View
        // bool        pmdTopDownImageConverter; //!< // 0 = TopDown from CHIPS Component 1 = PMD internal topDown calculation

        
        //uint16_t    topDownViewResolution_mm; //!< Resolution value */
        //uint32_t    topDownBoundaryTolerance_pix;  //!< Tolerance to discern if line ends are intersecting with top down boundary

        // Configuration
        //GaussianConfig      gaussianConfig;     //!< Gaussian configuration */
        //TophatConfig        tophatConfig;       //!< Tophat configuration */
       
        //ThinningConfig      thinningConfig;     //!< Thinning configuration */
        //HoughConfig         houghConfig;        //!< Hough configuration */
        //LineMappingConfig   lineMappingConfig;  //!< Line mapping configuration */
        //uint8_t             useStableAgorithms;  //!< Configuration for using stable algorithms */
        //uint8_t             enableConfidenceMap; //!< Configuration for enabling confidence map */
        //uint8_t             enableShapeDetection; //!< Configuration for enabling shape detection */
        //uint8_t             useEmlPoseInterpolator; //!< Enable using of the EML interpolation ov the vehicle position.
        //uint8_t             useSplitLines;          //!< Config for enabling line splitting at line intersections.
        //uint8_t             enableAutoThresholdingROIPosition; //!< Enable automatic computing the position of the ROI of thresholding.
        //uint8_t             useCnnLineFusion;       //!< Use CNN image to refine line detection.

        //// GetLineID
        //float32_t  tolerenceX_mm;  //!<Tolerance of LineId in the X direction */
        //float32_t  tolerenceY_mm;  //!<Tolerance of LineId in the Y direction */
        //float32_t tolerenceTheta_deg;  //!<Tolerance of Theta for LineId */

        //// Binarisation
        //uint8_t     binValuemin; //!< Minimum value of binary */
        //uint8_t     binValuemax; //!< Maximum value of binary */

        //float32_t    thetaToleranceDeg; //!< line splitting tolerance angle */
        //float32_t    minLineSplittingLength; //!< minimal line segment length accepted after the splitting.

        //float32_t   radiusAroundCar; //!< Radius around car */
        //LineSegmentShapesConfig lsShapesConfig; //!< Line segment based shapes detection configuration

        //float32_t wheelbase_mm; //!< distance between the ego-vehicle axles, in millimeters.
        //uint8_t convertVedodoPositionToASL; //!< do the input position converstion from Frankfurt to ASL coordinates origin.
        //float32_t steeringAngleBiasCorrection_rad; //!< add this to the mInData.pose.pose_rot_rad.

        //// Peaks tracking
        //PeakTrackingConfig peakTracking;
    }PmdCameraConfig_t;

    typedef struct PepDemoConf {
        uint32_t pmdmr;
        uint32_t pmdml;
        uint32_t pmdfv;
        uint32_t pmdrv;
        uint32_t sfm;
        uint32_t uss;
        uint32_t cem;
        uint32_t si;
    }PepDemoConf_t;
    
    typedef struct PmdConfig {
        pmd::PmdCameraConfig leftPmdCamera{};
        pmd::PmdCameraConfig rightPmdCamera{};
        pmd::PmdCameraConfig frontPmdCamera{};
        pmd::PmdCameraConfig rearPmdCamera{};

        void init() {
            leftPmdCamera.camId = pmd::CAMERAIDX::CAM_LEFT;
            rightPmdCamera.camId = pmd::CAMERAIDX::CAM_RIGHT;
            frontPmdCamera.camId = pmd::CAMERAIDX::CAM_FRONT;
            rearPmdCamera.camId = pmd::CAMERAIDX::CAM_REAR;
        }
    }PmdConfig_t;
}