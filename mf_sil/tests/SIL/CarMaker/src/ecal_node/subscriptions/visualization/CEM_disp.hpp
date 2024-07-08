/*****************************************************************************
Description   : Visualization of CEM_LSM outputs (Subscription and deliver to
                IPG Control and MF_Plotter)
Created by    : Zhizhe02 Jia
Co-Author     : Hannes Brauckmann
Company       : Continental Autonomous Mobility Germany GmbH
*****************************************************************************/

#pragma once
#ifndef __INCLUDE_CEM_DISP_HPP
#define __INCLUDE_CEM_DISP_HPP
#ifdef USE_ENV_PLOTTER
#include "mf_plot/MF_PlotterSi.h"
#include "mf_plot/MF_PlotterCem.h"
#endif
#include "aupdf/aupdf_generated_types.h"

extern aupdf::DynamicEnvironment *dynamic_environment_subscriber;
extern aupdf::EgoMotionAtCemOutput *ego_motion_at_cem_output_subscriber;
extern aupdf::ParkingSlotDetectionOutput *parking_slot_detection_output_subscriber;
extern aupdf::PclOutput *pcl_output_subscriber;
extern aupdf::SgfOutput *sgf_output_subscriber;
extern aupdf::StopLineOutput *stop_Line_output_subscriber;
extern aupdf::PedestrianCrossingOutput *pedes_cross_output_subscriber;

#ifdef USE_ENV_PLOTTER
void CEMSubscribeStaticObjects2Plotter(const aupdf::SgfOutput *sgfOutput_sub, MF_Plot::plotterCemObjectList& cemObjects);
void CEMSubscribeLines2Plotter(const aupdf::PclOutput *pclOutput_disp, MF_Plot::plotterCemLineList& cemLines);
void CEMSubscribeDynObj2Plotter(const aupdf::DynamicEnvironment *dyn_evn_sub, MF_Plot::DYN_OBJ_LIST_FROM_CEM& cemDynObjects);
void CEMSubscribeODSlots2Plotter(const aupdf::ParkingSlotDetectionOutput *parking_slot_detection, MF_Plot::OD_SLOT_LIST_FROM_CEM& cemODSlots);
#endif

void registerCemOutputsAsDvaVariables(
    aupdf::DynamicEnvironment *dynamicEnvironment,
    aupdf::EgoMotionAtCemOutput *egoMotionAtCemOutput,
    aupdf::ParkingSlotDetectionOutput *parkingSlotDetectionOutput,
    aupdf::PclOutput *pclOutput,
    aupdf::SgfOutput *sgfOutput,
    aupdf::StopLineOutput *stopLineOutput,
    aupdf::PedestrianCrossingOutput *pedestrianCrossingOutput);
#endif /* __INCLUDE_CEM_DISP_HPP */
