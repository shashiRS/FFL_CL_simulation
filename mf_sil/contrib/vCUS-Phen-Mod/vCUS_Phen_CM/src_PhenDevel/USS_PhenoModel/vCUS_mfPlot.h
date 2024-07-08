#include <stdint.h>

#define _MAX_ARR_VCUS2MFPLOT_ 128 // make sure it's <= NUM_CUS_PTS_PER_ID in MF_PlotterCus.h
#define _MF_PLOT_HISTORY_ 100 // n of 40 ms cycles: 100 = 4 s history to display; <= NUMBER_CUS_TRACES in MF_PlotterCus.h

typedef struct tNpDataMfPlot {
	float		CoordArrayFr1[_MAX_ARR_VCUS2MFPLOT_][2];							// array containing the Nearest points coordinates in FR1 for mf_plot (mf_plot uses floats)
	float		CoordArrayFr0[_MAX_ARR_VCUS2MFPLOT_][2];							// array containing the Nearest points coordinates in FR0 for mf_plot
	int			nReflPnts;															// number of actual  points in array
	uint8_t		mfp_CycleId;														// cycle id - keeps track of 40ms cycle history for mf_Plot 0 to (_MF_PLOT_HISTORY_-1)
	int			vCUS_CycleId;														// cycle id - keeps track of 40ms absolute cycle count from vCUS
} tNpDataMfPlot;
//__declspec(dllexport) tNpDataMfPlot *MfPlotDataStru;
extern tNpDataMfPlot *MfPlotDataStru;
extern int uspExec_counter;
