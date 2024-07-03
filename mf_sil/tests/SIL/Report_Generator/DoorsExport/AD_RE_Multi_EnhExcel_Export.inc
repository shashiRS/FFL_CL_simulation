// ADAS RM DXL Enhanced Export to Excel for DOORS 7.x
/**
 * Created by: daeumlingd
 * Creation date: 04.03.2011
 *
 * DXL Editor is provided by SODIUS (www.dxleditor.com)
 */

/*************************************************************************

  COMPANY:                Advanced Driver Assistance Systems (ADAS)
                          Passive Safety & ADAS (PSAD), A.D.C. GmbH

  DEPARTMENT:             System RM (C PSAD A D X2)

  PROJECT:                Generic

  MODULENAME:             $Source: AD_RE_Multi_EnhExcel_Export.inc $

  DESCRIPTION:            ADAS RM DXL coding module template

  VERSION:                $Revision: 1.6 $


  ---*/ /*---
  CHANGES:                $Log: AD_RE_Multi_EnhExcel_Export.inc  $
  CHANGES:                Revision 1.6 2019/10/10 13:14:50CEST M, Nandhini (uib49278) (uib49278) 
  CHANGES:                Disabled exporting ole objects and pictures
  CHANGES:                Revision 1.5 2018/09/12 09:50:48CEST Horber, Harald (horberh) 
  CHANGES:                synchronisation with "...Multi_Enhanced_Export_MD"
  CHANGES:                Revision 1.2 2016/08/10 12:31:41CEST Horber, Harald (horberh) 
  CHANGES:                analyze _Project standard attribute
  CHANGES:                Revision 1.1 2016/02/22 09:21:30CET Horber, Harald (horberh) 
  CHANGES:                Initial revision
  CHANGES:                Member added to project /nfs/projekte1/org/10_Requirements_Management/tool/doors/scripts/public/dxl/ADAS_Includes/Common/project.pj
  CHANGES:                Revision 1.2 2011/07/21 13:02:12CEST uidt5578 
  CHANGES:                Updated to Version 2.3 from "Michael Sutherland - Galactic Solutions Group, LLC"
  CHANGES:                --- Added comments ---  uidt5578 [Jul 21, 2011 1:02:12 PM CEST]
  CHANGES:                Change Package : :bypass http://mks-psad:7002/im/viewissue?selection=
  CHANGES:                Revision 1.1 2011/03/04 09:28:16CET Daeumling Dirk (daeumlingd) (daeumlingd) 
  CHANGES:                Initial revision
  CHANGES:                Member added to project /nfs/projekte1/org/10_Requirements_Management/tool/doors/scripts/nonPublic/dxl/addins/ADAS_RM/ImportExport/Excel/project.pj
  CHANGES:                Revision 1.7 2011/02/07 12:09:05CET Harald Horber (horberh) 
  CHANGES:                Insert default Conti-Logo
  CHANGES:                --- Added comments ---  horberh [Feb 7, 2011 12:09:05 PM CET]
  CHANGES:                Change Package : :none http://mks-psad:7002/im/viewissue?selection=

*****************************************************************************/

/*
    Enhanced Export to Excel for DOORS

    Copyright (c) 2002-2011  Galactic Solutions Group, LLC

    Exports current DOORS Module to Excel (version 97 and above)
    Exports all visible Objects (rows) and Columns in current View
    Exports and sizes OLE Objects and Pictures
    Exports Table cells
    Exports Rich Text and Symbol Font
    Exports View (Column Layout, Widths and Justifications)
    Exports Outlining/Hierarchy
    Exports to designated Worksheet and start Cell
    Exports Header rows for Column Titles and/or Attribute Names
    Exports Cell Borders for export area
    Exports Color for Header Rows, Object Headings, Color Columns and deleted Objects
    Exports Page Setup (Paper Size, Margin Sizes, Page Orientation, Header/Footers)
    Exports to existing Excel Template file
*/

const string ENHANCED_EXPORT_TO_EXCEL_ABOUT = "
   Copyright (c) 2002-2011	Galactic Solutions Group, LLC
   	http://galactic-solutions.com
   	Author: michael.sutherland@galactic-solutions.com

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/lgpl.html>.

   If you find defects with or improve upon this library, please contact
   support@galactic-solutions.com
"

/*
      2.3.0.2  26 May 2011         Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to allow line feeds for multi-valued enumerated attribute values
          - Updated getCellTextToExport()
            - Credit to Kevin Murphy for the original code submission
          - Added const string useLFsforMultiValuedEnums
          - Added DBE useCRsforMultiDBE
          - Added bool XLdoUseCRsforMulti
          - Updated excelPerformExportForModule()
          - Updated doExcelExportColumnsTab()
          - Updated validateExportToExcelDialogBox()
       - Updated to strip rich text from stored header/footers in page setups
          - Updated setPageLayoutUISettingsFromSavedPageLayout()
            - Credit to Martin Williams for the original code submission
              - Used stripRichText() instead of the undocumented plainText()
       - Updated to not fail with error message when an OLE object is failing to paste
          - Updated excelPasteOLEObjectOLE(), excelExportOLEObject()
       - Updated to set Format Picture - Size
         'Lock aspect ratio' = false and 'Relative to original picture size' = false
         - Updated excelSizeShape()
         - Updated excelShapeRangeLockAspectRatioOLE()
         - Added excelShapeRangeScaleWidth() and excelShapeRangeScaleHeight()
           - Added const string cExcelMethodScaleWidth
           - Added const string cExcelMethodScaleHeight
       - Added ()s to return statement globally in all functions
           - No functional value, done for consistency

      2.3.0.1a 16 May 2011         Michael Sutherland - Galactic Solutions Group, LLC
         - Passed Skip shapesExported to excelExportPicture()

      2.3.0.1  13 May 2011         Michael Sutherland - Galactic Solutions Group, LLC
         - Fixed error where some shapes were not getting resized, and
           Fixed error where some shapes were not getting placed at the desired cell
            - Updated excelSizeShape()
              - Updated excelExportPicture()
              - Updated excelExportOLEObjects()
              - Updated excelPerformExportForModule()
            - Added excelGetCountOLE()
               - Updated excelSheetExistsOLE()
               - Updated excelAdjustVerticalPageBreakOLE()
            - Added excelGetItemOLE() and excelGetItemFromCollectionOLE()
               - Updated excelAdjustVerticalPageBreakOLE()
               - Updated excelFormatHeaderRows()
               - Removed excelGetActiveWindowOLE()
            - Added excelGetNameOLE()
            - Added const string cExcelMethodItem 
         - Set default XLdoReduceOLEObjects = true

      2.3  07 January 2011       Michael Sutherland - Galactic Solutions Group, LLC
         - Updated ENHANCED_EXPORT_TO_EXCEL_ABOUT for the GNU LGPL 3
         - Updated version checking for DOORS 9
           - Deleted EXCEL_DOORS_VERSIONS and EXCEL_DOORS_VERSION
           - Added getDOORSmajorVersion()
           - Added const string EXCEL_DOORS_VERSIONS_70SP1
           - Added const int EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_ALL
           - Added const int EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_CURRENT
           - Added const int EXCEL_NEWEST_MAJOR_VERSION_TESTED
           - Added const Regexp DOORS_MAJOR_VERSION
           - Updated const excelExporterVersionTitle
           - Updated reference in excelPerformExportForModule()
           - Updated checkDOORSVersion(), warnIncorrectDOORSVersion(), exportPreChecks()
         - Updated with option to not shrink OLEs to fit the column width and
           updated with option to expand column widths to fit OLEs
           - Added const string expandColumnWidthsLabel
           - Added const string OLEreduceLabel
           - Added DBE expandColumnWidthsDBE
           - Added DBE OLEreduceDBE
           - Added XLdoExpandColumnWidths
           - Added excelSetColumnWidths()
           - Addded excelGetShapeWidthOLE()
           - Updated excelPerformExportForModule()
             - Added largestOLEinColumn[]
           - Updated excelSizeShape()
           - Updated excelExportOLEObjects()
           - Updated validateExportToExcelDialogBox()
           - Updated exportModuleToExcel()
           - Updated onOLEexportToggle()
           - Updated doExcelExportColumnsTab()
           - Updated doExcelExportOLETab()
           - Added excelGetShapeRangeAtIndex()
           - Added const EXCEL_SET_COLUMN_SIZE_OFFSET
           - Added trackLargestOLEInColumn()
         - Updated to fix problem with initial whitespace before OLEs
           - Updated trimWhitespace()

      2.2.7  06 June 2004        Michael Sutherland - Galactic Solutions Group, LLC
       - Created unified version for DOORS 7.x (DOORS 7.0, DOORS 7.0 SP1, DOORS 7.1)
         - Updated warnIncorrectDOORSVersion()
         - Updated and renamed DOORS_VERSION to EXCEL_DOORS_VERSION
         - Added Skip EXCEL_DOORS_VERSIONS
         - Updated excelExporterVersion, added excelExporterVersionTitle
         - Removed use of <utils/ole.inc>
           - Removed cObjExcelApplication,    added cExcelProgrammaticIdentifier
           - Removed cPropertyActiveWorkbook, added cExcelPropertyActiveWorkbook
           - Removed cPropertyBorders,        added cExcelPropertyBorders
           - Removed cPropertyColumns,        added cExcelPropertyColumns
           - Removed cPropertyCount,          added cExcelPropertyCount
           - Removed cPropertyFont,           added cExcelPropertyFont
           - Removed cPropertyHeight,         added cExcelPropertyHeight
           - Removed cPropertyLeft,           added cExcelPropertyLeft
           - Removed cPropertyLeftMargin,     added cExcelPropertyLeftMargin
           - Removed cPropertyLineStyle,      added cExcelPropertyLineStyle
           - Removed cPropertyName,           added cExcelPropertyName
           - Removed cPropertyOutlineLevel,   added cExcelPropertyOutlineLevel
           - Removed cPropertyPageSetup,      added cExcelPropertyPageSetup
           - Removed cPropertyRange,          added cExcelPropertyRange
           - Removed cPropertyScreenUpdating, added cExcelPropertyScreenUpdating 
           - Removed cPropertySelection,      added cExcelPropertySelection
           - Removed cPropertyShapes,         added cExcelPropertyShapes
           - Removed cPropertyTop,            added cExcelPropertyTop
           - Removed cPropertyValue,          added cExcelPropertyValue
           - Removed cPropertyView,           added cExcelPropertyView
           - Removed cPropertyVisible,        added cExcelPropertyVisible
           - Removed cPropertyWidth,          added cExcelPropertyWidth
           - Removed cPropertyWorkbooks,      added cExcelPropertyWorkbooks
           - Removed cMethodAdd,              added cExcelMethodAdd
           - Removed cMethodPasteSpecial,     added cExcelMethodPasteSpecial
           - Removed cMethodSelect,           added cExcelMethodSelect
           - Removed cParamDirection,         added cExcelParamDirection
           - Removed cParamFileName,          added cExcelParamFileName
           - Removed global variable oleYesToAll, added oleContinueAndIgnoreErrors
           - Removed disconnectFromApp() and closeIfNonNull(), replaced with oleClose()
           - Removed connectToApp(), replaced with oleCreate()
           - Removed checkRes(),     replaced with oleResult()
           - Removed makeVisible(),  replaced with excelSetVisibleOLE()
           - Removed checkPlatform(), updated exportPreChecks(), added warnIncorrectPlatform()
           - Added const cExcelApplicationName
       - Updated to handle situation when no printers are installed.
         - Added constants EXCEL_EXPORT_*_TAB_NAME, updated excelExportTabLabels[]
         - Added warnUnavailablePapeSize()
         - Added warnNoPrintersInstalled()
         - Added cExcelPropertyActivePrinter
         - Added excelGetActivePrinterOLE()
         - Added excelHasActivePrinter()
         - Updated excelSetPageSetup()
         - Updated excelPerformExportForModule()
         - Added excelSetPaperSizeOLE(), deleted
                 excelSetPaperSizeA4OLE(), excelSetPaperSizeA3OLE(), excelSetPaperSizeA5OLE(),
                 excelSetPaperSizeLegalOLE(), excelSetPaperSizeLetterOLE()
       - Renamed for consistency:
         - Renamed cProperty* to cExcelProperty*
         - Renamed cMethod* to cExcelMethod*
         - Renamed cParam* to cExcelParam*
         - Renamed *VBA() to *OLE()
         - Renamed OleAutoObj obj* to OleAutoObj objExcel*
       - Updated parameters for makeExcelExportUI()

      2.2.6  28 March 2004       Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to fix Rich Text Export problem
          - Updated excelSetCellRichText() to count newline characters
       - Updated for DOORS 7.0 SP1 specific Rich Text Export problem 
         where the richText(), richTextNoOle(), and rt.text functions do not
         return the correct value when Symbol Font markup is contained in a Rich Text string
          - Added cellTextHasSymbolFont(), detectSymbolFont()
          - Updated excelPerformExportForModule(), const excelExporterVersion 

      2.2.5  22 February 2004    Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to fix DOORS 7.0 SP1 specific error:
         "unassigned variable (excelExportDB)" upon execution and subsequent close of Module
         - Updated exportModuleToExcel(), added call to closeDB()
         - Moved declaration of closeDB() to precede declaration of exportModuleToExcel()

      2.2.4  11 January 2004     Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to fix error when sizing OLE objects with height > EXCEL_MAX_ROW_HEIGHT_POINTS
         - Updated excelSizeShape()

      2.2.3  15 November 2003    Michael Sutherland - Galactic Solutions Group, LLC
       - Updated for DOORS 7.0 SP1
         - Removed Excel VBA Properties cPropertyLeftMargin and cPropertyPageSetup,
           which are now included in the DOORS 7.0 SP1 <utils/ole.inc>

      2.2.2  19 October 2003     Michael Sutherland - Galactic Solutions Group, LLC
       - Updated for DOORS 7.0
         - Removed Excel VBA Properties cPropertyFont and cPropertyName,
           which are now included in the DOORS 7.0 <utils/ole.inc>

      2.2.1  10 May 2003         Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to allow the export of DOORS heading numbers to be optional
           (thanks to Geoff Wilson - Atex Media Command for requirement and prototype version)
         - Updated excelPerformExportForModule(), getCellTextToExport(),
           getObjectMainColumnContent(), and getObjectNumber()
       - Reordered XL parameters in excelPerformExportForModule() 
         and validateExportToExcelDialogBox() to correspond to order in GUI

      2.2    13 February 2003    Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to correctly export text beginning with an
         "=" (equal sign) or a "-" (minus sign) or a "'" (single quote)
         - Updated excelSetCellVBA() and added character constant EXCEL_STRING_CHARACTER
       - Updated so that the "Object Number" is not exported for empty table cells
         - Updated isHeadingRow()
       - Updated to export Rich Text for Layout DXL Columns
         - Updated getCellTextToExport() and deleted columnContainsTextAttribute()
       - Updated to ignore "Object Text" when in Outlining mode
         - Updated getCellTextToExport() parameters and
           getObjectMainColumnContent() and excelExportOLEObjects() functionality
       - Updated Rows Tab with option to export deleted Objects in color
         - Updated excelPerformExportForModule()
         - Added constants XLdoDeletedObjectRowsColor and XLdeletedObjectRowsColor
       - Updated to export OLE Object embedded in Layout DXL Columns (D6)
         - Updated excelExportOLEObjects() 
       - Updated getObjectMainColumnContent() to simplify logic and utilize isHeadingRow()
       - Updated excelPageSetupCenterHorizontally() and excelSelectRangeVBA() with
         the addition of checkRes()

      2.1    21 December 2002    Michael Sutherland - Galactic Solutions Group, LLC
       - Updated to handle exporting and sizing of Windows Enhanced Metafiles
         - Updated excelPasteOLEObjectVBA()
         - Renamed excelSizeOLEObject() to excelSizeShape(), updated to handle Shape sizing
           - Updated excelExportOLEObjects() based on rename
         - Updated excelGetShapeRangeVBA() to handle Shapes instead of OLE objects
         - Added excelGetShapesVBA(), cParamFormat ,xlPasteEnhancedMetafile
         - Deleted excelGetOLEobjectVBA(), cPropertyShapeRange
       - Updated to export Pictures
         - Updated excelPerformExportForModule()
         - Added DOORS_PICTURE
         - Added XLdoExportPictures
           - Updated exportModuleToExcel(), validateExportToExcelDialogBox()
         - Updated doExcelExportOLETab()
           - Added PictureExportDBE, PictureExportLabel
       - Added excelGetBordersVBA()
         - Updated excelSetBorders()
       - Corrected Dialog Box spacing
         - Updated doExcelExportAboutTab() and added spacing label to makeExcelExportUI()
         - Updated excelExportPicture()
       - Updated getObjectMainColumnContent() so Rich Text Markup (bolding) is consistent

      2.0.1  11 December 2002    Michael Sutherland - Galactic Solutions Group, LLC
       - Added "=" sign after declaration of "warnInvalidFileNameDoesNotExistErrorMessage"

      2.0    23 November 2002    Michael Sutherland - Galactic Solutions Group, LLC
       - Updated for DOORS v5.x and v6.x
       - Organized user options with Tabs
       - Isolated and grouped Excel VBA Functions
       - Updated to that exporter can be run on a Module without an interface
       - Added getExcelColumnAlphaAddressFromInt() so that the full
         256 Columns of Excel 97 ( A - IV ) can be used
       - Added Progress Bar and Cancel functionality
       - Added Sheet Tab
         - Added Sheet option and functionality for Excel to be visible during export
         - Added Sheet option and functionality to export Rich Text
           (including preservation of Symbol font characters)
       - Added Header Tab
         - Added Header option and functionality for AutoFilter activation
         - Added Header option and functionality for Split Header activation
         - Added Header option and functionality for Freeze Panes activation
         - Added Header option and functionality for Selection of Object Heading Color
       - Added Columns Tab
         - Added Columns option for Adjustment of Column Widths
         - Added Columns option and functionality for Justification of Columns
       - Added Rows Tab
         - Added Rows option and functionality for Outline creation
         - Added Rows option and functionality for Selection of Header Rows Color
       - Added OLE Tab
       - Added Page Setup Tab
         - Added Page Setup options and functionality
       - Added Template Tab
       - Added About Tab

      1.2.1  03 June 2001        Michael Sutherland - Galactic Solutions Group, LLC
       - Updated getColorConstant() and added getColorColumnAttributeDefinition() to doExcel() 
         to display Color Columns

      1.2    10 May 2001         Michael Sutherland - Galactic Solutions Group, LLC
       - Added Version to Window Titles
       - Commented out unused Global variables
       - Changed "ack" to "warningBox" in 
         validateCell(), validateSheetNumber(), validateFileName(), and validateExportToExcelDialogBox()
       - Changed "ack" to "infoBox" in exportModuleToExcel()

      1.1    08 April 2001       Michael Sutherland - Galactic Solutions Group, LLC
       - Updated Library file locations and names to updated standards
       - Updated doExcel() to export Layout DXL for non-color columns

      1.0    22 December 2000    Roger Ellero
       - Initial Version
*/

/*

Known issues:

A defect in the DOORS 7.0 SP1 Rich Text commands creates problems
exporting Rich Text and Symbol Font, so Rich Text export is disabled
when Symbol Font is encountered. (DOORS 7.0 SP1 only, confirmed fixed in DOORS 7.1)

Problems exporting Rich Text with initial whitespace (spaces, tabs, etc.)

Problems exporting tab characters (appears as a square if embedded in text)

DOORS 6 Rich Text features bullets and indents not exported

Page Dimensions only in millimeters (inches not currently supported)
Custom Paper Size not currently supported

*/

pragma runLim,0

//-------------------------- Version Constants -----------------------------

// Version

const string excelExporterVersion = "2.3.0.2"
const string EXCEL_DOORS_VERSIONS_70SP1 = "DOORS 7.0 SP1"
const int EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_ALL = 5
const int EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_CURRENT = 7
const int EXCEL_NEWEST_MAJOR_VERSION_TESTED = 9
const Regexp DOORS_MAJOR_VERSION = regexp "^DOORS ([0-9].*)."

//------------------------------ Constants ---------------------------------

// Print debugging info?
const bool DEBUG = false

Regexp INTEGER = regexp "^(-|+|[0-9])[0-9]*$"
          
const string TABLE_MARKER_TEXT = "{\\b " ">> Table" "}"
const string OLE_MARKER_TEXT   = "{\\b " ">> OLE" "}"

//-------------------------- DOORS Constants ------------------------------

// Attributes
const string DOORS_OBJECT_HEADING = "Object Heading"
const string DOORS_OBJECT_TEXT    = "Object Text"
const string DOORS_PICTURE        = "Picture"

// Colors
const int DOORS_MIN_REAL_COLOR = realColor_Light_Blue2
const int DOORS_MAX_REAL_COLOR = realColor_NewGrey4

// Paper Size and Margin Limits
// These constants match print/page.h
const int DOORS_BIG_PAPER_MM    = 3000
const int DOORS_SMALL_PAPER_MM  = 50
const int DOORS_BIG_MARGIN_MM   = 150
const int DOORS_SMALL_MARGIN_MM = 5

// Standard Page Layout Name
const string DOORS_STANDARD_LAYOUT = "Standard layout"

// Paper Sizes able to be stored in a DOORS Page Setup
const string DOORS_PAGE_SETUP_PAPER_SIZES[]   = { "A4",  "A3",  "A5",  "Legal", "Letter" }
const string DOORS_PAGE_SETUP_PAPER_WIDTHS_MM[]  = { "210", "297", "148", "216", "216" }
const string DOORS_PAGE_SETUP_PAPER_HEIGHTS_MM[] = { "296", "420", "210", "356", "279" }
const int DOORS_PAPER_SIZE_A4     = 0
const int DOORS_PAPER_SIZE_A3     = 1
const int DOORS_PAPER_SIZE_A5     = 2
const int DOORS_PAPER_SIZE_LEGAL  = 3
const int DOORS_PAPER_SIZE_LETTER = 4

// Note:  Paper size 'Custom' not currently supported.
//        References to "Custom' kept for future versions
const int DOORS_PAPER_SIZE_CUSTOM = 5

// Page Orientations
const int DOORS_PAGE_ORIENTATION_PORTRAIT  = 0
const int DOORS_PAGE_ORIENTATION_LANDSCAPE = 1

// RGB Values for DOORS Real Color Constants
const int DOORS_RGB_RED_INDEX   = 0
const int DOORS_GREEN_RED_INDEX = 1
const int DOORS_BLUE_RED_INDEX  = 2
const Array REAL_COLOR_RGB_VALUES = create( DOORS_MAX_REAL_COLOR, 3 )
put( REAL_COLOR_RGB_VALUES, 178, realColor_Light_Blue2,    DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 223, realColor_Light_Blue2,    DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 238, realColor_Light_Blue2,    DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 173, realColor_Light_Blue,     DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 216, realColor_Light_Blue,     DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 230, realColor_Light_Blue,     DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Dark_Turquoise, DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 206, realColor_Dark_Turquoise, DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 209, realColor_Dark_Turquoise, DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Pink,           DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 192, realColor_Pink,           DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 203, realColor_Pink,           DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Blue,           DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Blue,           DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Blue,           DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 176, realColor_Maroon,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  48, realColor_Maroon,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  96, realColor_Maroon,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Red,            DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Red,            DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Red,            DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Yellow,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Yellow,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Yellow,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Green,          DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Green,          DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Green,          DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Cyan,           DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Cyan,           DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Cyan,           DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Magenta,        DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Magenta,        DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Magenta,        DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_White,          DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 255, realColor_White,          DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 255, realColor_White,          DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 255, realColor_Orange,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 165, realColor_Orange,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Orange,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 165, realColor_Brown,          DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  42, realColor_Brown,          DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  42, realColor_Brown,          DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 160, realColor_Purple,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  32, realColor_Purple,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 240, realColor_Purple,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Navy,           DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Navy,           DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 128, realColor_Navy,           DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES,  46, realColor_Sea_Green,      DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 139, realColor_Sea_Green,      DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  87, realColor_Sea_Green,      DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 180, realColor_Lime_Green,     DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 220, realColor_Lime_Green,     DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 215, realColor_Lime_Green,     DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 228, realColor_Rosy_Brown,     DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 183, realColor_Rosy_Brown,     DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 183, realColor_Rosy_Brown,     DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 205, realColor_Peru,           DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 133, realColor_Peru,           DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  63, realColor_Peru,           DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 192, realColor_Red_Grey,       DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 176, realColor_Red_Grey,       DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 176, realColor_Red_Grey,       DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 178, realColor_Firebrick,      DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 034, realColor_Firebrick,      DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 034, realColor_Firebrick,      DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 216, realColor_Thistle,        DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 191, realColor_Thistle,        DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 216, realColor_Thistle,        DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 209, realColor_Grey82,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 209, realColor_Grey82,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 209, realColor_Grey82,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 196, realColor_Grey77,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 196, realColor_Grey77,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 196, realColor_Grey77,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 168, realColor_Grey66,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 168, realColor_Grey66,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 168, realColor_Grey66,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 140, realColor_Grey55,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 140, realColor_Grey55,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 140, realColor_Grey55,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 112, realColor_Grey44,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 112, realColor_Grey44,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 112, realColor_Grey44,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES,  84, realColor_Grey33,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  84, realColor_Grey33,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  84, realColor_Grey33,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES,  56, realColor_Grey22,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  56, realColor_Grey22,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  56, realColor_Grey22,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES,  28, realColor_Grey11,         DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  28, realColor_Grey11,         DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  28, realColor_Grey11,         DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Black,          DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Black,          DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 000, realColor_Black,          DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 250, realColor_NewGrey1,       DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 250, realColor_NewGrey1,       DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 250, realColor_NewGrey1,       DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 240, realColor_NewGrey2,       DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 240, realColor_NewGrey2,       DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 240, realColor_NewGrey2,       DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES, 224, realColor_NewGrey3,       DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES, 224, realColor_NewGrey3,       DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES, 224, realColor_NewGrey3,       DOORS_BLUE_RED_INDEX  )
put( REAL_COLOR_RGB_VALUES,  14, realColor_NewGrey4,       DOORS_RGB_RED_INDEX   )
put( REAL_COLOR_RGB_VALUES,  14, realColor_NewGrey4,       DOORS_GREEN_RED_INDEX )
put( REAL_COLOR_RGB_VALUES,  14, realColor_NewGrey4,       DOORS_BLUE_RED_INDEX  )

//--------------------------- Excel Constants ---------------------------

// Used in getExcelColumnAlphaAddressFromInt() and getExcelColumnIntFromAlphaAddress()
const int EXCEL_ASCII_OFFSET = intOf( 'A' )
const int EXCEL_BASE_FOR_ALPHABET = 26 

// Used in getExcelColumnIntFromAlphaAddress()
const int EXCEL_MIN_COLUMN = 1      // Excel 97 Program workspace specification
const int EXCEL_MAX_COLUMN = 256    // Excel 97 Program workspace specification

// Used in validExcelRowIdentifier()
const int EXCEL_MIN_ROW = 1         // Excel 97 Program workspace specification
const int EXCEL_MAX_ROW = 65536     // Excel 97 Program workspace specification

// Used in parseAndValidateExcelCellIdentifier()
const Regexp EXCEL_CELL_IDENTIFIER = regexp "^( *)([A-Z]|[A-Z][A-Z])([0-9].*)( *)$"
const Regexp EXCEL_SHEET_IDENTIFIER = regexp "^( *)([0-9].*)( *)$"

// Used in excelSizeShape()
const int EXCEL_MAX_ROW_HEIGHT_POINTS = 409      // Excel 97 Program workspace specification
const int EXCEL_OLE_POSITION_OFFSET_WIDTH = 3
const int EXCEL_OLE_POSITION_OFFSET_HEIGHT = 3
const real EXCEL_SET_COLUMN_SIZE_OFFSET = 1.15

// Used in doExcelExportSheetTab()
const string EXCEL_FIRST_CELL_IDENTIFIER = "A1"

// Used in parseAndValidateExcelSheetIdentifier()
const int EXCEL_MIN_WORKBOOK_SHEETS = 1          // Excel 97 Program workspace specification
const int EXCEL_MAX_WORKBOOK_SHEETS = 255        // Excel 97 Program workspace specification

// Used in excelSetCellOLE()
const int EXCEL_MAX_CHARACTERS_IN_CELL = 32767   // Excel 97 Program workspace specification
const char EXCEL_STRING_CHARACTER = '\''

//-------------------------- Dialog Box Constants -----------------------------
// Dialog Boxes MKS script or file revision number placeholder or title
// Note: The revision will be automatically patched by MKS. The blanks are added for higher revision numerber and support a revision string after the dot 
// of size of 2 character. Branches of the file are not expected and not considered!!!
const string gc_szScriptRevision = "$Revision: 1.6 $";
// Start and End Index of the used part within the Script Revision number
const int gc_iScriptRevisonStartndex = 11;
const int gc_iScriptRevisonEndIndex = 14;
const string excelMainTitle            = "Enhanced Export To Excel"
const string excelExporterVersionTitle = excelExporterVersion " - " doorsVersion ""
const string excelTitle            = excelMainTitle " (" excelExporterVersionTitle ")     " gc_szScriptRevision[gc_iScriptRevisonStartndex:gc_iScriptRevisonEndIndex]""

const void warnIncorrectDOORSVersion( void ) {
   warningBox "This version (" excelExporterVersion ") of the '" //-
               excelMainTitle //-
              "' has been written for DOORS versions " //-
              EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_ALL ".x - " EXCEL_NEWEST_MAJOR_VERSION_TESTED ".x" "\n\n" //-
              "You are using " doorsVersion "\n\n" //-
              "Please download the version for " doorsVersion " from http://galactic-solutions.com\n"
}

// Tab Indices
const int EXCEL_EXPORT_SHEET_TAB      = 0
const int EXCEL_EXPORT_HEADER_TAB     = 1
const int EXCEL_EXPORT_COLUMNS_TAB    = 2
const int EXCEL_EXPORT_ROWS_TAB       = 3
const int EXCEL_EXPORT_OLE_TAB        = 4
const int EXCEL_EXPORT_PAGE_SETUP_TAB = 5
const int EXCEL_EXPORT_TEMPLATE_TAB   = 6
const int EXCEL_EXPORT_ABOUT_TAB      = 7

const string EXCEL_EXPORT_SHEET_TAB_NAME      = "Sheet"
const string EXCEL_EXPORT_HEADER_TAB_NAME     = "Header"
const string EXCEL_EXPORT_COLUMNS_TAB_NAME    = "Columns"
const string EXCEL_EXPORT_ROWS_TAB_NAME       = "Rows"
const string EXCEL_EXPORT_OLE_TAB_NAME        = "OLE"
const string EXCEL_EXPORT_PAGE_SETUP_TAB_NAME = "Page Setup"
const string EXCEL_EXPORT_TEMPLATE_TAB_NAME   = "Template"
const string EXCEL_EXPORT_ABOUT_TAB_NAME      = "About"

// DBE Options and Labels
const string excelExportTabLabels[] = { EXCEL_EXPORT_SHEET_TAB_NAME,
                                        EXCEL_EXPORT_HEADER_TAB_NAME,
                                        EXCEL_EXPORT_COLUMNS_TAB_NAME,
                                        EXCEL_EXPORT_ROWS_TAB_NAME,
                                        EXCEL_EXPORT_OLE_TAB_NAME,
                                        EXCEL_EXPORT_PAGE_SETUP_TAB_NAME,
                                        EXCEL_EXPORT_TEMPLATE_TAB_NAME,
                                        EXCEL_EXPORT_ABOUT_TAB_NAME
                                      }

// Sheet Tab Options and Labels
const string dataSheetFieldLabel = "Export Data to Sheet:"
const string startCellFieldLabel = "Export Start Cell:"
const string startCellOptions[] = { "First Cell in Worksheet (" EXCEL_FIRST_CELL_IDENTIFIER ")",
                                    "Specific Cell (C3, E12, etc.)"
                                  }
const string addCellBordersLabel = "Add Cell Borders to Export Area"
const string exportRichTextLabel = "Export Rich Text"
const string exportTableCellsLabel = "Export Table Cells"
const string setPrintAreaLabel = "Set Print Area"

// Header Tab Options and Labels
const string exportColumnTitlesLabel = "Export Column Titles"
const string exportAttributeNamesLabel = "Export Attribute Names"
const string activateAutoFilterLabel = "Activate AutoFilter"
const string splitHeaderRowsLabel = "Split Header"
const string freezePanesLabel = "Freeze Panes"
const string headerRowsColorFrameLabel = "Header Row(s) Color"
const string useHederRowColorLabel = "Use Color:"
const string colorHeaderslabel = "Color"

// Columns Tab Options and Labels
const string setColumnWidthsLabel = "Adjust Column Widths to Fit Page"
const string expandColumnWidthsLabel = "Expand Column Widths to Fit OLEs"
const string justifyColumnsLabel = "Justify Columns"
const string useLFsforMultiValuedEnums = "Use Line Feed as separator for Multi-valued enumerated attribute values"
const string useColumnColorLabel = "Use Color for Columns"
const string forceColumnColorLabel = "Force Color for Columns"

// Rows Tab Options and Labels
const string exportHeadingNumberLabel = "Export Heading Numbers"
const string setOutlineLabel = "Outline"
const string useDeletedObjectRowsColorLabel = "Color Deleted Objects"
const string objectHeadingColorFrameRowsLabel = "Object Heading Color"
const string useObjectHeadingColorRowsLabel = "Use Color:"
const string colorRowsLabel = "Color"

// OLE Tab Options and Labels
const string OLEexportLabel = "Export OLE objects"
const string OLEreduceLabel = "Reduce OLE object size to fit column width"
const string OLEseparateSheetLabel = "Export OLE objects to a separate worksheet"
const string OLEsheetFieldLabel = "Export OLE to sheet:"
const string PictureExportLabel = "Export Pictures"

// Page Setup Tab Options and Labels
const string usePageSetupLabel = "Apply Page Setup"
const string pageSetupFrameLabel = "Page Setups"
const string paperSizeLabel = "Paper size"
const string paperWidthLabel = "Width"
const string paperHeightLabel = "Height"
const string columnTitlesLabel = "Print column titles:"
const string columnTitlesOptions[] = { "Every page", "First page only" }
const string marginsLabel = "Margins:"
const string marginLeftLabel = "Left"
const string marginRightLabel = "Right   "
const string marginTopLabel = "Top"
const string marginBottomLabel = "Bottom"
const string pageOrientationLabel = "Orientation"
const string pageOrientationOptions[] = { "Portrait", "Landscape" }
const string hfPrintOptionsLabel = "Header and Footer Options"
const string hfPrintOptions1 = "&N - Page number   "
const string hfPrintOptions2 = "&M - Module name"
const string hfPrintOptions3 = "&L - Module path   "
const string hfPrintOptions4 = "&P - Project name"
const string hfPrintOptions5 = "&V - Module version"
const string hfPrintOptions6 = "&U - User name"
const string hfPrintOptions7 = "&A - Product name"
const string hfPrintOptions8 = "&C - Page count"
const string hfPrintOptions9 = "&D - Date"
const string hfPrintOptions10 = "&T - Time"
const string hfPrintOptions11 = "&B - Product version"
const string bodyHeaderLabel = "Header"
const string bodyFooterLabel = "Footer"
const string pageSetupListLabel = "Name"

// Template Tab Options and Labels
const string sheetOptions[] = { "New spreadsheet", "Existing spreadsheet" }
const string exportFileChoiceLabel = "Export to:"
const string exportFileLabel = "Excel file name:"

// Main DB Buttons
const string exportButtonLabel = "Export"

// DB Validation Error Messages
const void warnIncorrectPlatform( string applicationName ) { 
   warningBox "The " applicationName " interface is only available on Microsoft Windows platforms."
}
const string warnRunWithinFormalModule = 
   "Must be run from within a Formal Module"
const void warnCannotReadAttribute( string attributeName ) {
   warningBox "Cannot read Attribute '" attributeName "'"
}
const string errorCannotGetWorkbooks = "Unable to get workbooks collection"
const string errorCannotGetActiveWorkbook = "Unable to get active workbook"
const string dataSheetName = "Data"
const string OLESheetName = "OLE"
const void infoSheetDoesNotExist( int sheetNum, string sheetName, int exportSheetNum ) {
   infoBox( "Sheet " sheetNum " does not exist\n" //-
            "Exporting " sheetName " to Sheet " exportSheetNum ""
          )
}
const void errorCannotGetSheet( int sheetNum ) {
   errorBox "Unable to get Sheet " sheetNum ""
}
const string warnInvalidExcelIdentifierErrorMessage = 
   "Invalid starting cell. Cell must follow the format:\n\n" //-
   "XX99999, where \n" //-
   "X is an Excel Column (A-IV) " //-
   "and 99999 is an Excel Row (" EXCEL_MIN_ROW "-" EXCEL_MAX_ROW ").\n\n" //-
   "Examples of valid starting cells: A1, C3, B15, F10"
const string warnInvalidExcelSheetIdentifierErrorMessage =
   "Worksheet numbers must be a number between 1 and " EXCEL_MAX_WORKBOOK_SHEETS "."
const string warnInvalidExcelDataAndOLESheetErrorMessage = 
   "You have opted to export Data and OLE objects to separate sheets.\n" //-
   "Please enter a different sheet number for each of these items."
const string warnInvalidFileNameBlankErrorMessage =
   "If you are not exporting to a new spreadsheet, you must supply the name\n" //-
   "of the Excel file to which you wish to export."
const string warnInvalidFileNameDoesNotExistErrorMessage =
   "The file does not exist.\n" //-
   "Please select a file (with full path name) before continuing."
const string pageWidthDimensionName = "Page Width"
const string pageHeightDimensionName = "Page Height"
const string leftMarginDimensionName = "Left Margin"
const string rightMarginDimensionName = "Right Margin"
const string topMarginDimensionName = "Top Margin"
const string bottomMarginDimensionName = "Bottom Margin"
const void warnInvalidPageDimension( string pageDimStr, string dimensionName ) {
   warningBox pageDimStr " is not a valid " dimensionName "."
}
const void warnInvalidPageDimension( string pageDimStr, string dimensionName, int lowerLim, int upperLim ) {
   warningBox pageDimStr " is not a valid " dimensionName ".\n" //-
              dimensionName " must be between " lowerLim " and " upperLim "\n"
}
const void warnUnavailablePapeSize( int paperSize ) {
   warningBox "Paper Size '" DOORS_PAGE_SETUP_PAPER_SIZES[ paperSize ] //-
              "' is not available for your default printer."
}
const void warnNoPrintersInstalled( void ) {
	/*
   warningBox "No printers are installed.\n\n" //
              "The following functions will be disabled:\n\n" //-
              "- " EXCEL_EXPORT_PAGE_SETUP_TAB_NAME "\n" //-
              "- " setColumnWidthsLabel "\n" //-
              "- " exportColumnTitlesLabel "\n"//-
              "- " setPrintAreaLabel
              */
}

// Progress messages
const string progressStartMessage = "Initializing"
const string progressHeaderRowMessage = "Exporting Header Row"
string progressObjectMessage( int numObjectsExported, int numObjectsToExport ) {
   return( "Exporting Object " numObjectsExported " of " numObjectsToExport "" )
}
const string progressFormattingDataMessage = "Formatting Data"

//----------- Enhanced Export to Excel Dialog Box Global Variables ---------------

DB excelExportDB = null
DBE excelExportTabDBE
// Sheet
DBE dataSheetFieldDBE
DBE startCellDBE
DBE startCellFieldDBE
DBE addCellBordersDBE
DBE exportRichTextDBE
DBE exportTableCellsDBE
DBE setPrintAreaDBE
// Columns
DBE setColumnWidthsDBE
DBE expandColumnWidthsDBE
DBE justifyColumnsDBE
DBE useCRsforMultiDBE
DBE useColumnColorDBE
DBE forceColumnColorDBE
// Rows
DBE exportHeadingNumberDBE
DBE setOutlineDBE
DBE objectHeadingColorFrameRowsDBE
DBE useObjectHeadingColorRowsDBE
DBE objectHeadingColorListRowsDBE
DBE useDeletedObjectRowsColorDBE
// Header
DBE exportColumnTitlesDBE
DBE exportAttributeNamesDBE
DBE activateAutoFilterDBE
DBE splitHeaderRowsDBE
DBE freezePanesDBE
DBE headerRowsColorFrameDBE
DBE useHeaderRowColorDBE
DBE headerRowsColorListDBE
// OLE
DBE OLEexportDBE
DBE OLEreduceDBE
DBE OLESeparateSheetDBE
DBE OLESheetFieldDBE
DBE PictureExportDBE
// Template
DBE exportFileChoiceDBE
DBE exportFileLabelDBE
DBE exportFileSelectDBE
// Page Setup
DBE pageSetupFrameDBE
DBE usePageSetupDBE
DBE savedPageLayoutListDBE
DBE paperSizeDBE
DBE pageOrientationDBE
DBE paperWidthDBE
DBE paperHeightDBE
DBE paperWidth_RO_DBE
DBE paperHeight_RO_DBE
DBE pagePrintColumnTitleRowsDBE
DBE marginsDBE
DBE marginLeftDBE
DBE marginRightDBE
DBE marginTopDBE
DBE marginBottomDBE
DBE printOptionLabel0DBE
DBE printOptionLabel1DBE
DBE printOptionLabel2DBE
DBE printOptionLabel3DBE
DBE printOptionLabel4DBE
DBE printOptionLabel5DBE
DBE printOptionLabel6DBE
DBE printOptionLabel7DBE
DBE printOptionLabel8DBE
DBE printOptionLabel9DBE
DBE printOptionLabel10DBE
DBE printOptionLabel11DBE
DBE bodyLeftHeaderDBE
DBE bodyCenterHeaderDBE
DBE bodyRightHeaderDBE
DBE bodyLeftFooterDBE
DBE bodyCenterFooterDBE
DBE bodyRightFooterDBE
// About
DBE excelExportAboutDBE
// Buttons
DBE exportButtonDBE

Skip excelExportTabSkip
Skip excelExportSheetTabSkip
Skip excelExportHeaderTabSkip
Skip excelExportColumnsTabSkip
Skip excelExportRowsTabSkip
Skip excelExportOLETabSkip
Skip excelExportPageSetupTabSkip
Skip excelExportTemplateTabSkip
Skip excelExportAboutTabSkip

Trigger excelTrigger

//####################################################

   OleAutoObj objExcelWorkbook = null

//####################################################
//------ Globals derived from User Selections (with initial values set) ---------

// Sheet
int XLdataExportSheetNumber = 1
int XLcolOffset = 0
int XLrowOffset = 0
bool XLdoAddCellBorders = true
bool XLdoExportRichText = true
bool XLdoExportTableCells = true
bool XLdoSetPrintArea = true

// Header
bool XLdoExportColumnTitles = true
bool XLdoExportAttributeNames = false
bool XLdoEnableExcelAutoFilter = false
bool XLdoSplitHeaderRows = false
bool XLdoFreezePanes = false
bool XLdoHeaderRowsColor = true
int  XLheaderRowsColor = realColor_Grey66

// Columns
bool XLdoSetColumnWidths = true
bool XLdoExpandColumnWidths = true
bool XLdoJustifyColumns = true
bool XLdoUseCRsforMulti = true
bool XLdoExportColorColumns = true
bool XLdoForceColumnColor = false

// Rows
bool XLdoExportHeadingNumber = true
bool XLdoOutline = false
bool XLdoDeletedObjectRowsColor = true
int  XLdeletedObjectRowsColor = getRealColor( logicalDeletedTextColor )
bool XLdoObjectHeadingRowsColor = true
int  XLObjectHeadingRowsColor = realColor_Grey82

// OLE
bool XLdoExportOLEObjects = b_ExportOLE //true
bool XLdoReduceOLEObjects = true
bool XLdoExportOLEToSeparateSheet = false
int  XLOLEExportSheetNumber = 2
bool XLdoExportPictures = false

// Template
string XLexistingFileName = ""
bool XLdoUseExistingExcelFile = false

// Page Setup
bool XLdoPageSetup = true

// ( the following Page Setup defaults are set by applying XLdefaultLayout )
const string XLdefaultLayout = DOORS_STANDARD_LAYOUT
int    XLpageSetupOrientation = 0
int    XLpaperSize = 0
int    XLpageWidth = 0        // milliMeters
int    XLpageHeight = 0       // milliMeters
int    XLleftMargin = 0       // milliMeters
int    XLrightMargin = 0      // milliMeters
int    XLtopMargin = 0        // milliMeters
int    XLbottomMargin = 0     // milliMeters
bool   XLdoSetPrintColumnTitleRows = true
string XLleftHeader = ""
string XLcenterHeader = ""
string XLrightHeader = ""
string XLleftFooter = ""
string XLcenterFooter = ""
string XLrightFooter = ""

//----------------------- Version Functions ---------------------

int getDOORSmajorVersion( void ) {
   int v = 0
   string dmv = doorsVersion
   if ( DOORS_MAJOR_VERSION dmv ) {
      v = intOf( dmv[ match 1 ] )
   }
   return( v )
}


//------------------------- Debug Output -----------------------

void debug( string s ) {
   if ( DEBUG ) print s "\n"
}

//---------------- Excel VBA Export support functions --------------

bool checkSetPaperSizeRes(string res) {
   if ( !null res ) {
      debug( "Cannot set Paper Size: " res )
      return( false )
   }
   return( true )
}

int convertMillimetersToPoints( int milliMeters ) {
   // points = mm * ( 72.0 Points per Inch ) / ( 25.4 mm per Inch )
   return( intOf( realOf( milliMeters ) * ( 72.0 / 25.4 ) ) )
}

string getExcelColumnAlphaAddressFromInt( int columnInt ) {

   if ( columnInt < EXCEL_BASE_FOR_ALPHABET ) {
      return( charOf( EXCEL_ASCII_OFFSET + columnInt - 1 ) "" )
   }
   else {
      if ( ( columnInt % EXCEL_BASE_FOR_ALPHABET ) == 0 ) {
         if ( columnInt == EXCEL_BASE_FOR_ALPHABET ) {
            return( charOf( EXCEL_ASCII_OFFSET + EXCEL_BASE_FOR_ALPHABET - 1 ) "" )
         }
         else {
            return( getExcelColumnAlphaAddressFromInt( ( columnInt - EXCEL_BASE_FOR_ALPHABET + 1 ) / EXCEL_BASE_FOR_ALPHABET ) "" //-
                   charOf( EXCEL_ASCII_OFFSET + EXCEL_BASE_FOR_ALPHABET - 1 ) "" )
         }
      }
      else {
         return( getExcelColumnAlphaAddressFromInt( columnInt / EXCEL_BASE_FOR_ALPHABET ) "" //-
                charOf( EXCEL_ASCII_OFFSET + ( columnInt % EXCEL_BASE_FOR_ALPHABET ) - 1 ) "" )
      }
   }

   return( "" )
}

int getExcelColumnIntFromAlphaAddress( string columnAlpha ) {

   if ( length( columnAlpha ) == 1 ) {
      return( intOf( columnAlpha[0] ) - EXCEL_ASCII_OFFSET + 1 )
   }
   else {
      return( EXCEL_BASE_FOR_ALPHABET * getExcelColumnIntFromAlphaAddress( columnAlpha[ 0 : length columnAlpha - 2 ] ) + //-
             intOf( columnAlpha[ length columnAlpha - 1 ] ) - EXCEL_ASCII_OFFSET + 1 )
   }
}

string createExcelA1StyleRowAddress( int rowStart, int rowEnd ) {
   return( "$" rowStart ":" "$" rowEnd "" )
}

string createExcelA1StyleAddress( int rowStart,
                                  int columnStart,
                                  int rowEnd,
                                  int columnEnd
                                )
{
   string returnStr = "$" getExcelColumnAlphaAddressFromInt( columnStart ) //-
                      "$" rowStart ":" //-
                      "$" getExcelColumnAlphaAddressFromInt( columnEnd ) //-
                      "$" rowEnd ""
   return( returnStr )
}


bool parseAndValidateExcelSheetIdentifier( string sheetID, int &excelSheetNumber ) {
   if ( EXCEL_SHEET_IDENTIFIER sheetID ) {
      excelSheetNumber = intOf( sheetID[ match 2 ] )
      return( ( excelSheetNumber >= EXCEL_MIN_WORKBOOK_SHEETS ) &&
              ( excelSheetNumber <= EXCEL_MAX_WORKBOOK_SHEETS )
            )
   }
   return( false )
}

bool validateExcelSheetIdentifier( string sheetID ) {
   int excelSheetIdentifier = 0
   return( parseAndValidateExcelSheetIdentifier( sheetID, excelSheetIdentifier ) )

}

bool validExcelColumnIdentifier( string excelColumnIdentifier ) {   
   int excelColumnInteger = getExcelColumnIntFromAlphaAddress( excelColumnIdentifier )
   return( ( excelColumnInteger >= EXCEL_MIN_COLUMN ) && ( excelColumnInteger <= EXCEL_MAX_COLUMN ) )
}

bool validExcelRowIdentifier( int excelRowIdentifier ) {
   return( ( excelRowIdentifier >= EXCEL_MIN_ROW ) && ( excelRowIdentifier <= EXCEL_MAX_ROW ) )
}

bool parseAndValidateExcelCellIdentifier( string excelCellIdentifier,
                                          string &excelColumnIdentifier,
                                          int &excelRowIdentifier
                                        )
{   
   if ( EXCEL_CELL_IDENTIFIER excelCellIdentifier ) {
      excelColumnIdentifier = excelCellIdentifier[ match 2 ]
      excelRowIdentifier = intOf( excelCellIdentifier[ match 3 ] )
      if ( validExcelColumnIdentifier( excelColumnIdentifier ) &&
           validExcelRowIdentifier( excelRowIdentifier )
         )
      {
         return( true )
      }
   }

   excelColumnIdentifier = ""
   excelRowIdentifier = 0
   return( false )
}

bool validateExcelCellIdentifier( string excelCellIdentifier ) {
   string excelColumnIdentifier = ""
   int excelRowIdentifier = 0
   return( parseAndValidateExcelCellIdentifier( excelCellIdentifier,
                                                excelColumnIdentifier,
                                                excelRowIdentifier
                                              )
         )
}

//--------------------- OLE Global Variables ---------------------

OleAutoArgs objArgBlock   = create
bool oleContinueAndIgnoreErrors = false
OleAutoArgs objArgs   = create

//------------------------ OLE Functions -------------------------

OleAutoObj oleCreate( string oleApplication, string applicationName ) {
   OleAutoObj objOleApplication = oleCreateAutoObject( oleApplication )
   if ( null objOleApplication ) {
      ack "Unable to connect to " applicationName "."
   }
   return( objOleApplication )
}

void oleResult( string result ) {
	
   if ( ( !oleContinueAndIgnoreErrors ) && ( !null result ) ) { 
      string userChoices[] = { "Yes", "Yes to all", "No" }
      int i  = messageBox( "OLE property or method failed with error message: \n" //-
                            result "\n\n" //-
                            "Continue?",
                            userChoices,
                            msgWarning
                         )
      if ( i == 2 ) {
         progressStop() 
         halt()
      }
      if ( i == 1 ) {
         oleContinueAndIgnoreErrors = true    
      }    
   }
}

void oleClose( OleAutoObj objApplication ) {
   if ( !null objApplication ) {
      oleCloseAutoObject( objApplication )
   }
}

//---------------- Excel OLE Application Constants  --------------

const string cExcelProgrammaticIdentifier = "Excel.Application"
const string cExcelApplicationName        = "Microsoft Excel"

//--------------------- Excel VBA Properties ---------------------

const string cExcelPropertyActivePrinter       = "ActivePrinter"
const string cExcelPropertyActiveWorkbook      = "ActiveWorkbook"
const string cExcelPropertyAutoFilterMode      = "AutoFilterMode"
const string cExcelPropertyBold                = "Bold"
const string cExcelPropertyBoldItalic          = "Bold Italic"
const string cExcelPropertyBorders             = "Borders"
const string cExcelPropertyBottomMargin        = "BottomMargin"
const string cExcelPropertyCenterFooter        = "CenterFooter"
const string cExcelPropertyCenterHeader        = "CenterHeader"
const string cExcelPropertyCenterHorizontally  = "CenterHorizontally"
const string cExcelPropertyCharacters          = "Characters"
const string cExcelPropertyColor               = "Color"
const string cExcelPropertyColorIndex          = "ColorIndex"
const string cExcelPropertyColumnWidth         = "ColumnWidth"
const string cExcelPropertyColumns             = "Columns"
const string cExcelPropertyCount               = "Count"
const string cExcelPropertyEntireRow           = "EntireRow"
const string cExcelPropertyFont                = "Font"
const string cExcelPropertyFontStyle           = "FontStyle"
const string cExcelPropertyFreezePanes         = "FreezePanes"
const string cExcelPropertyHeight              = "Height"
const string cExcelPropertyHorizontalAlignment = "HorizontalAlignment"
const string cExcelPropertyInterior            = "Interior"
const string cExcelPropertyItalic              = "Italic"
const string cExcelPropertyItem                = "Item"
const string cExcelPropertyLeft                = "Left"
const string cExcelPropertyLeftFooter          = "LeftFooter"
const string cExcelPropertyLeftHeader          = "LeftHeader"
const string cExcelPropertyLeftMargin          = "LeftMargin"
const string cExcelPropertyLineStyle           = "LineStyle"
const string cExcelPropertyLockAspectRatio     = "LockAspectRatio"
const string cExcelPropertyMergeCells          = "MergeCells"
const string cExcelPropertyName                = "Name"
const string cExcelPropertyOrientation         = "Orientation"
const string cExcelPropertyOutlineLevel        = "OutlineLevel"
const string cExcelPropertyPageSetup           = "PageSetup"
const string cExcelPropertyPaperSize           = "PaperSize"
const string cExcelPropertyPrintArea           = "PrintArea"
const string cExcelPropertyPrintTitleRows      = "PrintTitleRows"
const string cExcelPropertyRange               = "Range"
const string cExcelPropertyRegular             = "Regular"
const string cExcelPropertyRightFooter         = "RightFooter"
const string cExcelPropertyRightHeader         = "RightHeader"
const string cExcelPropertyRightMargin         = "RightMargin"
const string cExcelPropertyRowHeight           = "RowHeight"
const string cExcelPropertyScreenUpdating      = "ScreenUpdating"
const string cExcelPropertySelection           = "Selection"
const string cExcelPropertyShapes              = "Shapes"
const string cExcelPropertySheets              = "Sheets"
const string cExcelPropertyShrinkToFit         = "ShrinkToFit"
const string cExcelPropertySplitRow            = "SplitRow"
const string cExcelPropertyStrikeThrough       = "StrikeThrough"
const string cExcelPropertySubscript           = "Subscript"
const string cExcelPropertySuperscript         = "Superscript"
const string cExcelPropertySymbol              = "Symbol"
const string cExcelPropertyTop                 = "Top"
const string cExcelPropertyTopMargin           = "TopMargin"
const string cExcelPropertyUnderline           = "Underline"
const string cExcelPropertyVPageBreaks         = "VPageBreaks"
const string cExcelPropertyValue               = "Value"
const string cExcelPropertyVerticalAlignment   = "VerticalAlignment"
const string cExcelPropertyView                = "View"
const string cExcelPropertyVisible             = "Visible"
const string cExcelPropertyWeight              = "Weight"
const string cExcelPropertyWidth               = "Width"
const string cExcelPropertyWindowState         = "WindowState"
const string cExcelPropertyWindows             = "Windows"
const string cExcelPropertyWorkbooks           = "Workbooks"
const string cExcelPropertyWrapText            = "WrapText"

//--------------------- Excel VBA Methods -------------------------

const string cExcelMethodActivate     = "Activate"
const string cExcelMethodAdd          = "Add"
const string cExcelMethodAutoFilter   = "AutoFilter"
const string cExcelMethodDragOff      = "DragOff"
const string cExcelMethodItem         = "Item"
const string cExcelMethodOpen         = "Open"
const string cExcelMethodPasteSpecial = "PasteSpecial"
const string cExcelMethodScaleWidth   = "ScaleWidth"
const string cExcelMethodScaleHeight  = "ScaleHeight"
const string cExcelMethodSelect       = "Select"
const string cExcelMethodSave         = "SaveAs"

//--------------------- Excel OLE Parameters  --------------------

const string cExcelParamDirection   = "Direction"
const string cExcelParamFileName    = "FileName"
const string cExcelParamFormat      = "Format"
const string cExcelParamLength      = "Length"
const string cExcelParamOLELink     = "Link"
const string cExcelParamRegionIndex = "RegionIndex"
const string cExcelParamStart       = "Start"

//----------------- Excel VBA Symbolic Constants -----------------

const int xlActiveWindow         = 1
const int xlMinimized            = -4140
const int xlNormal               = -4143
const int xlMaximumOutlineLevel  = 8
const int xlHAlignCenter         = -4108
const int xlHAlignJustify        = -4130
const int xlHAlignLeft           = -4131
const int xlHAlignRight          = -4152
const int xlUnderlineStyleSingle = 2
const int xlUnderlineStyleNone   = -4142
const int xlVAlignTop            = -4160
const int xlPortrait             = 1
const int xlLandscape            = 2
const int xlHorizontal           = -4128
const int xlPaperA4              = 9
const int xlPaperA3              = 8
const int xlPaperA5              = 11
const int xlPaperLegal           = 5
const int xlPaperLetter          = 1
const int xlContinuous           = 1
const int xlThin                 = 2
const int xlAutomatic            = -4105
const int xlNormalView           = 1
const int xlPageBreakPreview     = 2
const int xlToRight              = -4161

// name based on wdPasteEnhancedMetafile, no such constant in Excel
const string xlPasteEnhancedMetafile = "Picture (Enhanced Metafile)"

//----------- Excel VBA Functions (Properties and Methods) ---------

string excelGetNameOLE( OleAutoObj objExcelItem ) {
   string itemName = ""
   oleResult( oleGet( objExcelItem, cExcelPropertyName, itemName ) )
   return( itemName )
}

OleAutoObj excelSetRangeOLE( OleAutoObj objExcelSheet, string range ) {
   // Range is a 'A1-style reference'
   clear( objArgBlock )
   put( objArgBlock, range )
   OleAutoObj objExcelRange = null
   oleResult( oleGet( objExcelSheet, cExcelPropertyRange, objArgBlock, objExcelRange ) )
   return( objExcelRange )
}

void excelSelectRangeOLE( OleAutoObj objExcelRange ) {
   oleResult( oleMethod( objExcelRange, cExcelMethodSelect ) )
}

OleAutoObj excelGetSelectionOLE( OleAutoObj objExcelApplication ) {
   OleAutoObj objExcelSelection
   oleResult( oleGet( objExcelApplication, cExcelPropertySelection, objExcelSelection ) )
   return( objExcelSelection )
}

void excelDisableFiltersOLE( OleAutoObj objExcelApplication, OleAutoObj objExcelSheet ) {

   // This function disables any Filters or AutoFilters in an Excel sheet

   OleAutoObj objExcelSelection = excelGetSelectionOLE( objExcelApplication )
   if ( !null objExcelSelection ) {

      bool autoFilterActive
      oleResult( oleGet( objExcelSheet, cExcelPropertyAutoFilterMode, autoFilterActive ) )

      // autoFilterActive will either be true or null (it will not specifically be false)
      if ( autoFilterActive == true ) {

         // In order to use AutoFilter method on Range class, we must specify
         // a range. Since the AutoFilter method works on the Sheet, and not
         // on a range, it doesn't matter what range we specify.
         OleAutoObj objExcelRange = excelSetRangeOLE( objExcelSheet, EXCEL_FIRST_CELL_IDENTIFIER )

         // Disable AutoFilter
         oleResult( oleMethod( objExcelRange, cExcelMethodAutoFilter ) )
      }
   }
}

int excelGetCountOLE( OleAutoObj objExcelCollection ) {
   int c = 0
   oleResult( oleGet( objExcelCollection, cExcelPropertyCount, c ) )
   return( c )
}

bool excelSheetExistsOLE( OleAutoObj objExcelWorkbook, int sheetNumber ) {
   OleAutoObj objExcelSheets
   oleResult( oleGet( objExcelWorkbook, cExcelPropertySheets, objExcelSheets ) )
   int numSheetsInWorkbook = excelGetCountOLE( objExcelSheets )
   //oleResult( oleGet( objExcelSheets, cExcelPropertyCount, numSheetsInWorkbook ) )
   debug( "Number of Sheets in Workbook [" numSheetsInWorkbook "]" )
   return( sheetNumber <= numSheetsInWorkbook )
}

OleAutoObj excelGetSheetOLE( OleAutoObj objExcelWorkbook, int sheetNumber ) {
   if ( !excelSheetExistsOLE( objExcelWorkbook, sheetNumber ) ) {
      sheetNumber = 1
   }
   OleAutoObj objExcelSheet
   clear( objArgBlock )
   put( objArgBlock, sheetNumber )
   oleResult( oleGet( objExcelWorkbook, cExcelPropertySheets, objArgBlock, objExcelSheet ) )
   return( objExcelSheet )
}

void excelSetScreenUpdatingOLE( OleAutoObj objExcelApplication, bool screenUpdatingActive ) {
   oleResult( olePut( objExcelApplication, cExcelPropertyScreenUpdating, screenUpdatingActive ) )
}

void excelSetVisibleOLE( OleAutoObj objExcelApplication ) {
   bool excelIsVisible = false
   oleResult( oleGet( objExcelApplication, cExcelPropertyVisible, excelIsVisible ) )
   if ( !excelIsVisible ) {
      oleResult( olePut( objExcelApplication, cExcelPropertyVisible, true ) )
   } 
}

bool initExcelOLE( bool       runInteractive,
                   bool       exportOLEObjects,
                   int        dataExportSheetNumber,
                   bool       exportOLEToSeparateSheet,
                   int        OLEExportSheetNumber,
                   string     existingFileName,
                   bool       useExistingExcelFile,
                   OleAutoObj &objExcelApplication,
                   OleAutoObj &objExcelWorkbook,
                   OleAutoObj &objExcelDataExportSheet,
                   OleAutoObj &objExcelOLEExportSheet
                 )
{
   objExcelApplication = oleCreate( cExcelProgrammaticIdentifier, "Excel" )
   if ( null objExcelApplication ) return( false )

   // Make Excel Application window visible
   excelSetVisibleOLE( objExcelApplication )

   // Minimize Excel Application
   oleResult( olePut( objExcelApplication, cExcelPropertyWindowState, xlMinimized ) )

   // Get Workbooks collection
   OleAutoObj objExcelWorkbooks = null
   oleResult( oleGet( objExcelApplication, cExcelPropertyWorkbooks, objExcelWorkbooks ) )
   if ( null objExcelWorkbooks ) {
      if ( runInteractive ) errorBox errorCannotGetWorkbooks
      return( false )
   }

   // Get Workbook object for Excel file to which data will be exported
   if ( useExistingExcelFile == true ) {
      // Open existing Excel file
      clear( objArgBlock )
      put( objArgBlock, cExcelParamFileName, existingFileName )
      oleResult( oleMethod( objExcelWorkbooks, cExcelMethodOpen, objArgBlock ) )
   }
   else {
      // Open new Excel file
      oleResult( oleMethod( objExcelWorkbooks, cExcelMethodAdd ) )
   }

   // Get active workbook
   oleResult( oleGet( objExcelApplication, cExcelPropertyActiveWorkbook, objExcelWorkbook ) )

   if ( null objExcelWorkbook ) {
      if ( runInteractive ) errorBox errorCannotGetActiveWorkbook
      return( false )
   }

   // Get Data Sheet from Sheets collection
   if ( !excelSheetExistsOLE( objExcelWorkbook, dataExportSheetNumber ) ) {      
      if ( runInteractive ) {
         infoSheetDoesNotExist( dataExportSheetNumber, dataSheetName, 1 )
      }
      dataExportSheetNumber = 1
   }
   objExcelDataExportSheet = excelGetSheetOLE( objExcelWorkbook, dataExportSheetNumber )
   if ( null objExcelDataExportSheet ) {
      if ( runInteractive ) errorCannotGetSheet( dataExportSheetNumber )
      return( false )
   }

   if ( exportOLEObjects && exportOLEToSeparateSheet ) {

      // Get OLE Separate Sheet from Sheets collection
      if ( !excelSheetExistsOLE( objExcelWorkbook, OLEExportSheetNumber ) ) {
         if ( runInteractive ) {
            infoSheetDoesNotExist( OLEExportSheetNumber, OLESheetName, 2 )
         }
         OLEExportSheetNumber = 2
      }
      objExcelOLEExportSheet = excelGetSheetOLE( objExcelWorkbook, OLEExportSheetNumber )
      if ( null objExcelOLEExportSheet ) {
         if ( runInteractive ) errorCannotGetSheet( OLEExportSheetNumber )
         return( false )
      }
   }

   if ( ( useExistingExcelFile ) && ( exportOLEObjects ) ) {
      // Filters in the Excel Sheet will disrupt OLE object formatting.
      // Disable Excel filters in the sheet scheduled for OLE export, if
      // they exist.
      if ( exportOLEToSeparateSheet ) {
         excelDisableFiltersOLE( objExcelApplication, objExcelOLEExportSheet )
      }
      else {
         excelDisableFiltersOLE( objExcelApplication, objExcelDataExportSheet )
      }
   }

   excelSetScreenUpdatingOLE( objExcelApplication, false )

   return( true )  
}
    
void excelDoneOLE( OleAutoObj objExcelApplication ) {
   if ( !null objExcelApplication ) {
      excelSetScreenUpdatingOLE( objExcelApplication, true )
      excelSetVisibleOLE( objExcelApplication )
      oleResult( olePut( objExcelApplication, cExcelPropertyWindowState, xlNormal ) )
	  oleMethod( objExcelApplication, "Quit")
	  oleClose( objExcelApplication )
   }
}

OleAutoObj excelGetCellOLE( OleAutoObj objExcelSheet, string cell ) {
   clear( objArgBlock )
   put( objArgBlock, cell )
   OleAutoObj objExcelCell = null
   oleResult( oleGet( objExcelSheet, cExcelPropertyRange, objArgBlock, objExcelCell ) )
   return( objExcelCell )
}

OleAutoObj excelGetCellByCoordinatesOLE( OleAutoObj objExcelSheet,
                                         int iRow,
                                         int iColumn
                                       )
{
   OleAutoObj objExcelCell =
      excelGetCellOLE( objExcelSheet, getExcelColumnAlphaAddressFromInt( iColumn ) ( iRow ) "" )
   return( objExcelCell )
}

OleAutoObj excelSelectRangeByCoordinatesOLE( OleAutoObj objExcelSheet,
                                             int iRow,
                                             int iColumn
                                           )
{
   string range = getExcelColumnAlphaAddressFromInt( iColumn ) ( iRow ) ""
   debug( "Range:[" range "]" )

   OleAutoObj objExcelRange = excelSetRangeOLE( objExcelSheet, range )
   excelSelectRangeOLE( objExcelRange )
   return( objExcelRange )
}

OleAutoObj excelSelectRangeByCoordinatesOLE( OleAutoObj objExcelSheet,
                                             int iRowStart,
                                             int iColumnStart,
                                             int iRowEnd,
                                             int iColumnEnd
                                           )
{
   string range = getExcelColumnAlphaAddressFromInt( iColumnStart ) ( iRowStart ) ":" //-
                  getExcelColumnAlphaAddressFromInt( iColumnEnd )   ( iRowEnd   ) ""
   debug( "Range:[" range "]" )

   OleAutoObj objExcelRange = excelSetRangeOLE( objExcelSheet, range )
   excelSelectRangeOLE( objExcelRange )
   return( objExcelRange )
}

OleAutoObj excelGetCharactersOLE( OleAutoObj objExcelCell, int iStart, int iLength ) {
   clear( objArgBlock )
   put( objArgBlock, cExcelParamStart, iStart )
   put( objArgBlock, cExcelParamLength, iLength )
   OleAutoObj objExcelChars = null
   oleResult( oleGet( objExcelCell, cExcelPropertyCharacters, objArgBlock, objExcelChars ) )
   return( objExcelChars )
}

OleAutoObj excelGetFontOLE( OleAutoObj objExcelCell, int iStart, int iLength ) {
   OleAutoObj objExcelChars = excelGetCharactersOLE( objExcelCell, iStart, iLength )
   OleAutoObj objExcelFont = null
   oleResult( oleGet( objExcelChars, cExcelPropertyFont, objExcelFont ) )
   return( objExcelFont )
}

void excelSetFontSymbolOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyName, cExcelPropertySymbol ) )
}

void excelSetFontRegularOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyFontStyle, cExcelPropertyRegular ) )
}

void excelSetFontBoldOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyFontStyle, cExcelPropertyBold ) )
}

void excelSetFontItalicOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyFontStyle, cExcelPropertyItalic ) )
}

void excelSetFontBoldItalicOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyFontStyle, cExcelPropertyBoldItalic ) )
}

void excelSetFontUnderlineSingleOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyUnderline, xlUnderlineStyleSingle ) )
}

void excelSetFontUnderlineNoneOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyUnderline, xlUnderlineStyleNone ) )
}

void excelSetFontStrikeThroughOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertyStrikeThrough, true ) )
}

void excelSetFontSuperscriptOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertySuperscript, true ) )
}

void excelSetFontSubscriptOLE( OleAutoObj objExcelFont ) {
   oleResult( olePut( objExcelFont, cExcelPropertySubscript, true ) )
}

OleAutoObj excelSetCellOLE( OleAutoObj objExcelSheet, int iRow, int iColumn, string cellContents ) {
   if ( !null cellContents ) {
      // Strings beginning with EXCEL_STRING_CHARACTER need to be prepended with
      // the EXCEL_STRING_CHARACTER so that the desired EXCEL_STRING_CHARACTER appears in Excel
      if ( cellContents[0] == EXCEL_STRING_CHARACTER ) {
         cellContents = EXCEL_STRING_CHARACTER cellContents
      }
      // Strings beginning with '=' or '-' may be interpreted by Excel as formulas,
      // so prepend the EXCEL_STRING_CHARACTER to the string
      if ( ( cellContents[0] == '=' ) || ( cellContents[0] == '-' ) ) {
         cellContents = EXCEL_STRING_CHARACTER cellContents
      }
      // Trim cell text to paste to not exceed EXCEL_MAX_CHARACTERS_IN_CELL (if necessary)
      if ( length( cellContents ) > EXCEL_MAX_CHARACTERS_IN_CELL ) {
         cellContents = cellContents[ 0 : EXCEL_MAX_CHARACTERS_IN_CELL - 1 ]
      }
      OleAutoObj objExcelCell = excelGetCellByCoordinatesOLE( objExcelSheet, iRow, iColumn )
      if ( !null objExcelCell ) {
         oleResult( olePut( objExcelCell, cExcelPropertyValue, cellContents ) )
         return( objExcelCell )
      }
   }
   return( null )
}

string excelGetActivePrinterOLE( OleAutoObj objExcelApplication ) {
   string activePrinterName = ""
   oleGet( objExcelApplication, cExcelPropertyActivePrinter, activePrinterName )
   return( activePrinterName )
}

OleAutoObj excelGetPageSetupOLE( OleAutoObj objExcelSheet ) {
   OleAutoObj objExcelPageSetup
   oleResult( oleGet( objExcelSheet, cExcelPropertyPageSetup, objExcelPageSetup ) )
   return( objExcelPageSetup )
} 

OleAutoObj excelGetColumnOLE( OleAutoObj objExcelColumns, int columnNumber ) {
   clear( objArgBlock )
   put( objArgBlock, columnNumber )
   OleAutoObj objExcelColumn
   oleResult( oleGet( objExcelColumns, cExcelPropertyColumns, objArgBlock, objExcelColumn ) )
   return( objExcelColumn )
}

void excelPageSetupCenterHorizontally( OleAutoObj objExcelPageSetup, bool doCenter ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyCenterHorizontally, doCenter ) )
}

void excelSetLeftHeaderOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyLeftHeader, content ) )
}

void excelSetCenterHeaderOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyCenterHeader, content ) )
}

void excelSetRightHeaderOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyRightHeader, content ) )
}

void excelSetLeftFooterOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyLeftFooter, content ) )
}

void excelSetCenterFooterOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyCenterFooter, content ) )
}

void excelSetRightFooterOLE( OleAutoObj objExcelPageSetup, string content ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyRightFooter, content ) )
}

void excelSetPageOrientationOLE( OleAutoObj objExcelPageSetup, int pageOrientation ) {
   if ( pageOrientation == DOORS_PAGE_ORIENTATION_PORTRAIT ) {
      debug( "Page Orientation: Portrait" )
      oleResult( olePut( objExcelPageSetup, cExcelPropertyOrientation, xlPortrait ) )
   }
   if ( pageOrientation == DOORS_PAGE_ORIENTATION_LANDSCAPE ) {
      debug( "Page Orientation: Landscape" )
      oleResult( olePut( objExcelPageSetup, cExcelPropertyOrientation, xlLandscape ) )
   }
}

bool excelSetPaperSizeOLE( OleAutoObj objExcelPageSetup, int xlPaper ) {
   debug( "Set Paper Size[" xlPaper "]" )
   return( checkSetPaperSizeRes( olePut( objExcelPageSetup, cExcelPropertyPaperSize, xlPaper ) ) )
}

void excelSetPrintColumnTitleRowsOLE( OleAutoObj objExcelPageSetup, string titleRows ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyPrintTitleRows, titleRows ) )
}

int excelGetLeftMarginOLE( OleAutoObj objExcelPageSetup ) {
   int leftMargin
   oleResult( oleGet( objExcelPageSetup, cExcelPropertyLeftMargin, leftMargin ) )
   return( leftMargin )
}

int excelGetRightMarginOLE( OleAutoObj objExcelPageSetup ) {
   int rightMargin
   oleResult( oleGet( objExcelPageSetup, cExcelPropertyRightMargin, rightMargin ) )
   return( rightMargin )
}

void excelSetLeftMarginOLE( OleAutoObj objExcelPageSetup, int leftMargin ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyLeftMargin, leftMargin ) )
}

void excelSetRightMarginOLE( OleAutoObj objExcelPageSetup, int rightMargin ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyRightMargin, rightMargin ) )
}

void excelSetTopMarginOLE( OleAutoObj objExcelPageSetup, int topMargin ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyTopMargin, topMargin ) )
}

void excelSetBottomMarginOLE( OleAutoObj objExcelPageSetup, int bottomMargin ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyBottomMargin, bottomMargin ) )
}

OleAutoObj excelGetColumnsOLE( OleAutoObj objExcelSheet ) {
   OleAutoObj objExcelColumns
   oleResult( oleGet( objExcelSheet, cExcelPropertyColumns, objExcelColumns ) )
   return( objExcelColumns )
}

int excelGetColumnCharacterWidthOLE( OleAutoObj objExcelColumn ) {
   // Note: Width is in units equal to
   //       the width of one character in the Normal style
   //       For proportional fonts, the width of the character 0 (zero) is used.
   int iColumnWidth
   oleResult( oleGet( objExcelColumn, cExcelPropertyColumnWidth, iColumnWidth ) )
   return( iColumnWidth )
}

void excelSetColumnCharacterWidthOLE( OleAutoObj objExcelColumn, int iColumnWidth ) {
   // Note: Width is in units equal to
   //       the width of one character in the Normal style
   //       For proportional fonts, the width of the character 0 (zero) is used.
   oleResult( olePut( objExcelColumn, cExcelPropertyColumnWidth, iColumnWidth ) )
}

int excelGetColumnWidthOLE( OleAutoObj objExcelColumn ) {
   int iWidth
   oleResult( oleGet( objExcelColumn, cExcelPropertyWidth, iWidth ) )
   return( iWidth )
}

void excelActivateSheetOLE( OleAutoObj objExcelSheet ) {
   oleResult( oleMethod( objExcelSheet, cExcelMethodActivate ) )
}

void excelEnableExcelAutoFilterOLE( OleAutoObj objExcelSheet, OleAutoObj objExcelRange ) {
   bool autoFilterActive
   oleResult( oleGet( objExcelSheet, cExcelPropertyAutoFilterMode, autoFilterActive ) )

   // autoFilterActive will either be true or null (it will not specifically be false)
   if ( null autoFilterActive ) {
      // Enable AutoFilter
      oleResult( oleMethod( objExcelRange, cExcelMethodAutoFilter ) )
   }
}

OleAutoObj excelGetWindowsOLE( OleAutoObj objExcelWorkbook ) {
   OleAutoObj objExcelWindows = null
   oleResult( oleGet( objExcelWorkbook, cExcelPropertyWindows, objExcelWindows ) )
   return( objExcelWindows )
}

OleAutoObj excelGetItemOLE( OleAutoObj objExcelCollection, int i ) {
   clear( objArgBlock )
   put( objArgBlock, i )
   OleAutoObj objExcelItem = null
   oleResult( oleGet( objExcelCollection, cExcelPropertyItem, objArgBlock, objExcelItem ) )
   return( objExcelItem )
}

OleAutoObj excelGetItemFromCollectionOLE( OleAutoObj objExcelCollection, int i ) {
   clear( objArgBlock )
   put( objArgBlock, i )
   OleAutoObj objExcelItem = null
   oleResult( oleMethod( objExcelCollection, cExcelMethodItem, objArgBlock, objExcelItem ) )
   return( objExcelItem )
}

void excelEnableSplitHeaderRowsOLE( OleAutoObj objExcelWindow, int headerRowsToSplit ) {
   oleResult( olePut( objExcelWindow, cExcelPropertySplitRow, headerRowsToSplit ) )
}

void excelEnableFreezePanesOLE( OleAutoObj objExcelWindow ) {
   oleResult( olePut( objExcelWindow, cExcelPropertyFreezePanes, true ) )
}

bool excelPasteOLEObjectOLE( OleAutoObj objExcelSheet, int iRow, int iColumn ) {	
   OleAutoObj objExcelCell = excelGetCellByCoordinatesOLE( objExcelSheet, iRow, iColumn )
   oleResult( oleMethod( objExcelCell, cExcelMethodSelect ) )
   clear( objArgBlock )
   put( objArgBlock, cExcelParamOLELink, false )
   string errMess = oleMethod( objExcelSheet, cExcelMethodPasteSpecial, objArgBlock )
   if ( !null errMess ) {
      // Windows Enhanced Metafiles are known to fail the initial attempt to Paste,
      // so handle by exception (add Format parameter to Argument Block and repaste)
      debug( "Could not Paste, attempting to paste Enhanced Metafile" )
      clear( objArgBlock )
      put( objArgBlock, cExcelParamOLELink, false )
      put( objArgBlock, cExcelParamFormat, xlPasteEnhancedMetafile )
      string errMess = oleMethod( objExcelSheet, cExcelMethodPasteSpecial, objArgBlock )
      if ( !null errMess ) {
         return( false )
      }
   }
   return( true )
}

OleAutoObj excelGetShapesOLE( OleAutoObj objExcelSheet ) {
   OleAutoObj objExcelShapes = null
   oleResult( oleGet( objExcelSheet, cExcelPropertyShapes, objExcelShapes ) )
   return( objExcelShapes )
}

OleAutoObj excelGetShapeRangeOLE( OleAutoObj objExcelShapes, int iShapeCount ) {
   OleAutoObj objExcelShapeRange = null
   clear( objArgBlock )
   put( objArgBlock, iShapeCount )
   oleResult( oleGet( objExcelShapes, cExcelPropertyRange, objArgBlock, objExcelShapeRange ) )
   return( objExcelShapeRange )
}

int excelGetShapeRangeWidthOLE( OleAutoObj objExcelShapeRange ) {
   int iWidth
   oleResult( oleGet( objExcelShapeRange, cExcelPropertyWidth, iWidth ) )
   return( iWidth )
}

void excelSetShapeRangeWidthOLE( OleAutoObj objExcelShapeRange, int iWidth ) {
   oleResult( olePut( objExcelShapeRange, cExcelPropertyWidth, iWidth ) )
}

void excelSetShapeRangeHeightOLE( OleAutoObj objExcelShapeRange, int iHeight ) {
   oleResult( olePut( objExcelShapeRange, cExcelPropertyHeight, iHeight ) )
}

int excelGetShapeRangeHeightOLE( OleAutoObj objExcelShapeRange ) {
   int iHeight
   oleResult( oleGet( objExcelShapeRange, cExcelPropertyHeight, iHeight ) )
   return( iHeight )
}

int excelGetCellWidthOLE( OleAutoObj objExcelCell ) {
   int iWidth
   oleResult( oleGet( objExcelCell, cExcelPropertyWidth, iWidth  ) )
   return( iWidth )
}

int excelGetCellHeightOLE( OleAutoObj objExcelCell ) {
   int iHeight
   oleResult( oleGet( objExcelCell, cExcelPropertyHeight, iHeight ) )
   return( iHeight )
}

void excelShapeRangeLockAspectRatioOLE( OleAutoObj objExcelShapeRange, bool lockAspectRatio ) {
   oleResult( olePut( objExcelShapeRange, cExcelPropertyLockAspectRatio, lockAspectRatio ) )
}

void excelShapeRangeScaleWidth( OleAutoObj objExcelShapeRange, bool relativeToOriginalSize ) {
   clear( objArgBlock )
   put( objArgBlock, 1 )
   put( objArgBlock, relativeToOriginalSize )
   oleResult( oleMethod( objExcelShapeRange, cExcelMethodScaleWidth, objArgBlock ) )
}

void excelShapeRangeScaleHeight( OleAutoObj objExcelShapeRange, bool relativeToOriginalSize ) {
   clear( objArgBlock )
   put( objArgBlock, 1 )
   put( objArgBlock, relativeToOriginalSize )
   oleResult( oleMethod( objExcelShapeRange, cExcelMethodScaleHeight, objArgBlock ) )
}

void excelSetRowHeightOLE( OleAutoObj objExcelCell, int iHeight ) {
   oleResult( olePut( objExcelCell, cExcelPropertyRowHeight, iHeight ) )
}

int excelGetShapeRangeLeftOLE( OleAutoObj objExcelShapeRange ) {
   int iLeft
   oleResult( oleGet( objExcelShapeRange, cExcelPropertyLeft, iLeft ) )
   return( iLeft )
}

void excelSetShapeRangeLeftOLE( OleAutoObj objExcelShapeRange, int iLeft ) {
   oleResult( olePut( objExcelShapeRange, cExcelPropertyLeft, iLeft ) )
}

int excelGetShapeRangeTopOLE( OleAutoObj objExcelShapeRange ) {
   int iTop
   oleResult( oleGet( objExcelShapeRange, cExcelPropertyTop, iTop ) )
   return( iTop )
}

void excelSetShapeRangeTopOLE( OleAutoObj objExcelShapeRange, int iTop ) {
   oleResult( olePut( objExcelShapeRange, cExcelPropertyTop, iTop ) )
}

void excelSetRangeBoldOLE( OleAutoObj objExcelRange ) {   
   OleAutoObj objExcelFont
   oleResult( oleGet( objExcelRange, cExcelPropertyFont, objExcelFont ) )
   oleResult( olePut( objExcelFont, cExcelPropertyBold, true ) )
}

void excelSetRangeHorizontalAlignmentLeftOLE( OleAutoObj objExcelRange ) { 
   debug( "Justification: Left" )
   oleResult( olePut( objExcelRange, cExcelPropertyHorizontalAlignment, xlHAlignLeft ) )
}

void excelSetRangeHorizontalAlignmentRightOLE( OleAutoObj objExcelRange ) { 
   debug( "Justification: Right" )
   oleResult( olePut( objExcelRange, cExcelPropertyHorizontalAlignment, xlHAlignRight ) )
}

void excelSetRangeHorizontalAlignmentCenterOLE( OleAutoObj objExcelRange ) { 
   debug( "Justification: Center" )
   oleResult( olePut( objExcelRange, cExcelPropertyHorizontalAlignment, xlHAlignCenter ) )
}

void excelSetRangeHorizontalAlignmentJustifyOLE( OleAutoObj objExcelRange ) { 
   debug( "Justification: Full" )
   oleResult( olePut( objExcelRange, cExcelPropertyHorizontalAlignment, xlHAlignJustify ) )
}

void excelSetRangeColorOLE( OleAutoObj objExcelRange, int iRGBValue ) {
   OleAutoObj objExcelInterior = null
   oleResult( oleGet( objExcelRange, cExcelPropertyInterior, objExcelInterior ) )
   oleResult( olePut( objExcelInterior, cExcelPropertyColor, iRGBValue ) ) 
}

OleAutoObj excelGetRowOLE( OleAutoObj objExcelRange ) {
   OleAutoObj objExcelRow
   oleResult( oleGet( objExcelRange, cExcelPropertyEntireRow, objExcelRow ) )
   return( objExcelRow )
}

void excelOutlineLevelOLE( OleAutoObj objExcelSheet, int iRow, int iColumn, int outlineLevel  ) {
   if ( outlineLevel > xlMaximumOutlineLevel ) outlineLevel = xlMaximumOutlineLevel
   OleAutoObj objExcelRange = excelSelectRangeByCoordinatesOLE( objExcelSheet, iRow, iColumn )
   OleAutoObj objExcelRow = excelGetRowOLE( objExcelRange )
   oleResult( olePut( objExcelRow, cExcelPropertyOutlineLevel, outlineLevel ) )                   
}

void excelWrapTextOLE( OleAutoObj objExcelRange, bool wrapText ) {
   oleResult( olePut( objExcelRange, cExcelPropertyWrapText, wrapText ) )
}

void excelVerticalAlignmentTopOLE( OleAutoObj objExcelRange ) {
   oleResult( olePut( objExcelRange, cExcelPropertyVerticalAlignment, xlVAlignTop ) )
}

void excelOrientationHorizontalOLE( OleAutoObj objExcelRange ) {
   oleResult( olePut( objExcelRange, cExcelPropertyOrientation, xlHorizontal ) )
}

void excelShrinkToFitOLE( OleAutoObj objExcelRange, bool shrinkToFit ) {
   oleResult( olePut( objExcelRange, cExcelPropertyShrinkToFit, shrinkToFit ) )
}

void excelMergeCellsOLE( OleAutoObj objExcelRange, bool mergeCells ) {
   oleResult( olePut( objExcelRange, cExcelPropertyMergeCells, mergeCells ) )
}

OleAutoObj excelGetBordersOLE( OleAutoObj objExcelRange ) {
   OleAutoObj objExcelBorders = null
   oleResult( oleGet( objExcelRange, cExcelPropertyBorders, objExcelBorders ) )
   return( objExcelBorders )
}

void excelSetLineStyleContinuousOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyLineStyle, xlContinuous ) )
}

void excelSetWeightThinOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyWeight, xlThin ) )
}

void excelSetColorIndexAutomaticOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyColorIndex, xlAutomatic ) )
}

int excelGetPaperSizeOLE( OleAutoObj objExcelPageSetup ) {
   int paperSize = 0
   oleResult( oleGet( objExcelPageSetup, cExcelPropertyPaperSize, paperSize ) )
   return( paperSize )
}

void excelSetLineStyleContinuousOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyLineStyle, xlContinuous ) )
}

void excelSetWeightThinOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyWeight, xlThin ) )
}

void excelSetColorIndexAutomaticOLE( OleAutoObj objExcelBorders ) {
   oleResult( olePut( objExcelBorders, cExcelPropertyColorIndex, xlAutomatic ) )
}

void excelSetPrintAreaOLE( OleAutoObj objExcelPageSetup, string address ) {
   oleResult( olePut( objExcelPageSetup, cExcelPropertyPrintArea, address ) )
}

void excelAdjustVerticalPageBreakOLE( OleAutoObj objExcelWorkbook, OleAutoObj objExcelSheet ) {

   // Note:  This is done because Columns in Excel are set by character width integers, 
   //        which is not as precise as width in points.
   //        DOORS does not allow real numbers to be passed with olePut.
   //        As a result, an unnecessary Page Break may be set.

   OleAutoObj objExcelWindows = excelGetWindowsOLE( objExcelWorkbook )

   if ( !null objExcelWindows ) {

      OleAutoObj objExcelWindow = excelGetItemOLE( objExcelWindows, xlActiveWindow )

      if ( !null objExcelWindow ) {

         oleResult( olePut( objExcelWindow, cExcelPropertyView, xlPageBreakPreview ) )

         OleAutoObj objExcelVPageBreaks = null
         oleResult( oleGet( objExcelSheet, cExcelPropertyVPageBreaks, objExcelVPageBreaks ) )

         if ( !null objExcelVPageBreaks ) {

            int vPageBreakCount = excelGetCountOLE( objExcelVPageBreaks )

            if ( vPageBreakCount > 0 ) {

               OleAutoObj objExcelVPageBreak = excelGetItemOLE( objExcelVPageBreaks, 1 )

               if ( !null objExcelVPageBreak ) {

                  clear( objArgBlock )
                  put( objArgBlock, cExcelParamDirection, xlToRight )
                  put( objArgBlock, cExcelParamRegionIndex, 1 )
                  oleResult( oleMethod( objExcelVPageBreak, cExcelMethodDragOff, objArgBlock ) )
               }
            }
         }

         oleResult( olePut( objExcelWindow, cExcelPropertyView, xlNormalView ) )
      }
   }
}

// ---------------------- DOORS Export Functions  -------------------------

string trimWhitespace(string s) {
   int first = 0
   int last = length( s ) - 1
   while ( last >= 0 && isspace( s[ last ] ) ) last--
   while ( isspace( s[ first ] ) && first < last ) first++
   if ( s[ first:last ]==" " ) return( "" )
   return( s[ first:last ] )
}

bool isInteger( string s ) {
   return( INTEGER trimWhitespace( s ) )
}

int getInteger( string s ) {
   if ( isInteger( s ) ) {
      return( intOf( trimWhitespace( s ) ) )
   }
}

string getObjectNumber( Object o, bool getRichText ) {
  if ( getRichText ) {
     return( "{\\b " number( o ) "}" )
  }
  else {
     return( number( o ) )
  }
}

string getRichTextNoOle( Object o, string attributeName ) {
   return( richTextFragment( richTextNoOle( o.attributeName ) ) )
}

bool isHeadingRow( Object o ) {
   // An Object is a Heading Row if:
   //    It has text in the DOORS_OBJECT_HEADING Attribute, OR
   //    it has no text in either the DOORS_OBJECT_HEADING and DOORS_OBJECT_TEXT Attributes
   //    AND the Main Column DOORS_OBJECT_TEXT does not contain any OLE Objects
   //    AND the Object is not a table cell
   //    (based on the behavior of the User Interface, the Object Number is still displayed)

   // Ignores Rich Text in making determination

   return( ( !null o.DOORS_OBJECT_HEADING "" ) ||
           ( ( null o.DOORS_OBJECT_HEADING "" ) &&
             ( null o.DOORS_OBJECT_TEXT "" ) &&
             ( !oleIsObject( o ) ) &&
             ( !( table( o ) || row( o ) || cell( o ) ) )
           )
         )
}

string getObjectMainColumnContent( Module m, Object o, bool getHeadingNumber, bool getRichText ) {

   string returnString = ""

   if ( getHeadingNumber && isHeadingRow( o ) ) {
      returnString = getObjectNumber( o, getRichText ) " "
   }

   if ( !null o.DOORS_OBJECT_HEADING "" ) {
      if ( getRichText ) {
         returnString = returnString "{\\b " getRichTextNoOle( o, DOORS_OBJECT_HEADING ) "}"
      }
      else {
         returnString = returnString o.DOORS_OBJECT_HEADING ""
      }
   }
   
   if ( !outlining( m ) ) {
      if ( !null getRichTextNoOle( o, DOORS_OBJECT_TEXT ) ) {
         if ( !null returnString ) {
            returnString = returnString "\n"
         }
         if ( getRichText ) {
            returnString = returnString "" getRichTextNoOle( o, DOORS_OBJECT_TEXT )
         }
         else {
            returnString = returnString "" o.DOORS_OBJECT_TEXT ""
         }
      }
   }
   
   if ( table( o ) ) {
      returnString = TABLE_MARKER_TEXT
   }

   return( returnString ) 	
}

int getColorConstant( Object o, AttrDef ad ) {

   string attrName = ad.name
   AttrType at = ad.type
   string currentValue = o.attrName ""
   int colorCode = -1     // -1 means no color found

   if ( null currentValue ) return( colorCode )

   if ( at.type == attrEnumeration ) {
      int count
      for ( count = 0 ;
            count < at.size ;
            count++
          )
      {
         if ( currentValue == at.strings[count] ) {
            // DOORS 5.0 handles color constants differently than DOORS 4.1.4 SR2.
            // Added getRealColor function to translate result of colors function so
            // that Actual Color constant is passed to colorCode
            colorCode = getRealColor( at.colors[count] )
            if ( ( colorCode >= DOORS_MIN_REAL_COLOR ) && ( colorCode <= DOORS_MAX_REAL_COLOR ) ) {
               // do nothing
            }
            else {
               colorCode = -1
            }
         }   
      }
   }

   return( colorCode )
}

AttrDef getColorColumnAttributeDefinition( Module m, Column c, bool forceColor ) {

   // Check to see if an Column is showing Color
   string colorAttributeName = color c
   if ( !null colorAttributeName ) {
      return( find( m, colorAttributeName ) )
   }
   else {
      if ( forceColor ) {
         string colorAttributeName = attrName( c )
         if ( !null colorAttributeName ) {
            return( find( m, colorAttributeName ) )
         }
      }
   }

   return( null )
}

int getRGBValue( int colorConstant ) {
   int r = get( REAL_COLOR_RGB_VALUES, colorConstant, DOORS_RGB_RED_INDEX )
   int g = get( REAL_COLOR_RGB_VALUES, colorConstant, DOORS_GREEN_RED_INDEX )
   int b = get( REAL_COLOR_RGB_VALUES, colorConstant, DOORS_BLUE_RED_INDEX )
   return( r |
           g * intOf( pow( realOf( 2 ), realOf( 8  ) ) ) |
           b * intOf( pow( realOf( 2 ), realOf( 16 ) ) )
         )
}

string getHeaderCellTextToExport( Column c,
                                  bool exportAttributeNames,
                                  bool exportColumnTitles,
                                  int numberHeaderRows,
                                  int exportAreaRow
                                )
{
   // Get Header Cell Content
   string headerText = ""   
   if ( main c ) {
      headerText = title c
   } 
   else {
      // There are three possible Header Row Combinations
      //  - Row One ( Column Titles )
      //  - Row One ( Attribute Names )
      //  - Row One ( Attribute Names ) Row Two ( Column Titles )
      if ( ( exportAttributeNames ) &&
           ( 
              ( ( numberHeaderRows == 1 ) && ( exportAreaRow == 1 ) ) ||
              ( ( numberHeaderRows == 2 ) && ( exportAreaRow == 1 ) )
           )
         )
      {
         headerText = attrName c ""
      }
      if ( ( exportColumnTitles ) &&
           ( 
              ( ( numberHeaderRows == 1 ) && ( exportAreaRow == 1 ) ) ||
              ( ( numberHeaderRows == 2 ) && ( exportAreaRow == 2 ) )
           )
         )
      {
         headerText = title c
      }
   }
   return( headerText )
}

bool detectSymbolFont( string richTextString ) {
   bool hasSymbolFont = false
   RichText rt
   for rt in richTextString do {
      if ( rt.charset == charsetSymbol ) hasSymbolFont = true
   }
   return( hasSymbolFont )
}

bool cellTextHasSymbolFont( Module m, Object o, Column c ) {
   bool hasSymbolFont = false
   if ( main c ) {
      if ( !null o.DOORS_OBJECT_HEADING "" ) {
         if ( detectSymbolFont( richText( o.DOORS_OBJECT_HEADING ) ) ) hasSymbolFont = true
      }
      if ( !outlining( m ) ) {
         if ( !null getRichTextNoOle( o, DOORS_OBJECT_TEXT ) ) {
            if ( detectSymbolFont( richText( o.DOORS_OBJECT_TEXT ) ) ) hasSymbolFont = true
         }
      }
   }
   else {
      if ( detectSymbolFont( richText( c, o ) ) ) hasSymbolFont = true
   }
   
   return( hasSymbolFont )
}

string getCellTextToExport( Module m, Object o, Column c, bool exportHeadingNumber, bool exportRichText, bool useCRsforMulti ) {
   if ( main( c ) ) {
      return( getObjectMainColumnContent( m, o, exportHeadingNumber, exportRichText ) )
   }
   else {
      Buffer b = create
				
      if ( exportRichText ) {				
         b += richText( c, o )				
      }				
      else {				
         b += text( c, o )				
      }				

      if ( useCRsforMulti ) {
   		  string attributeName = attrName( c )		
   		  if ( !null attributeName ) {
   		     AttrDef ad = find( m, attributeName )
   		     if ( !null ad ) {
   		        if ( ad.multi ) {
   		           delete( b )
   		           b = create()
   		           AttrType at = ad.type
   		           int i
                  for ( i = 0 ; i < at.size ; i++ ) {
                     string attributeValue = at.strings[ i ]
                     if ( isMember( o.attributeName, attributeValue ) ) {
   		                 if ( length( b ) > 0 ) b += "\n"
   		                 b += attributeValue
   				          }
   				       }
   			      }
   		     }		
   	    }
   	  }

      return( stringOf( b ) )		
   }				
}

bool cellTextIsNull( Object o, Column c ) {
   // Ignores Rich Text in making determination for null cell content
   if ( main c ) {
      if ( isHeadingRow( o ) ) {
         return( false )
      }
      else {
         if ( null trimWhitespace( o.DOORS_OBJECT_TEXT "" ) ) return( true )
      }
   }
   else {
      if ( null trimWhitespace( text( c, o ) ) ) return( true )
   }
   return( false )
}

string stripRichText( string rts ) {
   Buffer plainText = create
   RichText rt
   for rt in rts do {
      plainText += rt.text
      if ( ( rt.newline ) && !( rt.last ) ) {
         plainText += "\n"
      }
   }
   return( trimWhitespace( stringOf( plainText ) ) )
}

int addOLERow( int rowOffset, int numRows ) {
   // Only add a Row (return 1) if:
   // No row offset and
   // Row offset and 
   if ( ( ( rowOffset == 0 ) && ( numRows > 1 ) ) ||
        ( ( rowOffset == 1 ) && ( numRows > 0 ) )
      )
   {
      return( 1 )
   }
   else {
      return( 0 )
   }
}

// -------------------- DOORS Excel Export Functions  ----------------------

bool excelHasActivePrinter( OleAutoObj objExcelApplication ) {
  return( !matches( "unknown printer", excelGetActivePrinterOLE( objExcelApplication ) ) )
}

int excelGetPageWidthPoints( OleAutoObj objExcelSheet, int pageOrientation ) {
   OleAutoObj objExcelPageSetup = excelGetPageSetupOLE( objExcelSheet ) 
   int paperSize = excelGetPaperSizeOLE( objExcelPageSetup )

   int paperSizeIndex = -1  // -1 means invalid Paper Size index
   if ( paperSize == xlPaperA4 ) {
      paperSizeIndex = DOORS_PAPER_SIZE_A4
   }
   if ( paperSize == xlPaperA3 ) {
      paperSizeIndex = DOORS_PAPER_SIZE_A3
   }
   if ( paperSize == xlPaperA5 ) {
      paperSizeIndex = DOORS_PAPER_SIZE_A5
   }
   if ( paperSize == xlPaperLegal ) {
      paperSizeIndex = DOORS_PAPER_SIZE_LEGAL
   }
   if ( paperSize == xlPaperLetter ) {
      paperSizeIndex = DOORS_PAPER_SIZE_LETTER
   }

   int pageWidthMM = 0
   if ( paperSizeIndex > -1 ) {
      debug( "Page Setup returned paper size '" DOORS_PAGE_SETUP_PAPER_SIZES[ paperSizeIndex ] "'" )
      if ( pageOrientation == DOORS_PAGE_ORIENTATION_PORTRAIT ) {
         pageWidthMM = intOf( DOORS_PAGE_SETUP_PAPER_WIDTHS_MM[ paperSizeIndex ] )
      }
      if ( pageOrientation == DOORS_PAGE_ORIENTATION_LANDSCAPE ) {
         pageWidthMM = intOf( DOORS_PAGE_SETUP_PAPER_HEIGHTS_MM[ paperSizeIndex ] )
      }
   }
   else {
      debug( "Using default Paper Size '" DOORS_PAGE_SETUP_PAPER_SIZES[ DOORS_PAPER_SIZE_LETTER ] "' Landscape" )
      // Assume a Paper Size of Letter in Landscape Orientation
      pageWidthMM = intOf( DOORS_PAGE_SETUP_PAPER_HEIGHTS_MM[ DOORS_PAPER_SIZE_LETTER ] )
   }

   return( convertMillimetersToPoints( pageWidthMM ) )
}

void excelSetColumnWidthPercentOfPageWidth( OleAutoObj objExcelSheet,
                                            int columnNumber,
                                            real columnPercent
                                          )
{
   // Get Page Setup for Data Export Sheet
   OleAutoObj objExcelPageSetup = excelGetPageSetupOLE( objExcelSheet )

   // Get Page Width in Points
   int pageWidthPoints = excelGetPageWidthPoints( objExcelSheet, XLpageSetupOrientation )
   debug( "pageWidthPoints:[" pageWidthPoints "]" )

   // Get Left and Right Margin
   int leftMarginPoints  = excelGetLeftMarginOLE( objExcelPageSetup )
   int rightMarginPoints = excelGetRightMarginOLE( objExcelPageSetup )
   
   // Compute the available Page Width
   int availablePageWidthPoints = pageWidthPoints - ( leftMarginPoints + rightMarginPoints )
   debug( "availablePageWidthPoints:[" availablePageWidthPoints "]" )

   OleAutoObj objExcelColumns = excelGetColumnsOLE( objExcelSheet )
   OleAutoObj objExcelColumn = excelGetColumnOLE( objExcelColumns, columnNumber )

   // Get the Width of the Column in Points
   int columnWidthPoints = excelGetColumnWidthOLE( objExcelColumn )

   // Get the Width of the Column in Characters
   int columnWidthChars  = excelGetColumnCharacterWidthOLE( objExcelColumn )
   debug( "columnWidthPoints:[" columnWidthPoints "]" )
   debug( "columnWidthChars [" columnWidthChars "]" )

   // Compute the number Characters per Point
   real columnCharsPerPoint = realOf( columnWidthChars ) / realOf( columnWidthPoints )
   debug( "columnCharsPerPoint:[" columnCharsPerPoint "]" )

   // Compute the new Width of the Column in Characters
   int columnWidthCharsToSet =
      intOf( realOf( availablePageWidthPoints ) * columnPercent * columnCharsPerPoint )

   // Set the new Width of the Column in Characters
   excelSetColumnCharacterWidthOLE( objExcelColumn, columnWidthCharsToSet )
}

void excelSetColumnWidths( OleAutoObj objExcelSheet,
                           int numberOfDataColumns,
                           int largestOLEinColumn[]
                         )
{
   int iColumn = 0
   for ( iColumn = 1 ; iColumn <= numberOfDataColumns ; iColumn++ ) {
      if ( largestOLEinColumn[ iColumn ] > 0 ) {
         OleAutoObj objExcelColumns = excelGetColumnsOLE( objExcelSheet )
         OleAutoObj objExcelColumn = excelGetColumnOLE( objExcelColumns, iColumn )

         // Get the Width of the Column in Points
         int columnWidthPoints = excelGetColumnWidthOLE( objExcelColumn )
         debug( "columnWidthPoints:[" columnWidthPoints "]" )

         // Get the Width of the Column in Characters
         int columnWidthChars  = excelGetColumnCharacterWidthOLE( objExcelColumn )
         debug( "columnWidthChars [" columnWidthChars "]" )

         // Compute the number Characters per Point
         real columnCharsPerPoint = realOf( columnWidthChars ) / realOf( columnWidthPoints ) * EXCEL_SET_COLUMN_SIZE_OFFSET
         debug( "columnCharsPerPoint:[" columnCharsPerPoint "]" )

         // Compute the new Width of the Column in Characters
         int columnWidthCharsToSet =
            intOf( realOf( largestOLEinColumn[ iColumn ] ) * columnCharsPerPoint )
                        
         // Set the new Width of the Column in Characters
         excelSetColumnCharacterWidthOLE( objExcelColumn, columnWidthCharsToSet )
      }
   }
}

void excelSetCellRichText( OleAutoObj objExcelCell, string cellRichTextContents ) {

   OleAutoObj objExcelFont = null
        
   int characterCount = 1
   RichText rt
   for rt in cellRichTextContents do {     

      objExcelFont = excelGetFontOLE( objExcelCell, characterCount, length( rt.text ) )

      if ( rt.charset == charsetSymbol ) excelSetFontSymbolOLE( objExcelFont )

      if ( !( rt.bold ) && !( rt.italic ) ) excelSetFontRegularOLE( objExcelFont )
      if (  ( rt.bold ) && !( rt.italic ) ) excelSetFontBoldOLE( objExcelFont )
      if ( !( rt.bold ) &&  ( rt.italic ) ) excelSetFontItalicOLE( objExcelFont )
      if (  ( rt.bold ) &&  ( rt.italic ) ) excelSetFontBoldItalicOLE( objExcelFont )

      if ( rt.underline ) {
         excelSetFontUnderlineSingleOLE( objExcelFont )
      }
      else {
         excelSetFontUnderlineNoneOLE( objExcelFont )
      }

      if ( rt.strikethru ) excelSetFontStrikeThroughOLE( objExcelFont )
      if ( rt.superscript ) excelSetFontSuperscriptOLE( objExcelFont )
      if ( rt.subscript ) excelSetFontSubscriptOLE( objExcelFont )

      characterCount += length rt.text

      if ( rt.newline ) characterCount++
   }
}

void excelSetCellContents( OleAutoObj objExcelSheet,
                           int row,
                           int column,
                           string cellContents,
                           bool exportRichText
                         )
{
   if ( exportRichText ) {
      OleAutoObj objExcelCell = excelSetCellOLE( objExcelSheet, row, column, stripRichText( cellContents ) )
      if ( !null objExcelCell ) {
         excelSetCellRichText( objExcelCell, cellContents )
      }
   }
   else {
      excelSetCellOLE( objExcelSheet, row, column, cellContents )
   }
}

OleAutoObj excelGetShapeRangeAtIndex( OleAutoObj objExcelSheet, int i ) {
   OleAutoObj objExcelShapeRange = null
   OleAutoObj objExcelShapes = excelGetShapesOLE( objExcelSheet )
   if ( !null objExcelShapes ) {
      objExcelShapeRange = excelGetShapeRangeOLE( objExcelShapes, i )
   }
   
   return( objExcelShapeRange )
}

int excelGetShapeWidthOLE( OleAutoObj objExcelSheet, int i ) {
   OleAutoObj objExcelShapeRange = excelGetShapeRangeAtIndex( objExcelSheet, i )

   // Get current Width and Height for OLE Object (in points)
   int OLEwidth =  excelGetShapeRangeWidthOLE( objExcelShapeRange )
   
   return( OLEwidth )
}

void excelSizeShape( OleAutoObj objExcelSheet, int iRow, int iColumn, bool doReduce, Skip &shapesExported ) {

   // Pasted shapes are added to the Shapes collection in Excel
   // New shapes are not always added as a new last shape by index in the ShapeRange
   // The order of the Shape in the ShaprRange seems to be set by the Name given to the Shape by Excel at the time of the paste
   // Some are named "Picture ##", and some are named "Object ##", ## being an integer
   // The list index starts with the "Picture ##" items in order, then the "Object ##" items in order
   // To work around this, loop through the Shapes collection after each addition and check for the new string name

   OleAutoObj objExcelShapeRange = null
   OleAutoObj objExcelShapes = excelGetShapesOLE( objExcelSheet )
   if ( !null objExcelShapes ) {
      int shapeCount = excelGetCountOLE( objExcelShapes )
      int i
      for ( i = 1 ; i <= shapeCount ; i++ ) {
         OleAutoObj objExcelShape = excelGetItemFromCollectionOLE( objExcelShapes, i )
         if ( !null objExcelShape ) {
            string itemName = excelGetNameOLE( objExcelShape )
            // New Shape will not be in the list, Skip list put() operation will return true for new Shape
            if ( put( shapesExported, itemName, objExcelShape ) ) {
               objExcelShapeRange = objExcelShape
            }  
         }
      }
   }

   // Some pasted shapes do not respect the initial cell selection and get pasted to A1
   OleAutoObj objExcelRange = excelSelectRangeByCoordinatesOLE( objExcelSheet, iRow, iColumn )
   excelSetShapeRangeLeftOLE( objExcelShapeRange, excelGetShapeRangeLeftOLE( objExcelRange ) )
   excelSetShapeRangeTopOLE( objExcelShapeRange, excelGetShapeRangeTopOLE( objExcelRange ) )

   // Get current Width and Height for OLE Object (in points)
   int OLEwidth  = excelGetShapeRangeWidthOLE( objExcelShapeRange )
   int OLEheight = excelGetShapeRangeHeightOLE( objExcelShapeRange )
   OleAutoObj objExcelCell = excelGetCellByCoordinatesOLE( objExcelSheet, iRow, iColumn )

   if ( doReduce ) {
      // Reduce OLE Object Width (if necessary)
      if ( OLEwidth > excelGetCellWidthOLE( objExcelCell ) - ( 2 * EXCEL_OLE_POSITION_OFFSET_WIDTH ) ) {
         OLEwidth = excelGetCellWidthOLE( objExcelCell ) - ( 2 * EXCEL_OLE_POSITION_OFFSET_WIDTH )
         excelShapeRangeLockAspectRatioOLE( objExcelShapeRange, true )
         excelSetShapeRangeWidthOLE( objExcelShapeRange, OLEwidth )
         OLEheight = excelGetShapeRangeHeightOLE( objExcelShapeRange )
      }

      // Reduce OLE Object Height (if necessary)
      if ( OLEheight > EXCEL_MAX_ROW_HEIGHT_POINTS - ( 2 * EXCEL_OLE_POSITION_OFFSET_HEIGHT ) ) {
         OLEheight = EXCEL_MAX_ROW_HEIGHT_POINTS - ( 2 * EXCEL_OLE_POSITION_OFFSET_HEIGHT )
         excelShapeRangeLockAspectRatioOLE( objExcelShapeRange, true )
         excelSetShapeRangeHeightOLE( objExcelShapeRange, OLEheight )
         OLEwidth = excelGetShapeRangeWidthOLE( objExcelShapeRange )
      }
   }

   // Increase Row Height to match OLE Object Height with Offset (if necessary)
   // Note:  Since there may be more than one OLE Object per row,
   //        the Row Height is only increased if necessary, no reduction is allowed
   if ( OLEheight > excelGetCellHeightOLE( objExcelCell ) - ( 2 * EXCEL_OLE_POSITION_OFFSET_HEIGHT ) ) {
      int rowHeight = OLEheight + ( 2 * EXCEL_OLE_POSITION_OFFSET_HEIGHT )
      if ( rowHeight > EXCEL_MAX_ROW_HEIGHT_POINTS ) {
         rowHeight = EXCEL_MAX_ROW_HEIGHT_POINTS
      }
      excelSetRowHeightOLE( objExcelCell, rowHeight )
   }

   // Position the OLE object away from the edge
   if ( EXCEL_OLE_POSITION_OFFSET_WIDTH > 0 ) {
      int OLEpositionLeft = excelGetShapeRangeLeftOLE( objExcelShapeRange )
      excelSetShapeRangeLeftOLE( objExcelShapeRange, OLEpositionLeft + EXCEL_OLE_POSITION_OFFSET_WIDTH )
   }
   if ( EXCEL_OLE_POSITION_OFFSET_HEIGHT > 0 ) {
      int OLEpositionTop = excelGetShapeRangeTopOLE( objExcelShapeRange )
      excelSetShapeRangeTopOLE( objExcelShapeRange, OLEpositionTop + EXCEL_OLE_POSITION_OFFSET_HEIGHT )
   }

   excelShapeRangeLockAspectRatioOLE( objExcelShapeRange, false )
   excelShapeRangeScaleWidth( objExcelShapeRange, false )
   excelShapeRangeScaleHeight( objExcelShapeRange, false )
}

bool excelExportOLEObject( EmbeddedOleObject ole, OleAutoObj objExcelSheet, int row, int column ) {
   if ( oleCopy( ole ) ) {
      if ( excelPasteOLEObjectOLE( objExcelSheet, row, column ) ) {
         return( true )
      }
   }

   excelSetCellContents( objExcelSheet,
                         row,
                         column,
                         OLE_MARKER_TEXT,
                         XLdoExportRichText
                       )
   return( false )
}

void trackLargestOLEInColumn( OleAutoObj objExcelSheet, int index, int &widthLargestOLEInColumn ) {
   int widthCurrentOLEInColumn = excelGetShapeWidthOLE( objExcelSheet, index )
   if ( widthCurrentOLEInColumn > widthLargestOLEInColumn ) widthLargestOLEInColumn = widthCurrentOLEInColumn
}

void excelExportOLEObjects( OleAutoObj objExcelDataExportSheet,
                            OleAutoObj objExcelOLEExportSheet,
                            Module m,
                            Object o,
                            Column c,
                            int row,
                            int column,
                            bool exportOLEObjects,
                            bool exportOLEToSeparateSheet,
                            int exportAreaOLERowOffset,
                            int &exportAreaOLERowsAdded,
                            int &numOLEObjectsExported,
                            Skip &shapesExported,
                            int largestOLEinColumn[]

                          ) 
{
   int numOLEObjectsExportedThisAttribute = 0

   string s = ""
   if ( !outlining( m ) && main( c ) ) {
      s = richTextWithOle( o.DOORS_OBJECT_TEXT )
   }
   else {
      s = richTextWithOle( c, o )
   }

   RichText rtf
   for rtf in s do {

      if ( rtf.isOle ) {

         EmbeddedOleObject ole = rtf.getEmbeddedOle

         if ( exportOLEObjects && exportOLEToSeparateSheet ) {
            excelActivateSheetOLE( objExcelOLEExportSheet )
            // For Separate Sheet, OLE Objects always in Column 1 (no offset)
            if ( excelExportOLEObject( ole,
                                       objExcelOLEExportSheet,
                                       numOLEObjectsExported + 1,
                                       1
                                     )
               )
            {
               numOLEObjectsExported++               
               excelSizeShape( objExcelOLEExportSheet,
                               numOLEObjectsExported,
                               1,
                               XLdoReduceOLEObjects,
                               shapesExported
                             )
               trackLargestOLEInColumn( objExcelOLEExportSheet, numOLEObjectsExported, largestOLEinColumn[ column ] )
               excelSetCellContents( objExcelDataExportSheet,
                                     row + exportAreaOLERowOffset + numOLEObjectsExportedThisAttribute,
                                     column,
                                     OLE_MARKER_TEXT,
                                     XLdoExportRichText
                                   )
               numOLEObjectsExportedThisAttribute++
            }	
            excelActivateSheetOLE( objExcelDataExportSheet )
         }
         else {        
            if ( exportOLEObjects ) {

               if ( excelExportOLEObject( ole,
                                          objExcelDataExportSheet,
                                          row + exportAreaOLERowOffset + numOLEObjectsExportedThisAttribute,
                                          column
                                        )
                  )
               {
                  debug( "Exported OLE Object at Row: " ( row + exportAreaOLERowsAdded + numOLEObjectsExportedThisAttribute ) "" )
                  numOLEObjectsExported++      
                  excelSizeShape( objExcelDataExportSheet,
                                  row + exportAreaOLERowOffset + numOLEObjectsExportedThisAttribute,
                                  column,
                                  XLdoReduceOLEObjects,
                                  shapesExported
                                )
                  trackLargestOLEInColumn( objExcelDataExportSheet, numOLEObjectsExported, largestOLEinColumn[ column ] )
                  numOLEObjectsExportedThisAttribute++
               }
            }
            else {
               debug( "Export OLE Marker" )
               excelSetCellContents( objExcelDataExportSheet,
                                     row + exportAreaOLERowOffset + numOLEObjectsExportedThisAttribute,
                                     column,
                                     OLE_MARKER_TEXT,
                                     XLdoExportRichText
                                   )
               numOLEObjectsExportedThisAttribute++
            }
         }

         exportAreaOLERowsAdded += addOLERow( exportAreaOLERowOffset,
                                              numOLEObjectsExportedThisAttribute - exportAreaOLERowsAdded
                                            )
      }
   }
}

bool excelExportPicture( Object o,
                         OleAutoObj objExcelSheet,
                         int row,
                         int column,
                         int &exportAreaOLERowsAdded,
                         int &numOLEObjectsExported,
                         Skip &shapesExported
                       )
{
   bool hasPicture = o.DOORS_PICTURE
   if ( hasPicture ) {
      if ( pictureCopy( o ) ) {
         if ( exportAreaOLERowsAdded == 0 ) exportAreaOLERowsAdded++
         numOLEObjectsExported++
         excelPasteOLEObjectOLE( objExcelSheet, row, column )
         excelSizeShape( objExcelSheet, row, column, XLdoReduceOLEObjects, shapesExported )
         return( true )
      }
   }

   return( false )
}

void excelSetBorders( OleAutoObj objExcelRange ) {
   OleAutoObj objExcelBorders = excelGetBordersOLE( objExcelRange )
   excelSetLineStyleContinuousOLE( OleAutoObj objExcelBorders )
   excelSetWeightThinOLE( OleAutoObj objExcelBorders )
   excelSetColorIndexAutomaticOLE( OleAutoObj objExcelBorders )
}

void excelFormatHeaderRows( OleAutoObj objExcelWorkbook,
                            OleAutoObj objExcelSheet,
                            int rowStart,
                            int colStart,
                            int rowEnd,
                            int columnEnd,
                            bool splitHeaderRows,
                            bool freezePanes,
                            bool exportHeaderRowColor,
                            int headerRowColor
                          )
{
   debug( "Format Header Rows:" )
   OleAutoObj objExcelRange = 
      excelSelectRangeByCoordinatesOLE( objExcelSheet,
                                        rowStart,
                                        colStart,
                                        rowEnd,
                                        columnEnd
                                      )
   excelSetRangeBoldOLE( objExcelRange )
   excelSetRangeHorizontalAlignmentCenterOLE( objExcelRange )
   if ( exportHeaderRowColor ) {
      excelSetRangeColorOLE( objExcelRange,
                             getRGBValue( headerRowColor )
                           )
   }
   excelWrapTextOLE( objExcelRange, true )

   if ( ( splitHeaderRows ) || ( freezePanes ) ) {
      OleAutoObj objExcelWindows = excelGetWindowsOLE( objExcelWorkbook )
      if ( !null objExcelWindows ) {
         OleAutoObj objExcelWindow = excelGetItemOLE( objExcelWindows, xlActiveWindow )
         if ( !null objExcelWindow ) {

            // Split Header Row(s)
            if ( splitHeaderRows ) {
               excelEnableSplitHeaderRowsOLE( objExcelWindow, rowStart )
            }
   
            // Freeze Panes
            if ( freezePanes ) {
               excelEnableFreezePanesOLE( objExcelWindow )
            }
         }
      }
   }
}

void excelFormatDataRows( OleAutoObj objExcelSheet,
                          int rowStart,
                          int colStart,
                          int rowEnd,
                          int columnEnd
                        )
{
   debug( "Format Data Rows:" )
   OleAutoObj objExcelRange = 
      excelSelectRangeByCoordinatesOLE( objExcelSheet,
                                        rowStart,
                                        colStart,
                                        rowEnd,
                                        columnEnd
                                      )
   excelWrapTextOLE( objExcelRange, true )
   excelShrinkToFitOLE( objExcelRange, false )
   excelMergeCellsOLE( objExcelRange, false )
   excelVerticalAlignmentTopOLE( objExcelRange )
   excelOrientationHorizontalOLE( objExcelRange )
}

void excelFormatSheet( OleAutoObj objExcelSheet,
                       int rowStart,
                       int colStart,
                       int rowEnd,
                       int columnEnd,
                       bool addCellBorders,
                       bool setPrintArea
                     )
{
   OleAutoObj objExcelPageSetup = excelGetPageSetupOLE( objExcelSheet )
   if ( !null objExcelPageSetup ) {
      if ( setPrintArea ) {
         excelSetPrintAreaOLE( objExcelPageSetup,
                               createExcelA1StyleAddress( rowStart, colStart, rowEnd, columnEnd )
                             )
         excelPageSetupCenterHorizontally( objExcelPageSetup, true )
      }
   }                  

   if ( addCellBorders ) {
      OleAutoObj objExcelRange = 
         excelSelectRangeByCoordinatesOLE( objExcelSheet,
                                           rowStart,
                                           colStart,
                                           rowEnd,
                                           columnEnd
                                         )
      excelSetBorders( objExcelRange )
   }
}

void excelSetOutlineLevel( OleAutoObj objExcelSheet, Object o, int row, int col, int numRows ) {

   // Determine Outline Level of Object   
   int objLevel = level( o )   
   if ( cell( o ) ) {
      objLevel = objLevel - 2
   }
   int i = 0
   for ( i = row ; i < ( row + numRows ) ; i++ ) {
      excelOutlineLevelOLE( objExcelSheet, i, col, objLevel ) 
   }
}

void excelJustifyColumns( Module m,
                          OleAutoObj objExcelSheet,
                          int rowStart,
                          int rowEnd,
                          int columnStart
                        )
{
   int exportAreaColumn = 0
   Column c
   for c in m do {
      exportAreaColumn++

      OleAutoObj objExcelRange = 
         excelSelectRangeByCoordinatesOLE( objExcelSheet,
                                           rowStart,
                                           exportAreaColumn + columnStart,
                                           rowEnd,
                                           exportAreaColumn + columnStart
                                         )
      Justification columnJustification = justify c
      if ( columnJustification == left ) {
         excelSetRangeHorizontalAlignmentLeftOLE( objExcelRange )
      }
      if ( columnJustification == right ) {
         excelSetRangeHorizontalAlignmentRightOLE( objExcelRange )
      }
      if ( ( columnJustification == center ) || ( columnJustification == centre ) ) {
         excelSetRangeHorizontalAlignmentCenterOLE( objExcelRange )
      }
      if ( columnJustification == full ) {
         excelSetRangeHorizontalAlignmentJustifyOLE( objExcelRange )
      }
   }
}

void excelApplyColorForAttributeValue( Module m,
                                       Object o,
                                       Column c,
                                       OleAutoObj objExcelSheet,
                                       int row,
                                       int column,
                                       bool forceColumnColor
                                     )
{
   AttrDef colorAttrDef =
      getColorColumnAttributeDefinition( m, c, forceColumnColor )
   if ( !null colorAttrDef ) {
      int colorConstant = getColorConstant( o, colorAttrDef )
      if ( colorConstant >= 0 ) {
         OleAutoObj objExcelRange = 
            excelSelectRangeByCoordinatesOLE( objExcelSheet, row, column )
         excelSetRangeColorOLE( objExcelRange,
                                getRGBValue( colorConstant )
                              )
      }
   }
}

void excelSetPageSetup( OleAutoObj objExcelSheet,
                        bool runInteractive,
                        int paperSize,
                        int pageWidth,
                        int pageHeight,
                        int pageSetupOrientation,
                        int leftMargin,
                        int rightMargin,
                        int topMargin,
                        int bottomMargin,
                        string leftHeader,
                        string centerHeader,
                        string rightHeader,
                        string leftFooter,
                        string centerFooter,
                        string rightFooter
                      )
{
   // Note: pageWidth and pageHeight not currently used

   OleAutoObj objExcelPageSetup = excelGetPageSetupOLE( objExcelSheet )

   // Set Page Setup Paper Size
   bool paperSizeSet = false
   if ( paperSize == DOORS_PAPER_SIZE_A4 ) {
      paperSizeSet = excelSetPaperSizeOLE( objExcelPageSetup, xlPaperA4 )
   }
   if ( paperSize == DOORS_PAPER_SIZE_A3 ) {
      paperSizeSet = excelSetPaperSizeOLE( objExcelPageSetup, xlPaperA3 )
   }
   if ( paperSize == DOORS_PAPER_SIZE_A5 ) {
      paperSizeSet = excelSetPaperSizeOLE( objExcelPageSetup, xlPaperA5 )
   }
   if ( paperSize == DOORS_PAPER_SIZE_LEGAL ) {
      paperSizeSet = excelSetPaperSizeOLE( objExcelPageSetup, xlPaperLegal )
   }
   if ( paperSize == DOORS_PAPER_SIZE_LETTER ) {
      paperSizeSet = excelSetPaperSizeOLE( objExcelPageSetup, xlPaperLetter )
   }
   if ( !paperSizeSet ) {
      if ( runInteractive ) warnUnavailablePapeSize( paperSize )
   }
   else {
      // Set Page Setup Orientation
      excelSetPageOrientationOLE( objExcelPageSetup, pageSetupOrientation )
      
      // Set Page Setup Margins
      excelSetLeftMarginOLE( objExcelPageSetup, convertMillimetersToPoints( leftMargin ) )
      excelSetRightMarginOLE( objExcelPageSetup, convertMillimetersToPoints( rightMargin ) )
      excelSetTopMarginOLE( objExcelPageSetup, convertMillimetersToPoints( topMargin ) )
      excelSetBottomMarginOLE( objExcelPageSetup, convertMillimetersToPoints( bottomMargin ) )
   
      // Set Page Headers and Footers
      excelSetLeftHeaderOLE( objExcelPageSetup, leftHeader )
      excelSetCenterHeaderOLE( objExcelPageSetup, centerHeader )
      excelSetRightHeaderOLE( objExcelPageSetup, rightHeader )
      excelSetLeftFooterOLE( objExcelPageSetup, leftFooter )
      excelSetCenterFooterOLE( objExcelPageSetup, centerFooter )
      excelSetRightFooterOLE( objExcelPageSetup, rightFooter )
   }
}

void excelPerformExportForModule( Module XLmoduleToExport,
                                  bool   XLrunInteractive,
                                  bool   XLshowProgressBar,
                                  int    XLcolOffset,
                                  int    XLrowOffset,
                                  bool   XLdoAddCellBorders,
                                  bool   XLdoExportRichText,
                                  bool   XLdoExportTableCells,
                                  bool   XLdoSetPrintArea,
                                  bool   XLdoExportColumnTitles,
                                  bool   XLdoExportAttributeNames,
                                  bool   XLdoEnableExcelAutoFilter,
                                  bool   XLdoSplitHeaderRows,
                                  bool   XLdoFreezePanes,
                                  bool   XLdoHeaderRowsColor,
                                  int    XLheaderRowsColor,
                                  bool   XLdoSetColumnWidths,
                                  bool   XLdoExpandColumnWidths,
                                  bool   XLdoJustifyColumns,
                                  bool   XLdoUseCRsforMulti,
                                  bool   XLdoExportColorColumns,
                                  bool   XLdoForceColumnColor,
                                  bool   XLdoExportHeadingNumber,
                                  bool   XLdoOutline,
                                  bool   XLdoDeletedObjectRowsColor,
                                  int    XLdeletedObjectRowsColor,
                                  bool   XLdoObjectHeadingRowsColor,
                                  int    XLObjectHeadingRowsColor,
                                  bool   XLdoExportOLEObjects,
                                  bool   XLdoReduceOLEObjects,
                                  bool   XLdoExportOLEToSeparateSheet,
                                  bool   XLdoExportPictures,
                                  bool   XLdoPageSetup,
                                  int    XLpageSetupOrientation,
                                  int    XLpaperSize,
                                  int    XLpageWidth,
                                  int    XLpageHeight,
                                  int    XLleftMargin,
                                  int    XLrightMargin,
                                  int    XLtopMargin,
                                  int    XLbottomMargin,
                                  bool   XLdoSetPrintColumnTitleRows,
                                  string XLleftHeader,
                                  string XLcenterHeader,
                                  string XLrightHeader,
                                  string XLleftFooter,
                                  string XLcenterFooter,
                                  string XLrightFooter,
                                  OleAutoObj objExcelApplication,
                                  OleAutoObj objExcelWorkbook,
                                  OleAutoObj objExcelDataExportSheet,
                                  OleAutoObj objExcelOLEExportSheet
                                )
{
   // Count Data Objects to Export
   int numObjectsToExport = 0
   Object o
   for o in XLmoduleToExport do {
      if ( !cell( o ) || ( cell( o ) && XLdoExportTableCells ) ) {
         numObjectsToExport++
      }
   }

   // Initialize Progress Bar
   //if ( XLshowProgressBar ) {
   //   progressStart( excelExportDB, excelMainTitle, progressStartMessage, numObjectsToExport )
   //}

   // Gather Column information
   int exportAreaWidthPixels = 0
   int numberOfDataColumns = 0
   Column c
   for c in XLmoduleToExport do {
      numberOfDataColumns++
      exportAreaWidthPixels += ( width c )
   }
   
   // Track OLE widths
   int largestOLEinColumn[ numberOfDataColumns + 1 ]
   int i
   for ( i = 1 ; i <= numberOfDataColumns ; i++ ) {
      largestOLEinColumn[ i ] = 0
   }

   // Count Header Rows
   int numberHeaderRows = 0
   if ( XLdoExportColumnTitles ) {
      numberHeaderRows++
   }
   if ( XLdoExportAttributeNames ) {
      numberHeaderRows++
   }

   // Activate Data Sheet
   excelActivateSheetOLE( objExcelDataExportSheet )

   // Export Area Row and Column
   int exportAreaRow = 0
   int exportAreaColumn = 0

   // Export Column Header Information (runs 0-2 times)
   int headerRowIndex = 0
   for ( headerRowIndex = 0 ; headerRowIndex < numberHeaderRows ; headerRowIndex++ ) {
        
      exportAreaColumn = 0
      exportAreaRow++

      if ( XLshowProgressBar ) progressMessage( progressHeaderRowMessage )

      for c in XLmoduleToExport do {
         exportAreaColumn++
         excelSetCellContents( objExcelDataExportSheet,
                               exportAreaRow    + XLrowOffset,
                               exportAreaColumn + XLcolOffset,
                               getHeaderCellTextToExport( c,
                                                          XLdoExportAttributeNames,
                                                          XLdoExportColumnTitles,
                                                          numberHeaderRows,
                                                          exportAreaRow
                                                        ),
                               XLdoExportRichText
                             )
      }    
   }

   if ( !excelHasActivePrinter( objExcelApplication ) ) {
      XLdoPageSetup               = false
      XLdoSetColumnWidths         = false
      XLdoSetPrintColumnTitleRows = false
      XLdoSetPrintArea            = false
      if ( XLrunInteractive ) warnNoPrintersInstalled()
   }
    
   // Set Page Setup
   if ( XLdoPageSetup ) {
      excelSetPageSetup( objExcelDataExportSheet, 
                         XLrunInteractive,
                         XLpaperSize,
                         XLpageWidth,
                         XLpageHeight,
                         XLpageSetupOrientation,
                         XLleftMargin,
                         XLrightMargin,
                         XLtopMargin,
                         XLbottomMargin,
                         XLleftHeader,
                         XLcenterHeader,
                         XLrightHeader,
                         XLleftFooter,
                         XLcenterFooter,
                         XLrightFooter
                       )
   }

   // Set Print Column Title Rows
   if ( XLdoSetPrintColumnTitleRows ) {
      excelSetPrintColumnTitleRowsOLE( excelGetPageSetupOLE( objExcelDataExportSheet ),
                                       createExcelA1StyleRowAddress( XLrowOffset + 1,
                                                                     XLrowOffset + numberHeaderRows
                                                                   )
                                     )
   }
   else {
      if ( excelHasActivePrinter( objExcelApplication ) ) {
         excelSetPrintColumnTitleRowsOLE( excelGetPageSetupOLE( objExcelDataExportSheet ), "" )
      }
   }

   // Set Column Widths
   if ( XLdoExpandColumnWidths ) {

      // Set Column Widths on Data Sheet
      int exportAreaColumn = 0
      for c in XLmoduleToExport do {
         exportAreaColumn++
         excelSetColumnWidthPercentOfPageWidth( objExcelDataExportSheet,
                                                exportAreaColumn + XLcolOffset,
                                                realOf( width( c ) ) / realOf( exportAreaWidthPixels )
                                              )
      }
 
      // Adjust Vertical Page Break
      excelAdjustVerticalPageBreakOLE( objExcelWorkbook, objExcelDataExportSheet )

      // Set Column Width on OLE Sheet
      if ( ( XLdoExportOLEObjects ) && ( XLdoExportOLEToSeparateSheet ) ) {
         excelActivateSheetOLE( objExcelOLEExportSheet )
         // The objExcelOLEExportSheet has one working Column
         // which should occupy 100% (1.0) of printable space
         excelSetColumnWidthPercentOfPageWidth( objExcelOLEExportSheet, 1, 1.0 )
         excelActivateSheetOLE( objExcelDataExportSheet )
      }
   }

   // Format Header Row(s)
   if ( numberHeaderRows > 0 ) {
      excelFormatHeaderRows( objExcelWorkbook,
                             objExcelDataExportSheet,
                             XLrowOffset + 1,
                             XLcolOffset + 1,
                             XLrowOffset + numberHeaderRows,
                             XLcolOffset + numberOfDataColumns,
                             XLdoSplitHeaderRows,
                             XLdoFreezePanes,
                             XLdoHeaderRowsColor,
                             XLheaderRowsColor
                           )
   } 

   OleAutoObj objExcelRange = null
   int numObjectsExported = 0
   int numOLEObjectsExported = 0
   Skip shapesExported = createString
   // Export contents (data) of current View
   for o in XLmoduleToExport do {

      if ( !cell( o ) || ( cell( o ) && XLdoExportTableCells ) ) {

         exportAreaRow++
         int exportAreaOLERowsAdded = 0

         if ( XLshowProgressBar ) {
            numObjectsExported++
            progressStep( numObjectsExported )
            progressMessage( progressObjectMessage( numObjectsExported, numObjectsToExport ) )
         }

         exportAreaColumn = 0
         for c in XLmoduleToExport do {
            exportAreaColumn++
            debug( "Exporting Object [ ONum: " number( o ) " ; ANum: " o."Absolute Number" " ] Column [" exportAreaColumn "]" )              
            // Export Text to Cell
            bool exportRichText = XLdoExportRichText
            if ( ( XLdoExportRichText ) &&
                 ( doorsVersion == EXCEL_DOORS_VERSIONS_70SP1 ) &&
                 ( cellTextHasSymbolFont( XLmoduleToExport, o, c ) )
               )
            {
               // Note: A defect in the DOORS 7.0 SP1 Rich Text commands creates problems
               //       exporting Rich Text and Symbol Font, so Rich Text export is disabled
               //       when Symbol Font is encountered (DOORS 7.0 SP1 only)
               exportRichText = false
            }
            string cellTextToExport = getCellTextToExport( XLmoduleToExport,
                                                           o,
                                                           c,
                                                           XLdoExportHeadingNumber,
                                                           exportRichText,
                                                           XLdoUseCRsforMulti
                                                         )
            excelSetCellContents( objExcelDataExportSheet,
                                  exportAreaRow    + XLrowOffset, 
                                  exportAreaColumn + XLcolOffset,
                                  cellTextToExport,
                                  exportRichText
                                )   

            // If no Cell Text was exported, the existing row
            // can be used for the first OLE Object Exported
            // New row(s) will be added for each additional OLE Object Exported (D6D7)
            int exportAreaOLERowOffset = 0
            if ( !cellTextIsNull( o, c ) ) exportAreaOLERowOffset++

            // Export Picture from Main Column
            if ( XLdoExportPictures ) {
               if ( main( c ) && showPictures( XLmoduleToExport ) ) {
                  excelExportPicture( o,
                                      objExcelDataExportSheet,
                                      exportAreaRow + XLrowOffset + exportAreaOLERowOffset,
                                      exportAreaColumn + XLcolOffset,
                                      exportAreaOLERowsAdded,
                                      numOLEObjectsExported,
                                      shapesExported
                                    )
               }
            }

            // Export OLE Objects (or markers)
            excelExportOLEObjects( objExcelDataExportSheet,
                                   objExcelOLEExportSheet,
                                   XLmoduleToExport,
                                   o,
                                   c,
                                   exportAreaRow    + XLrowOffset,
                                   exportAreaColumn + XLcolOffset,
                                   XLdoExportOLEObjects,
                                   XLdoExportOLEToSeparateSheet,
                                   exportAreaOLERowOffset,
                                   exportAreaOLERowsAdded,
                                   numOLEObjectsExported,
                                   shapesExported,
                                   largestOLEinColumn
                                 )
                
            // Export Column Color
            if ( XLdoExportColorColumns ) {
               excelApplyColorForAttributeValue( XLmoduleToExport,
                                                 o,
                                                 c,
                                                 objExcelDataExportSheet,
                                                 exportAreaRow    + XLrowOffset,
                                                 exportAreaColumn + XLcolOffset,
                                                 XLdoForceColumnColor
                                               )
            }
                         
            // After last Cell is Exported, apply formatting to Row based on Object Heading Level
            if ( exportAreaColumn == numberOfDataColumns ) {

               // Set Outline Level for Object ( and any extra OLE Rows added )
               if ( XLdoOutline ) {
                  excelSetOutlineLevel( objExcelDataExportSheet,
                                        o,
                                        exportAreaRow + XLrowOffset,
                                        exportAreaColumn + XLcolOffset,
                                        1 + exportAreaOLERowsAdded
                                      )
               }
   
               // Apply Heading Row Color
               if ( XLdoObjectHeadingRowsColor && isHeadingRow( o ) ) {
                  objExcelRange = 
                     excelSelectRangeByCoordinatesOLE( objExcelDataExportSheet,
                                                       exportAreaRow + XLrowOffset,
                                                       XLcolOffset   + 1,
                                                       exportAreaRow + XLrowOffset,
                                                       XLcolOffset   + numberOfDataColumns
                                                     )
                  excelSetRangeColorOLE( objExcelRange,
                                         getRGBValue( XLObjectHeadingRowsColor )
                                       )
               }
               
               // Apply Deleted Objects Row Color 
               if ( XLdoDeletedObjectRowsColor && isDeleted( o ) ) {
                  objExcelRange = 
                     excelSelectRangeByCoordinatesOLE( objExcelDataExportSheet,
                                                       exportAreaRow + XLrowOffset,
                                                       XLcolOffset   + 1,
                                                       exportAreaRow + XLrowOffset,
                                                       XLcolOffset   + numberOfDataColumns
                                                     )
                  excelSetRangeColorOLE( objExcelRange,
                                         getRGBValue( XLdeletedObjectRowsColor )
                                       )
               }
            }
         }
         exportAreaRow += exportAreaOLERowsAdded
      }
      if ( XLshowProgressBar ) {
         if ( progressCancelled ) break
      }
   }

   if ( XLshowProgressBar ) progressMessage( progressFormattingDataMessage )

   if ( numObjectsToExport > 0 ) {
 
      // Format Data Columns
      if ( XLdoJustifyColumns ) {
         excelJustifyColumns( XLmoduleToExport,
                              objExcelDataExportSheet,
                              numberHeaderRows + XLrowOffset + 1,
                              exportAreaRow    + XLrowOffset,
                              XLcolOffset
                            )
      }

      // Format Data Rows
      excelFormatDataRows( objExcelDataExportSheet,
                           numberHeaderRows    + XLrowOffset + 1,
                           XLcolOffset         + 1,
                           exportAreaRow       + XLrowOffset,
                           numberOfDataColumns + XLcolOffset
                         )

      // Format Sheet
      excelFormatSheet( objExcelDataExportSheet,
                        XLrowOffset + 1,
                        XLcolOffset + 1,
                        exportAreaRow       + XLrowOffset,
                        numberOfDataColumns + XLcolOffset,
                        XLdoAddCellBorders,
                        XLdoSetPrintArea
                      )

      // Set Column Widths
      if ( XLdoSetColumnWidths ) {
         excelSetColumnWidths( objExcelDataExportSheet,
                               numberOfDataColumns,
                               largestOLEinColumn
                             )
      }

   }

   // Enable Excel AutoFilter (this must be done after all data is exported)
   if ( XLdoEnableExcelAutoFilter ) {
      // Set Range to Top Left Cell of Export Area
      objExcelRange = 
         excelSelectRangeByCoordinatesOLE( objExcelDataExportSheet, ( XLrowOffset + 1 ), ( XLcolOffset + 1 ) )
      excelEnableExcelAutoFilterOLE( objExcelDataExportSheet, objExcelRange )
   }

   if ( XLshowProgressBar ) progressStop
}

// ------------------------ DB Functions  ----------------------

bool validateFileName( string userFileName ) {

   // This function only validates that the supplied file name
   //     represents a file which exists. It does not validate whether
   //     the file is a Microsoft Excel compatible file.

   bool returnValue = true

   if ( null userFileName ) {
       warningBox warnInvalidFileNameBlankErrorMessage
       returnValue = false
   }
   else {
      if ( !fileExists_ userFileName ) {
         warningBox warnInvalidFileNameDoesNotExistErrorMessage
         returnValue = false
      }
   }
   return( returnValue )
}

bool validatePageDimension( string pageDimStr,
                            int lowerLim,
                            int upperLim,
                            string dimensionName,
                            int &pageDimInt
                          )
{
   if ( !isInteger( pageDimStr ) ) {
      warnInvalidPageDimension( pageDimStr, dimensionName )
      return( false )
   }
   else {
      pageDimInt = getInteger( pageDimStr )
      if ( ( pageDimInt < lowerLim ) || ( pageDimInt > upperLim ) ) {
         warnInvalidPageDimension( pageDimStr, dimensionName, lowerLim, upperLim )
         return( false )
      }
   }
   return( true )
}

bool validateExportToExcelDialogBox( DB xx ) {

   bool returnValue = true

   // Validate Data Sheet Identifier
   if ( !parseAndValidateExcelSheetIdentifier( get( dataSheetFieldDBE ), XLdataExportSheetNumber ) ) {
      warningBox warnInvalidExcelSheetIdentifierErrorMessage
      returnValue = false
   }

   // Validate OLE Sheet Identifier
   XLdoExportOLEObjects = get( OLEexportDBE )
   if ( XLdoExportOLEObjects ) {
      XLdoExportOLEToSeparateSheet = get( OLESeparateSheetDBE )
      if ( parseAndValidateExcelSheetIdentifier( get( OLESheetFieldDBE ), XLOLEExportSheetNumber ) ) {
         if ( XLdataExportSheetNumber == XLOLEExportSheetNumber ) {
            warningBox warnInvalidExcelDataAndOLESheetErrorMessage
            returnValue = false
        }
      }
      else {
         returnValue = false
      }
   }
   else {
      XLdoExportOLEToSeparateSheet = false
   }

   // Validate Start Cell
   if ( get( startCellDBE ) == 1 ) {
      string userCellOffset = get( startCellFieldDBE )
      string colCoordinateStr
      int rowCoordinateInt
      if ( parseAndValidateExcelCellIdentifier( userCellOffset , colCoordinateStr , rowCoordinateInt ) ) {
         XLcolOffset = getExcelColumnIntFromAlphaAddress( colCoordinateStr ) - 1
         XLrowOffset = ( rowCoordinateInt ) - 1
      }
      else {
         warningBox warnInvalidExcelIdentifierErrorMessage
         returnValue = false
      }
   }

   // Validate Template File Name
   if ( get( exportFileChoiceDBE ) == 1 ) {
      string userFileName = get( exportFileSelectDBE )
      if ( validateFileName( userFileName ) ) {
         XLexistingFileName = userFileName
         XLdoUseExistingExcelFile = true
      }
      else { 
         returnValue = false
      }
   }
   else {
      XLdoUseExistingExcelFile = false
   }

   XLpaperSize = ( get paperSizeDBE )
   // Validate Paper Size Dimensions   
   if ( XLpaperSize == DOORS_PAPER_SIZE_CUSTOM ) {
      debug( "Custom Paper Size Selected" )
      if ( !validatePageDimension( get( paperWidthDBE ),
                                        DOORS_SMALL_PAPER_MM,
                                        DOORS_BIG_PAPER_MM,
                                        pageWidthDimensionName,
                                        XLpageWidth
                                 )
         )
      {
         returnValue = false
      }

      if ( !validatePageDimension( get( paperHeightDBE ),
                                   DOORS_SMALL_PAPER_MM,
                                   DOORS_BIG_PAPER_MM,
                                   pageHeightDimensionName,
                                   XLpageHeight
                                 )
         )
      {
         returnValue = false
      }
   }

   // Validate Margin Dimensions
   if ( !validatePageDimension( get( marginLeftDBE ),
                                DOORS_SMALL_MARGIN_MM,
                                DOORS_BIG_MARGIN_MM,
                                leftMarginDimensionName,
                                XLleftMargin
                              )
      )
   {
      returnValue = false
   }
   if ( !validatePageDimension( get( marginRightDBE ),
                                DOORS_SMALL_MARGIN_MM,
                                DOORS_BIG_MARGIN_MM,
                                rightMarginDimensionName,
                                XLrightMargin
                              )
      )
   {
      returnValue = false
   }
   if ( !validatePageDimension( get( marginTopDBE ),
                                DOORS_SMALL_MARGIN_MM,
                                DOORS_BIG_MARGIN_MM,
                                topMarginDimensionName,
                                XLtopMargin
                              )
      )
   {
      returnValue = false
   }
   if ( !validatePageDimension( get( marginBottomDBE ),
                                DOORS_SMALL_MARGIN_MM,
                                DOORS_BIG_MARGIN_MM,
                                bottomMarginDimensionName,
                                XLbottomMargin
                              )
      )
   {
      returnValue = false
   }

   if ( returnValue ) {
      // These DB settings require no Validation

      // Sheet
      XLdoAddCellBorders               = get( addCellBordersDBE )
      XLdoExportRichText               = get( exportRichTextDBE )
      XLdoExportTableCells             = get( exportTableCellsDBE )
      XLdoSetPrintArea                 = get( setPrintAreaDBE )

      // Header 
      XLdoExportColumnTitles           = get( exportColumnTitlesDBE )
      XLdoExportAttributeNames         = get( exportAttributeNamesDBE )
      XLdoEnableExcelAutoFilter        = get( activateAutoFilterDBE )
      XLdoSplitHeaderRows              = get( splitHeaderRowsDBE )
      XLdoFreezePanes                  = get( freezePanesDBE )
      XLdoHeaderRowsColor              = get( useHeaderRowColorDBE )
      XLheaderRowsColor                = get( headerRowsColorListDBE )

      // Columns
      XLdoSetColumnWidths              = get( setColumnWidthsDBE )
      XLdoExpandColumnWidths           = get( expandColumnWidthsDBE )
      XLdoJustifyColumns               = get( justifyColumnsDBE )
      XLdoUseCRsforMulti               = get( useCRsforMultiDBE )
      XLdoExportColorColumns           = get( useColumnColorDBE )
      XLdoForceColumnColor             = get( forceColumnColorDBE )

      // Rows
      XLdoExportHeadingNumber          = get( exportHeadingNumberDBE )
      XLdoOutline                      = get( setOutlineDBE )
      XLdoDeletedObjectRowsColor       = get( useDeletedObjectRowsColorDBE )
      XLdoObjectHeadingRowsColor       = get( useObjectHeadingColorRowsDBE )
      XLObjectHeadingRowsColor         = get( objectHeadingColorListRowsDBE )

      // OLE
      XLdoReduceOLEObjects             = get( OLEreduceDBE ) 
      XLdoExportPictures               = get( PictureExportDBE )
 
      // Page Setup
      XLdoPageSetup                    = get( usePageSetupDBE )
      XLpageSetupOrientation           = get( pageOrientationDBE )
      XLleftHeader   = pageExpandHF( get( bodyLeftHeaderDBE ), "&P", "&N" )
      XLcenterHeader = pageExpandHF( get( bodyCenterHeaderDBE ), "&P", "&N" )
      XLrightHeader  = pageExpandHF( get( bodyRightHeaderDBE ), "&P", "&N" )
      XLleftFooter   = pageExpandHF( get( bodyLeftFooterDBE ), "&P", "&N" )
      XLcenterFooter = pageExpandHF( get( bodyCenterFooterDBE ), "&P", "&N" )
      XLrightFooter  = pageExpandHF( get( bodyRightFooterDBE ), "&P", "&N" )
   }
   
   if ( XLdoPageSetup ) {
      XLdoSetPrintColumnTitleRows = ( get( pagePrintColumnTitleRowsDBE ) == 0 )
   }
   else {
      // If no Page Setup is exported, force the setting of Print Column Title Rows
      XLdoSetPrintColumnTitleRows = true
   }

   return( returnValue )
}

bool checkDOORSVersion() {
   int v = getDOORSmajorVersion()
   if ( ( v >= EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_CURRENT ) && ( v <= EXCEL_NEWEST_MAJOR_VERSION_TESTED ) ) {
      return( true )
   }
   else {
      if ( ( v >= EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_ALL ) && ( v < EXCEL_OLDEST_MAJOR_VERSION_SUPPORTED_CURRENT ) ) {
         warnIncorrectDOORSVersion()
         return( false )
      }
   }
   return( true )
}

void exportPreChecks( Module m ) {
   if ( !checkDOORSVersion() ) {
      halt
   }
   if ( platform != "WIN32" ) {
      warnIncorrectPlatform( cExcelApplicationName )
      halt
   }
   if ( null m ) {
      warningBox warnRunWithinFormalModule
      halt
   }
   if ( !canRead( m, DOORS_OBJECT_HEADING ) ) {
      warnCannotReadAttribute( DOORS_OBJECT_HEADING )
      halt
   }
   if ( !canRead( m, DOORS_OBJECT_TEXT ) ) {
      warnCannotReadAttribute( DOORS_OBJECT_TEXT )
      halt
   }
}

void closeDB( DB db ) {

   delete excelTrigger
   release excelExportDB
}

void exportModuleToExcel( Module modRef ) {

   //if ( !validateExportToExcelDialogBox excelExportDB ) return()

   //showSelectedTab( excelExportTabSkip, EXCEL_EXPORT_ABOUT_TAB )

   OleAutoObj objExcelApplication = null
   OleAutoObj objExcelWorkbook = null
   OleAutoObj objExcelDataExportSheet = null
   OleAutoObj objExcelOLEExportSheet = null
   if ( initExcelOLE( true,
                      XLdoExportOLEObjects,
                      XLdataExportSheetNumber,
                      XLdoExportOLEToSeparateSheet,
                      XLOLEExportSheetNumber,
                      XLexistingFileName,
                      XLdoUseExistingExcelFile,
                      objExcelApplication,
                      objExcelWorkbook,
                      objExcelDataExportSheet,
                      objExcelOLEExportSheet
                    )
      )
   {
      excelPerformExportForModule( modRef,
                                   true,
                                   true,
                                   XLcolOffset,
                                   XLrowOffset,
                                   XLdoAddCellBorders,
                                   XLdoExportRichText,
                                   XLdoExportTableCells,
                                   XLdoSetPrintArea,
                                   XLdoExportColumnTitles,
                                   XLdoExportAttributeNames,
                                   XLdoEnableExcelAutoFilter,
                                   XLdoSplitHeaderRows,
                                   XLdoFreezePanes,
                                   XLdoHeaderRowsColor,
                                   XLheaderRowsColor,
                                   XLdoSetColumnWidths,
                                   XLdoExpandColumnWidths,
                                   XLdoJustifyColumns,
                                   XLdoUseCRsforMulti,
                                   XLdoExportColorColumns,
                                   XLdoForceColumnColor,
                                   XLdoExportHeadingNumber,
                                   XLdoOutline,
                                   XLdoDeletedObjectRowsColor,
                                   XLdeletedObjectRowsColor,
                                   XLdoObjectHeadingRowsColor,
                                   XLObjectHeadingRowsColor,
                                   XLdoExportOLEObjects,
                                   XLdoReduceOLEObjects,
                                   XLdoExportOLEToSeparateSheet,
                                   XLdoExportPictures,
                                   XLdoPageSetup,
                                   XLpageSetupOrientation,
                                   XLpaperSize,
                                   XLpageWidth,
                                   XLpageHeight,
                                   XLleftMargin,
                                   XLrightMargin,
                                   XLtopMargin,
                                   XLbottomMargin,
                                   XLdoSetPrintColumnTitleRows,
                                   XLleftHeader,
                                   XLcenterHeader,
                                   XLrightHeader,
                                   XLleftFooter,
                                   XLcenterFooter,
                                   XLrightFooter,
                                   objExcelApplication,
                                   objExcelWorkbook,
                                   objExcelDataExportSheet,
                                   objExcelOLEExportSheet
                                 )
   }
   //excelDoneOLE( objExcelApplication )
   //infoBox excelMainTitle " complete."
	string strModName = name(modRef)
	if (length (strModName) > 31)
		strModName = strModName [0:30]
	oleGet (objExcelWorkbook, "ActiveSheet", objExcelDataExportSheet);
	olePut (objExcelDataExportSheet, "Name", strModName);
	string strModuleName =  str_path "" strModuleName1 "_" name(modRef) ".xlsx"
	clear(objArgBlock)
	put(objArgBlock, "FileName", strModuleName)
	oleMethod(objExcelWorkbook, cExcelMethodSave, objArgBlock)
	oleMethod(objExcelWorkbook,"Close",objArgBlock)
	oleMethod(objExcelApplication,"Quit")
	excelDoneOLE( objExcelApplication )

   //release excelExportDB
   //closeDB excelExportDB
}

bool closeModuleTrig( Trigger t ) {

   release excelExportDB
   return( true )
}

void onOLEseparateSheetToggle( DBE xx ) {

   if ( get( OLESeparateSheetDBE ) ) {
      active OLESheetFieldDBE
   }
   else {
      inactive OLESheetFieldDBE
   }
}

void onStartCellUpdate( DBE xx ) {

   if ( get( startCellDBE ) == 0 ) {
      inactive startCellFieldDBE
 
   }
   else {
      active startCellFieldDBE
   }
}

void onOLEexportToggle( DBE xx ) {

   if ( get( OLEexportDBE ) ) {
      active expandColumnWidthsDBE
      active OLEreduceDBE
      active OLESeparateSheetDBE
      onOLEseparateSheetToggle( OLESeparateSheetDBE )
   }
   else {
      inactive expandColumnWidthsDBE
      inactive OLEreduceDBE
      inactive OLESeparateSheetDBE
      inactive OLESheetFieldDBE
   }
}

void onUseColumnColorToggle( DBE xx ) {
   if ( get useColumnColorDBE ) {
      active forceColumnColorDBE
   }
   else {
      inactive forceColumnColorDBE
   }
}

void onObjectHeadingColorToggleRows( DBE xx ) {
   if ( get useObjectHeadingColorRowsDBE ) {
      active objectHeadingColorListRowsDBE
   }
   else {
      inactive objectHeadingColorListRowsDBE
   }
}

void onHeaderRowsColorToggle( DBE xx ) {
   if ( get useHeaderRowColorDBE ) {
      active headerRowsColorListDBE
   }
   else {
      inactive headerRowsColorListDBE
   }
}

void fileRadioUpdate( DBE xx ) {

   if ( get( exportFileChoiceDBE ) == 0 ) {
      inactive exportFileLabelDBE
      inactive exportFileSelectDBE
   }
   else {
      active exportFileLabelDBE
      active exportFileSelectDBE
   }
}

void onUsePageSetupToggle( DBE xx ) {
   if ( get usePageSetupDBE ) {
      active pageSetupFrameDBE
      active savedPageLayoutListDBE
      active paperSizeDBE
      active pageOrientationDBE
      active paperWidthDBE
      active paperHeightDBE
      active paperWidth_RO_DBE
      active paperHeight_RO_DBE
      active pagePrintColumnTitleRowsDBE
      active marginsDBE
      active marginLeftDBE
      active marginRightDBE
      active marginTopDBE
      active marginBottomDBE
      active printOptionLabel0DBE
      active printOptionLabel1DBE
      active printOptionLabel2DBE
      active printOptionLabel3DBE
      active printOptionLabel4DBE
      active printOptionLabel5DBE
      active printOptionLabel6DBE
      active printOptionLabel7DBE
      active printOptionLabel8DBE
      active printOptionLabel9DBE
      active printOptionLabel10DBE
      active printOptionLabel11DBE
      active bodyLeftHeaderDBE
      active bodyCenterHeaderDBE
      active bodyRightHeaderDBE
      active bodyLeftFooterDBE
      active bodyCenterFooterDBE
      active bodyRightFooterDBE
   }
   else {
      inactive pageSetupFrameDBE
      inactive savedPageLayoutListDBE
      inactive paperSizeDBE
      inactive pageOrientationDBE
      inactive paperWidthDBE
      inactive paperHeightDBE
      inactive paperWidth_RO_DBE
      inactive paperHeight_RO_DBE
      inactive pagePrintColumnTitleRowsDBE
      inactive marginsDBE
      inactive marginLeftDBE
      inactive marginRightDBE
      inactive marginTopDBE
      inactive marginBottomDBE
      inactive printOptionLabel0DBE
      inactive printOptionLabel1DBE
      inactive printOptionLabel2DBE
      inactive printOptionLabel3DBE
      inactive printOptionLabel4DBE
      inactive printOptionLabel5DBE
      inactive printOptionLabel6DBE
      inactive printOptionLabel7DBE
      inactive printOptionLabel8DBE
      inactive printOptionLabel9DBE
      inactive printOptionLabel10DBE
      inactive printOptionLabel11DBE
      inactive bodyLeftHeaderDBE
      inactive bodyCenterHeaderDBE
      inactive bodyRightHeaderDBE
      inactive bodyLeftFooterDBE
      inactive bodyCenterFooterDBE
      inactive bodyRightFooterDBE
   }
}

void onPaperSizeToggle( DBE )
{ 
   int paperSizeSelected = get( paperSizeDBE )
   int orientation = get( pageOrientationDBE )

   if ( paperSizeSelected == DOORS_PAPER_SIZE_CUSTOM ) {
      debug( "Custom Paper Size Toggled" )
      string w = get( paperWidth_RO_DBE )
      string h = get( paperHeight_RO_DBE )

      if ( orientation == DOORS_PAGE_ORIENTATION_PORTRAIT ) {
         set( paperWidthDBE, w ) 
         set( paperHeightDBE, h )
      }
      else {
         set( paperWidthDBE, h )
         set( paperHeightDBE, w )
      }

      show paperWidthDBE
      show paperHeightDBE
      hide paperWidth_RO_DBE
      hide paperHeight_RO_DBE
    }
    else {
      if ( orientation == DOORS_PAGE_ORIENTATION_PORTRAIT ) {
         set( paperWidth_RO_DBE, DOORS_PAGE_SETUP_PAPER_WIDTHS_MM[ paperSizeSelected ] )
         set( paperHeight_RO_DBE, DOORS_PAGE_SETUP_PAPER_HEIGHTS_MM[ paperSizeSelected ] )
      }
      else {
         set( paperWidth_RO_DBE, DOORS_PAGE_SETUP_PAPER_HEIGHTS_MM[ paperSizeSelected ] )
         set( paperHeight_RO_DBE, DOORS_PAGE_SETUP_PAPER_WIDTHS_MM[ paperSizeSelected ] )
      }
      show paperWidth_RO_DBE
      show paperHeight_RO_DBE
      hide paperWidthDBE
      hide paperHeightDBE
   }
}

void onOrientationToggle( DBE )  { 

   string w, h

   if ( get( paperSizeDBE ) == DOORS_PAPER_SIZE_CUSTOM ) {
      w = get( paperWidthDBE )
      h = get( paperHeightDBE )
      set( paperWidthDBE, h )
      set( paperHeightDBE, w )
   }
   else {
      w = get( paperWidth_RO_DBE )
      h = get( paperHeight_RO_DBE )
      set( paperWidth_RO_DBE, h )
      set( paperHeight_RO_DBE, w )
   }	
}

void setPageLayoutUISettingsFromSavedPageLayout( string pageLayoutName ) {

   PageLayout pl = pageLayout pageLayoutName
   if ( !null pl ) {

      onUsePageSetupToggle( usePageSetupDBE )

      set( paperWidthDBE, pageWidth( pl ) "" )
      set( paperHeightDBE, pageHeight( pl ) "" )

      set( paperSizeDBE, pageSize( pl ) )
      onPaperSizeToggle( paperSizeDBE )
      if ( pagePortrait( pl ) ) {
         set( pageOrientationDBE, DOORS_PAGE_ORIENTATION_PORTRAIT )
      }
      else {
         set( pageOrientationDBE, DOORS_PAGE_ORIENTATION_LANDSCAPE )
      }
      onOrientationToggle( pageOrientationDBE )
      set( marginLeftDBE,   pageLeftMargin( pl ) "" )
      set( marginRightDBE,  pageRightMargin( pl ) "" )
      set( marginTopDBE,    pageTopMargin( pl ) "" )
      set( marginBottomDBE, pageBottomMargin( pl ) "" )

      if ( pageRepeatTitles( pl ) ) {
         set( pagePrintColumnTitleRowsDBE, 0 )
      }
      else {
         set( pagePrintColumnTitleRowsDBE, 1 )
      }
  
      set( bodyLeftHeaderDBE,   stripRichText( pageHeaderFooter( pl, 0 ) "" ) )
      set( bodyCenterHeaderDBE, stripRichText( pageHeaderFooter( pl, 1 ) "" ) )
      set( bodyRightHeaderDBE,  stripRichText( pageHeaderFooter( pl, 2 ) "" ) )
      set( bodyLeftFooterDBE,   stripRichText( pageHeaderFooter( pl, 3 ) "" ) )
      set( bodyCenterFooterDBE, stripRichText( pageHeaderFooter( pl, 4 ) "" ) )
      set( bodyRightFooterDBE,  stripRichText( pageHeaderFooter( pl, 5 ) "" ) )
   }
}

void onPageSetupSelect( DBE x, int i ) {
   setPageLayoutUISettingsFromSavedPageLayout( get savedPageLayoutListDBE )
}

void onPageSetupDeselect( DBE x, int i ) {}

void onPageSetupActivate( DBE x, int i ) {
   setPageLayoutUISettingsFromSavedPageLayout( get savedPageLayoutListDBE )
}

void updatePageLayouts( void ) {

   string pageLayoutName
   int pageLayoutCount = 0

   PageLayout pl = pageLayout( DOORS_STANDARD_LAYOUT )
   if ( !null pl ) {
      insert( savedPageLayoutListDBE, pageLayoutCount, DOORS_STANDARD_LAYOUT, iconNone )
      if ( XLdefaultLayout == DOORS_STANDARD_LAYOUT ) set( savedPageLayoutListDBE, pageLayoutCount )
      pageLayoutCount++
   }

   for pageLayoutName in pageSetups( database ) do {
      if ( pageLayoutName != DOORS_STANDARD_LAYOUT ) {
         insert( savedPageLayoutListDBE, pageLayoutCount++, pageLayoutName, iconNone )
         if ( pageLayoutName == XLdefaultLayout ) set( savedPageLayoutListDBE, pageLayoutCount )
         pageLayoutCount++
      }
   }   
}

void updateColorList( DBE colorListDBE, int defaultColor ) {
   int i
   for ( i = DOORS_MIN_REAL_COLOR ; i <= DOORS_MAX_REAL_COLOR ; i++ ) {
      insert( colorListDBE, i, getRealColorName( i ), getRealColorIcon( i ) )
   }
   set( colorListDBE, defaultColor )
}

void excelExportTabSelectFn( DBE theTab ) {
   int selection = get( theTab )
   //showSelectedTab( excelExportTabSkip, selection ) 
   if ( selection == EXCEL_EXPORT_PAGE_SETUP_TAB ) {
      onPaperSizeToggle( paperSizeDBE )
   }
}

void doExcelExportSheetTab( DB excelExportDB,
                            DBE excelExportTabDBE,
                            Skip copyViewsSheetTabSkip
                          )
{
   dataSheetFieldDBE = field( excelExportDB,
                              dataSheetFieldLabel,
                              XLdataExportSheetNumber "",
                              3
                            )

   startCellFieldDBE = field( excelExportDB, startCellFieldLabel, EXCEL_FIRST_CELL_IDENTIFIER, 10 )

   startCellDBE = verticalRadioBox( excelExportDB, "", startCellOptions, 0 )
   set( startCellDBE, onStartCellUpdate )
  

   inactive startCellFieldDBE

   addCellBordersDBE = toggle( excelExportDB,
                               addCellBordersLabel, 
                               XLdoAddCellBorders
                             )

   exportRichTextDBE = toggle( excelExportDB,
                               exportRichTextLabel,
                               XLdoExportRichText
                             )

   exportTableCellsDBE = toggle( excelExportDB,
                                 exportTableCellsLabel,
                                 XLdoExportTableCells
                               )

   setPrintAreaDBE = toggle( excelExportDB,
                             setPrintAreaLabel,
                             XLdoSetPrintArea
                           )

   dataSheetFieldDBE->"top"->"inside"->excelExportTabDBE
   dataSheetFieldDBE->"left"->"inside"->excelExportTabDBE
   startCellFieldDBE->"left"->"inside"->excelExportTabDBE
   startCellDBE->"left"->"aligned"->startCellFieldDBE

   int index = 0

   put( excelExportSheetTabSkip, index++, dataSheetFieldDBE )
   put( excelExportSheetTabSkip, index++, startCellFieldDBE )
   put( excelExportSheetTabSkip, index++, startCellDBE )
   put( excelExportSheetTabSkip, index++, addCellBordersDBE )
   put( excelExportSheetTabSkip, index++, exportRichTextDBE )
   put( excelExportSheetTabSkip, index++, exportTableCellsDBE )
   put( excelExportSheetTabSkip, index++, setPrintAreaDBE )
}

void doExcelExportHeaderTab( DB excelExportDB,
                             DBE excelExportTabDBE,
                             Skip copyViewsHeaderTabSkip
                           )
{
   exportColumnTitlesDBE = toggle( excelExportDB, exportColumnTitlesLabel, XLdoExportColumnTitles )

   exportAttributeNamesDBE = toggle( excelExportDB, exportAttributeNamesLabel, XLdoExportAttributeNames )
   
   separator excelExportDB
   
   activateAutoFilterDBE = toggle( excelExportDB, activateAutoFilterLabel, XLdoEnableExcelAutoFilter )

   splitHeaderRowsDBE = toggle( excelExportDB, splitHeaderRowsLabel, XLdoSplitHeaderRows )

   freezePanesDBE = toggle( excelExportDB, freezePanesLabel, XLdoFreezePanes )

   headerRowsColorFrameDBE = frame( excelExportDB, headerRowsColorFrameLabel, 50, 50 )

   useHeaderRowColorDBE = toggle( excelExportDB, useHederRowColorLabel, XLdoHeaderRowsColor )
   set( useHeaderRowColorDBE, onHeaderRowsColorToggle )

   string dummyList[] = {}
   headerRowsColorListDBE = listView( excelExportDB, 0, 200, 10, dummyList )

   exportColumnTitlesDBE->"top"->"inside"->excelExportTabDBE
   exportColumnTitlesDBE->"left"->"form"
   exportColumnTitlesDBE->"bottom"->"unattached"

   headerRowsColorFrameDBE->"top"->"inside"->excelExportTabDBE
   headerRowsColorFrameDBE->"left"->"spaced"->exportColumnTitlesDBE
   headerRowsColorFrameDBE->"right"->"unattached"

   useHeaderRowColorDBE->"top"->"inside"->headerRowsColorFrameDBE
   useHeaderRowColorDBE->"left"->"inside"->headerRowsColorFrameDBE

   headerRowsColorListDBE->"top"->"spaced"->useHeaderRowColorDBE
   headerRowsColorListDBE->"left"->"inside"->headerRowsColorFrameDBE
   headerRowsColorListDBE->"bottom"->"inside"->headerRowsColorFrameDBE
   headerRowsColorListDBE->"right"->"inside"->headerRowsColorFrameDBE

   int index = 0
   put( excelExportHeaderTabSkip, index++, exportColumnTitlesDBE )
   put( excelExportHeaderTabSkip, index++, exportAttributeNamesDBE )
   put( excelExportHeaderTabSkip, index++, activateAutoFilterDBE )
   put( excelExportHeaderTabSkip, index++, splitHeaderRowsDBE )
   put( excelExportHeaderTabSkip, index++, freezePanesDBE )
   put( excelExportHeaderTabSkip, index++, headerRowsColorFrameDBE )
   put( excelExportHeaderTabSkip, index++, useHeaderRowColorDBE )
   put( excelExportHeaderTabSkip, index++, headerRowsColorListDBE )
}

void doExcelExportColumnsTab( DB excelExportDB,
                              DBE excelExportTabDBE,
                              Skip copyViewsColumnsTabSkip
                            )
{
   setColumnWidthsDBE = toggle( excelExportDB,
                                setColumnWidthsLabel,
                                XLdoSetColumnWidths
                              )

   expandColumnWidthsDBE = toggle( excelExportDB,
                                   expandColumnWidthsLabel,
                                   XLdoExpandColumnWidths
                                 )

   justifyColumnsDBE = toggle( excelExportDB,
                               justifyColumnsLabel,
                               XLdoSetColumnWidths
                             )

   useCRsforMultiDBE = toggle( excelExportDB,
                               useLFsforMultiValuedEnums,
                               XLdoUseCRsforMulti
                             )

   useColumnColorDBE = toggle( excelExportDB,
                               useColumnColorLabel,
                               XLdoExportColorColumns
                             )
   set( useColumnColorDBE, onUseColumnColorToggle )

   forceColumnColorDBE = toggle( excelExportDB,
                                 forceColumnColorLabel,
                                 XLdoForceColumnColor
                               )

   setColumnWidthsDBE->"top"->"inside"->excelExportTabDBE
   setColumnWidthsDBE->"left"->"inside"->excelExportTabDBE
   setColumnWidthsDBE->"bottom"->"unattached"

   expandColumnWidthsDBE->"left"->"form"
   justifyColumnsDBE->"left"->"form"
   useCRsforMultiDBE->"left"->"form"
   useColumnColorDBE->"left"->"form"
   forceColumnColorDBE->"left"->"form"

   int index = 0
   put( excelExportColumnsTabSkip, index++, setColumnWidthsDBE )
   put( excelExportColumnsTabSkip, index++, expandColumnWidthsDBE )
   put( excelExportColumnsTabSkip, index++, justifyColumnsDBE )
   put( excelExportColumnsTabSkip, index++, useCRsforMultiDBE )
   put( excelExportColumnsTabSkip, index++, useColumnColorDBE )
   put( excelExportColumnsTabSkip, index++, forceColumnColorDBE )
}

void doExcelExportRowsTab( DB excelExportDB,
                           DBE excelExportTabDBE,
                           Skip copyViewsRowTabSkip
                         )
{
   exportHeadingNumberDBE = toggle( excelExportDB, exportHeadingNumberLabel, XLdoExportHeadingNumber )

   setOutlineDBE = toggle( excelExportDB, setOutlineLabel, XLdoOutline )

   useDeletedObjectRowsColorDBE = toggle( excelExportDB,
                                          useDeletedObjectRowsColorLabel,
                                          XLdoDeletedObjectRowsColor
                                        )

   objectHeadingColorFrameRowsDBE = frame( excelExportDB, objectHeadingColorFrameRowsLabel, 50, 50 )

   useObjectHeadingColorRowsDBE = toggle( excelExportDB,
                                          useObjectHeadingColorRowsLabel,
                                          XLdoObjectHeadingRowsColor
                                        )
   set( useObjectHeadingColorRowsDBE, onObjectHeadingColorToggleRows )

   string dummyList[] = {}
   objectHeadingColorListRowsDBE = listView( excelExportDB, 0, 200, 10, dummyList )

   exportHeadingNumberDBE->"top"->"inside"->excelExportTabDBE
   exportHeadingNumberDBE->"left"->"inside"->excelExportTabDBE
   exportHeadingNumberDBE->"bottom"->"unattached"

   setOutlineDBE->"left"->"form"
   useDeletedObjectRowsColorDBE->"left"->"form"

   objectHeadingColorFrameRowsDBE->"top"->"inside"->excelExportTabDBE
   objectHeadingColorFrameRowsDBE->"left"->"spaced"->exportHeadingNumberDBE
   objectHeadingColorFrameRowsDBE->"right"->"unattached"

   useObjectHeadingColorRowsDBE->"top"->"inside"->objectHeadingColorFrameRowsDBE
   useObjectHeadingColorRowsDBE->"left"->"inside"->objectHeadingColorFrameRowsDBE

   objectHeadingColorListRowsDBE->"top"->"spaced"->useObjectHeadingColorRowsDBE
   objectHeadingColorListRowsDBE->"left"->"inside"->objectHeadingColorFrameRowsDBE
   objectHeadingColorListRowsDBE->"bottom"->"inside"->objectHeadingColorFrameRowsDBE
   objectHeadingColorListRowsDBE->"right"->"inside"->objectHeadingColorFrameRowsDBE

   int index = 0
   put( excelExportRowsTabSkip, index++, exportHeadingNumberDBE )
   put( excelExportRowsTabSkip, index++, setOutlineDBE )
   put( excelExportRowsTabSkip, index++, useDeletedObjectRowsColorDBE )
   put( excelExportRowsTabSkip, index++, objectHeadingColorFrameRowsDBE )
   put( excelExportRowsTabSkip, index++, useObjectHeadingColorRowsDBE )
   put( excelExportRowsTabSkip, index++, objectHeadingColorListRowsDBE )
}

void doExcelExportOLETab( DB excelExportDB,
                          DBE excelExportTabDBE,
                          Skip copyViewsOLETabSkip
                        )
{
   OLEexportDBE = toggle( excelExportDB, OLEexportLabel, XLdoExportOLEObjects )
   set( OLEexportDBE, onOLEexportToggle )

   OLEreduceDBE = toggle( excelExportDB, OLEreduceLabel, XLdoReduceOLEObjects )

   OLESeparateSheetDBE = toggle( excelExportDB, OLEseparateSheetLabel, XLdoExportOLEToSeparateSheet )
   set( OLESeparateSheetDBE, onOLEseparateSheetToggle )
  
   OLESheetFieldDBE = field( excelExportDB, OLEsheetFieldLabel, XLOLEExportSheetNumber "", 3 )

   inactive OLESheetFieldDBE

   PictureExportDBE = toggle( excelExportDB, PictureExportLabel, XLdoExportPictures )

   OLEexportDBE->"top"->"inside"->excelExportTabDBE
   OLEexportDBE->"left"->"form"
   OLEexportDBE->"bottom"->"unattached"

   int index = 0
   put( excelExportOLETabSkip, index++, OLEexportDBE )
   put( excelExportOLETabSkip, index++, OLEreduceDBE )
   put( excelExportOLETabSkip, index++, OLESeparateSheetDBE )
   put( excelExportOLETabSkip, index++, OLESheetFieldDBE )
   put( excelExportOLETabSkip, index++, PictureExportDBE )
}

void doExcelExportPageSetupTab( DB excelExportDB,
                                DBE excelExportTabDBE,
                                Skip excelExportPageSetupTabSkip
                              )
{
   usePageSetupDBE = toggle( excelExportDB, usePageSetupLabel, XLdoPageSetup )
   set( usePageSetupDBE, onUsePageSetupToggle )

   pageSetupFrameDBE = frame( excelExportDB, pageSetupFrameLabel, 50, 100 )

   string dummyList[] = {}
   savedPageLayoutListDBE = listView( excelExportDB, 0, 150, 10, dummyList )
   set( savedPageLayoutListDBE, onPageSetupSelect, onPageSetupDeselect, onPageSetupActivate )

   paperSizeDBE = choice( excelExportDB, paperSizeLabel, DOORS_PAGE_SETUP_PAPER_SIZES, 0 )
   set( paperSizeDBE, onPaperSizeToggle )

   // Read-only Width and Height fields
   paperWidth_RO_DBE  = field( excelExportDB, paperWidthLabel,  "",  3, true )
   paperHeight_RO_DBE = field( excelExportDB, paperHeightLabel, "", 3, true )

   // Writable Width and Height fields for Custom paper size entry
   paperWidthDBE  = field( excelExportDB, paperWidthLabel,  "", 3 )
   paperHeightDBE = field( excelExportDB, paperHeightLabel, "", 3 )

   // Column Titles
   pagePrintColumnTitleRowsDBE =
      verticalRadioBox( excelExportDB, columnTitlesLabel, columnTitlesOptions, 0 )
   
   // Margins
   marginsDBE = label( excelExportDB, marginsLabel )
   marginLeftDBE = field( excelExportDB, marginLeftLabel, "", 3 )
   marginRightDBE  = field( excelExportDB, marginRightLabel, "", 3 )
   marginTopDBE = field( excelExportDB, marginTopLabel, "", 3 )
   marginBottomDBE = field( excelExportDB, marginBottomLabel, "", 3 )

   // Page Orientation
   pageOrientationDBE =
      verticalRadioBox( excelExportDB, pageOrientationLabel, pageOrientationOptions, DOORS_PAGE_ORIENTATION_PORTRAIT )
   set( pageOrientationDBE, onOrientationToggle )

   // Header Footer Print Option Labels
   printOptionLabel0DBE = label( excelExportDB, hfPrintOptionsLabel )
   printOptionLabel1DBE = label( excelExportDB, hfPrintOptions1 )
   printOptionLabel2DBE = label( excelExportDB, hfPrintOptions2 )
   printOptionLabel3DBE = label( excelExportDB, hfPrintOptions3 )
   printOptionLabel4DBE = label( excelExportDB, hfPrintOptions4 )
   printOptionLabel5DBE = label( excelExportDB, hfPrintOptions5 )
   printOptionLabel6DBE = label( excelExportDB, hfPrintOptions6 )
   printOptionLabel7DBE = label( excelExportDB, hfPrintOptions7 )
   printOptionLabel8DBE = label( excelExportDB, hfPrintOptions8 )
   printOptionLabel9DBE = label( excelExportDB, hfPrintOptions9 )
   printOptionLabel10DBE = label( excelExportDB, hfPrintOptions10 )
   printOptionLabel11DBE = label( excelExportDB, hfPrintOptions11 )

   // Headers and Footers
   bodyLeftHeaderDBE   = field( excelExportDB, bodyHeaderLabel, "", 20 )
   bodyCenterHeaderDBE = field( excelExportDB, "",              "", 20 )
   bodyRightHeaderDBE  = field( excelExportDB, "",              "", 20 )
   bodyLeftFooterDBE   = field( excelExportDB, bodyFooterLabel, "", 20 )
   bodyCenterFooterDBE = field( excelExportDB, "",              "", 20 )
   bodyRightFooterDBE  = field( excelExportDB, "",              "", 20 )

   usePageSetupDBE->"top"->"inside"->excelExportTabDBE
   usePageSetupDBE->"left"->"inside"->excelExportTabDBE

   pageSetupFrameDBE->"top"->"spaced"->usePageSetupDBE
   pageSetupFrameDBE->"left"->"inside"->excelExportTabDBE
   pageSetupFrameDBE->"bottom"->"inside"->excelExportTabDBE
   pageSetupFrameDBE->"right"->"unattached"

   savedPageLayoutListDBE->"top"->"inside"->pageSetupFrameDBE
   savedPageLayoutListDBE->"left"->"inside"->pageSetupFrameDBE
   savedPageLayoutListDBE->"bottom"->"inside"->pageSetupFrameDBE
   savedPageLayoutListDBE->"right"->"inside"->pageSetupFrameDBE

   paperSizeDBE->"left"->"spaced"->pageSetupFrameDBE
   paperSizeDBE->"top"->"aligned"->usePageSetupDBE
   paperSizeDBE->"left"->"spaced"->pageSetupFrameDBE
   paperSizeDBE->"bottom"->"unattached"

   paperWidth_RO_DBE->"left"->"spaced"->paperSizeDBE
   paperWidth_RO_DBE->"top"->"aligned"->paperSizeDBE
   paperHeight_RO_DBE->"left"->"spaced"->paperWidth_RO_DBE
   paperHeight_RO_DBE->"top"->"aligned"->paperWidth_RO_DBE
   paperWidthDBE->"left"->"spaced"->paperSizeDBE
   paperWidthDBE->"top"->"aligned"->paperSizeDBE
   paperHeightDBE->"left"->"spaced"->paperWidthDBE
   paperHeightDBE->"top"->"aligned"->paperWidthDBE
   marginsDBE->"left"->"spaced"->paperSizeDBE
   marginsDBE->"top"->"spaced"->paperSizeDBE
   marginLeftDBE->"left"->"spaced"->paperSizeDBE
   marginLeftDBE->"top"->"flush"->marginsDBE
   marginRightDBE->"left"->"spaced"->marginLeftDBE
   marginRightDBE->"top"->"aligned"->marginLeftDBE
   marginTopDBE->"left"->"aligned"->marginLeftDBE
   marginTopDBE->"top"->"spaced"->marginLeftDBE
   marginBottomDBE->"left"->"spaced"->marginTopDBE
   marginBottomDBE->"top"->"aligned"->marginTopDBE
   pageOrientationDBE->"left"->"spaced"->paperSizeDBE
   pageOrientationDBE->"top"->"aligned"->marginsDBE
   pagePrintColumnTitleRowsDBE->"left"->"spaced"->paperHeightDBE

   marginLeftDBE->"left"->"spaced"->pageSetupFrameDBE
   marginsDBE->"left"->"spaced"->pageSetupFrameDBE
   printOptionLabel0DBE->"left"->"spaced"->pageSetupFrameDBE
   printOptionLabel1DBE->"left"->"spaced"->pageSetupFrameDBE
   printOptionLabel1DBE->"top"->"flush"->printOptionLabel0DBE
   printOptionLabel2DBE->"left"->"spaced"->printOptionLabel1DBE
   printOptionLabel2DBE->"top"->"aligned"->printOptionLabel1DBE
   printOptionLabel3DBE->"left"->"spaced"->printOptionLabel2DBE
   printOptionLabel3DBE->"top"->"aligned"->printOptionLabel1DBE
   printOptionLabel4DBE->"left"->"spaced"->printOptionLabel3DBE
   printOptionLabel4DBE->"top"->"aligned"->printOptionLabel1DBE
   printOptionLabel5DBE->"left"->"aligned"->printOptionLabel1DBE
   printOptionLabel5DBE->"top"->"flush"->printOptionLabel1DBE
   printOptionLabel6DBE->"left"->"aligned"->printOptionLabel2DBE
   printOptionLabel6DBE->"top"->"aligned"->printOptionLabel5DBE   
   printOptionLabel7DBE->"left"->"aligned"->printOptionLabel3DBE
   printOptionLabel7DBE->"top"->"aligned"->printOptionLabel5DBE    
   printOptionLabel8DBE->"left"->"aligned"->printOptionLabel1DBE
   printOptionLabel8DBE->"top"->"flush"->printOptionLabel5DBE    
   printOptionLabel9DBE->"left"->"aligned"->printOptionLabel2DBE
   printOptionLabel9DBE->"top"->"aligned"->printOptionLabel8DBE
   printOptionLabel10DBE->"left"->"aligned"->printOptionLabel3DBE
   printOptionLabel10DBE->"top"->"aligned"->printOptionLabel8DBE
   printOptionLabel11DBE->"left"->"aligned"->printOptionLabel4DBE
   printOptionLabel11DBE->"top"->"aligned"->printOptionLabel8DBE
   bodyLeftHeaderDBE->"left"->"spaced"->pageSetupFrameDBE
   bodyCenterHeaderDBE->"left"->"spaced"->bodyLeftHeaderDBE
   bodyCenterHeaderDBE->"top"->"aligned"->bodyLeftHeaderDBE
   bodyRightHeaderDBE->"left"->"spaced"->bodyCenterHeaderDBE
   bodyRightHeaderDBE->"top"->"aligned"->bodyCenterHeaderDBE
   bodyLeftFooterDBE->"left"->"aligned"->bodyLeftHeaderDBE
   bodyLeftFooterDBE->"top"->"flush"->bodyLeftHeaderDBE
   bodyCenterFooterDBE->"left"->"spaced"->bodyLeftFooterDBE
   bodyCenterFooterDBE->"top"->"aligned"->bodyLeftFooterDBE
   bodyRightFooterDBE->"left"->"spaced"->bodyCenterFooterDBE
   bodyRightFooterDBE->"top"->"aligned"->bodyCenterFooterDBE
   bodyRightFooterDBE->"bottom"->"unattached"

   setPageLayoutUISettingsFromSavedPageLayout( XLdefaultLayout )

   int index = 0
   put( excelExportPageSetupTabSkip, index++, pageSetupFrameDBE )
   put( excelExportPageSetupTabSkip, index++, usePageSetupDBE )
   put( excelExportPageSetupTabSkip, index++, savedPageLayoutListDBE )
   put( excelExportPageSetupTabSkip, index++, paperSizeDBE )
   put( excelExportPageSetupTabSkip, index++, pageOrientationDBE )
   put( excelExportPageSetupTabSkip, index++, paperWidthDBE )
   put( excelExportPageSetupTabSkip, index++, paperHeightDBE )
   put( excelExportPageSetupTabSkip, index++, paperWidth_RO_DBE )
   put( excelExportPageSetupTabSkip, index++, paperHeight_RO_DBE )
   put( excelExportPageSetupTabSkip, index++, pagePrintColumnTitleRowsDBE )
   put( excelExportPageSetupTabSkip, index++, marginsDBE )
   put( excelExportPageSetupTabSkip, index++, marginLeftDBE )
   put( excelExportPageSetupTabSkip, index++, marginRightDBE )
   put( excelExportPageSetupTabSkip, index++, marginTopDBE )
   put( excelExportPageSetupTabSkip, index++, marginBottomDBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel0DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel1DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel2DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel3DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel4DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel5DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel6DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel7DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel8DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel9DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel10DBE )
   put( excelExportPageSetupTabSkip, index++, printOptionLabel11DBE )
   put( excelExportPageSetupTabSkip, index++, bodyLeftHeaderDBE )
   put( excelExportPageSetupTabSkip, index++, bodyCenterHeaderDBE )
   put( excelExportPageSetupTabSkip, index++, bodyRightHeaderDBE )
   put( excelExportPageSetupTabSkip, index++, bodyLeftFooterDBE )
   put( excelExportPageSetupTabSkip, index++, bodyCenterFooterDBE )
   put( excelExportPageSetupTabSkip, index++, bodyRightFooterDBE )
}	

void doExcelExportTemplateTab( DB excelExportDB,
                               DBE excelExportTabDBE,
                               Skip copyViewsTemplateTabSkip
                             )
{
   exportFileChoiceDBE = verticalRadioBox( excelExportDB, exportFileChoiceLabel, sheetOptions, 0 )
   set( exportFileChoiceDBE, fileRadioUpdate )

   exportFileLabelDBE = label( excelExportDB, exportFileLabel )

   exportFileSelectDBE = fileName( excelExportDB, "c:\\", "*.xl*", "Excel files", true )

   inactive exportFileLabelDBE
   inactive exportFileSelectDBE

   exportFileChoiceDBE->"top"->"inside"->excelExportTabDBE
   exportFileChoiceDBE->"left"->"inside"->excelExportTabDBE
   exportFileChoiceDBE->"bottom"->"unattached"

   int index = 0
   put( excelExportTemplateTabSkip, index++, exportFileChoiceDBE )
   put( excelExportTemplateTabSkip, index++, exportFileLabelDBE )
   put( excelExportTemplateTabSkip, index++, exportFileSelectDBE )
}

void doExcelExportAboutTab( DB excelExportDB,
                            DBE excelExportTabDBE,
                            Skip excelExportAboutTabSkip
                        )
{
   excelExportAboutDBE = richText( excelExportDB, excelTitle, ENHANCED_EXPORT_TO_EXCEL_ABOUT, 40, 239, true )

   excelExportAboutDBE->"top"->"inside"->excelExportTabDBE
   excelExportAboutDBE->"left"->"inside"->excelExportTabDBE
   excelExportAboutDBE->"right"->"inside"->excelExportTabDBE
   excelExportAboutDBE->"bottom"->"unattached"

   int index = 0
   put( excelExportAboutTabSkip, index++, excelExportAboutDBE )
}


void makeExcelExportUI( Module m ) {

   if ( null excelExportDB ) {

      excelExportDB = create( m, excelTitle, styleFixed | styleCentered )

      excelExportTabSkip = create

      excelExportTabDBE = tab( excelExportDB,
                               excelExportTabLabels,
                               200,
                               300,
                               excelExportTabSelectFn
                             )

      excelExportSheetTabSkip = create
      doExcelExportSheetTab( excelExportDB, excelExportTabDBE, excelExportSheetTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_SHEET_TAB, excelExportSheetTabSkip )

      excelExportHeaderTabSkip = create
      doExcelExportHeaderTab( excelExportDB, excelExportTabDBE, excelExportHeaderTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_HEADER_TAB, excelExportHeaderTabSkip )

      excelExportColumnsTabSkip = create
      doExcelExportColumnsTab( excelExportDB, excelExportTabDBE, excelExportColumnsTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_COLUMNS_TAB, excelExportColumnsTabSkip )

      excelExportRowsTabSkip = create
      doExcelExportRowsTab( excelExportDB, excelExportTabDBE, excelExportRowsTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_ROWS_TAB, excelExportRowsTabSkip )

      excelExportOLETabSkip = create
      doExcelExportOLETab( excelExportDB, excelExportTabDBE, excelExportOLETabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_OLE_TAB, excelExportOLETabSkip )

      excelExportPageSetupTabSkip = create
      doExcelExportPageSetupTab( excelExportDB, excelExportTabDBE, excelExportPageSetupTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_PAGE_SETUP_TAB, excelExportPageSetupTabSkip )

      excelExportTemplateTabSkip = create
      doExcelExportTemplateTab( excelExportDB, excelExportTabDBE, excelExportTemplateTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_TEMPLATE_TAB, excelExportTemplateTabSkip )

      excelExportAboutTabSkip = create
      doExcelExportAboutTab( excelExportDB, excelExportTabDBE, excelExportAboutTabSkip )
      put( excelExportTabSkip, EXCEL_EXPORT_ABOUT_TAB, excelExportAboutTabSkip )

      label( excelExportDB, "" )

      //exportButtonDBE = apply( excelExportDB, exportButtonLabel, exportModuleToExcel )

      excelTrigger = trigger( module, close, 10, closeModuleTrig )

      //showSelectedTab( excelExportTabSkip, EXCEL_EXPORT_SHEET_TAB )

      close( excelExportDB, true, closeDB )
      
      realize excelExportDB

      insertColumn( objectHeadingColorListRowsDBE, 0, colorRowsLabel, 190, iconNone )
      updateColorList( objectHeadingColorListRowsDBE, XLObjectHeadingRowsColor )

      insertColumn( headerRowsColorListDBE, 0, colorHeaderslabel, 190, iconNone )
      updateColorList( headerRowsColorListDBE, XLheaderRowsColor )

      insertColumn( savedPageLayoutListDBE, 0, pageSetupListLabel, 150, iconNone )
      updatePageLayouts
   }

   //showSelectedTab( excelExportTabSkip, EXCEL_EXPORT_SHEET_TAB )
   block excelExportDB
}

//exportPreChecks( current Module )
//makeExcelExportUI( current Module )