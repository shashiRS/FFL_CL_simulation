import os
import shutil
import cv2
import subprocess as sp
import numpy as np

#This method creates a web browser compatible mp4 video file out of a sequence of a camera stream of a recording
#specify the inputs of rec2mp4 :
#
#   path2RecExtractor   - absolute path to RecFileExtractor.exe (without the file name "RecFileExtractor.exe")
#   recordingFile       - absolute path to the .rec/.rrec file including the file name
#   outputFolder        - absolute path to the output folder for the .mpf
#   eventTime           - the time stamp of the relevant event
#   preTime=5000000     - the time span before eventTime which the video sequence should start with [ms]
#   postTime = 5000000  - the time span after eventTime after which the video sequence should end [ms]
#   frameRate=15        - the frame rate of the camera
#   videoSource="video" - the name of the video source in the .rec/.rrec file

TMP_AVI_DIR = '.tmp_avi'

def loadRec(path2RecExtractor, recordingFile, eventTime, preTime=5000000, postTime=5000000, videoSource="video"):
    """
    Loads a recording and returns a list of frames (a frame is a numpy array).

    Well this function does something pretty ugly: an avi is generated from the recording by an external RecExtractor. This
    avi will be read by open cv and converted to a numpy array.

    @param path2RecExtractor path to the RecFileExtractor.exe
    @param recodingFile rec file to load
    @param evenTime TBD
    @param preTime TBD
    @param postTime TBD
    @param videoSource TBD
    @return a tuple with a list of numpy arrays for the frames as first element and the framrate as second element
        Well, framerate is always 20fps but the recordings are actually made with 15 fps... strange world O.o
    """

    # Get file name of recording
    recFileName = os.path.basename(recordingFile)

    # Create a temporary avi file
    recExtractorPath = os.path.join(path2RecExtractor, 'RecFileExtractor.exe')
    command = [recExtractorPath, '{:s}'.format(recordingFile), '/T:{:d}'.format(int(eventTime - preTime)),
               '/U:{:d}'.format(int(eventTime + postTime)), '/F:avi', '/O:{:s}'.format(TMP_AVI_DIR), '/D:{:s}'.format(videoSource)]
    # Redirect output to null
    with open(os.devnull, 'w') as FNULL:
        sp.call(command, stdout=FNULL, stderr=sp.STDOUT)
        FNULL.close()

    # Get path to the temporary avi
    aviFilePath = None
    for filename in os.listdir(TMP_AVI_DIR):
        if filename.endswith('.avi'):
            aviFilePath = os.path.join(TMP_AVI_DIR, filename)
            break

    if aviFilePath is None:
        shutil.rmtree(TMP_AVI_DIR)
        raise RuntimeError("No AVI file found in {:s}".format(TMP_AVI_DIR))

    # Read temporary avi
    cap = cv2.VideoCapture(aviFilePath) # TODO: check filename of temp avi
    if not cap.isOpened():
        shutil.rmtree(rec2mp4.TMP_AVI_DIR)
        raise RuntimeError('Cannot open "{:s}"!'.format(aviFilePath))
    framerate = cap.get(cv2.cv.CV_CAP_PROP_FPS)

    frames = []
    for i in range(int(cap.get(cv2.cv.CV_CAP_PROP_FRAME_COUNT))):
        bgrImg = np.asarray(cap.read()[1])
        rgbImg = np.array(bgrImg.shape)
        rgbImg[:, :, 0] = bgrImg[:, :, 2]
        rgbImg[:, :, 1] = bgrImg[:, :, 1]
        rgbImg[:, :, 2] = bgrImg[:, :, 0]
        frames.append()
    cap.release()

    # Clean up
    shutil.rmtree(TMP_AVI_DIR)

    return (frames, framerate)


def rec2mp4(path2RecExtractor, recordingFile, outputFolder, eventTime, preTime=5000000, postTime = 5000000, frameRate=15,videoSource="video"):
    curDir = os.getcwd();
    recFN = os.path.basename(recordingFile)

    # Generate AVI file
    outTmpFolder = os.path.join(outputFolder, '{:s}_TmpVideoOut'.format(recFN))
    keepFolder = os.path.isdir(outTmpFolder); #keep folder if it already exists
    os.chdir(path2RecExtractor)
    command = ['RecFileExtractor.exe', recordingFile, '/T:{:d}'.format(eventTime - preTime), '/U:{:d}'.format(int(eventTime + postTime)),
               '/F:avi', '/O:{:s}'.format(outTmpFolder), '/D:{:s}'.format(videoSource)]
    try:
        # Redirect output of subprocess to null to prevent flush of console
        with open(os.devnull, 'w') as FNULL:
            sp.check_call(command, stdout=FNULL, stderr=sp.STDOUT)
            FNULL.close()

        # Generate thumbnail
        timeDev = int(1.0 / frameRate*1000000 - 1)
        command = ['RecFileExtractor.exe', recordingFile, '/T:{:d}'.format(eventTime), '/U:{:d}'.format(int(eventTime + timeDev)),
                   '/F:bmp', '/O:{:s}'.format(outTmpFolder), '/D:{:s}'.format(videoSource)]
        with open(os.devnull, 'w') as FNULL:
            sp.check_call(command, stdout=FNULL, stderr=sp.STDOUT)
            FNULL.close()

        # Get avi file and thumbnail to copy it to rename it correctly
        for file in os.listdir(outTmpFolder):
            if file.endswith(".avi"):
                actualOutFile = file
            elif file.endswith(".bmp"):
                actualPosterFile = file
        shutil.move(os.path.join(outTmpFolder, actualPosterFile), os.path.join(outputFolder, '{:s}.bmp'.format(recFN)))
        # Run ffmpeg to create a mp4 from the avi
        os.chdir(curDir)
        frameRateRatio = 20.0/frameRate; # RecFileExtractor always creates videos with 20 fps
        command = ['ffmpeg.exe', '-loglevel', 'quiet', '-i', '{:s}'.format(os.path.join(outTmpFolder, actualOutFile)),
                   '-filter:v', 'setpts={:f}*PTS'.format(frameRateRatio), '-r', '{:d}'.format(int(frameRate)),
                   '-vcodec', 'libx264', '-pix_fmt', 'yuv420p', '-movflags', 'faststart',
                   '{:s}'.format(os.path.join(outputFolder, '{:s}.mp4'.format(recFN)))]

       # command = 'ffmpeg.exe -loglevel quiet -i "%s" -vcodec libx264  -sameq "%s" ' % (outTmpFolder+"\\"+actualOutFile, outputFolder + "\\"+ recFN + ".mp4")
        sp.call(command)
    finally:
        if ((os.path.isdir(outTmpFolder)) and (not keepFolder)): #remove temporary avi folder
            shutil.rmtree(outTmpFolder)
    
if __name__ == "__main__":
    #an example of how to use the extractor
    path2RecExtractor = "D:\\Projects\\03_Sideprojects\\08_ReportVideo\\rfe"
    recordingFile ="D:\\Projects\\03_Sideprojects\\08_ReportVideo\\rec\\Test_rate.rrec"
    outputDir = "D:\\Projects\\03_Sideprojects\\08_ReportVideo\\Videos"

    eventTS = 1416497717140059 + 30000000
    rec2mp4(path2RecExtractor, recordingFile, outputDir, eventTS )